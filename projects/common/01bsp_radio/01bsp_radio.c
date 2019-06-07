/**
\brief This program shows the use of the "radio" bsp module.

Since the bsp modules for different platforms have the same declaration, you
can use this project with any platform.

After loading this program, your board will switch on its radio on frequency
CHANNEL.

While receiving a packet (i.e. from the start of frame event to the end of 
frame event), it will turn on its sync LED.

Every TIMER_PERIOD, it will also send a packet containing LENGTH_PACKET bytes
set to ID. While sending a packet (i.e. from the start of frame event to the
end of frame event), it will turn on its error LED.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, August 2014.
*/

#include <stdlib.h>
#include <stdio.h>
#include "board.h"
#include "radio.h"
#include "leds.h"
#include "sctimer.h"
#include "i2c.h"
#include "mpu.h"
#include "inv_mpu.h"
#include "gptimer.h"
#include <headers/hw_memmap.h>
#include <headers/hw_ioc.h>
#include <headers/hw_ssi.h>
#include <headers/hw_sys_ctrl.h>
#include <headers/hw_gptimer.h>
#include <headers/hw_ints.h>
#include <headers/hw_rfcore_sfr.h>
#include <headers/hw_rfcore_sfr.h>
#include <headers/hw_rfcore_xreg.h>
#include <headers/MPU9250_RegisterMap.h>
#include <source/interrupt.h>
#include <source/ioc.h>
#include <source/gpio.h>
#include <source/gptimer.h>
#include <source/sys_ctrl.h>


//=========================== defines =========================================

#define LENGTH_PACKET   8+LENGTH_CRC ///< maximum length is 127 bytes --> TODO: use 100
#define OLD_LENGTH_PACKET 8+LENGTH_CRC
#define CHANNEL         16             ///< 11=2.405GHz
#define TX_CHANNEL	16	       /// tx channel of individual mote
#define TIMER_PERIOD    0x3ff         ///< 0xff ~ 125 Hz < 0x3ff ~ 30 Hz < 0x7fff ~ 1 Hz
#define ID              0xff           ///< byte sent in the packets
#define isTx	true
#define NUM_ATTEMPTS	3	       ///<number of times packet is resent, needs to match number of motes for multichan experiments
#define MOTE_NUM	1	           // index that sets rx mote channel
#define CHANNEL_HOP	5		//number of channels to hop by
#define LEFT_SENSOR_HIGH 2
#define RIGHT_SENSOR_HIGH 1

#define PULSE_TRACK_COUNT 5

#define MIN_SYNC_PERIOD_US 52
#define MAX_SYNC_PERIOD_US 138

#define PI 3.14159265358979323846264338f
#define SWEEP_PERIOD_US 8333.333333f

#define CLOCK_SPEED_MHZ 32.0f
#define MAX_SAMPLES 250

#define LEFT_LIM 1.601823580621385f
#define RIGHT_LIM 1.5278037261196f

#define DT 1.0f // FIXME: figure out kalman time step in us?
#define IMU_ADDRESS 0x69
#define LOW_POWER 0
#define ACCEL_SENS 16384.0f // TODO: check this, may have to be more precise??
#define GYRO_SENS 65.536f

#define PAGE_SIZE                2048
#define PAGE_TO_ERASE            14
#define PAGE_TO_ERASE_START_ADDR (FLASH_BASE + (PAGE_TO_ERASE * PAGE_SIZE))
#define DATAPOINTS			100
#define FLASH_PAGES_TOUSE	50
#define FLASH_PAGE_STORAGE_START 100 //first flash page to start at. TODO: make sure this doesn't overlap

//=========================== typedef =========================================

typedef enum {
   Sync, Vert, Horiz,
} Pulses;

typedef struct {
   uint32_t                rise;
   uint32_t                fall;
   int                     type; // -1 for unclassified, 0 for Sync, 1 for Horiz, 2 for Vert
} pulse_t;

typedef struct {
	double                    phi;
	double                  theta;
	double                 r_vert;
	double				 r_horiz;
	uint8_t               asn[5];
	int					   valid;
} location_t;

//=========================== variables =======================================
//bool isTx = true;

uint32_t tx_count; //count used to keep track of how many packet resends have happened
uint32_t tx_packet_count; //counter for verifying packet contents
volatile uint32_t rx_packet_count;
uint32_t debounce_complete; // used to debounce button press in interrupt handler

bool moving_right;
volatile bool transmitting;
volatile bool finished;

enum {
   APP_FLAG_START_FRAME = 0x01,
   APP_FLAG_END_FRAME   = 0x02,
   APP_FLAG_TIMER       = 0x04,
};

typedef enum {
   APP_STATE_TX         = 0x01,
   APP_STATE_RX         = 0x02,
} app_state_t;

typedef struct {
   uint8_t              num_startFrame;
   uint8_t              num_endFrame;
   uint8_t              num_timer;
} app_dbg_t;

app_dbg_t app_dbg;

typedef struct {
   uint8_t              flags;
   app_state_t          state;
   uint8_t              packet[LENGTH_PACKET];
   uint8_t              packet_len;
    int8_t              rxpk_rssi;
   uint8_t              rxpk_lqi;
   bool                 rxpk_crc;
} app_vars_t;

app_vars_t app_vars;
//===========================localization======================================
void precision_timers_init(void);
void input_edge_timers_init(void);
void pulse_handler_gpio_a(void);
void pulse_handler_gpio_d(void);

/* Hardware constants. */
static const uint32_t gptmTimerBase = GPTIMER1_BASE;
static const uint32_t timer_cnt = 32000000;

static const uint32_t gptmEdgeTimerBase = GPTIMER3_BASE;
static const uint32_t gptmFallingEdgeInt = INT_TIMER3B;
static const uint32_t gptmFallingEdgeEvent = GPTIMER_CAPB_EVENT;

static const uint32_t gptmTimer3AReg = 0x40033048;
static const uint32_t gptmTimer3BReg = 0x4003304C;

static const uint32_t timer_cnt_32 = 0xFFFFFFFF;
static const uint32_t timer_cnt_16 = 0xFFFF;
static const uint32_t timer_cnt_24 = 0xFFFFFF;

double accel;
volatile double azimuth;
volatile double elevation;
volatile bool new_data;
volatile bool update;
uint32_t ekf_fail;

static const double sweep_velocity = PI / SWEEP_PERIOD_US;

bool imu_ready;
double x; double y; double z;

volatile double valid_angles[MAX_SAMPLES][2];
volatile pulse_t pulses[PULSE_TRACK_COUNT];
volatile uint8_t modular_ptr;
volatile uint8_t pulse_count;
volatile uint32_t samples;

volatile uint32_t broken1;
volatile uint32_t broken2;
volatile uint32_t broken3;

volatile uint32_t test_count;
uint32_t estimate_transmit_count;
volatile uint32_t overflow_count;
volatile uint32_t prev_time; volatile uint32_t time;
volatile uint32_t init_time; volatile bool init_time_set;

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);
void     cb_timer(void);
void	 configure_pins(void);

// ATAN2: adapted from https://opensource.apple.com/source/Libm/Libm-315/Source/Intel/atan.c

static volatile const double Tiny = 0x1p-1022;

/*	double my_atan(double x).

	(This routine appears below, following subroutines.)

	Notes:

		Citations in parentheses below indicate the source of a requirement.

		"C" stands for ISO/IEC 9899:TC2.

		The Open Group specification (IEEE Std 1003.1, 2004 edition) adds no
		requirements since it defers to C and requires errno behavior only if
		we choose to support it by arranging for "math_errhandling &
		MATH_ERRNO" to be non-zero, which we do not.

	Return value:

		For arctangent of +/- zero, return zero with same sign (C F.9 12 and
		F.9.1.3).

		For arctangent of +/- infinity, return +/- pi/2 (C F.9.1.3).

		For a NaN, return the same NaN (C F.9 11 and 13).  (If the NaN is a
		signalling NaN, we return the "same" NaN quieted.)

		Otherwise:

			If the rounding mode is round-to-nearest, return arctangent(x)
			faithfully rounded.  This is not proven but seems likely.
			Generally, the largest source of errors is the evaluation of the
			polynomial using double precision.  Some analysis might bound this
			and prove faithful rounding.  The largest observed error is .814
			ULP.

			Return a value in [-pi/2, +pi/2] (C 7.12.4.3 3).
		
			Not implemented:  In other rounding modes, return arctangent(x)
			possibly with slightly worse error, not necessarily honoring the
			rounding mode (Ali Sazegari narrowing C F.9 10).

	Exceptions:

		Raise underflow for a denormal result (C F.9 7 and Draft Standard for
		Floating-Point Arithmetic P754 Draft 1.2.5 9.5).  If the input is the
		smallest normal, underflow may or may not be raised.  This is stricter
		than the older 754 standard.

		May or may not raise inexact, even if the result is exact (C F.9 8).

		Raise invalid if the input is a signalling NaN (C 5.2.4.2.2 3, in spite
		of C 4.2.1), but not if the input is a quiet NaN (C F.9 11).

		May not raise exceptions otherwise (C F.9 9).

	Properties:

		Not proven:  Monotonic.
*/


// Return arctangent(x) given that 2 < x, with the same properties as atan.
static double Tail(double x)
{
	{
		static const double HalfPi = 0x3.243f6a8885a308d313198a2e037ap-1;

		// For large x, generate inexact and return pi/2.
		if (0x1p53 <= x)
			return HalfPi + Tiny;
		if (isnan(x))
			return x - x;
	}

	static const double p03 = -0x1.5555555554A51p-2;
	static const double p05 = +0x1.999999989EBCAp-3;
	static const double p07 = -0x1.249248E1422E3p-3;
	static const double p09 = +0x1.C71C5EDFED480p-4;
	static const double p11 = -0x1.745B7F2D72663p-4;
	static const double p13 = +0x1.3AFD7A0E6EB75p-4;
	static const double p15 = -0x1.104146B1A1AE8p-4;
	static const double p17 = +0x1.D78252FA69C1Cp-5;
	static const double p19 = -0x1.81D33E401836Dp-5;
	static const double p21 = +0x1.007733E06CEB3p-5;
	static const double p23 = -0x1.83DAFDA7BD3FDp-7;

	static const double p000 = +0x1.921FB54442D18p0;
	static const double p001 = +0x1.1A62633145C07p-54;

	double y = 1/x;

	// Square y.
	double y2 = y * y;

	return p001 - ((((((((((((
		+ p23) * y2
		+ p21) * y2
		+ p19) * y2
		+ p17) * y2
		+ p15) * y2
		+ p13) * y2
		+ p11) * y2
		+ p09) * y2
		+ p07) * y2
		+ p05) * y2
		+ p03) * y2 * y + y) + p000;
}


/*	Return arctangent(x) given that 0x1p-27 < |x| <= 1/2, with the same
	properties as atan.
*/
static double atani0(double x)
{
	static const double p03 = -0x1.555555555551Bp-2;
	static const double p05 = +0x1.99999999918D8p-3;
	static const double p07 = -0x1.2492492179CA3p-3;
	static const double p09 = +0x1.C71C7096C2725p-4;
	static const double p11 = -0x1.745CF51795B21p-4;
	static const double p13 = +0x1.3B113F18AC049p-4;
	static const double p15 = -0x1.10F31279EC05Dp-4;
	static const double p17 = +0x1.DFE7B9674AE37p-5;
	static const double p19 = -0x1.A38CF590469ECp-5;
	static const double p21 = +0x1.56CDB5D887934p-5;
	static const double p23 = -0x1.C0EB85F543412p-6;
	static const double p25 = +0x1.4A9F5C4724056p-7;

	// Square x.
	double x2 = x * x;

	return ((((((((((((
		+ p25) * x2
		+ p23) * x2
		+ p21) * x2
		+ p19) * x2
		+ p17) * x2
		+ p15) * x2
		+ p13) * x2
		+ p11) * x2
		+ p09) * x2
		+ p07) * x2
		+ p05) * x2
		+ p03) * x2 * x + x;
}


/*	Return arctangent(x) given that 1/2 < x <= 3/4, with the same properties as
	atan.
*/
static double atani1(double x)
{
	static const double p00 = +0x1.1E00BABDEFED0p-1;
	static const double p01 = +0x1.702E05C0B8155p-1;
	static const double p02 = -0x1.4AF2B78215A1Bp-2;
	static const double p03 = +0x1.5D0B7E9E69054p-6;
	static const double p04 = +0x1.A1247CA5D9475p-4;
	static const double p05 = -0x1.519E110F61B54p-4;
	static const double p06 = +0x1.A759263F377F2p-7;
	static const double p07 = +0x1.094966BE2B531p-5;
	static const double p08 = -0x1.09BC0AB7F914Cp-5;
	static const double p09 = +0x1.FF3B7C531AA4Ap-8;
	static const double p10 = +0x1.950E69DCDD967p-7;
	static const double p11 = -0x1.D88D31ABC3AE5p-7;
	static const double p12 = +0x1.10F3E20F6A2E2p-8;

	double y = x - 0x1.4000000000027p-1;

	return ((((((((((((
		+ p12) * y
		+ p11) * y
		+ p10) * y
		+ p09) * y
		+ p08) * y
		+ p07) * y
		+ p06) * y
		+ p05) * y
		+ p04) * y
		+ p03) * y
		+ p02) * y
		+ p01) * y
		+ p00;
}


/*	Return arctangent(x) given that 3/4 < x <= 1, with the same properties as
	atan.
*/
static double atani2(double x)
{
	static const double p00 = +0x1.700A7C580EA7Ep-01;
	static const double p01 = +0x1.21FB781196AC3p-01;
	static const double p02 = -0x1.1F6A8499714A2p-02;
	static const double p03 = +0x1.41B15E5E8DCD0p-04;
	static const double p04 = +0x1.59BC93F81895Ap-06;
	static const double p05 = -0x1.63B543EFFA4EFp-05;
	static const double p06 = +0x1.C90E92AC8D86Cp-06;
	static const double p07 = -0x1.91F7E2A7A338Fp-08;
	static const double p08 = -0x1.AC1645739E676p-08;
	static const double p09 = +0x1.152311B180E6Cp-07;
	static const double p10 = -0x1.265EF51B17DB7p-08;
	static const double p11 = +0x1.CA7CDE5DE9BD7p-14;

	double y = x - 0x1.c0000000f4213p-1;

	return (((((((((((
		+ p11) * y
		+ p10) * y
		+ p09) * y
		+ p08) * y
		+ p07) * y
		+ p06) * y
		+ p05) * y
		+ p04) * y
		+ p03) * y
		+ p02) * y
		+ p01) * y
		+ p00;
}


/*	Return arctangent(x) given that 1 < x <= 4/3, with the same properties as
	atan.
*/
static double atani3(double x)
{
	static const double p00 = +0x1.B96E5A78C5C40p-01;
	static const double p01 = +0x1.B1B1B1B1B1B3Dp-02;
	static const double p02 = -0x1.AC97826D58470p-03;
	static const double p03 = +0x1.3FD2B9F586A67p-04;
	static const double p04 = -0x1.BC317394714B7p-07;
	static const double p05 = -0x1.2B01FC60CC37Ap-07;
	static const double p06 = +0x1.73A9328786665p-07;
	static const double p07 = -0x1.C0B993A09CE31p-08;
	static const double p08 = +0x1.2FCDACDD6E5B5p-09;
	static const double p09 = +0x1.CBD49DA316282p-13;
	static const double p10 = -0x1.0120E602F6336p-10;
	static const double p11 = +0x1.A89224FF69018p-11;
	static const double p12 = -0x1.883D8959134B3p-12;

	double y = x - 0x1.2aaaaaaaaaa96p0;

	return ((((((((((((
		+ p12) * y
		+ p11) * y
		+ p10) * y
		+ p09) * y
		+ p08) * y
		+ p07) * y
		+ p06) * y
		+ p05) * y
		+ p04) * y
		+ p03) * y
		+ p02) * y
		+ p01) * y
		+ p00;
}


/*	Return arctangent(x) given that 4/3 < x <= 5/3, with the same properties as
	atan.
*/
static double atani4(double x)
{
	static const double p00 = +0x1.F730BD281F69Dp-01;
	static const double p01 = +0x1.3B13B13B13B0Cp-02;
	static const double p02 = -0x1.22D719C06115Ep-03;
	static const double p03 = +0x1.C963C83985742p-05;
	static const double p04 = -0x1.135A0938EC462p-06;
	static const double p05 = +0x1.13A254D6E5B7Cp-09;
	static const double p06 = +0x1.DFAA5E77B7375p-10;
	static const double p07 = -0x1.F4AC1342182D2p-10;
	static const double p08 = +0x1.25BAD4D85CBE1p-10;
	static const double p09 = -0x1.E4EEF429EB680p-12;
	static const double p10 = +0x1.B4E30D1BA3819p-14;
	static const double p11 = +0x1.0280537F097F3p-15;

	double y = x - 0x1.8000000000003p0;

	return (((((((((((
		+ p11) * y
		+ p10) * y
		+ p09) * y
		+ p08) * y
		+ p07) * y
		+ p06) * y
		+ p05) * y
		+ p04) * y
		+ p03) * y
		+ p02) * y
		+ p01) * y
		+ p00;
}


/*	Return arctangent(x) given that 5/3 < x <= 2, with the same properties as
	atan.
*/
static double atani5(double x)
{
	static const double p00 = +0x1.124A85750FB5Cp+00;
	static const double p01 = +0x1.D59AE78C11C49p-03;
	static const double p02 = -0x1.8AD3C44F10DC3p-04;
	static const double p03 = +0x1.2B090AAD5F9DCp-05;
	static const double p04 = -0x1.881EC3D15241Fp-07;
	static const double p05 = +0x1.8CB82A74E0699p-09;
	static const double p06 = -0x1.3182219E21362p-12;
	static const double p07 = -0x1.2B9AD13DB35A8p-12;
	static const double p08 = +0x1.10F884EAC0E0Ap-12;
	static const double p09 = -0x1.3045B70E93129p-13;
	static const double p10 = +0x1.00B6A460AC05Dp-14;

	double y = x - 0x1.d555555461337p0;

	return ((((((((((
		+ p10) * y
		+ p09) * y
		+ p08) * y
		+ p07) * y
		+ p06) * y
		+ p05) * y
		+ p04) * y
		+ p03) * y
		+ p02) * y
		+ p01) * y
		+ p00;
}


// See documentation above.
double my_atan(double x)
{
	if (x < 0)
		if (x < -1)
			if (x < -5/3.)
				if (x < -2)
					return -Tail(-x);
				else
					return -atani5(-x);
			else
				if (x < -4/3.)
					return -atani4(-x);
				else
					return -atani3(-x);
		else
			if (x < -.5)
				if (x < -.75)
					return -atani2(-x);
				else
					return -atani1(-x);
			else
				if (x < -0x1.d12ed0af1a27fp-27)
					return atani0(x);
				else
					if (x <= -0x1p-1022)
						// Generate inexact and return x.
						return (Tiny + 1) * x;
					else
						if (x == 0)
							return x;
						else
							// Generate underflow and return x.
							return x*Tiny + x;
	else
		if (x <= +1)
			if (x <= +.5)
				if (x <= +0x1.d12ed0af1a27fp-27)
					if (x < +0x1p-1022)
						if (x == 0)
							return x;
						else
							// Generate underflow and return x.
							return x*Tiny + x;
					else
						// Generate inexact and return x.
						return (Tiny + 1) * x;
				else
					return atani0(x);
			else
				if (x <= +.75)
					return +atani1(+x);
				else
					return +atani2(+x);
		else
			if (x <= +5/3.)
				if (x <= +4/3.)
					return +atani3(+x);
				else
					return +atani4(+x);
			else
				if (x <= +2)
					return +atani5(+x);
				else
					return +Tail(+x);
}

double my_atan2(double y, double x) {
    if (x > 0) {
        return my_atan(y / x);
    } else if (x < 0 && y >= 0) {
        return my_atan(y / x) + PI;
    } else if (x < 0 && y < 0) {
        return my_atan(y / x) - PI;
    } else if (x == 0 && y > 0) {
        return PI / 2;
    } else if (x == 0 && y < 0) {
        return -PI / 2;
    } else if (x == 0 && y == 0) {
        return -1000000000000;
    }
}

float my_sqrt(float square)
{
    float root=square/3;
    int i;
    if (square <= 0) return 0;
    for (i=0; i<32; i++)
        root = (root + square / root) / 2;
    return root;
}

double reduce_digits(double f, int num) {
    if (num == 0) {
        return f;
    }
    return reduce_digits((f - ((double)((int) f)))*10, num-1);
}

//======================================START===========================================

void imu_init(void) {
    // initialize IMU
    imu_ready = false;

    // start bsp timer
    sctimer_set_callback(cb_timer);
    sctimer_setCompare(sctimer_readCounter()+TIMER_PERIOD);
    sctimer_enable();

    i2c_init();
    uint8_t readbyte;

    i2c_write_byte(IMU_ADDRESS, MPU9250_PWR_MGMT_1); // reset
    i2c_write_byte(IMU_ADDRESS, 0x80);

    i2c_write_byte(IMU_ADDRESS, MPU9250_PWR_MGMT_1); // enable/wake sensor
    i2c_write_byte(IMU_ADDRESS, 0x00);

    uint8_t bytes[2] = {MPU9250_PWR_MGMT_1, 0x01}; 
    i2c_write_bytes(IMU_ADDRESS, bytes, 2); // set gyro clock source  

    uint8_t *byteptr = &readbyte;

    i2c_write_byte(IMU_ADDRESS, MPU9250_PWR_MGMT_2); // reset
    i2c_read_byte(IMU_ADDRESS, byteptr);

    i2c_write_byte(IMU_ADDRESS, MPU9250_PWR_MGMT_2); // enable/wake sensor
    i2c_write_byte(IMU_ADDRESS, 0x00);

    i2c_write_byte(IMU_ADDRESS, MPU9250_PWR_MGMT_2);
    i2c_read_byte(IMU_ADDRESS, byteptr);
}

void get_scalar_accel(uint16_t *accel, uint32_t *timestamp) { // FIXME: put back when done debugging, is there a way to have this service an interrupt?
    uint8_t address = IMU_ADDRESS;
    uint8_t readbyte;  
    uint8_t *byteptr = &readbyte;

    int16_t ax; int16_t gx;
    int16_t ay; int16_t gy;
    int16_t az; int16_t gz;

    // Accel X
    i2c_read_register(address, MPU9250_ACCEL_XOUT_H, byteptr);
    ax = ((int16_t) readbyte) << 8;

    i2c_read_register(address, MPU9250_ACCEL_XOUT_L, byteptr);
    ax = ((int16_t) readbyte) | ax;

    // Accel Y
    i2c_read_register(address, MPU9250_ACCEL_YOUT_H, byteptr);
    ay = ((int16_t) readbyte) << 8;

    i2c_read_register(address, MPU9250_ACCEL_YOUT_L, byteptr);
    ay = ((int16_t) readbyte) | ay;

    // Accel Z
    i2c_read_register(address, MPU9250_ACCEL_ZOUT_H, byteptr);
    az = ((int16_t) readbyte) << 8;

    i2c_read_register(address, MPU9250_ACCEL_ZOUT_L, byteptr);
    az = ((int16_t) readbyte) | az;

    // Gyro X
    i2c_read_register(address,MPU9250_GYRO_XOUT_H,byteptr);
    gx = ((int16_t) readbyte) << 8;

    i2c_read_register(address,MPU9250_GYRO_XOUT_L,byteptr);
    gx = ((int16_t) readbyte) | gx;

    // Gyro Y
    i2c_read_register(address,MPU9250_GYRO_YOUT_H,byteptr);
    gy = ((int16_t) readbyte) << 8;

    i2c_read_register(address,MPU9250_GYRO_YOUT_L,byteptr);
    gy = ((int16_t) readbyte) | gy;

    // Gyro Z
    i2c_read_register(address,MPU9250_GYRO_ZOUT_H,byteptr);
    gz = ((int16_t) readbyte) << 8;

    i2c_read_register(address,MPU9250_GYRO_ZOUT_L,byteptr);
    gz = ((int16_t) readbyte) | gz;

    timestamp = TimerValueGet(GPTIMER2_BASE, GPTIMER_A);

    x = ((double) ax) / 16000.0; y = ((double) ay) / 16000.0; z = ((double) az) / 16000.0;
    accel[0] = ax; accel[1] = ay; accel[2] = az; // TODO: debias from gravity using gyro readings
}

void complementary_filter(short accelData[3], short gyroData[3], double *pitch, double *roll, double dt)
{
    double pitch_acc, roll_acc;
    
    // crux of complementary filter, weight gyro data vs. accel data
    double W_GYRO = 0.98;
    double W_ACCEL = 1 - W_GYRO; 

    // sensitivity = -16 to 16 G at 16Bit -> 16G = 262144 && 0.5G = 8192;
    double LOW_THRESH = 0.5 * ACCEL_SENS; double HIGH_THRESH = 16 * ACCEL_SENS;      

    // Integrate the gyroscope data -> int(angularSpeed) = angle
    *pitch += ((double) gyroData[0] / GYRO_SENS) * dt;
    *roll -= ((double) gyroData[1] / GYRO_SENS) * dt;

    // compensate for drift with accelerometer data if !bullshit
    int accel_check = abs(accelData[0]) + abs(accelData[1]) + abs(accelData[2]);
    if (accel_check > LOW_THRESH && accel_check < HIGH_THRESH) {
        pitch_acc = my_atan2(((double) accelData[1]), ((double) accelData[2])) * 180 / PI;
        roll_acc = my_atan2(((double) accelData[0]), ((double) accelData[2])) * 180 / PI;

        *pitch = *pitch * W_GYRO + pitch_acc * W_ACCEL;
        *roll = *roll * W_GYRO + roll_acc * W_ACCEL;
        // pitch_acc = my_atan2((double) W_GYRO, (double) W_GYRO);
    }
}

void precision_timers_init(void){
    // SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_GPT0); // enables timer0 module
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_GPT3); // enables timer3 module

    input_edge_timers_init();

    // TimerConfigure(gptmPeriodTimerBase, GPTIMER_CFG_PERIODIC_UP);
    // TimerLoadSet(gptmPeriodTimerBase,GPTIMER_A,timer_cnt_32);
    // TimerEnable(gptmPeriodTimerBase,GPTIMER_A);
}

void input_edge_timers_init(void) {
    GPIOPinTypeTimer(GPIO_A_BASE,GPIO_PIN_2); // enables hw muxing of pin inputs
    GPIOPinTypeTimer(GPIO_A_BASE,GPIO_PIN_5); // enables hw muxing of pin inputs

    TimerConfigure(gptmEdgeTimerBase, GPTIMER_CFG_SPLIT_PAIR |
          GPTIMER_CFG_A_CAP_TIME_UP | GPTIMER_CFG_B_CAP_TIME_UP); // configures timer3a/b as 16-bit edge timers

    TimerPrescaleSet(gptmEdgeTimerBase,GPTIMER_A,0); // add prescaler to timer3a (24-bit)
    TimerPrescaleSet(gptmEdgeTimerBase,GPTIMER_B,0); // add prescaler to timer3b (24-bit)

    TimerLoadSet(gptmEdgeTimerBase,GPTIMER_A,timer_cnt_16);
    TimerLoadSet(gptmEdgeTimerBase,GPTIMER_B,timer_cnt_16);

    // FIXME: can we use the same gpio pin for both??
    IOCPinConfigPeriphInput(GPIO_A_BASE, GPIO_PIN_2, IOC_GPT3OCP1); // map gpio pin output to timer3a
    IOCPinConfigPeriphInput(GPIO_A_BASE, GPIO_PIN_5, IOC_GPT3OCP2); // map gpio pin output to timer3b

    // NOTE: the TS3633-CM1 Prototyping Module inverts rising and falling edges when light pulses are received,
    // so negative edges correspond to rising edges, and positive edges correspond to falling edges
    TimerControlEvent(gptmEdgeTimerBase, GPTIMER_A, GPTIMER_EVENT_NEG_EDGE); // set timer3a to capture rising edges (inverted by PCB)
    TimerControlEvent(gptmEdgeTimerBase, GPTIMER_B, GPTIMER_EVENT_POS_EDGE); // set timer3b to capture falling edges (inverted by PCB)

    TimerIntDisable(gptmEdgeTimerBase, gptmFallingEdgeEvent);
    TimerIntClear(gptmEdgeTimerBase, gptmFallingEdgeEvent);

    // set up interrupt for falling edge timer
    TimerIntRegister(gptmEdgeTimerBase, GPTIMER_B, pulse_handler_gpio_a);
    TimerIntEnable(gptmEdgeTimerBase, gptmFallingEdgeEvent);
    IntPrioritySet(gptmFallingEdgeInt, 0<<5);
    IntEnable(gptmFallingEdgeInt);

    IntMasterEnable();

    TimerEnable(gptmEdgeTimerBase, GPTIMER_BOTH);

    TimerSynchronize(GPTIMER0_BASE, GPTIMER_3A_SYNC | GPTIMER_3B_SYNC); // for 2 diodes, sync logical or of GPTIMER_2A_SYNC and these two with GPTIMER0_BASE as base??? check if we get same functionality hmmmm
}

double get_period_us_32(uint32_t start, uint32_t end) {
    if (start > end) {
        // do overflow arithmetic
        return ((double) (end + (timer_cnt_32 - start))) / CLOCK_SPEED_MHZ;
    } else {
        return ((double) (end - start)) / CLOCK_SPEED_MHZ;
    }
}

double get_period_us(uint32_t start, uint32_t end) {
    if (start > end) {
        // do overflow arithmetic
        return ((double) (end + (timer_cnt_24 - start))) / CLOCK_SPEED_MHZ;
    } else {
        return ((double) (end - start)) / CLOCK_SPEED_MHZ;
    }
}

/** Returns a number defining our 3 information bits: skip, data, axis.
  Given by our pulse length in microseconds (us). */
unsigned short int sync_bits(double duration) {
  return (unsigned short int) (48*duration - 2501) / 500;
}

double distance_fit_horiz(double time_us) {
  double E = 0.2218; double c0 = -0.3024; double c1 = 18.2991;
  return E + c1 / (time_us - c0 * sweep_velocity);
}

double distance_fit_vert(double time_us) {
  double E = 0.3074; double c0 = 0.9001; double c1 = 16.1908;
  return E + c1 / (time_us - c0 * sweep_velocity);
}

location_t localize_mimsy(pulse_t *pulses_local) {
    location_t loc = (location_t){.phi = 0, .theta = 0,
											.r_vert = 0, .r_horiz = 0,
											.asn =  {0, 0, 0, 0, 0}, .valid = 0};

    uint8_t init_sync_index = PULSE_TRACK_COUNT;

    // loop through and classify our pulses
    Pulses valid_seq_a[4] = { Sync, Horiz, Sync, Vert };
    Pulses valid_seq_b[4] = { Sync, Vert, Sync, Horiz };
    uint8_t sweep_axes_check = 0; uint8_t i;
    for (i = 0; i < PULSE_TRACK_COUNT; i++) {
        double period = get_period_us(pulses_local[i].rise, pulses_local[i].fall);
        if (period < MIN_SYNC_PERIOD_US) { // sweep pulse
            if (init_sync_index != PULSE_TRACK_COUNT) {
                double parent_period = get_period_us(pulses_local[i-1].rise, pulses_local[i-1].fall);
                int axis = (sync_bits(parent_period) & 0b001) + 1;
                pulses_local[i].type = axis; // 2 if horizontal, 1 if vertical

                int ind = i - init_sync_index;
                if (axis == ((int) valid_seq_a[ind]) || axis == ((int) valid_seq_b[ind])) {
                    sweep_axes_check += axis; // check for 1 horizontal, 1 vertical sweep
                } else {
                    broken2 += 1;
                    return loc;
                }
            }
        } else if (period < MAX_SYNC_PERIOD_US) { // sync pulse
            if (init_sync_index == PULSE_TRACK_COUNT) {
            	init_sync_index = i; // set initial valid sync pulse index
            }
            pulses_local[i].type = (int) Sync;
        } else { // neither
            pulses_local[i].type = -1;
            broken2 += 1;
            return loc;
        }
    }

    if (init_sync_index == PULSE_TRACK_COUNT || sweep_axes_check != 3) {
        broken3 = sweep_axes_check; broken1 += 1;
        return loc;
    }

    for (i = init_sync_index; i < PULSE_TRACK_COUNT-1; i++) {
        pulse_t curr_pulse = pulses_local[i];
        pulse_t next_pulse = pulses_local[i+1];

        switch(next_pulse.type) {
            case ((int) Sync):
                break;
            case ((int) Horiz):
                loc.phi = get_period_us(curr_pulse.rise, next_pulse.rise) * sweep_velocity;
                loc.r_horiz = distance_fit_horiz(get_period_us(next_pulse.rise, next_pulse.fall));
                break;
            case ((int) Vert):
                loc.theta = get_period_us(curr_pulse.rise, next_pulse.rise) * sweep_velocity;
                loc.r_vert = distance_fit_vert(get_period_us(next_pulse.rise, next_pulse.fall));
                break;
            default:
                return loc;
                break;
        }
    }

    loc.valid = true;
    return loc;
}

void configure_pins(void){
    // initialize edges
    unsigned short int i;
    for (i = 0; i < PULSE_TRACK_COUNT; i++) {
        pulses[i] = (pulse_t){.rise = 0, .fall = 0, .type = -1};
    }

    // initialize angle array
    uint32_t j;
    for (j = 0; j < MAX_SAMPLES; j++) {
        valid_angles[j][0] = 0; valid_angles[j][1] = 0;
    }

    volatile uint32_t _i;
    //Delay to avoid pin floating problems
    for(_i = 0xFFFF; _i != 0; _i--);
    
    precision_timers_init();
}
/*
static void model(ekf_t * ekf, double accel, double phi, bool update) {
    // TODO: define your dynamics model
    ekf->fx[0] = ekf->x[0] + DT * ekf->x[1];
    ekf->fx[1] = ekf->x[1] + DT * accel;

    ekf->F[0][0] = 1; ekf->F[0][1] = DT;
    ekf->F[1][0] = 0; ekf->F[1][1] = 1;

    if (update) {
        ekf->hx[0] = my_atan(ekf->x[0]);
        ekf->hx[1] = 0;

        ekf->H[0][0] = 1.0 / (1.0 + x*x);
        ekf->H[0][1] = 0; ekf->H[1][0] = 0; ekf->H[1][1] = 0;
    }
}

void configure_ekf(ekf_t * ekf) {
    // configure Extended Kalman Filter for fusing lighthouse and acceleration data
    ekf_init(&ekf, NUM_STATES, NUM_OBS);

    const double S_lh = LH_RAD_VAR;
    const double S_a = ACCEL_G_VAR;
    const double Rk[2][2] = {{S_lh * S_lh, 0},
                             {0,           0}};
    const double Qk[2][2] = {{(S_a * S_a)*(DT*DT*DT*DT) / 4.0, (S_a * S_a)*(DT*DT*DT) / 2.0},
                             {(S_a * S_a)*(DT*DT*DT) / 2.0,            (S_a * S_a)*(DT*DT)}};

    // init covariances of state/measurement noise, can be arbitrary?
    int i; int j;
    for (i = 0; i < NUM_STATES; i += 1) {
        ekf->P[i][i] = 1;
    }
    for (i = 0; i < NUM_OBS; i += 1) {
        for (j = 0; i < NUM_OBS; j += 1) {
            ekf->R[i][j] = Rk[i][j]; // make this very low
        }
    }
    for (i = 0; i < NUM_OBS; i += 1) {
        for (j = 0; i < NUM_OBS; j += 1) {
            ekf->Q[i][j] = Qk[i][j]; // make this very low
        }
    }

    ekf->x[0] = 0; // position
    ekf->x[1] = 0; // velocity
} COMMENT */

void global_timer_init(void){
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_GPT1); // enables timer module

    TimerConfigure(gptmTimerBase, GPTIMER_CFG_PERIODIC_UP); // configures timers
    TimerLoadSet(gptmTimerBase,GPTIMER_A,timer_cnt_32);

    TimerEnable(gptmTimerBase,GPTIMER_A);
}

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {

    moving_right = true;
    transmitting = false;
    finished = false;

    modular_ptr = 0; pulse_count = 0; samples = 0; test_count = 0;
    estimate_transmit_count = 0; overflow_count = 0; time = 0; prev_time = 0; init_time = 0; init_time_set = false;

    azimuth = 0.0;
    broken1 = 0; broken2 = 0; broken3 = 0; ekf_fail = 0;

    accel = 0;
    
    new_data = false; update = false;

    uint16_t passphrase[4] = {0xAC,0xAC,0xA5,0XB1};  // Mote1 ID
    // uint16_t passphrase[4] = {0xBD,0xBD,0xB6,0XC2}; // Mote2 ID
    tx_packet_count=0; //reset packet sent counter
    rx_packet_count=0; //reset packet rx counter
    uint32_t byte_masks[4]={0xff000000,0x00ff0000,0x0000ff00,0x000000ff}; //used to access each byte of counter
    tx_count = 0; //count used to keep track of how many packet resends have happened
    uint32_t packet_valid;
	int j=0;
    uint8_t i;

    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));

    // initialize board
    board_init();

    //ekf_t ekf; COMMENT
    //configure_ekf(&ekf); COMMENT

    //imu_init(); COMMENT

    x = 0; y = 0; z = 0;

    /* while (true) {
        // lighthouse
    }

    while (!finished) {
        while (!imu_ready) {
            // wait for new IMU update
        }
        int accel[3]; int timestamp;
        get_scalar_accel(accel, &timestamp);
        imu_ready = false;
    } */

    // start bsp timer
    sctimer_set_callback(cb_timer);
    sctimer_setCompare(sctimer_readCounter()+TIMER_PERIOD);
    sctimer_enable();

    // add callback functions radio
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    // prepare packet. This loads the 32bit counter into the packet
    app_vars.packet_len = sizeof(app_vars.packet);
    for (i=0;i<app_vars.packet_len;i++) {
        if(i<4){
            app_vars.packet[i]=0;
        } else {
            app_vars.packet[i] = passphrase[i-4];
        }
    }

    radio_rfOn();
    radio_setFrequency(CHANNEL); // multichannel
    radio_rfOff();

    // configure localization interrupt timing scheme
    configure_pins();
    global_timer_init();

    // TODO: what's up with this???
    // HWREG(RFCORE_XREG_RXENABLE) = 0; //disable rx
    // HWREG(RFCORE_XREG_FRMCTRL1) = HWREG(RFCORE_XREG_FRMCTRL1) & 0b110; //prevents stxon instruction from enabling rx, this is really important because it prevents tx motes from ever receiving anything
   
    app_vars.flags &= ~APP_FLAG_TIMER; app_vars.flags &= ~APP_FLAG_START_FRAME; app_vars.flags &= ~APP_FLAG_END_FRAME;

    bool first = false; bool enabled = true;

    bool send_est = false;
    bool control_flag = false;
    double pos; uint32_t pos_time; // TODO: make sure you're not overwriting when sending estimate
    while (true) {
        if (new_data && !transmitting) {// TODO: still update ekf/position estimate when transmitting??? move if transmitting return inside
            new_data = false;
            /* model(&ekf, accel, azimuth, update);

            if (update) {
            double obs[2] = {azimuth, 0};
                if (ekf_step(&ekf, obs)) {
                    ekf_fail += 1;
                }
                update = false;
            } */

            // TODO: update positions & ignore velocities     
            // double pos = ekf.x[0];
            pos = azimuth; pos_time = time;

            // send packet with azimuth data

            if ((moving_right && (pos <= RIGHT_LIM)) || (!moving_right && (pos >= LEFT_LIM))) { // test with atan(LEFT_LIM_m), r_l_m
                transmitting = true; control_flag = true;
                app_vars.flags |= APP_FLAG_TIMER;
                rx_packet_count += 1;
            }

            if (imu_ready && !send_est) { // TODO: set flag here and do if statement on outer that handles ground truth transmission (don't break direction switch state machine while you're at it --> if most recent pos >= LIM, send switch signal with priority)
                                    // TODO: while emptying this array, start filling another one with new pose data, and when done, copy over and empty that one (assume transmission is quick so you can disable interrupts when transmitting --> won't lose too many pulses, if any, I hope)
                imu_ready = false;
                send_est = true;
                // TODO: make second timer for IMU reading that's different s.t. position transmissions only come on specified interval
                // TODO: or actually just make it so that whenever array count is above 200 or sth you start sending shit
            }
        }

        if (!control_flag && send_est && !transmitting) { // if control_flag set, wait till next round to transmit
            send_est = false; // if array done being emptied, else keep going
            app_vars.flags |= APP_FLAG_TIMER;

            // should transmit pose
            IntDisable(gptmFallingEdgeInt);
            enabled = false;

            radio_rfOn();
            radio_setFrequency(CHANNEL); // multichannel
            radio_rfOff();

            // split double into 8 pairs of two digits
            // (i.e. 3.141592653589793 --> 31, 41, 59, 26, 53, 58, 97, 93)
            // and send over TxChannel

            uint8_t d[8+5]; uint8_t offset = 0;
            d[offset + 0] = (uint8_t) (reduce_digits(pos, 0) * 10);
            d[offset + 1] = (uint8_t) (reduce_digits(pos, 2) * 10);
            d[offset + 2] = (uint8_t) (reduce_digits(pos, 4) * 10);
            d[offset + 3] = (uint8_t) (reduce_digits(pos, 6) * 10);
            d[offset + 4] = (uint8_t) (reduce_digits(pos, 8) * 10);
            d[offset + 5] = (uint8_t) (reduce_digits(pos, 10) * 10);
            d[offset + 6] = (uint8_t) (reduce_digits(pos, 12) * 10);
            d[offset + 7] = (uint8_t) (reduce_digits(pos, 14) * 10);

            /* d[offset + 8] = (uint8_t) (((uint32_t) (pos_time / 100000000.0)) % 100);
            d[offset + 9] = (uint8_t) (((uint32_t) (pos_time / 1000000.0)) % 100);
            d[offset + 10] = (uint8_t) (((uint32_t) (pos_time / 10000.0)) % 100);
            d[offset + 11] = (uint8_t) (((uint32_t) (pos_time / 100.0)) % 100);
            d[offset + 12] = (uint8_t) (((uint32_t) (pos_time / 1.0)) % 100); */

            /* uint8_t d[8*10]; uint8_t offset;
            for (i = 0; i < 10; i += 1) {
                offset = 8*i;
                d[offset + 0] = (uint8_t) (reduce_digits(pos, 0) * 10);
                d[offset + 1] = (uint8_t) (reduce_digits(pos, 2) * 10);
                d[offset + 2] = (uint8_t) (reduce_digits(pos, 4) * 10);
                d[offset + 3] = (uint8_t) (reduce_digits(pos, 6) * 10);
                d[offset + 4] = (uint8_t) (reduce_digits(pos, 8) * 10);
                d[offset + 5] = (uint8_t) (reduce_digits(pos, 10) * 10);
                d[offset + 6] = (uint8_t) (reduce_digits(pos, 12) * 10);
                d[offset + 7] = (uint8_t) (reduce_digits(pos, 14) * 10);
            } */

            // prepare packet. This loads the 32bit counter into the packet
            app_vars.packet_len = sizeof(app_vars.packet);
            for (i=0;i<app_vars.packet_len;i++) {
                app_vars.packet[i] = d[i];
            }

            // start transmitting packet
            radio_loadPacket(app_vars.packet,app_vars.packet_len);
            radio_txEnable();
            radio_txNow();

            // clear flag
            app_vars.flags &= ~APP_FLAG_TIMER;

            IntEnable(gptmFallingEdgeInt);
            enabled = true;
        }

        //==== APP_FLAG_START_FRAME (TX or RX)
        if (app_vars.flags & APP_FLAG_START_FRAME) {
            // start of frame
            // started sending a packet
        
            // clear flag
            app_vars.flags &= ~APP_FLAG_START_FRAME;
        }
     
        //==== APP_FLAG_END_FRAME (TX or RX)
        if (app_vars.flags & APP_FLAG_END_FRAME) {
            // end of frame
            // done sending a packet
            if(transmitting){
                tx_count++; //increment attempt counter
                // if number of attempts hasn't been reached, reset tx_timer flag to resend
                if((tx_count<NUM_ATTEMPTS)){
                    radio_rfOn();
                    radio_setFrequency(CHANNEL+tx_count*CHANNEL_HOP); // multichannel
                    radio_rfOff();
                    app_vars.flags |= APP_FLAG_TIMER;
                } else { // done transmitting
                    tx_count = 0; transmitting = false; control_flag = false;
                    // change motion state
                    moving_right = !moving_right;
                    IntEnable(gptmFallingEdgeInt); enabled = true;
                    // tx_packet_count += 1;
                    // NOTE: incorporate Kalman velocity direction into this? prolly overkill bc shouldn't differ o/w undefined behavior
                }
            }
            // clear flag
            app_vars.flags &= ~APP_FLAG_END_FRAME;
        }

        //==== APP_FLAG_TIMER
        if (app_vars.flags & APP_FLAG_TIMER) {
            // should transmit pose
            IntDisable(gptmFallingEdgeInt); enabled = false;

            if (tx_count == 0) {
                radio_rfOn();
                radio_setFrequency(CHANNEL); // multichannel
                radio_rfOff();
            }

            // prepare packet. This loads the 32bit counter into the packet
            app_vars.packet_len = sizeof(app_vars.packet);
            for (i=0;i<app_vars.packet_len;i++) {
                if (i<4) {
                    app_vars.packet[i]=0;
                } else {
                    app_vars.packet[i] = passphrase[i-4];
                }
            }

            // sent packet contains the left and right sensor states
            if(moving_right){
                app_vars.packet[0] = 0xFF;
            }else{
                app_vars.packet[0] = 0xAA;
            }

            tx_packet_count += 1;

            // start transmitting packet
            radio_loadPacket(app_vars.packet,app_vars.packet_len);
            radio_txEnable();
            radio_txNow();

            // GPIOPinWrite(GPIO_D_BASE,GPIO_PIN_0,0);

            // clear flag
            app_vars.flags &= ~APP_FLAG_TIMER;
        }
    }
}

//=========================== callbacks =======================================
void pulse_handler_gpio_a(void) {
    TimerIntClear(gptmEdgeTimerBase, gptmFallingEdgeEvent);

    pulses[modular_ptr].rise = (uint32_t)(HWREG(gptmTimer3AReg));
    pulses[modular_ptr].fall = (uint32_t)(HWREG(gptmTimer3BReg));
    modular_ptr++; if (modular_ptr == 5) modular_ptr = 0;

    pulse_count += 1;

    // check if fresh pulse data is available
    if (pulse_count >= 5) {
        // get pose
        pulse_t pulses_local[PULSE_TRACK_COUNT];
        uint8_t ptr = modular_ptr;

        unsigned short int i;
        for (i = ptr; i < ptr + PULSE_TRACK_COUNT; i++) {
            pulses_local[i-ptr].rise = pulses[i%PULSE_TRACK_COUNT].rise;
            pulses_local[i-ptr].fall = pulses[i%PULSE_TRACK_COUNT].fall;
            pulses_local[i-ptr].type = pulses[i%PULSE_TRACK_COUNT].type;
        }

        pulse_count = 0; // TODO: maybe only do this if pulses are valid???
        // recover azimuth and elevation
        location_t loc = localize_mimsy(pulses_local);
        if (!loc.valid) { test_count += 1; return; }

        azimuth = loc.phi; new_data = true; update = true;

        time = TimerValueGet(gptmTimerBase, GPTIMER_A) - init_time;
        time += 1; // FIXME: placeholder
        if (!init_time_set) {
            init_time_set = true;
            init_time = time; time = 0;
            // TODO: start hardware timer
            // TODO: set overflow counter to 0, transmit that as well for python multiplication
            // led's on
            leds_all_on(); // TODO: move later
        }
        valid_angles[(int)samples][0] = loc.phi; valid_angles[(int)samples][1] = (double) time;
        if (time < prev_time) {
            overflow_count += 1;
        }
        prev_time = time;

        elevation = azimuth * 180/PI;

        samples += 1;
        if (samples >= MAX_SAMPLES) { // write to flash
            samples = 0;
        }
    }
}

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {
   // set flag
   app_vars.flags |= APP_FLAG_START_FRAME;
   
   // update debug stats
   app_dbg.num_startFrame++;
}

void cb_endFrame(PORT_TIMER_WIDTH timestamp) {
   // set flag
   app_vars.flags |= APP_FLAG_END_FRAME;
   
   // update debug stats
   app_dbg.num_endFrame++;
}

void cb_timer(void) {
   // set flag
   imu_ready = true;
   
   sctimer_setCompare(sctimer_readCounter()+TIMER_PERIOD);
}
