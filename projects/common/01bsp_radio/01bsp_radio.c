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

#define LENGTH_PACKET   8+LENGTH_CRC ///< maximum length is 127 bytes
#define CHANNEL         16             ///< 11=2.405GHz
#define TX_CHANNEL	16	       /// tx channel of individual mote
#define TIMER_PERIOD    0xff         ///< 0xff = 125 Hz
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

#define PI 3.14159265f
#define SWEEP_PERIOD_US 8333.333333f

#define CLOCK_SPEED_MHZ 32.0f
#define MAX_SAMPLES 600

#define LEFT_LIM 102.0f
#define RIGHT_LIM 90.0f

#define IMU_ADDRESS 0x69
#define LOW_POWER 0
#define ACCEL_SENS 16384.0f // TODO: check this, may have to be more precise??
#define GYRO_SENS 65.536f

#define CALIB_DATA_STRUCT_SIZE 4
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
	float                    phi;
	float                  theta;
	float                 r_vert;
	float				 r_horiz;
	uint8_t               asn[5];
	int					   valid;
} location_t;

/*CalibData is a union data structure used to store MIMSY's mattress calibration data in flash. Access the struct
type of this union is used for accessing and setting the data. The uint32 array version
of the struct is used for reading and writing to flash*/
typedef union CalibData {
  struct {
  uint32_t azimuth;
} fields;
  struct {
  int32_t azimuth;
} signedfields;
uint32_t bits[1];
} CalibData;

/*This struct is used to keep track of where data was written to. This struct
must be passed to flashWriteCalib where it is updated to include the flash location
of the data. A written data card is passed to flashReadCalib inorder to read the
data from that location*/
typedef struct CalibDataCard{
    uint32_t page;
    uint32_t startTime;
    uint32_t endTime;
} CalibDataCard;

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
static const uint32_t gptmEdgeTimerBase = GPTIMER3_BASE;
static const uint32_t gptmFallingEdgeInt = INT_TIMER3B;
static const uint32_t gptmFallingEdgeEvent = GPTIMER_CAPB_EVENT;

static const uint32_t gptmTimer3AReg = 0x40033048;
static const uint32_t gptmTimer3BReg = 0x4003304C;

static const uint32_t timer_cnt_32 = 0xFFFFFFFF;
static const uint32_t timer_cnt_16 = 0xFFFF;
static const uint32_t timer_cnt_24 = 0xFFFFFF;

/* Definition of global variables for mattress calibration. */
volatile float azimuth;
volatile float elevation;

static const float sweep_velocity = PI / SWEEP_PERIOD_US;

float imu_data[MAX_SAMPLES][3];
uint8_t imu_samples;
bool imu_ready;
float x; float y; float z;

volatile float valid_angles[MAX_SAMPLES][2];
volatile pulse_t pulses[PULSE_TRACK_COUNT];
volatile uint8_t modular_ptr;
volatile uint8_t pulse_count;
volatile uint32_t samples;

volatile uint32_t broken1;
volatile uint32_t broken2;
volatile uint32_t broken3;

uint32_t test_count;

//=========================== flash ===========================================

void flashWriteCalib(CalibData data[],uint32_t size, uint32_t startPage, int wordsWritten);
void flashReadCalib(CalibDataCard card, CalibData *data, uint32_t size);
void flashReadCalibSection(CalibDataCard card, CalibData *data, uint32_t size,int wordsRead);

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);
void     cb_timer(void);
void	 configure_pins(void);

void imu_init(void) {
    // initialize IMU
    imu_ready = false;
    imu_samples = 0;

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

    x = ((float) ax) / 16000.0; y = ((float) ay) / 16000.0; z = ((float) az) / 16000.0;
    accel[0] = ax; accel[1] = ay; accel[2] = az; // TODO: debias from gravity using gyro readings

    imu_data[imu_samples][0] = ((float) ax) / 16000.0; imu_data[imu_samples][1] = ((float) ay) / 16000.0; imu_data[imu_samples][2] = ((float) az) / 16000.0;
    imu_samples += 1;

    if (imu_samples >= MAX_SAMPLES) {
        IntDisable(gptmFallingEdgeInt);
        imu_samples = 0; int _i;
        for (_i = 0; _i < MAX_SAMPLES; _i += 1) {
            mimsyPrintf("%d, %d, %d\n", (int) test_count, (int) (imu_data[imu_samples][0] * 100000), (int) timestamp);
        }
        IntEnable(gptmFallingEdgeInt);
    }
}

/* void complementary_filter(short accelData[3], short gyroData[3], float *pitch, float *roll, float dt)
{
    float pitch_acc, roll_acc;
    
    // crux of complementary filter, weight gyro data vs. accel data
    float W_GYRO = 0.98;
    float W_ACCEL = 1 - W_GYRO; 

    // sensitivity = -16 to 16 G at 16Bit -> 16G = 262144 && 0.5G = 8192;
    float LOW_THRESH = 0.5 * ACCEL_SENS; float HIGH_THRESH = 16 * ACCEL_SENS;      

    // Integrate the gyroscope data -> int(angularSpeed) = angle
    *pitch += ((float) gyroData[0] / GYRO_SENS) * dt;
    *roll -= ((float) gyroData[1] / GYRO_SENS) * dt;

    // compensate for drift with accelerometer data if !bullshit
    int accel_check = abs(accelData[0]) + abs(accelData[1]) + abs(accelData[2]);
    if (accel_check > LOW_THRESH && accel_check < HIGH_THRESH) {
        pitch_acc = atan2f((float) accelData[1], (float) accelData[2]) * 180 / PI;
        roll_acc = atan2f((float) accelData[0], (float) accelData[2]) * 180 / PI;

        *pitch = *pitch * W_GYRO + pitch_acc * W_ACCEL;
        *roll = *roll * W_GYRO + roll_acc * W_ACCEL;
    }
} */

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

float get_period_us_32(uint32_t start, uint32_t end) {
    if (start > end) {
        // do overflow arithmetic
        return ((float) (end + (timer_cnt_32 - start))) / CLOCK_SPEED_MHZ;
    } else {
        return ((float) (end - start)) / CLOCK_SPEED_MHZ;
    }
}

float get_period_us(uint32_t start, uint32_t end) {
    if (start > end) {
        // do overflow arithmetic
        return ((float) (end + (timer_cnt_24 - start))) / CLOCK_SPEED_MHZ;
    } else {
        return ((float) (end - start)) / CLOCK_SPEED_MHZ;
    }
}

/** Returns a number defining our 3 information bits: skip, data, axis.
  Given by our pulse length in microseconds (us). */
unsigned short int sync_bits(float duration) {
  return (unsigned short int) (48*duration - 2501) / 500;
}

float distance_fit_horiz(float time_us) {
  float E = 0.2218; float c0 = -0.3024; float c1 = 18.2991;
  return E + c1 / (time_us - c0 * sweep_velocity);
}

float distance_fit_vert(float time_us) {
  float E = 0.3074; float c0 = 0.9001; float c1 = 16.1908;
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
        float period = get_period_us(pulses_local[i].rise, pulses_local[i].fall);
        if (period < MIN_SYNC_PERIOD_US) { // sweep pulse
            if (init_sync_index != PULSE_TRACK_COUNT) {
                float parent_period = get_period_us(pulses_local[i-1].rise, pulses_local[i-1].fall);
                int axis = (sync_bits(parent_period) & 0b001) + 1;
                pulses_local[i].type = axis; // 2 if horizontal, 1 if vertical

                int ind = i - init_sync_index;
                if (axis == ((int) valid_seq_a[ind]) || axis == ((int) valid_seq_b[ind])) {
                    sweep_axes_check += axis; // check for 1 horizontal, 1 vertical sweep
                } else {
                    broken1 += 1;
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
        broken3 += 1;
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

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {

    moving_right = true;
    transmitting = false;
    finished = false;

    modular_ptr = 0; pulse_count = 0; samples = 0; test_count = 0;

    azimuth = ((float)(LEFT_LIM + RIGHT_LIM)) / 2;
    broken1 = 0; broken2 = 0; broken3 = 0;

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
    imu_init();
    configure_pins();

    uartMimsyInit();

    x = 0; y = 0; z = 0;

    while (!finished) {
        while (!imu_ready) {
            // wait for new IMU update
        }
        int accel[3]; int timestamp;
        get_scalar_accel(accel, &timestamp);
        imu_ready = false;
    }

    return;

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


    // TODO: what's up with this???
    // HWREG(RFCORE_XREG_RXENABLE) = 0; //disable rx
    // HWREG(RFCORE_XREG_FRMCTRL1)    = HWREG(RFCORE_XREG_FRMCTRL1) & 0b110; //prevents stxon instruction from enabling rx, this is really important because it prevents tx motes from ever receiving anything
   
    app_vars.flags &= ~APP_FLAG_TIMER; app_vars.flags &= ~APP_FLAG_START_FRAME; app_vars.flags &= ~APP_FLAG_END_FRAME;

    while (true) {
        //==== APP_FLAG_START_FRAME (TX or RX)
        if (app_vars.flags & APP_FLAG_START_FRAME) {
            // start of frame
            // started sending a packet

            // led
            leds_sync_on();
        
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
                    tx_count = 0; transmitting = false;
                    // change motion state
                    moving_right = !moving_right;
                    IntEnable(gptmFallingEdgeInt);
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
            IntDisable(gptmFallingEdgeInt);

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
        if (!loc.valid) { return; }

        valid_angles[(int)samples][0] = loc.phi; valid_angles[(int)samples][1] = loc.theta;
        azimuth = loc.phi * 180/PI; elevation = loc.theta * 180/PI;

        samples += 1; if (samples == MAX_SAMPLES) { finished = true; }
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

//=============================== flash ==========================================

/*This function writes a full 2048 KB page-worth of data to flash.
  Parameters:
    CalibData data[]: pointer to array of CalibData structures that are to be written to flash
    uint32_t size: size of data[] in number of CalibData structures
    uint32_t startPage: Flash page where data is to be written
    CalibDataCard *card: pointer to an CalibDataCard where the function will record
      which page the data was written to and which timestamps on the data are included
 */
void flashWriteCalib(CalibData data[],uint32_t size, uint32_t startPage,int wordsWritten){
  uint32_t pageStartAddr = FLASH_BASE + (startPage * PAGE_SIZE); // page base address
  int32_t i32Res;

  uint32_t structNum = size;

  // mimsyPrintf("\n Flash Page Address: %x",pageStartAddr);
  if (wordsWritten == 0){
	  i32Res = FlashMainPageErase(pageStartAddr); // erase page so there it can be written to
  }

  // mimsyPrintf("\n Flash Erase Status: %d",i32Res);
  for (uint32_t i = 0; i < size; i++){
    uint32_t* wordified_data=data[i].bits; //retrieves the int32 array representation of the PulseData struct
    IntMasterDisable(); //disables interrupts to prevent the write operation from being messed up
    i32Res = FlashMainPageProgram(wordified_data, pageStartAddr+i*CALIB_DATA_STRUCT_SIZE+wordsWritten*4, CALIB_DATA_STRUCT_SIZE); //write struct to flash
    IntMasterEnable();//renables interrupts
    // mimsyPrintf("\n Flash Write Status: %d",i32Res);
   }

   // update card with location information
   // card->page=startPage;
   // card->startTime=data[0].fields.timestamp;
   // card->endTime=data[size-1].fields.timestamp;

}

/*This function reads a page worth of CalibData from flash.
  Parameters:
    CalibDataCard card: The CalibDataCard that corresponds to the data that you want to read from flash
    CalibData * dataArray: pointer that points to location of data array that you want the read operation to be written to
    uint32_t size: size of dataArray in number of CalibData structures
*/
void flashReadCalib(CalibDataCard card, CalibData * dataArray, uint32_t size){

  uint32_t pageAddr=FLASH_BASE+card.page*PAGE_SIZE;

  for(uint32_t i=0;i<size;i++){
    for(uint32_t j=0;j<CALIB_DATA_STRUCT_SIZE/4;j++){
      IntMasterDisable();
      dataArray[i].bits[j] = FlashGet(pageAddr+i*CALIB_DATA_STRUCT_SIZE+j*4);
      IntMasterEnable();
    }
  }

}

/*This function reads a page worth of CalibData from flash.
  Parameters:
    CalibDataCard card: The CalibDataCard that corresponds to the data that you want to read from flash
    CalibData * dataArray: pointer that points to location of data array that you want the read operation to be written to
    uint32_t size: size of dataArray in number of CalibData structures
*/
void flashReadCalibSection(CalibDataCard card, CalibData * dataArray, uint32_t size, int wordsRead){

  uint32_t pageAddr=FLASH_BASE+card.page*PAGE_SIZE;

  for(uint32_t i=0;i<size;i++){
    for(uint32_t j=0;j<CALIB_DATA_STRUCT_SIZE/4;j++){
       IntMasterDisable();
       dataArray[i].bits[j]=FlashGet(pageAddr+i*CALIB_DATA_STRUCT_SIZE+j*4+wordsRead*4);
       IntMasterEnable();
    }
  }

}
