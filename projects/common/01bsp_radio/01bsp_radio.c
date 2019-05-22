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
#include "kalman.h"
#include <headers/hw_memmap.h>
#include <headers/hw_ioc.h>
#include <headers/hw_ssi.h>
#include <headers/hw_sys_ctrl.h>
#include <headers/hw_ints.h>
#include <headers/hw_rfcore_sfr.h>
#include <headers/hw_rfcore_sfr.h>
#include <headers/hw_rfcore_xreg.h>
#include <source/interrupt.h>
#include <source/ioc.h>
#include <source/gpio.h>
#include <source/gptimer.h>
#include <source/sys_ctrl.h>

//=========================== defines =========================================

#define LENGTH_PACKET   8+LENGTH_CRC ///< maximum length is 127 bytes
#define CHANNEL         16             ///< 11=2.405GHz
#define TX_CHANNEL	16	       /// tx channel of individual mote
#define TIMER_PERIOD    0xffff         ///< 0xffff = 2s@32kHz
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
#define MAX_SAMPLES 200

#define LEFT_LIM 102.0f
#define RIGHT_LIM 90.0f

#define DT 1.0f // FIXME: figure out kalman time step in us?

//=========================== typedef =========================================

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

typedef enum {
   Sync, Vert, Horiz,
} Pulses;
//=========================== variables =======================================
//bool isTx = true;

uint32_t tx_count; //count used to keep track of how many packet resends have happened
uint32_t tx_packet_count; //counter for verifying packet contents
volatile uint32_t rx_packet_count;
uint32_t debounce_complete; // used to debounce button press in interrupt handler

bool moving_right;
volatile bool transmitting;

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

volatile float azimuth;
volatile float elevation;
volatile bool update;

static const float sweep_velocity = PI / SWEEP_PERIOD_US;

volatile float valid_angles[MAX_SAMPLES][2];
volatile pulse_t pulses[PULSE_TRACK_COUNT];
volatile uint8_t modular_ptr;
volatile uint8_t pulse_count;
volatile uint32_t samples;

volatile uint32_t broken1;
volatile uint32_t broken2;
volatile uint32_t broken3;

uint32_t test_count;

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);
void	 configure_pins(void);

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

    TimerEnable(gptmEdgeTimerBase,GPTIMER_BOTH);

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

void configure_pins(void){ // TODO: set to do lighthouse setup
    // localize and store
    modular_ptr = 0; pulse_count = 0; samples = 0; test_count = 0;
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

static void model(*ekf, double accel, double phi, bool update) {
    // TODO: define your dynamics model
    ekf->fx[0] = ekf->x[0] + dt * ekf->x[1];
    ekf->fx[1] = ekf->x[1] + dt * accel;

    // TODO: process Jacobian? ask craig

    if (update) {
        ekf-> hx[0] = atan2(phi);
        ekf-> hx[1] = 0;
    }

    ekf->H[0][0] = 1.0 / (1.0 + ekf->x[0]*ekf->x[0]);
    ekf->H[0][1] = 0; ekf->H[1][0] = 0; ekf->H[1][1] = 0;
}

void configure_ekf(void) {
    // configure Extended Kalman Filter for fusing lighthouse and acceleration data
    ekf_t ekf;
    ekf_init(&ekf, NUM_STATES, NUM_OBS);

    const double S_lh = LH_RAD_VAR;
    const double S_a = ACCEL_G_VAR;
    const double Rk[4] = {S_lh * S_lh, 0, 0, 0};
    const double Qk[4] = {(S_a * S_a)*(DT*DT*DT*DT) / 4.0, (S_a * S_a)*(DT*DT*DT) / 2.0, (S_a * S_a)*(DT*DT*DT) / 2.0, (S_a * S_a)*(DT*DT)};

    // init covariances of state/measurement noise, can be arbitrary?
    int i;
    for (i = 0; i < NUM_STATES; i += 1) { ekf->P[i][i] = 1; }
    for (i = 0; i < NUM_OBS; i += 1) { ekf->R[i][i] = 1; } // make this very low

    ekf->x[0] = 0; // position
    ekf->x[1] = 0; // velocity

    // TODO: implement below in mote_main
}

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {

    moving_right = true;
    transmitting = false;

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
        if (new_data) {
            model(&ekf, accel, azimuth, update);

            if (update) {
                ekf_step(&ekf, azimuth);
                update = false;
            }

            // TODO: update positions & ignore velocities       
        }
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
        if (!loc.valid) { test_count += 1; return; }

        // TODO: set update flag

        valid_angles[(int)samples][0] = loc.phi; valid_angles[(int)samples][1] = loc.theta;
        azimuth = loc.phi * 180/PI; elevation = loc.theta * 180/PI;

        samples += 1; if (samples == MAX_SAMPLES) samples = 0;

        if ((moving_right && (azimuth <= 90.0)) || (!moving_right && (azimuth >= 102.0))) { // TODO: fix this workflow into Kalman filter logic
            transmitting = true;
            app_vars.flags |= APP_FLAG_TIMER;
            // GPIOPinWrite(GPIO_D_BASE,GPIO_PIN_0,GPIO_PIN_0);
            rx_packet_count += 1;
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
