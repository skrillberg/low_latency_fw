/**
\brief This program shows the use of the "radio" bsp module.

Since the bsp modules for different platforms have the same declaration, you
can use this project with any platform.

This application places the mote in receive mode, and prints, over the serial
port, all information about the received packet. The frame printed over the
serial port for each received packet is formatted as follows:
- [1B] the length of the packet, an unsigned integer
- [1B] the first byte of the packet, an unsigned integer
- [1B] the receive signal strength of tehe packet, an signed integer
- [1B] the link quality indicator, an unsigned integer
- [1B] whether the receive packet passed CRC (1) or not (0)
- [3B] closing flags, each of value 0xff

You can run the 01bsp_radio_rx.py script to listen to your mote and parse those 
serial frames. The application can connect directly to the mote's serial port,
or to its TCP port when running on the IoT-LAB platform.

Example when running locally:
----------------------------

 ___                 _ _ _  ___  _ _
| . | ___  ___ ._ _ | | | |/ __>| \ |
| | || . \/ ._>| ' || | | |\__ \|   |
`___'|  _/\___.|_|_||__/_/ <___/|_\_|
     |_|                  openwsn.org

running IoT-lAB? (Y|N): N
name of serial port (e.g. COM10): COM25
len=127 num=176 rssi=-43  lqi=107 crc=1
len=127 num=177 rssi=-43  lqi=107 crc=1
len=127 num=178 rssi=-43  lqi=106 crc=1
len=127 num=179 rssi=-43  lqi=107 crc=1
len=127 num=180 rssi=-43  lqi=108 crc=1
len=127 num=181 rssi=-43  lqi=107 crc=1
len=127 num=182 rssi=-43  lqi=107 crc=1
len=127 num=183 rssi=-43  lqi=107 crc=1


Example when running on the IoT-LAB platform:
--------------------------------------------

 ___                 _ _ _  ___  _ _
| . | ___  ___ ._ _ | | | |/ __>| \ |
| | || . \/ ._>| ' || | | |\__ \|   |
`___'|  _/\___.|_|_||__/_/ <___/|_\_|
     |_|                  openwsn.org

running IoT-lAB? (Y|N): Y
motename? (e.g. wsn430-35): wsn430-35
len=17  num=84  rssi=-80  lqi=107 crc=1
len=17  num=84  rssi=-81  lqi=107 crc=1
len=17  num=84  rssi=-80  lqi=107 crc=1
len=17  num=84  rssi=-81  lqi=105 crc=1
len=17  num=84  rssi=-80  lqi=108 crc=1
len=17  num=84  rssi=-81  lqi=108 crc=1


\author Xavi Vilajosana xvilajosana@eecs.berkeley.edu>, June 2012.
\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, August 2014.
*/

#include "stdint.h"
#include "string.h"
#include "board.h"
#include "radio.h"
#include "leds.h"
#include "uart.h"
#include "sctimer.h"
#include <headers/hw_memmap.h>
#include "source/gpio.h"
#include "i2c.h"
//=========================== defines =========================================

#define LENGTH_PACKET        8+LENGTH_CRC ///< maximum length is 127 bytes
#define CHANNEL              26           ///< 11 = 2.405GHz
#define LENGTH_SERIAL_FRAME  3              ///< length of the serial frame
#define I2C_ON				 false		///IF TRUE, user must connect I2C pin to slave device (GP1 : SCL GP2 : SDA)
#define I2C_ADDRESS			 0x08

//=========================== variables =======================================

typedef struct {
   uint8_t    num_radioTimerCompare;
   uint8_t    num_startFrame;
   uint8_t    num_endFrame;
} app_dbg_t;

app_dbg_t app_dbg;

typedef struct {
   // rx packet
   volatile   uint8_t    rxpk_done;
              uint8_t    rxpk_buf[LENGTH_PACKET];
              uint8_t    rxpk_len;
              uint8_t    rxpk_num;
              int8_t     rxpk_rssi;
              uint8_t    rxpk_lqi;
              bool       rxpk_crc;
   // uart
              uint8_t    i2c_txFrame[LENGTH_SERIAL_FRAME];
              uint8_t    i2c_lastTxByte;
              uint8_t	 i2c_pktpp;
   volatile   uint8_t    i2c_done;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

// radiotimer
void cb_radioTimerOverflows(void);
// radio
void cb_startFrame(PORT_TIMER_WIDTH timestamp);
void cb_endFrame(PORT_TIMER_WIDTH timestamp);


//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {
   int i=0;
   // clear local variables
   memset(&app_vars,0,sizeof(app_vars_t));
   
   // initialize board
   board_init();
//   i2c_init();

   // add callback functions radio
   radio_setStartFrameCb(cb_startFrame);
   radio_setEndFrameCb(cb_endFrame);
   
   // setup UART
   // uart_setCallbacks(cb_uartTxDone,cb_uartRxCb);
   
   // prepare radio
   radio_rfOn();
   radio_setFrequency(CHANNEL);
   
   // switch in RX
   radio_rxEnable();
   uint8_t packet[10] = {0,0,0,0,0,0,0,0,0,0};
   radio_loadPacket(packet,LENGTH_PACKET);
   radio_txEnable();
   radio_txNow();
 	int packet_valid;

   while (1) {
	   /*Added by SY*/
//	   memset(&app_vars,0,sizeof(app_vars_t));
	   packet_valid = 0;
      int j = 0;
      // sleep while waiting for at least one of the rxpk_done to be set
      app_vars.rxpk_done = 0;
      while (app_vars.rxpk_done==0) {
       // leds_debug_on();
      }
      
      // if I get here, I just received a packet
      
      //===== send notification over serial port
      
      // led
      leds_error_on();
      
      // format frame to send over serial port
      app_vars.i2c_pktpp = 0x00;

      app_vars.i2c_txFrame[0] = app_vars.i2c_pktpp;  // packet length
      app_vars.i2c_txFrame[1] = app_vars.rxpk_rssi; // RSSI
      app_vars.i2c_txFrame[2] = app_vars.rxpk_crc;  // CRC
      //app_vars.i2c_txFrame[3] = app_vars.rxpk_lqi;  // LQI
      //app_vars.i2c_txFrame[4] = app_vars.rxpk_num;  // packet number
      //app_vars.i2c_txFrame[5] = app_vars.rxpk_len;  // packet length
      //app_vars.i2c_txFrame[6] = 0xff;               // closing flag
      //app_vars.i2c_txFrame[7] = 0xff;               // closing flag
      
      app_vars.i2c_done          = 0;
      app_vars.i2c_lastTxByte    = 0;



//      packet_valid = ((app_vars.rxpk_crc != 0) && (app_vars.rxpk_buf[4] == 0x0C) && (app_vars.rxpk_buf[5] == 0xCE)  && (app_vars.rxpk_buf[6] == 0xAC) && (app_vars.rxpk_buf[7] == 0x5F)); //mote 1
	  packet_valid = ((app_vars.rxpk_crc != 0) && (app_vars.rxpk_buf[4] == 0xE6) && (app_vars.rxpk_buf[5] == 0x6F)  && (app_vars.rxpk_buf[6] == 0x80) && (app_vars.rxpk_buf[7] == 0xCE)); //mote 2
	  if(packet_valid){

		//set left output pin high
		if((app_vars.rxpk_buf[0] == 0xFF)){
			leds_sync_on();
			GPIOPinWrite(GPIO_D_BASE, GPIO_PIN_2,GPIO_PIN_2);
			GPIOPinWrite(GPIO_D_BASE, GPIO_PIN_0,GPIO_PIN_0);
			if(I2C_ON){
				app_vars.i2c_pktpp = 0x01;
				app_vars.i2c_txFrame[0] = app_vars.i2c_pktpp;
				i2c_write_bytes(I2C_ADDRESS, app_vars.i2c_txFrame, LENGTH_SERIAL_FRAME); //send data through i2c (add 12/28/18)
			}
			else for(j=0;j<1000;j++);
	  	}

	  	if((app_vars.rxpk_buf[0] == 0xAA)){
			leds_debug_on();
			GPIOPinWrite(GPIO_D_BASE, GPIO_PIN_1,GPIO_PIN_1);
			GPIOPinWrite(GPIO_D_BASE, GPIO_PIN_0,GPIO_PIN_0);
			if (I2C_ON){
				app_vars.i2c_pktpp = 0x01;
				app_vars.i2c_txFrame[0] = app_vars.i2c_pktpp;
				i2c_write_bytes(I2C_ADDRESS, app_vars.i2c_txFrame, LENGTH_SERIAL_FRAME); //send data through i2c (add 12/28/18)
			}
			else for(j=0;j<1000;j++);
		}

	  	GPIOPinWrite(GPIO_D_BASE, GPIO_PIN_1,0);
		GPIOPinWrite(GPIO_D_BASE, GPIO_PIN_2,0);
		GPIOPinWrite(GPIO_D_BASE, GPIO_PIN_0,0);
    	leds_error_off();
	  }
	  else {
		  if (I2C_ON){
			  	  i2c_write_bytes(I2C_ADDRESS, app_vars.i2c_txFrame, LENGTH_SERIAL_FRAME); //send data through i2c (add 12/28/18)
		  }
	  }
	  /*Added by SY*/
	  memset(&app_vars,0,sizeof(app_vars_t));
   }
}

//=========================== callbacks =======================================

//===== radio

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {
   /*Added by SY*/
//   GPIOPinWrite(GPIO_A_BASE,GPIO_PIN_2,GPIO_PIN_2);
   // update debug stats
   app_dbg.num_startFrame++;
}

void cb_endFrame(PORT_TIMER_WIDTH timestamp) {
	/*Added by SY*/
//	GPIOPinWrite(GPIO_A_BASE,GPIO_PIN_2,0);
   // update debug stats
   app_dbg.num_endFrame++;
   // indicate I just received a packet
   app_vars.rxpk_done = 1;
   
   leds_sync_on();

   // get packet from radio
   radio_getReceivedFrame(
      app_vars.rxpk_buf,
      &app_vars.rxpk_len,
      sizeof(app_vars.rxpk_buf),
      &app_vars.rxpk_rssi,
      &app_vars.rxpk_lqi,
      &app_vars.rxpk_crc
   );
   
   // read the packet number
   app_vars.rxpk_num = app_vars.rxpk_buf[0];
   
   // led
   leds_sync_off();
}




