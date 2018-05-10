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
#include "source/gpio.h"
#include <headers/hw_memmap.h>
#include <headers/hw_ints.h>
#include "source/interrupt.h"
#include <headers/hw_rfcore_sfr.h>
#include <headers/hw_rfcore_sfr.h>
#include <headers/hw_rfcore_xreg.h>

//=========================== defines =========================================

#define LENGTH_PACKET   8+LENGTH_CRC ///< maximum length is 127 bytes
#define CHANNEL         14             ///< 11=2.405GHz
#define TX_CHANNEL	20	       /// tx channel of individual mote 
#define TIMER_PERIOD    0xffff         ///< 0xffff = 2s@32kHz
#define ID              0xff           ///< byte sent in the packets
#define isTx	true
#define NUM_ATTEMPTS	2	       ///<number of times packet is resent, needs to match number of motes for multichan experiments
#define RxMOTE		false
#define MOTE_NUM	2	           // index that sets rx mote channel
#define MULTICHAN_TX    true
#define CHANNEL_HOP	6		//number of channels to hop by 
//=========================== variables =======================================
//bool isTx = true;

uint32_t tx_count; //count used to keep track of how many packet resends have happened
uint32_t tx_packet_count; //counter for verifying packet contents
uint32_t rx_packet_count;
uint32_t debounce_complete; // used to debounce button press in interrupt handler
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
uint8_t txed;
//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);
void     cb_timer(void);
void     cb_button(void);
void	 cb_count_rx(void);
void	 configure_pins(void);

void configure_pins(void){

   volatile uint32_t i;	

   for(i =0xFFFF;i!=0;i-- );
      GPIOPinIntDisable(GPIO_A_BASE,GPIO_PIN_2);
     GPIOPinIntClear(GPIO_A_BASE,GPIO_PIN_2);

      GPIOPinIntDisable(GPIO_A_BASE,GPIO_PIN_3);	
   
      GPIOPinIntClear(GPIO_A_BASE,GPIO_PIN_3);

  // if(RxMOTE == false){
	
   
 

      GPIOPinTypeGPIOInput(GPIO_A_BASE,GPIO_PIN_2);

      GPIOIntTypeSet(GPIO_A_BASE,GPIO_PIN_2, GPIO_RISING_EDGE);

      GPIOPortIntRegister(GPIO_A_BASE, cb_button);

      GPIOPinIntClear(GPIO_A_BASE,GPIO_PIN_2);

      GPIOPinIntEnable(GPIO_A_BASE,GPIO_PIN_2);
   //kill switch


      GPIOPinTypeGPIOInput(GPIO_A_BASE,GPIO_PIN_3);

	if(RxMOTE){
		GPIOPinTypeGPIOInput(GPIO_A_BASE,GPIO_PIN_4); //ad5 is rx counter increment

     	 //GPIOIntTypeSet(GPIO_A_BASE,GPIO_PIN_4, GPIO_RISING_EDGE);
	
     	 //GPIOPortIntRegister(GPIO_A_BASE, cb_count_rx);

      	//GPIOPinIntClear(GPIO_A_BASE,GPIO_PIN_4);
	 //GPIOPinIntEnable(GPIO_A_BASE,GPIO_PIN_4);

	}
   //}

}

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {

uint16_t passphrase[4] = {0xB5,0xAC,0xBA,0XE5}; 
tx_packet_count=0; //reset packet sent counter
rx_packet_count=0; //reset packet rx counter
//count = 0; //counter for verifying packet contents
uint32_t byte_masks[4]={0xff000000,0x00ff0000,0x0000ff00,0x000000ff}; //used to access each byte of counter
tx_count =0; //count used to keep track of how many packet resends have happened
txed = 0;
uint32_t packet_valid;
uint32_t count_from_packet = 0;
uint32_t last_pin_state = 0;
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
	   app_vars.packet[i]=(tx_packet_count&byte_masks[i])>>(8*(3-i));
	}
	else{
		if(!RxMOTE){
      	   		app_vars.packet[i] = passphrase[i-4];
		}else{
			app_vars.packet[i] = 0xAB;
		}
	}
   }
   
   // start bsp timer
  // sctimer_set_callback(cb_timer);
   //sctimer_setCompare(sctimer_readCounter()+TIMER_PERIOD);
   //sctimer_enable();
   
   // prepare radio (all this does is turn on the receiver using ths ISRXON instruction strobe)
   //radio_rfOn();
   //radio_rxEnable();
   if(RxMOTE){
      radio_rfOn();
      radio_setFrequency(CHANNEL+CHANNEL_HOP*(MOTE_NUM-1));
   }
   else if(MULTICHAN_TX){
      radio_setFrequency(CHANNEL);
   }else if(!MULTICHAN_TX){
	radio_setFrequency(TX_CHANNEL);
   }
   //set button int

   configure_pins();


   // switch in RX by default (only if rxmote?)
   if(RxMOTE){
      radio_rxEnable();
   } else{
	HWREG(RFCORE_XREG_RXENABLE) = 0; //disable rx
	HWREG(RFCORE_XREG_FRMCTRL1)    = HWREG(RFCORE_XREG_FRMCTRL1) & 0b110; //prevents stxon instruction from enabling rx, this is really important because it prevents tx motes from ever receiving anything 
   }
   app_vars.state = APP_STATE_RX;
   
   // start by a transmit

   if(!RxMOTE){
     app_vars.flags |= APP_FLAG_TIMER;
   }
     tx_count = 3; //needed to reset tx system because of the above line otherwise you get an intial packet send
   //}


   while (1) {

      //while (app_vars.flags==0x00) {
       //  board_sleep();
      //}

      // handle and clear every flag
      while (app_vars.flags) {
         
         
         //==== APP_FLAG_START_FRAME (TX or RX)
         
         if (app_vars.flags & APP_FLAG_START_FRAME) {
            // start of frame
            
            switch (app_vars.state) {
               case APP_STATE_RX:
                  // started receiving a packet
                  
                  // led
                  leds_error_on();

                  break;
               case APP_STATE_TX:
                  // started sending a packet
                  
                  // led

                  leds_sync_on();
		

                  break;
            }
            
            // clear flag
            app_vars.flags &= ~APP_FLAG_START_FRAME;
         }
         
         
         //==== APP_FLAG_END_FRAME (TX or RX)
         
         if (app_vars.flags & APP_FLAG_END_FRAME) {
            // end of frame
            
            switch (app_vars.state) {
               
               case APP_STATE_RX:
                  
                  // done receiving a packet
                  app_vars.packet_len = sizeof(app_vars.packet);
                  
                  // get packet from radio
                  radio_getReceivedFrame(
                     app_vars.packet,
                     &app_vars.packet_len,
                     sizeof(app_vars.packet),
                     &app_vars.rxpk_rssi,
                     &app_vars.rxpk_lqi,
                     &app_vars.rxpk_crc
                  );
                  if(app_vars.packet[4] ==0xB5 ){
			//leds_debug_on();
		  }else{

			//leds_debug_off();
		  }
                  // led
                  leds_error_off();
		  for(i=0;i<3;i++){
			//count_from_packet += (((uint32_t)app_vars.packet[i])<<(8*(3-i)));
		  }
		  count_from_packet += app_vars.packet[3];
		  count_from_packet += ((uint32_t)app_vars.packet[2] << 8);
		  count_from_packet += ((uint32_t)app_vars.packet[1] << 16);
		  count_from_packet += ((uint32_t)app_vars.packet[0] << 24);

		  
		  packet_valid = ((app_vars.rxpk_crc != 0) && (app_vars.packet[4] == 0xB5) && (app_vars.packet[5] == 0xAC) && (app_vars.packet[6] == 0xBA) && (app_vars.packet[7] == 0xE5));
		  if(packet_valid){
			
			switch (last_pin_state){
			case 0:
				last_pin_state = 1;
	          		GPIOPinWrite(GPIO_D_BASE, GPIO_PIN_2,GPIO_PIN_2);
				leds_debug_on();
				break;
			case 1: 
				last_pin_state = 0;
				GPIOPinWrite(GPIO_D_BASE, GPIO_PIN_2,0x00);
				leds_debug_off();
				break;
			}
		  }
		  count_from_packet = 0;
                  break;
               case APP_STATE_TX:
                  // done sending a packet
                  if(RxMOTE == false){
		     tx_count++; //increment attempt counter
		     if((MULTICHAN_TX) && (tx_count<=1)){
		        radio_setFrequency(CHANNEL+tx_count*CHANNEL_HOP);
		     }
		     //if number of attempts hasn't been reached, reset tx_timer flag to resend
		     if(GPIOPinRead(GPIO_A_BASE, GPIO_PIN_3)!=0) {
		        //leds_debug_on();
		     }else{//leds_debug_off();
			}
		     if((tx_count<NUM_ATTEMPTS) &&(GPIOPinRead(GPIO_A_BASE, GPIO_PIN_3)==0) ){

		        app_vars.flags |= APP_FLAG_TIMER;

		     }else{
                        tx_count=0;
		     }
		  }
                  // switch to RX mode (only if rxmote?)
                  if(RxMOTE){
                     radio_rxEnable();
                  }
                  app_vars.state = APP_STATE_RX;
                  
                  // leds reset after tx
                  leds_sync_off();
		  GPIOPinWrite(GPIO_D_BASE, GPIO_PIN_0,0);
		 
                  break;
            }
            // clear flag
            app_vars.flags &= ~APP_FLAG_END_FRAME;

	    //GPIOPinWrite(GPIO_D_BASE, GPIO_PIN_2,0x00);
	    //GPIOPinWrite(GPIO_A_BASE, GPIO_PIN_3,0);
         }
         
         
         //==== APP_FLAG_TIMER
         
         if (app_vars.flags & APP_FLAG_TIMER) {
            // timer fired
           
               if (app_vars.state==APP_STATE_RX) {

                  // stop listening (this doesn't do much)
                  radio_rfOff();

                  /* this method is from original bsp and isn't used for the counter based packet
                  // prepare packet
                  app_vars.packet_len = sizeof(app_vars.packet);
                  for (i=0;i<app_vars.packet_len;i++) {
                  app_vars.packet[i] = ID;
               }
               */
		   // prepare packet. This loads the 32bit counter into the packet
   		   app_vars.packet_len = sizeof(app_vars.packet);
   		   for (i=0;i<app_vars.packet_len;i++) {
			   if(i<4){
	  			 app_vars.packet[i]=(tx_packet_count&byte_masks[i])>>(8*(3-i));
			   }
			   else{
				if(!RxMOTE){
      	   		   		app_vars.packet[i] = passphrase[i-4];
				}
				else{
					app_vars.packet[i] = 0xAB;
				}
			   }
   		   }

               // start transmitting packet

                  radio_loadPacket(app_vars.packet,app_vars.packet_len);

                  radio_txEnable();

		//should i get rid of this if the mote is an rxmote? this could be a good idea 

                  radio_txNow();
    
                  app_vars.state = APP_STATE_TX;

               }
           // }
            // clear flag
            app_vars.flags &= ~APP_FLAG_TIMER;
	    
         }
      }
   }
}

//=========================== callbacks =======================================

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

   /*if((GPIOPinRead(GPIO_D_BASE,GPIO_PIN_1)==0) && (GPIOPinRead(GPIO_D_BASE,GPIO_PIN_0)==0)){
      leds_error_on();
   }
   else{
      leds_error_off();
   }*/

   // set flag
	
   //GPIOPinWrite(GPIO_D_BASE, GPIO_PIN_0,GPIO_PIN_0);
  // if(isTx){
     // GPIOPinTypeGPIOOutput(GPIO_D_BASE, GPIO_PIN_0);
      //GPIOPinWrite(GPIO_D_BASE, GPIO_PIN_0,0xff);
	//txed=1;
      //app_vars.flags |= APP_FLAG_TIMER;

		
   //}
   //app_vars.flags |= APP_STATE_RX;
   
   // update debug stats
   //app_dbg.num_timer++;
   
   //sctimer_setCompare(sctimer_readCounter()+TIMER_PERIOD);

 
}

void cb_count_rx(void){
rx_packet_count++;
   GPIOPinIntClear(GPIO_A_BASE, GPIO_PIN_4);
}
void cb_button(void){
   uint32_t k;
  // app_vars.flags |= APP_FLAG_TIMER;
   //count++; //increment counter 
   tx_packet_count++;
   tx_count = 0; //reset number of previous tx attempts 
   GPIOPinIntClear(GPIO_A_BASE, GPIO_PIN_2);
   //leds_sync_on();
   if(RxMOTE==false && MULTICHAN_TX){
      radio_setFrequency(CHANNEL);
   }
   debounce_complete = 0;
   for( k =0;k<10;k++){
   }
   if(GPIOPinRead(GPIO_A_BASE, GPIO_PIN_2)!=0){
      debounce_complete=1;
   }

   if(debounce_complete ){
      app_vars.flags |= APP_FLAG_TIMER;
   }

   //}
   //for(i =10000;i!=0;i-- );
   //leds_sync_off(); 

}

