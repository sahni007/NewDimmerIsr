/* 
 * File:   varun_4_1.c
 * Author: VARUNS SAHNI
 *
 * Created on 8 April, 2018, 8:40 PM
 * this is test code for new Isr PWM code 
 * AND WITH MANUAL SWITCH
 * last use: 07/01/19
 */

#include <stdio.h>
#include <stdlib.h>

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = HS        // Oscillator Selection (HS Oscillator, High-speed crystal/resonator connected between OSC1 and OSC2 pins)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable bit (VCAP pin function disabled)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
// Since we have used 16 MHz crystal
#define _XTAL_FREQ 16000000  

// Pin MACROS
#define OUTPUT_RELAY1 PORTFbits.RF1
#define OUTPUT_RELAY2 PORTFbits.RF0
#define OUTPUT_RELAY3 PORTAbits.RA3
#define OUTPUT_RELAY4 PORTAbits.RA2
#define OUTPUT_DIMMER PORTEbits.RE5   // PWM OUTPUT to MOC3021

#define INPUTSWITCH1 PORTFbits.RF7
#define INPUTSWITCH2 PORTFbits.RF5
#define INPUTSWITCH3 PORTFbits.RF3
#define INPUTSWITCH4 PORTFbits.RF2
#define INPUTSWITCH5 PORTAbits.RA5

#define INPUT_SWITCH_DIR_1 TRISFbits.TRISF7
#define INPUT_SWITCH_DIR_2 TRISFbits.TRISF5
#define INPUT_SWITCH_DIR_3 TRISFbits.TRISF3
#define INPUT_SWITCH_DIR_4 TRISFbits.TRISF2
#define INPUT_SWITCH_DIR_5 TRISAbits.TRISA5

#define OUTPUT_RELAY_DIR_1 TRISFbits.TRISF0
#define OUTPUT_RELAY_DIR_2 TRISFbits.TRISF1
#define OUTPUT_RELAY_DIR_3 TRISAbits.TRISA3
#define OUTPUT_RELAY_DIR_4 TRISAbits.TRISA2
#define OUTPUT_DIMMER_DIR_5 TRISEbits.TRISE5        // direction of PWM OUTPUT to MOC3021

/*
 * Extra Periferals Direction and PORT
 */
#define ZCD_CCP9_DIR TRISEbits.TRISE3
// USART Directions
#define USART_1_TRANSMIT_OUTPUT_DIR TRISCbits.TRISC6
#define USART_1_RECIEVE_INPUT_DIR TRISCbits.TRISC7

#define RECIEVED_DATA_LENGTH (16*2)
#define TOTAL_NUMBER_OF_SWITCH (5*2)

#define TRUE 1
#define FALSE 0



// Conditional compilation
//#define DEBUG
//#define RELEASE
#define SWITCH_1_RELAY


#define SWITCH_2_RELAY


#define SWITCH_3_RELAY
//#define SWITCH_3_DIMMER

#define SWITCH_4_RELAY
//#define SWITCH_4_DIMMER

//#define SWITCH_5_RELAY
#define SWITCH_5_DIMMER

// ALL error Definitions
/* 
 * #define WRONG_DATA_RECIEVED_ERROR_CODE ERRX
 * #define RECIVING_OVERRUN_ERROR EROV
 * #define RECEIVING_DATA_LOST_IN_MAIN ERLS
 */
/* DATA USED IN MANUAL  STARTS HERE*/
unsigned int M1;unsigned int M2;unsigned int M3;unsigned int M4;unsigned int M5;


#define ON 1
#define OFF 0
#define CHAR_OFF '0'
#define CHAR_ON '1'
        
/* DATA USED IN MANUAL END HERE*/

unsigned char ErrorNames[5]="####";

int mainReceivedDataPosition=0, mainDataReceived=FALSE;
unsigned char mainReceivedDataBuffer[RECIEVED_DATA_LENGTH]="#"; 
unsigned char tempReceivedDataBuffer[RECIEVED_DATA_LENGTH-8]="#";
unsigned char parentalLockBuffer[10]="000000000";
unsigned char currentStateBuffer[(TOTAL_NUMBER_OF_SWITCH*4)+2]="#";

unsigned int M1;unsigned int M2;unsigned int M3;unsigned int M4;unsigned int M5;

int start_PWM_Generation_in_ISR_FLAG=FALSE;
char levelofDimmer_MSB='0',levelofDimmer_LSB='0';

void errorsISR(char* errNum);
void errorsMain(char* errNum);
void sendAcknowledgment(char* currentStateBuffer);

void clearAllPorts();
void pinINIT_extra();
void GPIO_pin_Initialize();
void peripheralsEnable();
void AllInterruptEnable();
void EUSART_Initialize();

void TMR3_Initialize();
void TMR1_Initialize();
void CCP9_Initialize();
void allPeripheralInit();

void copyReceivedDataBuffer();

void applianceControl(char switchMSB, char switchLSB, char switchSTATE, char dimmerSpeedMSB, char dimmerSpeedLSB, char parentalControl, char finalFrameState);




interrupt void isr(){
    //*******************TIMER3 INTERRUPT**************************//
     if(PIE3bits.TMR3IE==1 && PIR3bits.TMR3IF==1)
    {           
        PIR3bits.TMR3IF=0;
        OUTPUT_DIMMER = TRUE;
        T3CONbits.TMR3ON=0;
       // TX1REG='Q';
    }    
   
     
    //*********************TIMER1 INTERRUPT**************************//
     if(PIE1bits.TMR1IE == 1 && PIR1bits.TMR1IF==1)
    {
        PIR1bits.TMR1IF=0;
        //TX1REG='T';        
        OUTPUT_DIMMER = FALSE;            
        TMR3H=0xFF;
        TMR3L=0xD8;
        T3CONbits.TMR3ON = 1;
        T1CONbits.TMR1ON = 0;        
    }
    //*************************ZCD INTERRRUPT****************************//
    
    if(CCP9IF){
        if(CCP9IF == 1){
             CCP9IF=0;
         if(start_PWM_Generation_in_ISR_FLAG == 1){
          switch(levelofDimmer_MSB)
                {
                case '0':           // 8.5
                    /**/
                        switch(levelofDimmer_LSB)
                             {
                             case '0':           // 8.5
                                    
                                    TMR1H = 123;
                                    TMR1L = 48;
                                    T1CONbits.TMR1ON = 1;
                                  //   OUTPUT_DIMMER=1;
                                     break;
                             case '1':           // 8.4
                                     TMR1H=124;
                                     TMR1L=192;
                                     T1CONbits.TMR1ON = 1;
                                    // OUTPUT_DIMMER=1;
                                     break;
                             case '2':           // 8.35
                                     TMR1H=125;
                                     TMR1L=136;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '3':           // 8.25
                                     TMR1H=127;
                                     TMR1L=24;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '4':          // 8.15
                                     TMR1H=128;
                                     TMR1L=168;
                                    T1CONbits.TMR1ON = 1;
                                     break;
                             case '5':               // 8.1
                                     TMR1H=129;
                                     TMR1L=112;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '6':               // 8.0    
                                     TMR1H=131;
                                     TMR1L=0;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '7':            //7.95
                                     TMR1H=131;
                                     TMR1L=200;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '8':           //7.9
                                     TMR1H=135;
                                     TMR1L=176;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '9':           // 7.85
                                     TMR1H=133;
                                     TMR1L=88;
                                     T1CONbits.TMR1ON = 1;
                                     break;

                             default:
                                 break;
                         }                    
                        break;
                case '1':           // 7.8-7.3

                            switch(levelofDimmer_LSB)
                                 {
                                 case '0':           // 7.8
                                         TMR1H=134;
                                         TMR1L=32;
                                         T1CONbits.TMR1ON = 1;
                                         break;
                                 case '1':           // 7.75
                                         TMR1H=134;
                                         TMR1L=232;
                                         T1CONbits.TMR1ON = 1;
                                         break;
                                 case '2':           // 7.7
                                         TMR1H=135;
                                         TMR1L=176;
                                         T1CONbits.TMR1ON = 1;
                                         break;
                                 case '3':           // 7.65
                                         TMR1H=136;
                                         TMR1L=120;
                                         T1CONbits.TMR1ON = 1;
                                         break;
                                 case '4':            // 7.6
                                         TMR1H=137;
                                         TMR1L=64;
                                         T1CONbits.TMR1ON = 1;
                                         break;
                                 case '5':               // 7.55
                                         TMR1H=138;
                                         TMR1L=8;
                                         T1CONbits.TMR1ON = 1;
                                         break;
                                 case '6':               // 7.5    
                                         TMR1H=138;
                                         TMR1L=208;
                                         T1CONbits.TMR1ON = 1;
                                         break;
                                 case '7':            //7.45
                                         TMR1H=139;
                                         TMR1L=152;
                                         T1CONbits.TMR1ON = 1;
                                         break;
                                 case '8':           //7.4
                                         TMR1H=140;
                                         TMR1L=96;
                                         T1CONbits.TMR1ON = 1;
                                         break;
                                 case '9':           // 7.35
                                         TMR1H=141;
                                         TMR1L=40;
                                         T1CONbits.TMR1ON = 1;
                                         break;

                                 default:
                                     break;
                                }
                        break;
                case '2':           // 7.3-
/**/
                        switch(levelofDimmer_LSB)
                             {
                             case '0':           // 7.3-6.85
                                     TMR1H=141;
                                     TMR1L=240;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '1':           // 7.25
                                     TMR1H=142;
                                     TMR1L=184;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '2':           // 7.20
                                     TMR1H=143;
                                     TMR1L=128;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '3':           // 7.15
                                     TMR1H=144;
                                     TMR1L=72;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '4'://TX1REG='n';      // 7.1
                                     TMR1H=145;
                                     TMR1L=16;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '5':               // 7.05
                                     TMR1H=145;
                                     TMR1L=216;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '6':               // 7.0    
                                     TMR1H=146;
                                     TMR1L=160;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '7':            //6.95
                                     TMR1H=147;
                                     TMR1L=104;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '8':           //6.9
                                     TMR1H=148;
                                     TMR1L=48;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '9':           // 6.85
                                     TMR1H=148;
                                     TMR1L=250;
                                     T1CONbits.TMR1ON = 1;
                                     break;

                             default:
                                 break;
                         }                    
                        break;
                case '3':           // 6.8-5.9                
/**/
                        switch(levelofDimmer_LSB)
                             {
                             case '0':           // 6.8
                                     TMR1H=149;
                                     TMR1L=192;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '1':           // 6.7
                                     TMR1H=151;
                                     TMR1L=80;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '2':           // 6.6
                                     TMR1H=152;
                                     TMR1L=224;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '3':           // 6.5
                                     TMR1H=154;
                                     TMR1L=112;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '4'://TX1REG='n';      // 6.4
                                     TMR1H=156;
                                     TMR1L=0;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '5':               // 6.3
                                     TMR1H=157;
                                     TMR1L=144;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '6':               // 6.2   
                                     TMR1H=159;
                                     TMR1L=32;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '7':            //6.1
                                     TMR1H=160;
                                     TMR1L=176;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '8':           //6.0
                                     TMR1H=162;
                                     TMR1L=64;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '9':           // 5.9
                                     TMR1H=163;
                                     TMR1L=208;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             default:
                                     break;
                            }
                        break;
                case '4'://TX1REG='n';      // 5.8-4.9                    
/**/
                        switch(levelofDimmer_LSB)
                             {
                             case '0':           // 5.8
                                     TMR1H=165;
                                     TMR1L=96;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '1':           // 5.7
                                     TMR1H=166;
                                     TMR1L=240;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '2':           // 5.6
                                     TMR1H=168;
                                     TMR1L=128;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '3':           // 5.5
                                     TMR1H=171;
                                     TMR1L=16;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '4':           // 5.4
                                     TMR1H=172;
                                     TMR1L=160;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '5':               // 5.3
                                     TMR1H=173;
                                     TMR1L=48;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '6':               // 5.2    
                                     TMR1H=174;
                                     TMR1L=192;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '7':              // 5.1
                                     TMR1H=176;
                                     TMR1L=80;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '8':              // 5.0
                                     TMR1H=177;
                                     TMR1L=224;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '9':              // 4.9
                                     TMR1H=179;
                                     TMR1L=112;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             default:
                                     break;
                         }
                        break;
                case '5':               // 4.8-3.9
/**/
                        switch(levelofDimmer_LSB)
                             {
                             case '0':           // 4.8
                                     TMR1H=181;
                                     TMR1L=0;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '1':           // 4.7
                                     TMR1H=182;
                                     TMR1L=144;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '2':           // 4.6
                                     TMR1H=184;
                                     TMR1L=32;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '3':           // 4.5
                                     TMR1H=185;
                                     TMR1L=176;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '4'://TX1REG='n';      // 4.4
                                     TMR1H=187;
                                     TMR1L=64;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '5':               // 4.3
                                     TMR1H=188;
                                     TMR1L=208;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '6':               // 4.2   
                                     TMR1H=189;
                                     TMR1L=96;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '7':            //4.1
                                     TMR1H=190;
                                     TMR1L=240;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '8':           //4.0
                                     TMR1H=191;
                                     TMR1L=128;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '9':           // 3.9
                                     TMR1H=195;
                                     TMR1L=16;
                                     T1CONbits.TMR1ON = 1;
                                     break;

                             default:
                                 break;
                            }                    
                        break;
                case '6':               // 3.8-2.9 
/**/
                        switch(levelofDimmer_LSB)
                             {
                             case '0':           // 3.8
                                     TMR1H=196;
                                     TMR1L=160;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '1':           // 3.7
                                     TMR1H=198;
                                     TMR1L=48;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '2':           // 3.6
                                     TMR1H=199;
                                     TMR1L=190;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '3':           // 3.5
                                     TMR1H=201;
                                     TMR1L=80;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '4'://TX1REG='n';      // 3.4
                                     TMR1H=202;
                                     TMR1L=224;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '5':               // 3.3
                                     TMR1H=204;
                                     TMR1L=112;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '6':               // 3.2   
                                     TMR1H=206;
                                     TMR1L=0;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '7':            //3.1
                                     TMR1H=207;
                                     TMR1L=144;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '8':           // 3.0
                                     TMR1H=209;
                                     TMR1L=32;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '9':           // 2.9
                                     TMR1H=210;
                                     TMR1L=176;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             default:
                                     break;
                            }                    
                        break;
                case '7':            //2.8-1.9
/**/
                        switch(levelofDimmer_LSB)
                             {
                             case '0':           // 2.8
                                     TMR1H=212;
                                     TMR1L=64;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '1':           // 2.7
                                     TMR1H=213;
                                     TMR1L=208;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '2':           // 2.6
                                     TMR1H=215;
                                     TMR1L=96;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '3':           // 2.5
                                     TMR1H=216;
                                     TMR1L=240;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '4'://TX1REG='n';      // 2.4
                                     TMR1H=219;
                                     TMR1L=128;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '5':               // 2.3
                                     TMR1H=221;
                                     TMR1L=16;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '6':               // 2.2  
                                     TMR1H=222;
                                     TMR1L=0xA0;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '7':            // 2.1
                                     TMR1H=224;
                                     TMR1L=48;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '8':           // 2.0
                                     TMR1H=225;
                                     TMR1L=192;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '9':           // 1.9
                                     TMR1H=227;
                                     TMR1L=80;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             default:
                                     break;
                            }
                        break;
                case '8':           //1.8-1.2
/**/
                        switch(levelofDimmer_LSB)
                             {
                             case '0':           // 1.8
                                     TMR1H=227;
                                     TMR1L=224;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '1':           // 1.75
                                     TMR1H=228;
                                     TMR1L=168;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '2':           // 1.7
                                     TMR1H=229;
                                     TMR1L=112;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '3':           // 1.65
                                     TMR1H=230;
                                     TMR1L=56;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '4'://TX1REG='n';      // 1.6
                                     TMR1H=231;
                                     TMR1L=0;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '5':               // 1.5
                                     TMR1H=232;
                                     TMR1L=144;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '6':               // 1.4   
                                     TMR1H=234;
                                     TMR1L=32;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '7':            //1.3
                                     TMR1H=235;
                                     TMR1L=176;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '8':           //1.25
                                     TMR1H=236;
                                     TMR1L=120;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '9':           // 1.2
                                     TMR1H=237;
                                     TMR1L=64;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             default:
                                     break;
                            }
                        break;
                case '9':           // 1.1-0.2
/**/
                        switch(levelofDimmer_LSB)
                             {
                             case '0':           // 1.1
                                     TMR1H=238;
                                     TMR1L=208;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '1':           // 1.0
                                     TMR1H=240;
                                     TMR1L=96;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '2':           // 0.9
                                     TMR1H=241;
                                     TMR1L=240;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '3':           // 0.8
                                     TMR1H=243;
                                     TMR1L=128;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '4'://TX1REG='n';      // 0.7
                                     TMR1H=245;
                                     TMR1L=16;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '5':               // 0.6
                                     TMR1H=246;
                                     TMR1L=160;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '6':               // 0.5    
                                     TMR1H=248;
                                     TMR1L=48;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '7':            //0.4
                                     TMR1H=249;
                                     TMR1L=192;
                                     T1CONbits.TMR1ON = 1;
                                     break;
                             case '8':           //0.3
                                     TMR1H=251;
                                     TMR1L=80;
                                    T1CONbits.TMR1ON = 1;
                                    //   OUTPUT_DIMMER=0;
                                     break;
                             case '9':           // 0.2
                                     TMR1H=252;
                                    TMR1L=224;
                                    T1CONbits.TMR1ON = 1;
                                    //   OUTPUT_DIMMER=0;
                                     break;
                             default:
                                     break;
                            }
                        break;
                default:
                        break;
            } 
         }
        }
       
    }
    
    
    // ************************************* UART INTERRUPT *********************************************** //
    if(RC1IF){        
        if(RC1STAbits.OERR){    // If over run error, then reset the receiver
            RC1STAbits.CREN = 0; // countinuous Recieve Disable
            RC1STAbits.CREN = 1; // countinuous Recieve Enable
            
            ErrorNames[0]='E';      ErrorNames[1]='R';      ErrorNames[2]='O';      ErrorNames[3]='V';
            errorsISR(ErrorNames); 
        } 
        mainReceivedDataBuffer[mainReceivedDataPosition]=RC1REG;
        #ifdef DEBUG
        TX1REG=mainReceivedDataBuffer[mainReceivedDataPosition];
        #endif
        if(mainReceivedDataBuffer[0]=='%'){
            mainReceivedDataPosition++;
            if(mainReceivedDataPosition>15){
                mainDataReceived=TRUE;
                mainReceivedDataPosition=0;                
                RC1IF=0;                
            }
        }
        else{
            RC1STAbits.CREN = 0; // countinuous Recieve Disable
            RC1STAbits.CREN = 1; // countinuous Recieve Enable
            mainReceivedDataPosition=0; // Reinitiate buffer counter
            
            ErrorNames[0]='E';      ErrorNames[1]='R';      ErrorNames[2]='R';      ErrorNames[3]='X';
            errorsISR(ErrorNames);            
        }
    }// End of RC1IF 
}




/*
 * Alfaone Main code starts here
 * For 4 switches 1 Dimmer
 */
int main() {
 
        M1=ON;    M2=ON;     M3=ON;    M4=ON;     M5=ON;
   //     OUTPUT_RELAY1 = OFF; OUTPUT_RELAY2 = OFF; OUTPUT_RELAY3 = OFF; OUTPUT_RELAY4 = OFF;OUTPUT_DIMMER = ON;
    GPIO_pin_Initialize();
    allPeripheralInit();

    
    while(1){
        
        if(mainDataReceived==TRUE){
            mainDataReceived=FALSE;
            if(mainReceivedDataBuffer[0]=='%' && mainReceivedDataBuffer[1]=='%' && mainReceivedDataBuffer[14]=='@' && mainReceivedDataBuffer[15]=='@'){
                copyReceivedDataBuffer();
                
                applianceControl(tempReceivedDataBuffer[0],
                        tempReceivedDataBuffer[1],
                        tempReceivedDataBuffer[2],
                        tempReceivedDataBuffer[3],
                        tempReceivedDataBuffer[4],
                        tempReceivedDataBuffer[5],
                        tempReceivedDataBuffer[6]);
                                
            }   // End of all buffer data check
            else{
                ErrorNames[0]='E';      ErrorNames[1]='R';      ErrorNames[2]='L';      ErrorNames[3]='S';
                errorsMain(ErrorNames);
                RC1STAbits.SPEN=0;  // Serial port disabled 
                RC1STAbits.CREN = 0; // countinuous Recieve Disable                
                for(int dataBufferCounter = 0; dataBufferCounter< 15; dataBufferCounter++)
                {
                    mainReceivedDataBuffer[dataBufferCounter] = '#'; // clean received data buffer
                }
                RC1STAbits.CREN = 1; // countinuous Recieve Enable
                RC1STAbits.SPEN=1;  // Serial port enabled (configures RXx/DTx and TXx/CKx pins as serial port pins)
            }
        } // End of mainDataReceived condition
        
        
        
        /******************** MANUAL RESPONE STARTS HERE************ */
        
        //check switch one status
        //off condition
       int man = 1;
        if(parentalLockBuffer[1] == CHAR_OFF  && INPUTSWITCH1 == OFF && M1 == OFF)
        {
            if(man == 1)
            {
            __delay_ms(5);
            TX1REG = 'R';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '1';__delay_ms(1);
            OUTPUT_RELAY1=OFF;
            }
            man=0;
            M1=1;
            
        }
        //on condition
        if(parentalLockBuffer[1] == CHAR_OFF && INPUTSWITCH1 == ON && M1 == ON)
        {
            if(man==1)
            {
            __delay_ms(5);
            TX1REG = 'R';__delay_ms(1);
            TX1REG = '1';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '1';__delay_ms(1);
            OUTPUT_RELAY1=ON;
            }
            man=0;
            M1=0;
        }
        
       // //check switch second status 
        //off condition
        if(parentalLockBuffer[2] == CHAR_OFF && INPUTSWITCH2 == OFF && M2 == OFF)
        {
            if(man==1)
            {
            __delay_ms(5);
            TX1REG = 'R';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '2';__delay_ms(1);
            OUTPUT_RELAY2=OFF;
            }
            man=0;
            M2=1;
        }
        //on condtion
        if(parentalLockBuffer[2] == CHAR_OFF && INPUTSWITCH2 == ON && M2 == ON)
        {
            if(man==1)
            {
            __delay_ms(5);
            TX1REG = 'R';__delay_ms(1);
            TX1REG = '1';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '2';__delay_ms(1);
            OUTPUT_RELAY2=ON;
            }
            man=0;
            M2=0;
        }
        
        
       // //check switch third status 
        //off condition
        if(parentalLockBuffer[3] == CHAR_OFF && INPUTSWITCH3 == OFF && M3 == OFF)
        {
            if(man == 1)
            {
            __delay_ms(5);
            TX1REG = 'R';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '3';__delay_ms(1);
            OUTPUT_RELAY3=OFF;
            }
            man=0;
            M3=1;
          
        }
        //on condtion
        if(parentalLockBuffer[3] == CHAR_OFF && INPUTSWITCH3 == ON && M3 == ON)
        {
            if(man==1)
            {
            __delay_ms(5);
            TX1REG = 'R';__delay_ms(1);
            TX1REG = '1';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '3';__delay_ms(1);
            OUTPUT_RELAY3=ON;
            }
            man=0;
            M3=0;
            
        }
        
        
       // //check switch fourth status 
        //off condition
        if(parentalLockBuffer[4] == CHAR_OFF && INPUTSWITCH4 == OFF && M4 == OFF)
        {
            if(man==1)
            {
            __delay_ms(5);
            TX1REG = 'R';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '4';__delay_ms(1);
            OUTPUT_RELAY4=OFF;
            }
            man=0;
            M4=1;
            
        }
        //on condtion
        if(parentalLockBuffer[4] == CHAR_OFF && INPUTSWITCH4 == ON && M4 == ON)
        {
            if(man==1)
            {
            __delay_ms(5);
            TX1REG = 'R';__delay_ms(1);
            TX1REG = '1';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '4';__delay_ms(1);
            OUTPUT_RELAY4=ON;
            }
            man=0;
            M4=0;
           
        }
        
             // //check switch fifth status(which can used in dimmer) 
        //off condition
        if(parentalLockBuffer[5] == CHAR_OFF && INPUTSWITCH5 == OFF && M5 == OFF)
        {
            if(man==1)
            {
            start_PWM_Generation_in_ISR_FLAG = 0;
            __delay_ms(5);
            TX1REG = 'R';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '5';__delay_ms(1);
            OUTPUT_DIMMER=ON;
            }
            man=0;
            M5=1;
           
        }
        //on condtion
        if(parentalLockBuffer[5] == CHAR_OFF && INPUTSWITCH5 == ON && M5 == ON)
        {
            if(man==1)
            {
            start_PWM_Generation_in_ISR_FLAG = 0;
            __delay_ms(5);
            TX1REG = 'R';__delay_ms(1);
            TX1REG = '1';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '5';__delay_ms(1);
            OUTPUT_DIMMER=OFF;
            }
            man=0;
            M5=0;
           
        }
    }    
}

void applianceControl(char charSwitchMSB, char charSwitchLSB, char charSwitchSTATE, char chDimmerSpeedMSB, char chDimmerSpeedLSB,
        char charParentalControl, char charFinalFrameState){
    
    //define used variables and initilize it with zero
    int integerSwitchNumber = 0;
    int integerSwitchState = 0;
    int integerSpeed = 0;
    int currentStateBufferPositions=0;
    // Get switch Number in Integer format 
    //define all used character data types and initlize it with "#"
    char switchNumberStringBuffer[2]="#";
    char dimmerSpeedStringBuffer[2]="#";
    
    switchNumberStringBuffer[0]=charSwitchMSB;
    switchNumberStringBuffer[1]=charSwitchLSB;    
    integerSwitchNumber = atoi(switchNumberStringBuffer);//convert string into integer
    
    // Get switch State in Integer Format
    
    integerSwitchState = charSwitchSTATE-'0';
    
    // Get speed of Fan or level of dimmer    
    dimmerSpeedStringBuffer[0]=chDimmerSpeedMSB;
    dimmerSpeedStringBuffer[1]=chDimmerSpeedLSB;    
    integerSpeed = atoi(dimmerSpeedStringBuffer);
    
    // save Parental lock state of each switch into parental lock buffer
//    int integerParentalControl=charParentalControl-'0';
    parentalLockBuffer[integerSwitchNumber] = charParentalControl;
    
    // ACKNOWLEDGMENT data Format :->> (Gateway+SwitchState+SwitchMSB+SwitchLSB)
    
    currentStateBufferPositions = ((1+4*(integerSwitchNumber))-5);
    currentStateBuffer[currentStateBufferPositions++] = 'G';
    currentStateBuffer[currentStateBufferPositions++] = charSwitchSTATE;
    currentStateBuffer[currentStateBufferPositions++] = charSwitchMSB;
    currentStateBuffer[currentStateBufferPositions] = charSwitchLSB;    
    
    currentStateBufferPositions-=3;     // since we have come forward by 3 address in current state buffer
    if(charFinalFrameState=='1')    // until 
    {
        sendAcknowledgment(currentStateBuffer+currentStateBufferPositions);    
    }
    
    switch(integerSwitchNumber){
        case 1:
            OUTPUT_RELAY1 = integerSwitchState;
            break;
        case 2:
            OUTPUT_RELAY2 = integerSwitchState;
            break;
        case 3:
            OUTPUT_RELAY3 = integerSwitchState;

            break;
        case 4:
        
            OUTPUT_RELAY4 = integerSwitchState;
            break;
#ifdef SWITCH_5_RELAY
        case 5:
        {
          start_PWM_Generation_in_ISR_FLAG = 0;
          switch(integerSwitchState){
                case 0:
                    OUTPUT_DIMMER=1;  // For Triac --> inverted condition for off
                    break;
                case 1:
                    OUTPUT_DIMMER=0;
                    break;
                default:
                    break;
            }
        }
#endif
 #ifdef SWITCH_5_DIMMER
        case 5:{
                start_PWM_Generation_in_ISR_FLAG = integerSwitchState;
               switch(integerSwitchState){
                case 0:
                    OUTPUT_DIMMER=1;  // For Triac --> inverted condition for off
                    break;
                case 1:
                    levelofDimmer_MSB = chDimmerSpeedMSB;
                    levelofDimmer_LSB = chDimmerSpeedLSB;
                    break;
                default:
                    break;
               }
#endif
        }break;//end of case 5
        default:
            break;
        }
    
}


/*
 * All input output pin initialization
 */
void GPIO_pin_Initialize(){
    clearAllPorts();
    pinINIT_extra();
    INPUT_SWITCH_DIR_1 = 1;
    INPUT_SWITCH_DIR_2 = 1;
    INPUT_SWITCH_DIR_3 = 1;
    INPUT_SWITCH_DIR_4 = 1;
    INPUT_SWITCH_DIR_5 = 1;
    
    OUTPUT_RELAY_DIR_1 = 0;
    OUTPUT_RELAY_DIR_2 = 0;
    OUTPUT_RELAY_DIR_3 = 0;
    OUTPUT_RELAY_DIR_4 = 0;
    OUTPUT_DIMMER_DIR_5 = 0; 
    
    // peripherals directions
    ZCD_CCP9_DIR = 1;
    // USART DIRECTIONS
    USART_1_TRANSMIT_OUTPUT_DIR = 0;
    USART_1_RECIEVE_INPUT_DIR = 1;
    
    clearAllPorts();
}

/*
 * ALL Peripheral Initialization
 */
void allPeripheralInit(){
    EUSART_Initialize();
    TMR1_Initialize();
    TMR3_Initialize();
    CCP9_Initialize();
}

/*
 * USART Control Registers initialization
 */
void EUSART_Initialize(){
    PIE1bits.RC1IE = 0;
    PIE1bits.TX1IE = 0;

    // Set the EUSART module to the options selected in the user interface.

    // ABDOVF no_overflow; SCKP Non-Inverted; BRG16 16bit_generator; WUE enabled; ABDEN disabled;
    BAUD1CON = 0x0A;

    // SPEN enabled; RX9 8-bit; CREN enabled; ADDEN disabled; SREN disabled;
    RC1STA = 0x90;

    // TX9 8-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; BRGH hi_speed; CSRC slave;
    TX1STA = 0x24;

    // Baud Rate = 9600; SP1BRGL 12;
    //SPBRGL = 0x0C;
    //SPBRGL = 0x19;                  // SP1BRGL is 25 (hex value=0x19) for 9600 baud on 16 MHz crystal frequency
    SP1BRGL = 0xA0;                  // SYNC =0 ; BRGH = 1 ; BRG16=1;
    // Baud Rate = 9600; SP1BRGH 1;
    SP1BRGH = 0x01;

    // Enable all active interrupts ---> INTCON reg .... bit 7            page 105
    GIE = 1;

    // Enables all active peripheral interrupts -----> INTCON reg .... bit 6         page 105
    PEIE = 1;

    // enable receive interrupt
    PIE1bits.RC1IE = 1;                    // handled into INTERRUPT_Initialize()

    // Transmit Enabled
    TX1STAbits.TXEN = 1;

    // Serial Port Enabled
    RC1STAbits.SPEN = 1;
}
void TMR1_Initialize(void)
{
   
    T1CON = 0x00;

    //T1GSS T1G; TMR1GE disabled; T1GTM disabled; T1GPOL low; T1GGO_nDONE done; T1GSPM disabled;
    T1GCON = 0x00;

        //TMR1H 29;
    TMR1H = 0x00;

    //TMR1L 112;
    TMR1L = 0x00;

    // Clearing IF flag before enabling the interrupt.
    PIR1bits.TMR1IF = 0;

    // Enabling TMR1 interrupt.
    PIE1bits.TMR1IE = 1;

    // Start TMR1
   // T1CONbits.TMR1ON = 1;

    // Enable all active interrupts ---> INTCON reg .... bit 7            page 105
    GIE = 1;

    // Enables all active peripheral interrupts -----> INTCON reg .... bit 6         page 105
    PEIE = 1;

}

void TMR3_Initialize(void)
{

    T3CON = 0x00;

    //T1GSS T1G; TMR1GE disabled; T1GTM disabled; T1GPOL low; T1GGO_nDONE done; T1GSPM disabled;
    T3GCON = 0x00;

        //TMR1H 29;
    TMR3H = 0x00;
 
    //TMR1L 112;
    TMR3L = 0x00;

    // Clearing IF flag before enabling the interrupt.
    PIR3bits.TMR3IF = 0;

    // Enabling TMR1 interrupt.
    PIE3bits.TMR3IE = 1;

    // Start TMR1
   // T1CONbits.TMR1ON = 1;

    // Enable all active interrupts ---> INTCON reg .... bit 7            page 105
    GIE = 1;

    // Enables all active peripheral interrupts -----> INTCON reg .... bit 6         page 105
    PEIE = 1;

}
void CCP9_Initialize(){
    // Set the CCP1 to the options selected in the User Interface

    // MODE Every edge; EN enabled; FMT right_aligned;
    CCP9CON = 0x84;

    // RH 0;
    CCPR9H = 0x00;

    // RL 0;
    CCPR9L = 0x00;
    
//    CCPTMRS2bits.C9TSEL0=0;
//    CCPTMRS2bits.C9TSEL1=0;

    // Clear the CCP1 interrupt flag
    PIR4bits.CCP9IF = 0;

    // Enable the CCP1 interrupt
    PIE4bits.CCP9IE = 1;
}

void peripheralsEnable(){
    // Transmit Enabled
    TX1STAbits.TXEN = 1;

    // Serial Port Enabled
    RC1STAbits.SPEN = 1;
}
void AllInterruptEnable(){
    // Enable all active interrupts ---> INTCON reg .... bit 7            page 105
    GIE = 1;

    // Enables all active peripheral interrupts -----> INTCON reg .... bit 6         page 105
    PEIE = 1;
    
    // enable receive interrupt
    PIE1bits.RC1IE = 1;                    // handled into INTERRUPT_Initialize()

}

void errorsISR(char* errNum){
    int Tx_count=0;
  	while(Tx_count!=4)
 	{ 
        while (!TX1STAbits.TRMT);
 		TX1REG = *errNum;
 		*errNum++;
        Tx_count++;
 	}
}
void errorsMain(char* errNum){
   int Tx_count=0;
  	while(Tx_count!=4)
 	{ 
        while (!TX1STAbits.TRMT);
 		TX1REG = *errNum;
 		*errNum++;
        Tx_count++;
 	}
}
void sendAcknowledgment(char* currentStateBuffer){
  int Tx_count=0;
  	while(Tx_count!=4)
 	{ 
        while (!TX1STAbits.TRMT);
//        TX1REG='S';
 		TX1REG = *currentStateBuffer;
 		*currentStateBuffer++;
        Tx_count++;
 	}
}

void copyReceivedDataBuffer(){
    int dataBufferCounter=2;
    for(dataBufferCounter=2;dataBufferCounter<9;dataBufferCounter++){
        tempReceivedDataBuffer[dataBufferCounter-2]=mainReceivedDataBuffer[dataBufferCounter]; // copy data buffer from main
        mainReceivedDataBuffer[dataBufferCounter]='#';  // clean data buffer
    }
}
/*
 * AANALOG and PULL up REGISTERS related initialization
 */
void pinINIT_extra(){
    ANSELG=0x00;    WPUG = 0;
    
    ANSELF=0x00;
    
    ANSELE=0x00;    WPUE=0x00;
    
    ANSELD=0x00;    WPUD=0x00;
    
    ANSELB=0x00;    WPUB=0x00;
    
    ANSELA=0x00;     
} 

/*
 * always clear all the ports before initialization
 */
void clearAllPorts(){
    OUTPUT_RELAY1=0;
    OUTPUT_RELAY2=0;
    OUTPUT_RELAY3=0;
    OUTPUT_RELAY4=0;
    OUTPUT_DIMMER=1;
}