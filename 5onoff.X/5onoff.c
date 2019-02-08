/**
 * File:   only_on_off.c
 * Author: Varun sahni
 *client: test
 * module: this is test code for New hardware with 8switches and one dimmer and one RGB
 * Note: for switching the led 
 * ON>>give low
 * OFF>>give 
 * Tip: use function as much possible for memory consumption(when i was using the R101 with normal TX1REG it use 39% of memory while using function it use 31% of memory(8% of memory relesed))
 * Note: LEVEL Shifter is used to supply 3.3 voltage to xbee.
 * #RA3 pin is used , keep RA3 pin to operate in 5 voltade mode, keep RA3 LOW to operate in 3.3 voltage
 * #In this hardware, input switch works diffrently then our original---->>input switch work as the keypad...all input switch initlize as high and when we press it it will give low output
 * so work opposite to the original one but output is same original -->>load is operating with transistor
 * # In this hardware we also debug the transmitting and received frame--->>>after receiveing the frame glow Blue Led
 *                                                                     ---->>>after transmitting the frame glow Green Led
 * 
 *  
 */


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

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include<pic16f1526.h>
#include<string.h>
#include <math.h>
// Since we have used 16 MHz crystal
#define _XTAL_FREQ 16000000  

// Pin MACROS
#define OUTPUT_RELAY1 PORTEbits.RE0
#define OUTPUT_RELAY2 PORTEbits.RE1
#define OUTPUT_RELAY3 PORTGbits.RG0
#define OUTPUT_RELAY4 PORTGbits.RG3
#define OUTPUT_RELAY5 PORTGbits.RG4
#define OUTPUT_RELAY6 PORTFbits.RF6
#define OUTPUT_RELAY7 PORTFbits.RF5
#define OUTPUT_RELAY8 PORTFbits.RF4
#define OUTPUT_DIMMER PORTFbits.RF3 //MOC


#define OUTPUT_RELAY_DIR_1 TRISEbits.TRISE0
#define OUTPUT_RELAY_DIR_2 TRISEbits.TRISE1
#define OUTPUT_RELAY_DIR_3 TRISGbits.TRISG0
#define OUTPUT_RELAY_DIR_4 TRISGbits.TRISG3
#define OUTPUT_RELAY_DIR_5 TRISGbits.TRISG4
#define OUTPUT_RELAY_DIR_6 TRISFbits.TRISF6
#define OUTPUT_RELAY_DIR_7 TRISFbits.TRISF5
#define OUTPUT_RELAY_DIR_8 TRISFbits.TRISF4
#define OUTPUT_DIMMER_DIR TRISFbits.TRISF3

//RGB MACORS
#define OUTPUT_FOR_RED_LED PORTDbits.RD4 ///pwm for red
#define OUTPUT_FOR_GREEN_LED PORTDbits.RD3 //pwm for green
#define OUTPUT_FOR_BLUE_LED PORTDbits.RD2   // PWM for blue

#define OUTPUT_RELAY_RED_LED_DIR TRISDbits.TRISD4
#define OUTPUT_RELAY_GREEN_LED_DIR TRISDbits.TRISD3
#define OUTPUT_RELAY_BLUE_LED_DIR TRISDbits.TRISD2        // direction of PWM OUTPUT to MOC3021

#define ZCD_CCP9_DIR TRISEbits.TRISE3//for rgb
#define ZCD_CCP7_DIR TRISEbits.TRISE5
#define ZCD_CCP8_DIR TRISEbits.TRISE4

#define ZCD_CCP10_DIR TRISEbits.TRISE3//for dimmer


#define INPUTSWITCH1 PORTDbits.RD6
#define INPUTSWITCH2 PORTDbits.RD7
#define INPUTSWITCH3 PORTBbits.RB0
#define INPUTSWITCH4 PORTBbits.RB1
#define INPUTSWITCH5 PORTBbits.RB2
#define INPUTSWITCH6 PORTBbits.RB3
#define INPUTSWITCH7 PORTBbits.RB4
#define INPUTSWITCH8 PORTBbits.RB5
#define INPUTSWITCH9 PORTDbits.RD5

#define INPUT_SWITCH_DIR_1 TRISDbits.TRISD6
#define INPUT_SWITCH_DIR_2 TRISDbits.TRISD7
#define INPUT_SWITCH_DIR_3 TRISBbits.TRISB0
#define INPUT_SWITCH_DIR_4 TRISBbits.TRISB1
#define INPUT_SWITCH_DIR_5 TRISBbits.TRISB2
#define INPUT_SWITCH_DIR_6 TRISBbits.TRISB3
#define INPUT_SWITCH_DIR_7 TRISBbits.TRISB4
#define INPUT_SWITCH_DIR_8 TRISBbits.TRISB5
#define INPUT_SWITCH_DIR_9 TRISDbits.TRISD5



//macros for leds

#define OUTPUT_TRANMIT_INDICATION_LED PORTFbits.RF0 //green
#define OUTPUT_RECEIVE_INDICATION_LED PORTFbits.RF1 //blue led
#define USER_RECEIVE_INDICATION_LED PORTFbits.RF2 //blue led

#define OUTPUT_TRANMIT_INDICATION_LED_DIR TRISFbits.TRISF0
#define OUTPUT_RECEIVE_INDICATION_LED_DIR TRISFbits.TRISF1
#define USER_RECEIVE_INDICATION_LED_DIR TRISFbits.TRISF2

#define LEVEL_SHIFTER_OUTPUT_PIN PORTAbits.RA3
#define LEVEL_SHIFTER_OUTPUT_PIN_DIR TRISAbits.TRISA3
// direction of PWM OUTPUT to MOC3021

/*
 * Extra Periferals Direction and PORT
 */
//#define ZCD_CCP9_DIR TRISEbits.TRISE3
// USART Directions
#define USART_1_TRANSMIT_OUTPUT_DIR TRISCbits.TRISC6
#define USART_1_RECIEVE_INPUT_DIR TRISCbits.TRISC7

#define RECIEVED_DATA_LENGTH (16+2)
#define TOTAL_NUMBER_OF_SWITCH (12+2)

#define TRUE 1
#define FALSE 0



// Conditional compilation
//#define DEBUG
//#define RELEASE


// ALL error Definitions
/* 
 * #define WRONG_DATA_RECIEVED_ERROR_CODE ERRX
 * #define RECIVING_OVERRUN_ERROR EROV
 * #define RECEIVING_DATA_LOST_IN_MAIN ERLS
 */
/* DATA USED IN MANUAL  STARTS HERE*/



#define ON 1
#define OFF 0
#define CHAR_OFF '0'
#define CHAR_ON '1'
#define SWITCH_PRESSED 0
#define SWITCH_RELEASED 1
#define GIVES_OUTPUT_ZERO_IF_EVEN 1
//#define USEMANUAL_SWITCH1_REDLIGHT_DIMMABLE
//#define USEMANUAL_SWITCH2_GREENLIGHT_DIMMABLE
//#define USEMANUAL_SWITCH3_BLUELIGHT_DIMMABLE
#define USEMANUAL_SWITCH1_REDLIGHT_SWITCH 
#define USEMANUAL_SWITCH1_GREENLIGHT_SWITCH
#define USEMANUAL_SWITCH1_BLUELIGHT_SWITCH     
// Conditional compilation
//#define DEBUG
#define MODULENAMELENGTH 3
#define SWITCHNUMBERLENGTH 2
//#define ACKNOWLEDGEMENTLENGTH 6
#define SWITCHSTATUS 1
#define REDCOLOINTENSITYLENGTH 2
#define GREENCOLORINTENSITYLENGHT 2
#define BLUECOLOINTENSITYLENGHT 2
#define CHILDLOCKLENGTH 1
#define FINALFRAMEBITLENGTH 1

#define FINALFRAMELENGTH ((3+2+6+1+3+3+3+1+1)*2)

unsigned char ModuleNameBuffer[MODULENAMELENGTH]=NULL;
unsigned char SwitchNumberBuffer[SWITCHNUMBERLENGTH]=NULL;
//unsigned char AcknowledgementBuffer[ACKNOWLEDGEMENTLENGTH]=NULL;
unsigned char SwitchStatusBuffer[SWITCHSTATUS]=NULL;
unsigned char RedColorIntensityBuffer[REDCOLOINTENSITYLENGTH]=NULL;
unsigned char GreenColorIntensityBuffer[GREENCOLORINTENSITYLENGHT]=NULL;
unsigned char BlueColorIntensityBuffer[BLUECOLOINTENSITYLENGHT]=NULL;
unsigned char ChildLockBuffer[CHILDLOCKLENGTH]=NULL;
unsigned char FinalFramebitBuffer[FINALFRAMEBITLENGTH]=NULL;

unsigned char sendFinalBufferToGAteway[FINALFRAMELENGTH]=NULL;
/* DATA USED IN MANUAL END HERE*/


unsigned char mainReceivedDataBuffer[RECIEVED_DATA_LENGTH]="#"; 
unsigned char tempReceivedDataBuffer[RECIEVED_DATA_LENGTH-8]="#";
unsigned char parentalLockBuffer[TOTAL_NUMBER_OF_SWITCH]="000000000000";
unsigned char copy_parentalLockBuffer[TOTAL_NUMBER_OF_SWITCH]="000000000000";
unsigned char currentStateBuffer[(TOTAL_NUMBER_OF_SWITCH*4)+2]="#";

///rgb related date
unsigned char ErrorNames[5]="####";
unsigned int FrameLegthCounter =0;
int mainReceivedDataPosition=0, mainDataReceived=FALSE;
int mainReceivedDataFlag=0;
int mainDatacopyPosition=0;
//unsigned char copyparentalLockBuffer[6]="00000";

int start_PWM_Generation_For_DIMMER=FALSE;
int start_PWM_Generation_For_RedLed=FALSE;
int start_PWM_Generation_For_GreenLed = FALSE;
int start_PWM_Generation_For_BlueLed = FALSE;
char levelofDimmer_MSB='0',levelofDimmer_LSB='0';
char levelofBlueLed_MSB = '0',levelofBlueLed_LSB = '0';
char levelofRedLed_MSB = '0',levelofRedLed_LSB = '0';
char levelofGreenLed_MSB = '0',levelofGreenLed_LSB = '0';
int Timer1H = 0,Timer1L=0;
int Timer3H = 0,Timer3L=0;
int Timer5H = 0,Timer5L=0;
int Timer8H = 0,Timer8L=0;


        
        unsigned char ConvertIntegertoStringRedColorBuffer[REDCOLOINTENSITYLENGTH] = NULL;
        unsigned char ConvertIntegertoStringGreenColorBuffer[GREENCOLORINTENSITYLENGHT] = NULL;
        unsigned char ConvertIntegertoStringBlueColorBuffer[BLUECOLOINTENSITYLENGHT] = NULL;
        
 char *ConvertmaindataReceiveIntoString = NULL;
 int mainDataReceiveStringLength=0;
 int RedColorIntensityStringLength =0;
 int GreenColorIntensityStringLength=0;
 int AllColorIntensityStringLength=0;
 int BlueColorIntensityStringLength =0;

char *token=NULL;
int partCounter=0;


char *ModuleNameString=NULL;
char *SwitchNumberString = NULL;
char switchNumberBuffer[3]=NULL;
char *AcknowledgementString =NULL;
char *SwitchStatusString = NULL;
char switchstatusbuffer[2]=NULL;
char *RedColorIntensityString = NULL;
char *GreenColorIntensityString = NULL;
char *BlueColorIntensityString = NULL;
char *AllColorIntensityString = NULL;
char *ChildLockString = NULL;
char ChildLockbuffer[2]=NULL;
char *FinalframebitString = NULL;
char Finalframebitbuffer[2] = NULL;
char *StopBitString=NULL;
char StopBitStringbufer=NULL;

int RedLedSwitchCounter=0;
int GreenLedSwitchCounter =0 ;
int BlueLedSwitchCounter =0;
        int IntegerSwitchNumber=0;
        int IntergerSwitchStatus = 0;
        unsigned int IntergerRedColorIntensity = 0;
        unsigned int IntegerGreenColorIntensity = 0;
        unsigned int IntegerBlueColorIntensity = 0;
        unsigned int IntegerAllColorIntensity = 0;
        int IntegerChildLock = 0;
        char charchildLock = '0';
        char charSwitchState = '0';



unsigned int M1;unsigned int M2;unsigned int M3;unsigned int M4;
unsigned int M5;unsigned int M6;unsigned int M7;unsigned int M8;
unsigned int M9;

void errorsISR(const char* errNum);
void errorsMain(const char* errNum);
void sendAcknowledgment(char* currentStateBuffer);
void Send_Acknowlde_To_RedPWM(char charlevelofRedLed_LSB);
void Send_Acknowlde_To_GreenPWM(char charlevelofGreenLed_LSB);
void Send_Acknowlde_To_BluePWM(char charlevelofBlueLed_LSB);
void SwitchOffStatustToGatway(int SwitchOffNumber);
void SwitchOnStatustToGatway(int SwitchOnNumber);
void TransmissionIndicationLedBlinking();
void ReceivingIndicationLedBlinking();
void clearAllPorts();
void pinINIT_extra();
void GPIO_pin_Initialize();
void EUSART_Initialize();

void TMR3_Initialize();
void TMR1_Initialize();
void TMR2_Initialize();
void TMR4_Initialize();
void TMR5_Initialize();
void TMR6_Initialize();
void TMR8_Initialize();
void TMR10_Initialize();
void CCP9_Initialize();
void CCP10_Initialize();
void CCP7_Initialize();
void CCP8_Initialize();
void allPeripheralInit();

void copyReceivedDataBuffer();

void applianceControl(char switchMSB, char switchLSB, char switchSTATE, char dimmerSpeedMSB, char dimmerSpeedLSB, char parentalControl, char finalFrameState);
void applicationControlRGB(char *ModuleName,char *SwitchNumber,char *SwitchStatus,
                char  *RedColorIntensity,char *GreenColorIntensity,char *BlueColorIntensity,char *ChildLock,char *AllColorStringlength);
#include"include.h"
interrupt void isr(){
 
    // ************************************* UART INTERRUPT *********************************************** //
    if(RC1IF){        
        if(RC1STAbits.OERR){    // If over run error, then reset the receiver
            RC1STAbits.CREN = 0; // countinuous Recieve Disable
            RC1STAbits.CREN = 1; // countinuous Recieve Enable
            errorsISR("EROV");
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
        else if((mainReceivedDataBuffer[0]=='R') && mainReceivedDataBuffer[0] != NULL)
        {
             mainReceivedDataPosition++;
            if(mainReceivedDataBuffer[mainReceivedDataPosition] == '|'){
                mainDataReceived=TRUE;
                mainReceivedDataPosition=0;           
                RC1IF=0;                
            }
        }
        else{
            RC1STAbits.CREN = 0; // countinuous Recieve Disable
            RC1STAbits.CREN = 1; // countinuous Recieve Enable
            mainReceivedDataPosition=0; // Reinitiate buffer counter
            errorsISR("ERRX");
        }
    }// End of RC1IF 
    
    
   //*******************************DIMMER 11111 *************************************
    
    if(PIE1bits.TMR2IE==1 && PIR1bits.TMR2IF==1)
    {        

       
        PIR1bits.TMR2IF=0;
        OUTPUT_DIMMER=ON;
        T2CONbits.TMR2ON=0;
    } 
    
     if(PIE1bits.TMR1IE == 1 && PIR1bits.TMR1IF==1)
    {

        PIR1bits.TMR1IF=0;
        T1CONbits.TMR1ON = 0;        
        OUTPUT_DIMMER=OFF;
        PR2=0x9F;
        T2CONbits.TMR2ON=1;
               
    }

//    //*******************************DIMMER 22222 *************************************
//    
//    if(PIE3bits.TMR4IE==1 && PIR3bits.TMR4IF==1)
//    {           
//
//        PIR3bits.TMR4IF=0;
//        OUTPUT_DIMMER2=ON;
//        T4CONbits.TMR4ON=0;
//
//    }
//    
//    if(PIE3bits.TMR3IE == 1 && PIR3bits.TMR3IF==1)
//    {
//
//        PIR3bits.TMR3IF=0;
//        
//        OUTPUT_DIMMER2=OFF;
//        PR4=0x9F;
//        T4CONbits.TMR4ON=1;
//        T3CONbits.TMR3ON = 0;        
//    }
//
//    //*******************************DIMMER 33333 *************************************    
//    
//    if(PIE3bits.TMR6IE == 1 && PIR3bits.TMR6IF == 1)
//    {           
//        PIR3bits.TMR6IF=0;
//        OUTPUT_DIMMER3=ON;
//        T6CONbits.TMR6ON=0;
//    } 
//    
//    if(PIE3bits.TMR5IE == 1 && PIR3bits.TMR5IF==1)
//    {
//         PIR3bits.TMR5IF=0;        
//        OUTPUT_DIMMER3=OFF;
//        PR6=0x9F;
//        T6CONbits.TMR6ON=1;
//        T5CONbits.TMR5ON=0;        
//    }
    
    
    
    //*************************ZCD INTERRRUPT****************************//
    if(PIR4bits.CCP8IF==1 || PIR4bits.CCP7IF == 1 || PIR4bits.CCP9IF==1 || PIR4bits.CCP10IF==1){
    if(CCP10IF){
        if(CCP10IF == 1){
             CCP10IF=0;
         if(start_PWM_Generation_For_DIMMER == 1)
                                    TMR1H = Timer1H;
                                    TMR1L = Timer1L;
                                    T1CONbits.TMR1ON = 1;
       
                            }//end of ccp10
                }
        }//end of CCP
}




/*
 * Alfaone Main code starts here
 * For 4 switches 1 Dimmer
 */
int main() {
    __delay_ms(2000);
        M1=ON;    M2=ON;    M3=ON;    M4=ON;    M5=ON; M6=ON; M7=ON;M8=ON;
        M9=ON; 
            GPIO_pin_Initialize();
            allPeripheralInit();
    
    while(1){       
        if(mainDataReceived==TRUE){
            mainDataReceived=FALSE;
            if(mainReceivedDataBuffer[0]=='%' && mainReceivedDataBuffer[1]=='%' && mainReceivedDataBuffer[14]=='@' && mainReceivedDataBuffer[15]=='@'){
            ReceivingIndicationLedBlinking();
                copyReceivedDataBuffer();
                
                applianceControl(tempReceivedDataBuffer[0],
                        tempReceivedDataBuffer[1],
                        tempReceivedDataBuffer[2],
                        tempReceivedDataBuffer[3],
                        tempReceivedDataBuffer[4],
                        tempReceivedDataBuffer[5],
                        tempReceivedDataBuffer[6]);
                                
            }   
            else if(mainReceivedDataBuffer[0]=='R' && mainReceivedDataBuffer[1]=='G'){
              ReceivingIndicationLedBlinking();
               ConvertmaindataReceiveIntoString = mainReceivedDataBuffer;
           //    mainDataReceiveStringLength = strlen(ConvertmaindataReceiveIntoString);
           
           //    sendAcknowledgment(ConvertmaindataReceiveIntoString);
            //**************strat breaking the string**************//
              
           //  token = strtok(ConvertmaindataReceiveIntoString,delimiter);
               token = strtok(ConvertmaindataReceiveIntoString,".");
             ModuleNameString = token;
          //  sendAcknowledgment(ModuleNameString);//ok
             partCounter=0;
             if((strcmp(ModuleNameString,"RGB") == 0)){  
             //  errorsMain("KILO");//OK
             // errorsMain(ModuleName);//ok
               while(token != NULL)
                {
                    partCounter++;
                    token = strtok(NULL,".");                 
                    switch(partCounter){
                        case 1:
                        {   
                            int j=0;
                            for(j=0;j<1;j++)
                            {
                              switchNumberBuffer[j]= *token;
                              *token++;
                            }
                            SwitchNumberString = switchNumberBuffer;
                        //    sendAcknowledgment(SwitchNumberString);
                            
                        }break;
                        
                        case 2:
                        {
                            AcknowledgementString = token;
                         //  errorsMain(AcknowledgementString); //ok                     
   
                        }break;
                        case 3:
                        {
                            int k=0;
                            for(k=0;k<1;k++)
                            {
                              switchstatusbuffer[k]= *token;
                              *token++;
                            }
                            SwitchStatusString = switchstatusbuffer;
                           // sendAcknowledgment(SwitchStatusString);
                        
                           
                        } break;
                        case 4:
                        {
                            RedColorIntensityString = token;
                            RedColorIntensityStringLength = strlen(RedColorIntensityString);
                          //  errorsMain(RedColorIntensityString);//255
                           
                        } break;
                        case 5:
                        {
                            GreenColorIntensityString = token;
                            GreenColorIntensityStringLength = strlen(GreenColorIntensityString);
                        //   errorsMain(GreenColorIntensityString);//ok-255
                           
                        } break;
                        case 6:
                        {
                            BlueColorIntensityString = token;
                            BlueColorIntensityStringLength = strlen(BlueColorIntensityString);
                        //    errorsMain(BlueColorIntensityString);//ok-255
                            
                        }  break; 
                        case 7:
                        {
                            
                            int k=0;
                            for(k=0;k<1;k++)
                            {
                              ChildLockString[k]= *token;
                              *token++;
                            }
                            ChildLockString = ChildLockbuffer;
                            //sendAcknowledgment(ChildLockString);//ok
                           
                        } break;
                        case 8:
                        {
                            AllColorIntensityString = token;
                            AllColorIntensityStringLength = strlen(AllColorIntensityString);

                          //  sendAcknowledgment(AllColorIntensityString);

                            
                        }break;

                        default:
                            break;
                         
                            
                    }
                }//end of while

             } 
             
           //        applicationControlRGB(ModuleNameString,SwitchNumberString,SwitchStatusString,
         //        RedColorIntensityString,GreenColorIntensityString,BlueColorIntensityString,ChildLockString,AllColorIntensityString ); 
            }
            
            else
            {

                errorsMain("ERLS");
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
        
 
       int man = 1;
        if(copy_parentalLockBuffer[1] == CHAR_OFF  && INPUTSWITCH1 == SWITCH_PRESSED && M1 == OFF)//switch is pressed
        {
            if(man == 1)
            {
                SwitchOnStatustToGatway(1);
            
            TransmissionIndicationLedBlinking();
            OUTPUT_RELAY1=ON;
            
            }
            man=0;
            M1=1;
            
        }
        //on condition
        if(copy_parentalLockBuffer[1] == CHAR_OFF && INPUTSWITCH1 == SWITCH_RELEASED && M1 == ON)////switch is released
        {
            //TX1REG='C';
            if(man==1)
            {
            SwitchOffStatustToGatway(1);
            TransmissionIndicationLedBlinking();
            OUTPUT_RELAY1=OFF;
            }
            man=0;
            M1=0;
        }
        
       // //check switch second status 
        //off condition
        if(copy_parentalLockBuffer[2] == CHAR_OFF && INPUTSWITCH2 == SWITCH_PRESSED && M2 == OFF)
        {
            if(man==1)
            {
            SwitchOnStatustToGatway(2);           
            TransmissionIndicationLedBlinking();
            OUTPUT_RELAY2=ON;
            }
            man=0;
            M2=1;
        }
        //on condtion
        if(copy_parentalLockBuffer[2] == CHAR_OFF && INPUTSWITCH2 == SWITCH_RELEASED && M2 == ON)
        {
            if(man==1)
            {
            SwitchOffStatustToGatway(2);
            TransmissionIndicationLedBlinking();
            OUTPUT_RELAY2=OFF;
            }
            man=0;
            M2=0;
        }
        
        
       // //check switch third status 
        //off condition
        if(copy_parentalLockBuffer[3] == CHAR_OFF && INPUTSWITCH3 == SWITCH_PRESSED && M3 == OFF)
        {
            if(man == 1)
            {
            SwitchOnStatustToGatway(3);
            TransmissionIndicationLedBlinking();
            OUTPUT_RELAY3=ON;
            }
            man=0;
            M3=1;
          
        }
        //on condtion
        if(copy_parentalLockBuffer[3] == CHAR_OFF && INPUTSWITCH3 == SWITCH_RELEASED && M3 == ON)
        {
            if(man==1)
            {
            SwitchOffStatustToGatway(3);
            TransmissionIndicationLedBlinking();
            OUTPUT_RELAY3=OFF;
            }
            man=0;
            M3=0;
            
        }
        
     
       // //check switch fourth status 
        //off condition
        if(copy_parentalLockBuffer[4] == CHAR_OFF && INPUTSWITCH4 == SWITCH_PRESSED && M4 == OFF)
        {
            if(man==1)
            {
            SwitchOnStatustToGatway(4);
            TransmissionIndicationLedBlinking();
            OUTPUT_RELAY4=ON;
            }
            man=0;
            M4=1;
            
        }
        //on condtion
        if(copy_parentalLockBuffer[4] == CHAR_OFF && INPUTSWITCH4 == SWITCH_RELEASED && M4 == ON)
        {
            if(man==1)
            {
            
            SwitchOffStatustToGatway(4);
            TransmissionIndicationLedBlinking();
            OUTPUT_RELAY4=OFF;
            }
            man=0;
            M4=0;
           
        }
               //off condition
        if(copy_parentalLockBuffer[5] == CHAR_OFF && INPUTSWITCH5 == SWITCH_PRESSED && M5 == OFF)
        {
            if(man==1)
            {
            SwitchOnStatustToGatway(5);
            TransmissionIndicationLedBlinking();
            OUTPUT_RELAY5=ON;
            }
            man=0;
            M5=1;
            
        }
        //on condtion
        if(copy_parentalLockBuffer[5] == CHAR_OFF && INPUTSWITCH5 == SWITCH_RELEASED && M5 == ON)
        {
            if(man==1)
            {
            
            SwitchOffStatustToGatway(5);
            TransmissionIndicationLedBlinking();
            OUTPUT_RELAY5=OFF;
            }
            man=0;
            M5=0;
           
        }
       
               //off condition
        if(copy_parentalLockBuffer[6] == CHAR_OFF && INPUTSWITCH6 == SWITCH_PRESSED && M6 == OFF)
        {
            if(man==1)
            {
            SwitchOnStatustToGatway(6);
            TransmissionIndicationLedBlinking();
            OUTPUT_RELAY6=ON;
            }
            man=0;
            M6=1;
            
        }
        //on condtion
        if(copy_parentalLockBuffer[6] == CHAR_OFF && INPUTSWITCH6 == SWITCH_RELEASED && M6 == ON)
        {
            if(man==1)
            {
            
            SwitchOffStatustToGatway(6);
            TransmissionIndicationLedBlinking();
            OUTPUT_RELAY6=OFF;
            }
            man=0;
            M6=0;
           
        }
       

               //off condition
        if(copy_parentalLockBuffer[7] == CHAR_OFF && INPUTSWITCH7 == SWITCH_PRESSED && M7 == OFF)
        {
            if(man==1)
            {
            SwitchOnStatustToGatway(7);
            TransmissionIndicationLedBlinking();
            OUTPUT_RELAY7=ON;
            }
            man=0;
            M7=1;
            
        }
        //on condtion
        if(copy_parentalLockBuffer[7] == CHAR_OFF && INPUTSWITCH7 == SWITCH_RELEASED && M7 == ON)
        {
            if(man==1)
            {
            
            SwitchOffStatustToGatway(7);
            TransmissionIndicationLedBlinking();
            OUTPUT_RELAY7=OFF;
            }
            man=0;
            M7=0;
           
        }
 
       

       //off condition
         if(copy_parentalLockBuffer[8] == CHAR_OFF && INPUTSWITCH8 == SWITCH_PRESSED && M8 == OFF)
         {
             if(man==1)
             {
              SwitchOnStatustToGatway(8);  
            TransmissionIndicationLedBlinking();
             OUTPUT_RELAY8=ON;
             }
             man=0;
             M8=1;
            
         }
         //on condtion
         if(copy_parentalLockBuffer[8] == CHAR_OFF && INPUTSWITCH8 == SWITCH_RELEASED && M8 == ON)
         {
             if(man==1)
             {
            
            SwitchOffStatustToGatway(8);
            TransmissionIndicationLedBlinking();
             OUTPUT_RELAY8=OFF;
             }
             man=0;
             M8=0;
           
         }
              //off condition
         if(copy_parentalLockBuffer[9] == CHAR_OFF && INPUTSWITCH9 == SWITCH_PRESSED && M9 == OFF)
         {
             if(man==1)
             {
            
           start_PWM_Generation_For_DIMMER = 0;
           SwitchOnStatustToGatway(10);    
           TransmissionIndicationLedBlinking();
            OUTPUT_DIMMER=OFF;
             }
             man=0;
             M9=1;
            
         }
         //on condtion
         if(copy_parentalLockBuffer[9] == CHAR_OFF && INPUTSWITCH9 == SWITCH_RELEASED && M9 == ON)
         {
             if(man==1)
             {
            start_PWM_Generation_For_DIMMER = 0;
            SwitchOffStatustToGatway(10);
            TransmissionIndicationLedBlinking();
             OUTPUT_DIMMER=ON;
             }
             man=0;
             M9=0;
           
         }
       
 
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
    INPUT_SWITCH_DIR_6 = 1;
    INPUT_SWITCH_DIR_7 = 1;
    INPUT_SWITCH_DIR_8 = 1;
    INPUT_SWITCH_DIR_9 = 1;

   
    
    OUTPUT_RELAY_DIR_1 = 0;
    OUTPUT_RELAY_DIR_2 = 0;
    OUTPUT_RELAY_DIR_3 = 0;
    OUTPUT_RELAY_DIR_4 = 0;
    OUTPUT_RELAY_DIR_5 = 0;
    OUTPUT_RELAY_DIR_6 = 0;
    OUTPUT_RELAY_DIR_7 = 0;
    OUTPUT_RELAY_DIR_8 = 0;
    OUTPUT_DIMMER_DIR = 0;
    
    
    ZCD_CCP10_DIR = 1;
    ZCD_CCP9_DIR = 1;
    ZCD_CCP7_DIR = 1;
    ZCD_CCP8_DIR = 1;
   OUTPUT_TRANMIT_INDICATION_LED_DIR = 0;
    OUTPUT_RECEIVE_INDICATION_LED_DIR = 0;
    USER_RECEIVE_INDICATION_LED_DIR = 0;
    
    
    OUTPUT_RELAY_RED_LED_DIR = 0;
    OUTPUT_RELAY_GREEN_LED_DIR = 0;
    OUTPUT_RELAY_BLUE_LED_DIR = 0;
    // peripherals directions
    
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
    TMR2_Initialize();
    TMR3_Initialize();
    TMR4_Initialize();
    TMR5_Initialize();
    TMR6_Initialize();
    
    TMR8_Initialize();
    TMR10_Initialize();
    CCP9_Initialize();
    CCP8_Initialize();
    CCP7_Initialize();
    CCP10_Initialize();
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
void TMR5_Initialize(void)
{
    //Set the Timer to the options selected in the GUI

    //T5CKPS 1:1; T5OSCEN disabled; nT5SYNC synchronize; TMR5CS FOSC/4; TMR5ON off; 
    T5CON = 0x00;

    //T5GSS T5G; TMR5GE disabled; T5GTM disabled; T5GPOL low; T5GGO_nDONE done; T5GSPM disabled; 
    T5GCON = 0x00;

    //TMR5H 123; 
    TMR5H = 0x00;

    //TMR5L 48; 
    TMR5L = 0x00;

    // Clearing IF flag.
    PIR3bits.TMR5IF = 0;    
    
    // Enabling TMR5 interrupt.
    PIE3bits.TMR5IE = 1;
}

void TMR2_Initialize(void)
{
//     Set TMR2 to the options selected in the User Interface

//     T2CKPS 1:1; T2OUTPS 1:1; TMR2ON off; 
    T2CON = 0x08;
//
//     PR2 39; 
//    PR2 = 0x00;
//
//     TMR2 10; 
    TMR2 = 0x00;

//     Clearing IF flag before enabling the interrupt.
    PIR1bits.TMR2IF = 0;

//     Enabling TMR2 interrupt.
    PIE1bits.TMR2IE = 1;
         GIE = 1;

//     Enables all active peripheral interrupts -----> INTCON reg .... bit 6         page 105
    PEIE = 1;
}


void TMR4_Initialize(void)
{
    // Set TMR2 to the options selected in the User Interface

    // T2CKPS 1:2; T2OUTPS 1:1; TMR2ON off; 
    T4CON = 0x08;

    // PR2 39; 
//    PR2 = 0x00;

    // TMR2 10; 
    TMR4 = 0x00;

    // Clearing IF flag before enabling the interrupt.
    PIR3bits.TMR4IF = 0;

    // Enabling TMR2 interrupt.
    PIE3bits.TMR4IE = 1;
            GIE = 1;

//     Enables all active peripheral interrupts -----> INTCON reg .... bit 6         page 105
    PEIE = 1;
}

void TMR6_Initialize(void)
{
    // Set TMR6 to the options selected in the User Interface

    // T6CKPS 1:2; T6OUTPS 1:1; TMR6ON off; 
    T6CON = 0x08;

    // PR6 39; 
//    PR6 = 0x27;

    // TMR6 0; 
    TMR6 = 0x00;

    // Clearing IF flag before enabling the interrupt.
    PIR3bits.TMR6IF = 0;

    // Enabling TMR6 interrupt.
    PIE3bits.TMR6IE = 1;
}
void TMR8_Initialize(void)
{
    // Set TMR6 to the options selected in the User Interface

    // T6CKPS 1:2; T6OUTPS 1:1; TMR6ON off; 
    T8CON = 0x08;

    // PR6 39; 
//    PR6 = 0x27;

    // TMR6 0; 
    TMR8 = 0x00;

    // Clearing IF flag before enabling the interrupt.
    PIR2bits.TMR8IF = 0;

    // Enabling TMR6 interrupt.
    PIE2bits.TMR8IE = 1;
}
void TMR10_Initialize(void)
{
    // Set TMR6 to the options selected in the User Interface

    // T6CKPS 1:2; T6OUTPS 1:1; TMR6ON off; 
    T10CON = 0x08;

    // PR6 39; 
//    PR6 = 0x27;

    // TMR6 0; 
    TMR10 = 0x00;

    // Clearing IF flag before enabling the interrupt.
    PIR2bits.TMR10IF = 0;

    // Enabling TMR6 interrupt.
    PIE2bits.TMR10IE = 1;
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
void CCP10_Initialize(){
    // Set the CCP1 to the options selected in the User Interface

    // MODE Every edge; EN enabled; FMT right_aligned;
    CCP10CON = 0x84;

    // RH 0;
    CCPR10H = 0x00;

    // RL 0;
    CCPR10L = 0x00;
    
//    CCPTMRS2bits.C9TSEL0=0;
//    CCPTMRS2bits.C9TSEL1=0;

    // Clear the CCP1 interrupt flag
    PIR4bits.CCP10IF = 0;

    // Enable the CCP1 interrupt
    PIE4bits.CCP10IE = 1;
}
void CCP7_Initialize(){
    // Set the CCP1 to the options selected in the User Interface

    // MODE Every edge; EN enabled; FMT right_aligned;
    CCP7CON = 0x84;

    // RH 0;
    CCPR7H = 0x00;

    // RL 0;
    CCPR7L = 0x00;
    
//    CCPTMRS2bits.C9TSEL0=0;
//    CCPTMRS2bits.C9TSEL1=0;

    // Clear the CCP1 interrupt flag
    PIR4bits.CCP7IF = 0;

    // Enable the CCP1 interrupt
    PIE4bits.CCP7IE = 1;
}
void CCP8_Initialize(){
    // Set the CCP1 to the options selected in the User Interface

    // MODE Every edge; EN enabled; FMT right_aligned;
    CCP9CON = 0x84;

    // RH 0;
    CCPR8H = 0x00;

    // RL 0;
    CCPR8L = 0x00;
    
//    CCPTMRS2bits.C9TSEL0=0;
//    CCPTMRS2bits.C9TSEL1=0;

    // Clear the CCP1 interrupt flag
    PIR4bits.CCP8IF = 0;

    // Enable the CCP1 interrupt
    PIE4bits.CCP8IE = 1;
}
void errorsISR(const char* errNum){
   
  	while(*errNum != NULL)
 	{ 
        while (!TX1STAbits.TRMT);
 		TX1REG = *errNum;
 		*errNum++;
        
 	}
}
void errorsMain(const char* errNum){
   
  	while(*errNum != NULL)
 	{ 
        while (!TX1STAbits.TRMT);
 		TX1REG = *errNum;
 		*errNum++;
       
 	}
}

void TransmissionIndicationLedBlinking(){
            OUTPUT_TRANMIT_INDICATION_LED = 0;
            __delay_ms(100);
            OUTPUT_TRANMIT_INDICATION_LED = 1;
}
void ReceivingIndicationLedBlinking(){
                  OUTPUT_RECEIVE_INDICATION_LED = 0;
                __delay_ms(100);
                OUTPUT_RECEIVE_INDICATION_LED = 1;
 
}
void sendAcknowledgment(char *currentStateBuffer){
  int Tx_count=0;
  while(*currentStateBuffer != NULL)
 	{ 
        while (!TX1STAbits.TRMT);
 		TX1REG = *currentStateBuffer;
 		*currentStateBuffer++;
        Tx_count++;
 	}
}
void SwitchOffStatustToGatway(const int SwitchOffNumber)
{
            TX1REG = 'R';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = SwitchOffNumber+'0';__delay_ms(1);
}
void SwitchOnStatustToGatway(const int SwitchOnNumber)
{
            TX1REG = 'R';__delay_ms(1);
            TX1REG = '1';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = SwitchOnNumber+'0';__delay_ms(1);
}
void Send_Acknowlde_To_RedPWM(char charlevelofRedLed_LSB)
{
    levelofRedLed_LSB = charlevelofRedLed_LSB;
    levelofRedLed_MSB = '0';
}
void Send_Acknowlde_To_BluePWM(char charlevelofBlueLed_LSB)
{
    levelofBlueLed_LSB = charlevelofBlueLed_LSB;
    levelofBlueLed_MSB = '0';
}
void Send_Acknowlde_To_GreenPWM(char charlevelofGreenLed_LSB)
{
    levelofBlueLed_LSB = charlevelofGreenLed_LSB;
    levelofBlueLed_MSB = '0';
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
    
    //this is wituout transistor
    //when used transistor use as opposite
    OUTPUT_RELAY1=0;
    OUTPUT_RELAY2=0;
    OUTPUT_RELAY3=0;
    OUTPUT_RELAY4=0;
    OUTPUT_RELAY5=0;
    OUTPUT_RELAY6=0;
    OUTPUT_RELAY7=0;
    OUTPUT_RELAY8=0;
    OUTPUT_TRANMIT_INDICATION_LED = 1;
    OUTPUT_RECEIVE_INDICATION_LED = 1;
    USER_RECEIVE_INDICATION_LED = 1;//off condition--->>>leds are high by default
    
    OUTPUT_FOR_RED_LED=0;
    OUTPUT_FOR_GREEN_LED=0;
    OUTPUT_FOR_BLUE_LED=0;
    OUTPUT_DIMMER = 0;
   
}
int hexadecimalToDecimal(char hexVal[]) ;

void applianceControl(char charSwitchMSB, char charSwitchLSB, char charSwitchSTATE, char chDimmerSpeedMSB, char chDimmerSpeedLSB,
        char charParentalControl, char charFinalFrameState){
    
    //define used variables and initilize it with zero
    int integerSwitchNumber = 0;
    int integerSwitchState = 0;
    int integerSpeed = 0;
    int currentStateBufferPositions=0;
        //**********************//
    	int ConvertStringIntoInt=0;
	float ConvertIntToTimeInMilisec=0;
	unsigned long long int Pulse=0,NeedPulse=0,CompleteClock =65535;
	float deno = 20.0;
	float clockPerCycle=0.25;//microsecinds
	int remainder=0; 
	char HexlevelBuffer[5];
	int start=0;
    int end = strlen(HexlevelBuffer)-1;
    char strH[3],strL[3];
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
    
        // Get speed of Fan or level of dimmer    
    dimmerSpeedStringBuffer[0]=chDimmerSpeedMSB;
    dimmerSpeedStringBuffer[1]=chDimmerSpeedLSB;    
    integerSpeed = atoi(dimmerSpeedStringBuffer);
    integerSpeed = 99-integerSpeed;
    ConvertIntToTimeInMilisec = (integerSpeed/deno);
    ConvertIntToTimeInMilisec = (ConvertIntToTimeInMilisec*1000);//convert into microseconds
    Pulse = (ConvertIntToTimeInMilisec/clockPerCycle);
    NeedPulse = CompleteClock - Pulse;//65535-pulse
    sprintf(HexlevelBuffer,"%X",NeedPulse);
    strncpy(strH,HexlevelBuffer,2);
	strH[2]='\0';
//	printf("Higer Nibble: %s",strH);
	printf("\n");
	strncpy(strL,HexlevelBuffer+2,2);
	strL[2]='\0';
    // save Parental lock state of each switch into parental lock buffer
//    int integerParentalControl=charParentalControl-'0';
    parentalLockBuffer[integerSwitchNumber] = charParentalControl;
    copy_parentalLockBuffer[integerSwitchNumber]=parentalLockBuffer[integerSwitchNumber];
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
        {
            OUTPUT_RELAY1 = integerSwitchState;
        }break;
            
        case 2:
            {
            OUTPUT_RELAY2 = integerSwitchState;

            } break;
        case 3:
        {
            OUTPUT_RELAY3 = integerSwitchState;

        } break;
          
        case 4:
        {
            OUTPUT_RELAY4 = integerSwitchState;
        }break;
        case 5:
        {
            OUTPUT_RELAY5 = integerSwitchState;
        }break;  
        case 6:
        {
            OUTPUT_RELAY6 = integerSwitchState;
        }break;

        case 7:
        {
            OUTPUT_RELAY7 = integerSwitchState;
        }break; 
      
        case 8:
        {
            OUTPUT_RELAY8 = integerSwitchState;
        }break; 
        case 9:{
                start_PWM_Generation_For_DIMMER = integerSwitchState;
               switch(integerSwitchState){
                case 0:
                    OUTPUT_DIMMER=1;  // For Triac --> inverted condition for off
                    break;
                case 1:
                    Timer1H = hexadecimalToDecimal(strH);
                    Timer1L = hexadecimalToDecimal(strL);
                    break;
                default:
                    break;
               }
        }break;  
            default:
            break;
        }
    
}
int hexadecimalToDecimal(char hexVal[]) 
{    
    int len = strlen(hexVal); 
      
    // Initializing base value to 1, i.e 16^0 
    int base = 1; 
      
    int dec_val = 0; 
      
    // Extracting characters as digits from last character 
    for (int i=len-1; i>=0; i--) 
    {    
        // if character lies in '0'-'9', converting  
        // it to integral 0-9 by subtracting 48 from 
        // ASCII value. 
        if (hexVal[i]>='0' && hexVal[i]<='9') 
        { 
            dec_val += (hexVal[i] - 48)*base; 
                  
            // incrementing base by power 
            base = base * 16; 
        } 
  
        // if character lies in 'A'-'F' , converting  
        // it to integral 10 - 15 by subtracting 55  
        // from ASCII value 
        else if (hexVal[i]>='A' && hexVal[i]<='F') 
        { 
            dec_val += (hexVal[i] - 55)*base; 
          
            // incrementing base by power 
            base = base*16; 
        } 
    } 
      
    return dec_val; 
} 
    void applicationControlRGB(char *ModuleNamestr,char *SwitchNumberstr,char *SwitchStatusstr,
                char  *RedColorIntensitystr,char *GreenColorIntensitystr,char *BlueColorIntensitystr,char *ChildLockstr,char *Allcolorstr ){
   

      //  sendAcknowledgment(Allcolorstr);
        
        IntegerSwitchNumber = atoi(SwitchNumberstr);
        IntergerSwitchStatus = atoi(SwitchStatusstr);
        IntergerRedColorIntensity = atoi(RedColorIntensitystr);
        IntegerBlueColorIntensity = atoi(BlueColorIntensitystr);
        IntegerGreenColorIntensity = atoi(GreenColorIntensitystr);
        IntegerAllColorIntensity = atoi(Allcolorstr);
        IntegerChildLock = atoi(ChildLockstr);
       
        
        charchildLock = IntegerChildLock + '0';
        charSwitchState = IntergerSwitchStatus + '0';
        
//while(*ModuleNameString != NULL)//convert string into buffer to concatenate the string>>>Not possible through string
//{
       // sendAcknowledgment(ModuleNamestr);
      //  sendAcknowledgment(SwitchNumberstr);
        
     //  sendAcknowledgment(SwitchStatusstr);
      //  sendAcknowledgment(RedColorIntensitystr);
      //  sendAcknowledgment(GreenColorIntensitystr);
      //  sendAcknowledgment(BlueColorIntensitystr);
    //   sendAcknowledgment(ChildLockstr);//>>issue
        //sendAcknowledgment(Finalframebitstr);
        
        
//    int i=0;
//    for(i=0;i<MODULENAMELENGTH;i++)
//    {
//        ModuleNameBuffer[i]=*ModuleNamestr;
//        *ModuleNamestr++;
//    }
//}
        //frame>>SW.
        strcpy(sendFinalBufferToGAteway,ModuleNamestr);//in firststep we need to ccreate a new buffer so copy is best option rather than concatenation   
        strcat(sendFinalBufferToGAteway,".");//now add delimiter with all frames in ;last
        strcat(sendFinalBufferToGAteway,SwitchNumberstr);
        strcat(sendFinalBufferToGAteway,".");
        strcat(sendFinalBufferToGAteway,"ACTACK");
        strcat(sendFinalBufferToGAteway,".");//ok
        strcat(sendFinalBufferToGAteway,SwitchStatusstr);
        strcat(sendFinalBufferToGAteway,".");
        strcat(sendFinalBufferToGAteway,RedColorIntensitystr);
        strcat(sendFinalBufferToGAteway,".");
        strcat(sendFinalBufferToGAteway,GreenColorIntensitystr);
        strcat(sendFinalBufferToGAteway,".");
        strcat(sendFinalBufferToGAteway,BlueColorIntensitystr);
        strcat(sendFinalBufferToGAteway,".");
        strcat(sendFinalBufferToGAteway,ChildLockstr);
        strcat(sendFinalBufferToGAteway,".");
         strcat(sendFinalBufferToGAteway,Allcolorstr);
     //   strcat(sendFinalBufferToGAteway,".");
//        strcat(sendFinalBufferToGAteway,"|");
        sendAcknowledgment(sendFinalBufferToGAteway);
          
        parentalLockBuffer[IntegerSwitchNumber] = charchildLock;
        
     //   TX1REG = copyparentalLockBuffer[IntegerSwitchNumber];
       //    sendAcknowledgment(sendFinalBufferToGAteway);  
        
        
       // sendAcknowledgment(sendFinalBufferToGAteway);//ok

//while(*SwitchNumberString != NULL)
//{
//int     i=0;
//    for(i=0;i<SWITCHNUMBERLENGTH;i++)
//    {
//        SwitchNumberBuffer[i]=*SwitchNumberstr;
//        *SwitchNumberstr++;
//    }
//}
        //>>frame SW.01.
//        strcat(sendFinalBufferToGAteway,SwitchNumberstr);
//        strcat(sendFinalBufferToGAteway,".");
      //  sendAcknowledgment(sendFinalBufferToGAteway);
//while(*AcknowledgementString != NULL)
//{
//    i=0;
//    for(i=0;i<ACKNOWLEDGEMENTLENGTH;i++)
//    {
//        AcknowledgementBuffer[i]=*AcknowledgementString;
//        *AcknowledgementString++;
//    }
//}
        //frame SW.01.ACTACK
//        strcat(sendFinalBufferToGAteway,"ACTACK");
//        strcat(sendFinalBufferToGAteway,".");//ok
       // sendAcknowledgment(sendFinalBufferToGAteway);//ok
//while(*SwitchStatusString != NULL)
//{
//     int i=0;
//    for(i=0;i<SWITCHSTATUS;i++)
//    {
//        SwitchStatusBuffer[i]=*SwitchStatusstr;
//        *SwitchStatusstr++;
//    }
//}
        //frame SW.01.ACTACK.1
//        strcat(sendFinalBufferToGAteway,SwitchStatusstr);
//        strcat(sendFinalBufferToGAteway,".");
       // sendAcknowledgment(sendFinalBufferToGAteway);//ok
//while(*RedColorIntensityString != NULL)
//{
        
        //***********************RED COLOR STRING************************************// 
        if(IntergerRedColorIntensity == 0)
        {
            RedColorIntensityBuffer[0] = '0';
            RedColorIntensityBuffer[1] = '0';
            //RedColorIntensityBuffer[0] = '0';
           // TX1REG = 'Z';
        }
        else if(IntergerRedColorIntensity == 255)
        {
            RedColorIntensityBuffer[0] = '9';
            RedColorIntensityBuffer[1] = '9';
           // RedColorIntensityBuffer[0] = '3';
          //  TX1REG = 'F';
        }
        
       
        int i=0;
        if(((IntergerRedColorIntensity & GIVES_OUTPUT_ZERO_IF_EVEN) == 1) && IntergerRedColorIntensity != 255 && IntergerRedColorIntensity != 0 )//ckeck for even
        {
            //odd - convert into even by adding one
            IntergerRedColorIntensity++;
           // TX1REG = 'O';
        }
        if(((IntergerRedColorIntensity & GIVES_OUTPUT_ZERO_IF_EVEN) == 0) && IntergerRedColorIntensity != 255 && IntergerRedColorIntensity != 0 )
        {
          //  TX1REG = 'E';
            IntergerRedColorIntensity = (IntergerRedColorIntensity/2);
            IntergerRedColorIntensity = round(IntergerRedColorIntensity*0.78);//to convert 0-123 to 0-99 eg: 123*0.8048 = 98
            itoa(ConvertIntegertoStringRedColorBuffer,IntergerRedColorIntensity,10);   // here 10 means decimal convert integer to string/buffer
           ///>>>>>>This loop is used to send two digit number to ISR.
            // for example, fro speed 30, after calibration it will give value less then 10, that means it ISR will read 90 instead 09, so that will create problem.
            if(IntergerRedColorIntensity > 0 && IntergerRedColorIntensity < 25)
            {
                 Send_Acknowlde_To_RedPWM(ConvertIntegertoStringRedColorBuffer[0]);
            }
            else
            {
            //even 
           for(i=0;i<RedColorIntensityStringLength;i++)
           {
              // TX1REG = ConvertIntegertoStringRedColorBuffer[i];//ok
               __delay_ms(0.5);
               RedColorIntensityBuffer[i]=ConvertIntegertoStringRedColorBuffer[i];
           //    TX1REG = RedColorIntensityBuffer[i];//ok

           }
            }
        }
        
 
  
   //********************GREEN COLOR STRING****************************************??  
        
    if(IntegerGreenColorIntensity == 0)
        {
            GreenColorIntensityBuffer[0] = '0';
            GreenColorIntensityBuffer[1] = '0';
            //RedColorIntensityBuffer[0] = '0';
           // TX1REG = 'Z';
        }
        else if(IntegerGreenColorIntensity == 255)
        {
            GreenColorIntensityBuffer[0] = '9';
            GreenColorIntensityBuffer[1] = '9';
           // RedColorIntensityBuffer[0] = '3';
          
        }
        
        int i=0;
        if(((IntegerGreenColorIntensity & GIVES_OUTPUT_ZERO_IF_EVEN) == 1) && IntegerGreenColorIntensity != 255 && IntegerGreenColorIntensity != 0 )//ckeck for even
        {
            //odd
            IntegerGreenColorIntensity++;
           // TX1REG = 'O';
        }
        if(((IntegerGreenColorIntensity & GIVES_OUTPUT_ZERO_IF_EVEN) == 0) && IntegerGreenColorIntensity != 255 && IntegerGreenColorIntensity != 0 )
        {
          //  TX1REG = 'E';
            IntegerGreenColorIntensity = (IntegerGreenColorIntensity/2);
            IntegerGreenColorIntensity = round(IntegerGreenColorIntensity*0.78);//ok//to convert 0-128 to 0-99 eg: 127*0.78 = 99
            itoa(ConvertIntegertoStringGreenColorBuffer,IntegerGreenColorIntensity,10);   // here 10 means decimal convert integer to string
            if(IntegerGreenColorIntensity > 0 && IntegerGreenColorIntensity < 25)
            {
                Send_Acknowlde_To_GreenPWM(ConvertIntegertoStringGreenColorBuffer[0]);
            }
            else
            {
            //even 
           for(i=0;i<GreenColorIntensityStringLength;i++)
           {
              // TX1REG = ConvertIntegertoStringRedColorBuffer[i];//ok
               __delay_ms(0.5);
               GreenColorIntensityBuffer[i]=ConvertIntegertoStringGreenColorBuffer[i];
 //              TX1REG = GreenColorIntensityBuffer[i];//ok

           }
           }
        }
  ////*******************BLUE COLOR*********************//
        
        if(IntegerBlueColorIntensity == 0)
        {
            BlueColorIntensityBuffer[0] = '0';
            BlueColorIntensityBuffer[1] = '0';
            //RedColorIntensityBuffer[0] = '0';
           // TX1REG = 'Z';
        }
        else if(IntegerBlueColorIntensity == 255)
        {
            BlueColorIntensityBuffer[0] = '9';
            BlueColorIntensityBuffer[1] = '9';
           // RedColorIntensityBuffer[0] = '3';
          //  TX1REG = 'F';
        }
        
 
        
        
        int i=0;
        if(((IntegerBlueColorIntensity & GIVES_OUTPUT_ZERO_IF_EVEN) == 1) && IntegerBlueColorIntensity != 255 && IntegerBlueColorIntensity != 0 )//ckeck for even
        {
            //odd
            IntergerRedColorIntensity++;
           // TX1REG = 'O';
        }
        if(((IntegerBlueColorIntensity & GIVES_OUTPUT_ZERO_IF_EVEN) == 0) && IntegerBlueColorIntensity != 255 && IntegerBlueColorIntensity != 0 )
        {
          //  TX1REG = 'E';
            IntegerBlueColorIntensity = (IntegerBlueColorIntensity/2);
            IntegerBlueColorIntensity = round(IntegerBlueColorIntensity*0.78);//to convert 0-123 to 0-99 eg: 123*0.78 = 98
            itoa(ConvertIntegertoStringBlueColorBuffer,IntegerBlueColorIntensity,10);   // here 10 means decimal convert integer to string
          if(IntegerBlueColorIntensity > 0 && IntegerBlueColorIntensity < 25)
            {
                Send_Acknowlde_To_BluePWM(ConvertIntegertoStringBlueColorBuffer[0]);
            }
          else{
            //even 
           for(i=0;i<BlueColorIntensityStringLength;i++)
           {
              // TX1REG = ConvertIntegertoStringRedColorBuffer[i];//ok
               __delay_ms(0.5);
               
               BlueColorIntensityBuffer[i]=ConvertIntegertoStringBlueColorBuffer[i];
           //    TX1REG = RedColorIntensityBuffer[i];//ok

           }
          }
        }
        ///////////////END*************************
//           for(i=0;i<REDCOLOINTENSITYLENGTH;i++)
//           {
//
//               RedColorIntensityBuffer[i]=*RedColorIntensitystr;
//               *RedColorIntensitystr++;
//
//           }
//}
        //frame SW.01.ACTACK.1.0
//        strcat(sendFinalBufferToGAteway,RedColorIntensitystr);
//        strcat(sendFinalBufferToGAteway,".");
        //sendAcknowledgment(sendFinalBufferToGAteway);//ok
////while(*GreenColorIntensityString != NULL)
////{
//    int i=0;
//    for(i=0;i<GREENCOLORINTENSITYLENGHT;i++)
//    {
//        GreenColorIntensityBuffer[i]=*GreenColorIntensitystr;
//        *GreenColorIntensitystr++;
//    }
//}
        //frame SW.01.ACTACK.1.0.0
//        strcat(sendFinalBufferToGAteway,GreenColorIntensitystr);
//        strcat(sendFinalBufferToGAteway,".");
       // sendAcknowledgment(sendFinalBufferToGAteway);//ok
//while(*BlueColorIntensityBuffer != NULL)
//{
//    int i=0;
//    for(i=0;i<BLUECOLOINTENSITYLENGHT;i++)
//    {
//        BlueColorIntensityBuffer[i]=*BlueColorIntensitystr;
//        *BlueColorIntensitystr++;
//    }
//}
        ////frame SW.01.ACTACK.1.0.0.0
//        strcat(sendFinalBufferToGAteway,BlueColorIntensitystr);
//        strcat(sendFinalBufferToGAteway,".");
      //  sendAcknowledgment(sendFinalBufferToGAteway);//ok
//while(*ChildLockString != NULL)
//{
//   int i=0;
//    for(i=0;i<CHILDLOCKLENGTH;i++)
//    {
//        ChildLockBuffer[i]=*ChildLockstr;
//        *ChildLockstr++;
//    }
//}
         ////frame SW.01.ACTACK.1.0.0.0.0
//        strcat(sendFinalBufferToGAteway,ChildLockstr);
//        strcat(sendFinalBufferToGAteway,".");
   //     sendAcknowledgment(sendFinalBufferToGAteway);
//while(*FinalframebitString != NULL)
//{
//     i=0;
//    for(i=0;i<FINALFRAMEBITLENGTH;i++)
//    {
//        FinalFramebitBuffer[i]=*Finalframebitstr;
//        *Finalframebitstr++;
//    }
//}
        //INPUT FRAME >> SW/,.01.ACT.1.0.0.0.0.1.|
        //>>Final frame be like>>>>SW/RGB.01.ACTACK .1.0.0.0.0.1.|
        
//        strcat(sendFinalBufferToGAteway,Finalframebitstr);
//        strcat(sendFinalBufferToGAteway,".");
//        strcat(sendFinalBufferToGAteway,"|");
       //sendAcknowledgment(sendFinalBufferToGAteway);
        
        
        switch(IntegerSwitchNumber)
        {
            

            case 1:
            
                start_PWM_Generation_For_BlueLed = start_PWM_Generation_For_GreenLed = start_PWM_Generation_For_RedLed = IntergerSwitchStatus;
                switch(IntergerSwitchStatus){
                    case 0:
                        
                        OUTPUT_FOR_RED_LED = OFF;
                        OUTPUT_FOR_GREEN_LED = OFF;
                        OUTPUT_FOR_BLUE_LED = OFF;
                        break;
                    case 1:
                       
                      levelofRedLed_MSB =  RedColorIntensityBuffer[0];__delay_ms(0.5);
                      levelofRedLed_LSB =  RedColorIntensityBuffer[1];__delay_ms(0.5);
                       levelofGreenLed_MSB =  GreenColorIntensityBuffer[0];__delay_ms(0.5);
                      levelofGreenLed_LSB =  GreenColorIntensityBuffer[1];__delay_ms(0.5);
                      levelofBlueLed_LSB =  BlueColorIntensityBuffer[0];__delay_ms(0.5);
                      levelofBlueLed_MSB =  BlueColorIntensityBuffer[1];__delay_ms(0.5);                      
//                     TX1REG = levelofRedLed_LSB;__delay_ms(0.5);
//                      TX1REG = levelofRedLed_MSB;__delay_ms(0.5);
//                      TX1REG = levelofGreenLed_LSB;__delay_ms(0.5);
//                      TX1REG = levelofGreenLed_MSB;__delay_ms(0.5); 
//                      TX1REG = levelofBlueLed_LSB;__delay_ms(0.5);
//                      TX1REG = levelofBlueLed_MSB;__delay_ms(0.5);
                      
                      break;
                    default:
                        break;
                        
                }

//                switch(IntergerSwitchStatus){
//                    case 0:
//                        
//                        OUTPUT_FOR_GREEN_LED = OFF;
//                        break;
//                    case 1:
//                      
//                      levelofGreenLed_MSB =  GreenColorIntensityBuffer[0];__delay_ms(0.5);
//                      levelofGreenLed_LSB =  GreenColorIntensityBuffer[1];__delay_ms(0.5);
////                    TX1REG = levelofGreenLed_LSB;__delay_ms(0.5);
////                      TX1REG = levelofGreenLed_MID;__delay_ms(0.5);
// //                   TX1REG = levelofGreenLed_MSB;__delay_ms(0.5);
//                      break;
//                    default:
//                        break;
//                        
//                }
//
//                switch(IntergerSwitchStatus){
//                    case 0:
//                     
//                        OUTPUT_FOR_BLUE_LED = OFF;
//                        break;
//                    case 1:
//                       
//                      levelofBlueLed_LSB =  BlueColorIntensityBuffer[0];__delay_ms(0.5);
//                      levelofBlueLed_MSB =  BlueColorIntensityBuffer[1];__delay_ms(0.5);
////                      TX1REG = levelofBlueLed_LSB;__delay_ms(0.5);
////                      TX1REG = levelofBlueLed_MID;__delay_ms(0.5);
////                      TX1REG = levelofBlueLed_MSB;__delay_ms(0.5);
//                      break;
//                    default:
//                        break;
//                        
//                }
//
//            break;
//            default:
//                break;
//            
//        }
//        
        
    }
}
#include<xc.h>
#define __XTAL_FREQ 16000000
#include "applicationcontrol.h"
#include "applicationcontrolRGB.h"
