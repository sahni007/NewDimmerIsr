/*
 * File:   main.c
 * Author: Alfaone
 *This is proper working code  code for 3 dimmer_2 switches with new ISR
 * Created on 8 February, 2019, 1:36 PM
 */


#include <stdio.h>
#include <stdlib.h>
#include<string.h>
#include<math.h>
#include<float.h>
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
#define OUTPUT_RELAY1 PORTFbits.RF0
#define OUTPUT_RELAY2 PORTFbits.RF1
#define OUTPUT_DIMMER1 PORTAbits.RA2 // PWM OUTPUT to MOC3021
#define OUTPUT_DIMMER2 PORTAbits.RA3 // PWM OUTPUT to MOC3021
#define OUTPUT_DIMMER3 PORTEbits.RE5   // PWM OUTPUT to MOC3021

#define INPUTSWITCH5 PORTFbits.RF7
#define INPUTSWITCH4 PORTFbits.RF5
#define INPUTSWITCH3 PORTFbits.RF3
#define INPUTSWITCH2 PORTFbits.RF2
#define INPUTSWITCH1 PORTAbits.RA5

#define INPUT_SWITCH_DIR_5 TRISFbits.TRISF7
#define INPUT_SWITCH_DIR_4 TRISFbits.TRISF5
#define INPUT_SWITCH_DIR_3 TRISFbits.TRISF3
#define INPUT_SWITCH_DIR_2 TRISFbits.TRISF2
#define INPUT_SWITCH_DIR_1 TRISAbits.TRISA5

#define OUTPUT_RELAY_DIR_1 TRISFbits.TRISF0
#define OUTPUT_RELAY_DIR_2 TRISFbits.TRISF1
#define OUTPUT_DIMMER_DIR_1 TRISAbits.TRISA2
#define OUTPUT_DIMMER_DIR_2 TRISAbits.TRISA3
#define OUTPUT_DIMMER_DIR_3 TRISEbits.TRISE5        // direction of PWM OUTPUT to MOC3021

/*
 * Extra Periferals Direction and PORT
 */
#define ZCD_CCP9_DIR TRISEbits.TRISE3
#define ZCD_CCP3_DIR  TRISGbits.TRISG0
#define ZCD_CCP1_DIR TRISCbits.TRISC2
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

int start_PWM_Generation_in_ISR_FLAG_DIMMER1=FALSE;
int start_PWM_Generation_in_ISR_FLAG_DIMMER2=FALSE;
int start_PWM_Generation_in_ISR_FLAG_DIMMER3=FALSE;
char levelofDimmer_MSB='0',levelofDimmer_LSB='0';
int Timer1H=0,Timer1L=0;
int Timer3H=0,Timer3L=0;
int Timer5H=0,Timer5L=0;
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
void TMR2_Initialize();
void TMR4_Initialize();
void TMR5_Initialize();
void TMR6_Initialize();
void CCP9_Initialize();
void CCP1_Initialize();
void CCP3_Initialize();
void allPeripheralInit();

void copyReceivedDataBuffer();

void applianceControl(char switchMSB, char switchLSB, char switchSTATE, char dimmerSpeedMSB, char dimmerSpeedLSB, char parentalControl, char finalFrameState);

int hexadecimalToDecimal(char hexVal[]) ;



 interrupt void isr() {
   //*******************************DIMMER 11111 *************************************
    
    if(PIE1bits.TMR2IE==1 && PIR1bits.TMR2IF==1)
    {        

        while(TX1REG==0);
        PIR1bits.TMR2IF=0;
        OUTPUT_DIMMER1=ON;
        T2CONbits.TMR2ON=0;
    } 
    
     if(PIE1bits.TMR1IE == 1 && PIR1bits.TMR1IF==1)
    {

        PIR1bits.TMR1IF=0;
        T1CONbits.TMR1ON = 0;        
        OUTPUT_DIMMER1=OFF;
        PR2=0x9F;
        T2CONbits.TMR2ON=1;
               
    }

    //*******************************DIMMER 22222 *************************************
    
    if(PIE3bits.TMR4IE==1 && PIR3bits.TMR4IF==1)
    {           

        PIR3bits.TMR4IF=0;
        OUTPUT_DIMMER2=ON;
        T4CONbits.TMR4ON=0;

    }
    
    if(PIE3bits.TMR3IE == 1 && PIR3bits.TMR3IF==1)
    {

        PIR3bits.TMR3IF=0;
        
        OUTPUT_DIMMER2=OFF;
        PR4=0x9F;
        T4CONbits.TMR4ON=1;
        T3CONbits.TMR3ON = 0;        
    }

    //*******************************DIMMER 33333 *************************************    
    
    if(PIE3bits.TMR6IE == 1 && PIR3bits.TMR6IF == 1)
    {           
        PIR3bits.TMR6IF=0;
        OUTPUT_DIMMER3=ON;
        T6CONbits.TMR6ON=0;
    } 
    
    if(PIE3bits.TMR5IE == 1 && PIR3bits.TMR5IF==1)
    {
         PIR3bits.TMR5IF=0;        
        OUTPUT_DIMMER3=OFF;
        PR6=0x9F;
        T6CONbits.TMR6ON=1;
        T5CONbits.TMR5ON=0;        
    }
    //*************************ZCD INTERRRUPT****************************//
     if(PIR1bits.CCP1IF==1 || PIR3bits.CCP3IF == 1 || PIR4bits.CCP9IF==1){
    if(CCP9IF){
        if(CCP9IF == 1){
             CCP9IF=0;
         if(start_PWM_Generation_in_ISR_FLAG_DIMMER1 == 1){
                                    TMR1H = Timer1H;
                                    TMR1L = Timer1L;
                                    T1CONbits.TMR1ON = 1;
                                                 }
                        }
       
                }
        if(CCP3IF){
        if(CCP3IF == 1){
             CCP3IF=0;
         if(start_PWM_Generation_in_ISR_FLAG_DIMMER2 == 1){
                                    TMR3H = Timer3H;
                                    TMR3L = Timer3L;
                                    T3CONbits.TMR3ON = 1;
                                                 }
                        }
       
                }
        if(CCP1IF){
        if(CCP1IF == 1){
             CCP1IF=0;
         if(start_PWM_Generation_in_ISR_FLAG_DIMMER3 == 1){
                                    TMR5H = Timer5H;
                                    TMR5L = Timer5L;
                                    T5CONbits.TMR5ON = 1;
                                                             }
                        }
       
                }
     }//end of ccp   
    
    
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


int main() {
 
        M1=ON;    M2=ON;     M3=ON;    M4=ON;     M5=ON;

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
                start_PWM_Generation_in_ISR_FLAG_DIMMER1 = 0;
            __delay_ms(5);
            TX1REG = 'R';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '3';__delay_ms(1);
            OUTPUT_DIMMER1=ON;
            }
            man=0;
            M3=1;
          
        }
        //on condtion
        if(parentalLockBuffer[3] == CHAR_OFF && INPUTSWITCH3 == ON && M3 == ON)
        {
            if(man==1)
            {
                start_PWM_Generation_in_ISR_FLAG_DIMMER1 = 0;
            __delay_ms(5);
            TX1REG = 'R';__delay_ms(1);
            TX1REG = '1';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '3';__delay_ms(1);
            OUTPUT_DIMMER1=OFF;
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
                start_PWM_Generation_in_ISR_FLAG_DIMMER2 = 0;
            __delay_ms(5);
            TX1REG = 'R';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '4';__delay_ms(1);
            OUTPUT_DIMMER2=ON;
            }
            man=0;
            M4=1;
            
        }
        //on condtion
        if(parentalLockBuffer[4] == CHAR_OFF && INPUTSWITCH4 == ON && M4 == ON)
        {
            if(man==1)
            {
                start_PWM_Generation_in_ISR_FLAG_DIMMER2 = 0;
            __delay_ms(5);
            TX1REG = 'R';__delay_ms(1);
            TX1REG = '1';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '4';__delay_ms(1);
            OUTPUT_DIMMER2=OFF;
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
            start_PWM_Generation_in_ISR_FLAG_DIMMER3 = 0;
            __delay_ms(5);
            TX1REG = 'R';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '5';__delay_ms(1);
            OUTPUT_DIMMER3=ON;
            }
            man=0;
            M5=1;
           
        }
        //on condtion
        if(parentalLockBuffer[5] == CHAR_OFF && INPUTSWITCH5 == ON && M5 == ON)
        {
            if(man==1)
            {
            start_PWM_Generation_in_ISR_FLAG_DIMMER3 = 0;
            __delay_ms(5);
            TX1REG = 'R';__delay_ms(1);
            TX1REG = '1';__delay_ms(1);
            TX1REG = '0';__delay_ms(1);
            TX1REG = '5';__delay_ms(1);
            OUTPUT_DIMMER3=OFF;
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
    
    
    
    
    
    
    switchNumberStringBuffer[0]=charSwitchMSB;
    switchNumberStringBuffer[1]=charSwitchLSB;    
    integerSwitchNumber = atoi(switchNumberStringBuffer);//convert string into integer
    
    // Get switch State in Integer Format
    
    integerSwitchState = charSwitchSTATE-'0';
    
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
// 	TimerH = hexadecimalToDecimal(strH);
// 	TimerL = hexadecimalToDecimal(strL);
//printf("Lower Nibble: %s",strL);
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
        case 3:{
                start_PWM_Generation_in_ISR_FLAG_DIMMER1 = integerSwitchState;
               switch(integerSwitchState){
                case 0:
                    OUTPUT_DIMMER1=1;  // For Triac --> inverted condition for off
                    break;
                case 1:
                	Timer1H = hexadecimalToDecimal(strH);
                	Timer1L = hexadecimalToDecimal(strL);
                    break;
                default:
                    break;
               }
        }break;//end of case 3
        case 4:{
                start_PWM_Generation_in_ISR_FLAG_DIMMER2 = integerSwitchState;
               switch(integerSwitchState){
                case 0:
                    OUTPUT_DIMMER2=1;  // For Triac --> inverted condition for off
                    break;
                case 1:
                	Timer3H = hexadecimalToDecimal(strH);
                	Timer3L = hexadecimalToDecimal(strL);
                    break;
                default:
                    break;
               }
        }break;//end of case 5
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
                start_PWM_Generation_in_ISR_FLAG_DIMMER3 = integerSwitchState;
               switch(integerSwitchState){
                case 0:
                    OUTPUT_DIMMER3=1;  // For Triac --> inverted condition for off
                    break;
                case 1:
                	Timer5H = hexadecimalToDecimal(strH);
                	Timer5L = hexadecimalToDecimal(strL);
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
    OUTPUT_DIMMER_DIR_1 = 0;
    OUTPUT_DIMMER_DIR_2 = 0;
    OUTPUT_DIMMER_DIR_3 = 0; 
    
    // peripherals directions
    ZCD_CCP9_DIR = 1;
    ZCD_CCP1_DIR =1;
    ZCD_CCP3_DIR =1;
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
    TMR2_Initialize();
    TMR4_Initialize();
    TMR6_Initialize();
    TMR5_Initialize();
    CCP9_Initialize();
    CCP3_Initialize();
    CCP1_Initialize();
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
void CCP1_Initialize()
{
    // Set the CCP1 to the options selected in the User Interface

    // MODE Every edge; EN enabled; FMT right_aligned;
  //  CCP1CON = 0x05;//raising edge
    CCP1CON = 0x04;//faling edge
   //   CCP1CON = 0x06;//4th rISING edge
 //   CCP1CON = 0x84;

    // RH 0;
    CCPR1H = 0x00;

    // RL 0;
    CCPR1L = 0x00;

    // Clear the CCP1 interrupt flag
    PIR1bits.CCP1IF = 0;

    // Enable the CCP1 interrupt
    PIE1bits.CCP1IE = 1;
   GIE = 1;

    // Enables all active peripheral interrupts -----> INTCON reg .... bit 6         page 105
    PEIE = 1;
}

void CCP3_Initialize(void)
{
    // Set the CCP3 to the options selected in the User Interface

    // MODE Every edge; EN enabled; FMT right_aligned;
    CCP3CON = 0x84;    

    // CCPR3L 0; 
    CCPR3L = 0x00;    

    // CCPR3H 0; 
    CCPR3H = 0x00;    
    
    // Selecting Timer 3
//    CCPTMRS0bits.C3TSEL = 0x1;

    // Clear the CCP3 interrupt flag
    PIR3bits.CCP3IF = 0;

    // Enable the CCP3 interrupt
    PIE3bits.CCP3IE = 0;
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
  while(*currentStateBuffer != NULL)
 	{ 
        while (!TX1STAbits.TRMT);

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
    OUTPUT_DIMMER1=1;
    OUTPUT_DIMMER2=1;
    OUTPUT_DIMMER3=1;
}
