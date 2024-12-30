/*=============================================================================
	File Name:	COMP3099Group9LabAssesment.c  
	Author:		Sachin Raj Charupadikkal
	Date:		29/11/2024
	Modified:	None
	© Fanshawe College, 2024

	Description: Configuring and controlling the PIC18F45K22 microcontroller using MPLAB
			by setting frequency to 4MHZ, configuring ports, Sampling an Analog channel input
			for potentiometer connected in AN0. Read the value and convert into temperature 
			along with switch and LED status which is transmited to Rapsberry pi 
			using USART1.
=============================================================================*/

/* Preprocessor ===============================================================
   Hardware Configuration Bits ==============================================*/
#pragma config FOSC		= INTIO67
#pragma config PLLCFG	= OFF
#pragma config PRICLKEN = ON
#pragma config FCMEN	= OFF
#pragma config IESO		= OFF
#pragma config PWRTEN	= OFF 
#pragma config BOREN	= ON
#pragma config BORV		= 285 
#pragma config WDTEN	= OFF
#pragma config PBADEN	= OFF
#pragma config LVP		= OFF
#pragma config MCLRE	= EXTMCLR

// Libraries ==================================================================
#include <p18f45k22.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <delays.h>

// Constants =================================================================
#define TRUE	1	
#define FALSE	0	

#define MAXIMUM_ADC 1023
#define ADCCHANNEL	0
#define VREF    3.2
#define TEMPERATURE_MAX 70.0
#define TEMPERATURE_MIN 20.0
#define TOKENSIZE		15
#define BUFFER_SIZE 64
#define SYSTEMLED	LATCbits.LATC2
#define T0FLAG	INTCONbits.TMR0IF

// Global Variables ==========================================================
int heatLED = 0;
int fanLED = 0;
char heatStatus[4] = "OFF";
char fanStatus[4] = "OFF";
char rcBuf[BUFFER_SIZE];  // Buffer to store Pi data recieved
unsigned char insert = 0;     // Index for data insertion
char sentenceReady = FALSE;
char PIC[6]="PIC";
char ON[6]="ON";
char OFF[6]="OFF";
char *tokens[TOKENSIZE];

void isr();
// Interrupt Vector  ==========================================================

#pragma code interruptVector = 0x0008
void interruptVector()
{
    _asm
    GOTO isr
	_endasm    
}
#pragma code  // save following in normal code space


// Functions =================================================================

//Function for setting 4MHZ internal oscillating frequency.
void config4MHZ(void)
{
	OSCCON = 0x52; // setting the OSCCON register to 4MHZ
	while(!OSCCONbits.HFIOFS); // wait till the HFIOFS bit of OSCCON register to stabilize
} // eo config4MHZ::

//Function for setting ports.
void configPorts(void)
{
	ANSELA = 0x01;
	LATA = 0x00;
	TRISA = 0xFF;	// low bits and high bits set as input

	ANSELC = 0x00;
	LATC = 0x00;
	TRISC = 0xC8;
} // eo configPorts::

//Function for setting USART1 serial port configuration.
void configSerialPort()
{
	BAUDCON1 = 0x48;
	TXSTA1 = 0x22;
	RCSTA1 = 0x90;
	SPBRG1 = 12;
	SPBRGH1 = 0x00;
} // eo configSerialPort::

//Function for enabling ADC registers.
void configADC(void)
{
	ADCON0 = 0x01;	// Enable ADC in ADCON0 register
	ADCON1 = 0x00;
	ADCON2 = 0xA9;	// 12 Tad, Fosc/8, Right justified
} // eo configADC::

//Function to conver the ADC value into temperature ranges from -10 to 50
float TempConvert(int valueADC) {
    float Voltage = ((float)valueADC / MAXIMUM_ADC) * VREF;
    return (((Voltage / VREF) * (TEMPERATURE_MAX - TEMPERATURE_MIN)) + TEMPERATURE_MIN);
}//eo TempConvert::

//Interrupt function
void configINTS(void)
{
	IPR1bits.RC1IP = FALSE;	//USART 1 Receiver Interrupt priority disabled
	PIR1bits.RC1IF = FALSE;	//USART 1 Receiver Interrupt flag reset 
	PIE1bits.RC1IE = TRUE;	//USART 1 Receiver Interrupt enabled
    INTCONbits.GIE = TRUE;       // Global Interrupt Enable
    INTCONbits.PEIE = TRUE;      // Peripheral Interrupt Enable
} // eo configINTS:: 


//calling all functions defined inside a single function
void systemInitialization(void)
{
	config4MHZ();
	configPorts();
	configSerialPort();
	configADC();;
	configINTS();
} // eo systemInitialization::

//Function for getting the ADC value
int getSampleADC(char chan)
{
	ADCON0bits.CHS = chan;
	ADCON0bits.GO = TRUE;
	while(ADCON0bits.GO);
	return ADRES;
} // eo getSampleADC::
	
//Function to calculate the checksum of the created String
unsigned char calculateChecksum(char *message)
{
    unsigned char  checksum = 0;
    while(*message && *message != '*')
    {
        checksum ^= *message++;
    }
    return checksum;
}//eo calculateChecksum::

//function to validate checksum received
int validateCheckSum(char *ptr){
    char checkSumReady = 0;
    char netCheckSum=0, receivedCheckSum=0, length;
	int result;
    char ncsStr[10],rcsStr[10];  // Buffer to hold the checksum string
    length = strlen(ptr);
    while(!checkSumReady)
	{
        if(*(ptr+length) == '*')
		{
            *(ptr+length) = '\0';
            receivedCheckSum = atoi(ptr+length+1);
            checkSumReady =1;
        }
        length--;
    }
	    netCheckSum = calculateChecksum(ptr+1);
	    return(netCheckSum == receivedCheckSum);

}// eo validateCheckSum::

// Parse Sentence into Tokens
void parseSent(char *ptr)
{
    char tokenCount=0;
    while(*ptr && tokenCount < TOKENSIZE){
        if(*ptr == '$' || *ptr == ','|| *ptr == '*')
		{
            *ptr='\0';
            tokens[tokenCount]=ptr+1;
            tokenCount++;
        }
        ptr++;
    }
}

void exeSent()
{
    if (strcmp(PIC,tokens[0])==0) // Check if sentence starts with $PIC
    {
		if (strcmp(ON,tokens[2])==0) // Check if sentence starts with $PIC
	    {
			// HEAT LED ON
			LATCbits.LATC0 = 1;
			sprintf(heatStatus,"ON");
		}
		else
		{
			LATCbits.LATC0 = 0;
			sprintf(heatStatus,"OFF");
		}
		if (strcmp(ON,tokens[4])==0) // Check if sentence starts with $PIC
	    {
			LATCbits.LATC1 = 1;
			sprintf(fanStatus,"ON");
		}
		else
		{
			LATCbits.LATC1 = 0;
			sprintf(fanStatus,"OFF");
		}		
    }
}

// Interrupt Service Routine
#pragma interrupt isr
void isr(void)
{
    if (PIR1bits.RC1IF)  
    {
        char receivedChar = RCREG1;
        // Reset buffer index if new data starts
        if (receivedChar == '$')
        {
            insert = 0;
        }
        // Store character in buffer
        if (insert < BUFFER_SIZE - 1) // Prevent overflow
        {
            rcBuf[insert++] = receivedChar;
        }
        // Check for end of sentence
        if (receivedChar == '\n')
        {
            rcBuf[insert] = '\0';   // Null-terminate the buffer
            sentenceReady = TRUE;  // Set flag for processing in main
        }
    }
}


void main(void)
{
	int valueADC1 = 0;
	float temp = 0;	
	char delayCounter,dataBuffer[50];  // Buffer for data string
	unsigned char checksum;
	SYSTEMLED = 1;
	systemInitialization();
	delayCounter=0;
	while(1) 
	{
		if(delayCounter>=50)
		{	
			delayCounter=0;
	        valueADC1 = getSampleADC(ADCCHANNEL);  // Read from AN0
	        // Scale ADC values to temperature
	        temp = TempConvert(valueADC1);   // 20°C to 70°C
			
	        // Read switch state (active low on RD0)
	        if(PORTCbits.RC3 == 0) 
			{
				sprintf(dataBuffer, "$TEMP,%d,SWITCH:ON,HEAT,%s,FAN,%s*", (int)temp,heatStatus,fanStatus);
		        checksum = calculateChecksum(dataBuffer+1); // Exclude the '$' for checksum calculation
		        
		        // Append checksum to the message
		        sprintf(dataBuffer + strlen(dataBuffer), "%02X", checksum);
				//strcat(dataBuffer, "\r\n");
		
		        printf("\n\r%s", dataBuffer);
		        //Delay10KTCYx(100); // Delay for 1 sec
			}
			else
			{
				// Format the message
				sprintf(dataBuffer, "$TEMP,%d,SWITCH:OFF,HEAT,%s,FAN,%s*", (int)temp,heatStatus,fanStatus);
		        checksum = calculateChecksum(dataBuffer + 1); // Exclude the '$' for checksum calculation
		        
		        // Append checksum to the message
		        sprintf(dataBuffer + strlen(dataBuffer), "%02X", checksum);
				strcat(dataBuffer, "\r\n");
		        printf("\n\r%s", dataBuffer);
		        //Delay10KTCYx(100); // Delay for 1 sec
			}
			if (sentenceReady) 
			{
			    sentenceReady = FALSE;
				if(validateCheckSum(rcBuf))
				{
			        parseSent(rcBuf);
			        exeSent();
				}
		    }
		}	
		Delay10KTCYx(20);
		delayCounter++;
		SYSTEMLED = !SYSTEMLED;	
	}
} // eo main::
