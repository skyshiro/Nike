#include <msp430.h> 
#include "ds18x20.h"
#include <math.h>

#define MIC0 BIT0
#define MIC1 BIT1
#define MIC2 BIT2
#define MIC3 BIT3
#define MIC4 BIT4

#define MIC_CAL 20
#define ADC_THRES 900
#define WIN_TIME 5
#define SAMPLE_AVG_COUNT 5 		//number of samples to avg
#define SAMPLE_AVG_COUNT_CALC 4

#define ARRAY_LENGTH 7000.00	//length between mic in TARs
#define ARRAY_LENGTH_ACTUAL 0.3 //length between mic in meters
#define ARRAY_LENGTH_ACTUAL_MM 300 //length between mic in millimeters

#define ACOS_COUNT 45

/* Timer runs at 8MHz to avoid overflow. Timer is an integer from 0->2^16 while mic_time is a float from 0->2^15
 *
 * P1.1 through P1.5 are the microphone inputs MIC0 through MIC4
 *
 * P1.0 will be the reference voltage
 *
 * ADC value of about 1020 will be rail
 *
 * CONSEQ_3 will convert selected channels continuously
 *
 * ADC10ON = 1
 *
 * set INCH_1 so ADC starts at channel 1 and goes to channel 0
 *
 * ENC = 1
 * ADC10SC = 1
 *
 * Takes 5 samples from each microphone then does radius/bearing calculation and averages the results
 *
 * Radius is calculated as an integer in mm and the bearing is calculated as float in degrees
 *
 *
 */
unsigned int virgin_flag;
unsigned int convert_flag;									//determines if ADC has converted value
unsigned int MIC0_sample_count = 0;
unsigned int MIC1_sample_count = 0;
unsigned int MIC2_sample_count = 0;

volatile int mic_time[5][SAMPLE_AVG_COUNT];					//2D array, first dimension holds timing values from mic and second is for the multiple samples int mic_adc[5];
unsigned int mic_check;										//BITN in mic_check is for MICN. Determines which MIC has a time value
unsigned int mic_use;										//vector that determines mic being used

volatile float B_horz;
int B_horz_angle;
volatile float R_horz;

int acos_table[100] = {180,169,164,160,157,154,152,149,147,145,143,141,139,138,136,134,133,131,130,128,127,125,124,123,121,120,119,117,116,115,114,112,111,110,109,107,106,105,104,103,102,100,99,98,97,96,95,93,92,91,90,89,88,87,85,84,83,82,81,80,78,77,76,75,74,73,71,70,69,68,66,65,64,63,61,60,59,57,56,55,53,52,50,49,47,46,44,42,41,39,37,35,33,31,28,26,23,20,16,11};

volatile float temperature_val;

int main(void) {

	unsigned int i;
	volatile int CD10[SAMPLE_AVG_COUNT], CD21[SAMPLE_AVG_COUNT];	//holy shit memory usage Batman!
	volatile int CD10_avg = 0, CD21_avg = 0;

    WDTCTL = WDTPW | WDTHOLD;						// Stop watchdog timer
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;
	_enable_interrupts();

    

	InitDS18B20();

	temperature_val = GetData()*9/5+32;

    while(1);

	return 0;
}


