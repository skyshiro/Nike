#include <msp430.h> 
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

#define ARRAY_LENGTH .3			//length between mic in meters

/*
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
 */

int virgin_flag;
int convert_flag;									//determines if ADC has converted value
int MIC0_virgin_flag;
int MIC1_viring_flag;
int MIC2_virgin_flag;

int mic_time[5][SAMPLE_AVG_COUNT];									//2d array, first dimension holds timing values from mic and second is for the multiple samples
int mic_adc[5];
int mic_check;										//BITN in mic_check is for MICN. Determines which MIC has a time value
int mic_use;										//vector that determines mic being used

int mic_adc[5];

float CD10[SAMPLE_AVG_COUNT], CD21[SAMPLE_AVG_COUNT],R_horz[SAMPLE_AVG_COUNT],B_horz[SAMPLE_AVG_COUNT];	//holy shit memory usage Batman!
float R_horz_avg,B_horz_avg;

int main(void) {
	int i;

    WDTCTL = WDTPW | WDTHOLD;						// Stop watchdog timer
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;
	_enable_interrupts();

    virgin_flag = 1;
    convert_flag = 1;

    mic_check = 0x00;

    ADC10CTL1 |= ADC10SSEL_2;
    ADC10CTL0 |= ADC10IE /* + ADC10SHT_2 */;
    ADC10CTL0 |= ADC10ON;								//turn on the ADC and interrupts for it
    //sweep microphone input

    //for 5 mic setup
    //while(!(mic_check & 0x1F))						//when all bits in mic_check vector are set, exit while loop to continue

    //for 2 mic setup
    while(mic_check != 0x7FFF)							//every 5 bits is is dedicated to 5 samples from each mic. 0x755 = 15 bits
    {
    	if(~(mic_check & 0x1F))							//0x1F is 5 bits, when all 5 samples have been taken mic_check4:0 will be high
    	{
    		mic_use = 0;
    		ADC10CTL0 |= ADC10SC+ENC;
    		while(convert_flag);
    		convert_flag = 1;
    	}
    	if(~(mic_check & 0x3E0))
		{
    		mic_use = 1;
    		ADC10CTL1 |= INCH_1;
			ADC10CTL0 |= ADC10SC+ENC;;
			while(convert_flag);
			convert_flag = 1;
			ADC10CTL1 &= ~INCH_1;
		}
    	if(~(mic_check & 0x7C00))
		{
			mic_use = 2;
			ADC10CTL1 |= INCH_2;
			ADC10CTL0 |= ADC10SC+ENC;;
			while(convert_flag);
			convert_flag = 1;
			ADC10CTL1 &= ~INCH_2;
		}

    }

    ADC10CTL0 &= ~ADC10ON;										//turn off ADC when done filling with values

    R_horz_avg = 0.0;
    B_horz_avg = 0.0;

    //INSERT LOOP TO CALCULATE RANGE AND BEARING FOR 5 SAMPLES
    for(i=0; i<SAMPLE_AVG_COUNT; i++)
    {
        CD10[i] = (mic_time[1][i]-mic_time[0][i])/4000.0*343/4000.0;
        CD21[i] = (mic_time[2][i]-mic_time[1][i])/4000.0*343/4000.0;

        R_horz[i] = ARRAY_LENGTH * ( 1 - ( (CD10[i] / ARRAY_LENGTH) * (CD10[i] / ARRAY_LENGTH)) ) + ARRAY_LENGTH * ( 1 - ( (CD21 / ARRAY_LENGTH ) * (CD21[i] / ARRAY_LENGTH) ));
        R_horz[i] = R_horz / ( 2 * ( ( CD21[i] / ARRAY_LENGTH ) - ( CD10[i] / ARRAY_LENGTH ) ) );
        //R_horz = R_horz * 100;	//convert to cm

        B_horz = acos(( ARRAY_LENGTH*ARRAY_LENGTH - 2*R_horz[i]*CD21 - CD21*CD21) / ( (2*R_horz[i]*ARRAY_LENGTH) ))*180/3.14159;

        R_horz_avg += R_horz[i];
        B_horz_ang += B_horz[i];
    }

    R_horz_avg = R_horz_avg / SAMPLE_AVG_COUNT;
    B_horz_ang = B_horz_ang / SAMPLE_AVG_COUNT;

    while(1);

	return 0;
}

//ISR for ADC
#pragma vector=ADC10_VECTOR
__interrupt void adc_isr (void)
{
	convert_flag = 0;

	ADC10CTL0 &= ~ENC;									//turn off ENC bit so the input channel can be changed

	mic_adc[mic_use] = ADC10MEM;

	if(ADC10MEM > ADC_THRES)									//check is ADC value is close to rail
	{
		mic_check |= 1 << mic_use;

		if(virgin_flag)
		{
			//turn on the timer
			TACTL = TASSEL_2 + MC_2;                  // SMCLK, continous mode @ 16MHz

			virgin_flag = 0;
			mic_time[mic_use] = 0;
		}
		else
		{
			//mic_time[mic_use] = TAR-MIC_CAL;		//the value of the next microphone is the value of TAR
			mic_time[mic_use] = TAR;
		}
	}
}
