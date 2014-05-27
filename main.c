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
#define SAMPLE_AVG_COUNT_CALC 4

#define ARRAY_LENGTH 7000.00	//length between mic in TARs
#define ARRAY_LENGTH_ACTUAL 0.3 //length between mic in meters

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
 */

unsigned int virgin_flag;
unsigned int convert_flag;									//determines if ADC has converted value
unsigned int MIC0_sample_count = 0;
unsigned int MIC1_sample_count = 0;
unsigned int MIC2_sample_count = 0;

int mic_time[5][SAMPLE_AVG_COUNT];					//2D array, first dimension holds timing values from mic and second is for the multiple samples
int mic_adc[5];
unsigned int mic_check;										//BITN in mic_check is for MICN. Determines which MIC has a time value
unsigned int mic_use;										//vector that determines mic being used

int mic_adc[5];

float R_horz[SAMPLE_AVG_COUNT],B_horz[SAMPLE_AVG_COUNT], R_horz_avg, B_horz_avg;

int main(void) {
	unsigned int i;
	int CD10[SAMPLE_AVG_COUNT], CD21[SAMPLE_AVG_COUNT];	//holy shit memory usage Batman!


    WDTCTL = WDTPW | WDTHOLD;						// Stop watchdog timer
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;
	_enable_interrupts();

    virgin_flag = 1;
    convert_flag = 1;

    mic_check = 0x00;

    TAR = 0x00;											//Clear TAR

    ADC10CTL1 |= ADC10SSEL_2;
    ADC10CTL0 |= ADC10IE /* + ADC10SHT_2 */;
    ADC10CTL0 |= ADC10ON;								//turn on the ADC and interrupts for it
    //sweep microphone input

    //for 5 mic setup
    //while(!(mic_check & 0x1F))						//when all bits in mic_check vector are set, exit while loop to continue

    //for 2 mic setup
    while(mic_check != 0x7FFF)							//every 5 bits is is dedicated to 5 samples from each mic. 0x755 = 15 bits
    {
    	//Checking MIC0
    	if(~(mic_check & 0x1F))							//0x1F is 5 bits, when all 5 samples have been taken mic_check4:0 will be high
    	{
    		mic_use = 0;
    		ADC10CTL0 |= ADC10SC+ENC;
    		while(convert_flag);
    		convert_flag = 1;
    	}

    	//Checking MIC1
    	if(~(mic_check & 0x3E0))
		{
    		mic_use = 1;
    		ADC10CTL1 |= INCH_1;
			ADC10CTL0 |= ADC10SC+ENC;;
			while(convert_flag);
			convert_flag = 1;
			ADC10CTL1 &= ~INCH_1;
		}

    	//Checking MIC2
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

    ADC10CTL0 &= ~ADC10ON;					//turn off ADC when done filling with values

    R_horz_avg = 0.0;
    B_horz_avg = 0.0;

    //INSERT LOOP TO CALCULATE RANGE AND BEARING FOR 5 SAMPLES
    for(i=0; i<SAMPLE_AVG_COUNT_CALC; i++)
    {
        CD10[i] = (mic_time[1][i]-mic_time[0][i]);
        CD21[i] = (mic_time[2][i]-mic_time[1][i]);

        //I wish I could take you out of your misery.
        R_horz[i] =  ARRAY_LENGTH_ACTUAL * ( 1 - ( (CD10[i] / ARRAY_LENGTH ) * ( CD10[i] / ARRAY_LENGTH ) ) );
        R_horz[i] += ARRAY_LENGTH_ACTUAL * ( 1 - ( (CD21[i] / ARRAY_LENGTH ) * ( CD21[i] / ARRAY_LENGTH ) ) );
        R_horz[i] = R_horz[i] / ( 2 * ( ( CD21[i] / ARRAY_LENGTH ) - ( CD10[i] / ARRAY_LENGTH ) ) );

        B_horz[i] = acos(( ARRAY_LENGTH_ACTUAL*ARRAY_LENGTH_ACTUAL - 2*R_horz[i]*CD21[i]/4000.0*343.0/2000.0 - ( CD21[i]/4000.0*343.0/2000.0)*( CD21[i]/4000.0*343.0/2000.0) ) / ( (2*R_horz[i]*ARRAY_LENGTH_ACTUAL)))*180/3.14159;

        R_horz_avg += R_horz[i];
        B_horz_avg += B_horz[i];

    }

    R_horz_avg = R_horz_avg / SAMPLE_AVG_COUNT_CALC;
    B_horz_avg = B_horz_avg / SAMPLE_AVG_COUNT_CALC;

    while(1);

	return 0;
}

//ISR for ADC
#pragma vector=ADC10_VECTOR
__interrupt void adc_isr (void)
{
	convert_flag = 0;

	ADC10CTL0 &= ~ENC;								//turn off ENC bit so the input channel can be changed

	//mic_adc[mic_use] = ADC10MEM;

	if(ADC10MEM > ADC_THRES)						//check is ADC value is close to rail
	{

		if(virgin_flag)
		{
			//turn on the timer
			TACTL = TASSEL_2 + MC_2 + ID_1;                 // SMCLK, continous mode @ 16MHz

			virgin_flag = 0;						//turn off flag so program doesn't repeat branch

			//the block of ifs checks to see what mic was triggered and use that as 0 time
			if(mic_use == 0)
			{
				if(MIC0_sample_count == SAMPLE_AVG_COUNT)
				{
				}
				else
				{
					mic_check |= 1 << (MIC0_sample_count);
					mic_time[mic_use][MIC0_sample_count] = 0;
					MIC0_sample_count++;
				}
			}
			else if(mic_use == 1)
			{
				if(MIC1_sample_count == SAMPLE_AVG_COUNT)
				{
				}
				else
				{
					mic_check |= 1 << (MIC1_sample_count + 5);
					mic_time[mic_use][MIC1_sample_count] = 0;
					MIC1_sample_count++;
				}
			}
			else if(mic_use == 2)
			{
				if(MIC2_sample_count == SAMPLE_AVG_COUNT)
				{
				}
				else
				{
					mic_check |= 1 << (MIC2_sample_count + 10);
					mic_time[mic_use][MIC2_sample_count] = 0;
					MIC2_sample_count++;
				}
			}
		}

		else
		{
			if(mic_use == 0)
			{
				if(MIC0_sample_count < SAMPLE_AVG_COUNT)
				{
					mic_check |= 1 << (MIC0_sample_count);
					mic_time[mic_use][MIC0_sample_count] = (unsigned int)TAR;
					MIC0_sample_count++;
				}
			}
			else if(mic_use == 1)
			{
				if(MIC1_sample_count < SAMPLE_AVG_COUNT)
				{
					mic_check |= 1 << (MIC1_sample_count + 5);
					mic_time[mic_use][MIC1_sample_count] = (unsigned int)TAR;
					MIC1_sample_count++;
				}
			}
			else if(mic_use == 2)
			{
				if(MIC2_sample_count < SAMPLE_AVG_COUNT)
				{
					mic_check |= 1 << (MIC2_sample_count + 10);
					mic_time[mic_use][MIC2_sample_count] = (unsigned int)TAR;
					MIC2_sample_count++;
				}
			}

		}
	}
}
