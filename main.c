#include <msp430.h> 
#include "ds18x20.h"
#include <math.h>

#define MIC0 BIT0
#define MIC1 BIT1
#define MIC2 BIT2
#define MIC3 BIT3
#define MIC4 BIT4

#define SERVO_HORZ BIT0
#define SERVO_VERT BIT1
#define SERVO_RST 9600
#define SERVO_WAIT 75

#define MIC_CAL 20
#define ADC_THRES 900
#define WIN_TIME 5
#define SAMPLE_AVG_COUNT 2 		//number of samples to avg

#define ARRAY_LENGTH_ACTUAL 0.3 //length between mic in meters

/* Documentation:
 *
 * Timer runs at 8MHz to avoid overflow. Timer is an integer from 0->2^16 while mic_time is a float from 0->2^15
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
 * Takes arbitrary samples from each microphone then does radius/bearing calculation and averages the results
 *
 * Radius is calculated as an float in m and the bearing is calculated as float in degrees
 *
 * Laser circuitry
 *	Vcc = 5V
 *	Green/White = GND
 *
 */

unsigned int virgin_flag;
unsigned int convert_flag;									//determines if ADC has converted value
unsigned int MIC0_sample_count = 0;
unsigned int MIC1_sample_count = 0;
unsigned int MIC2_sample_count = 0;
unsigned int MIC3_sample_count = 0;
unsigned int MIC4_sample_count = 0;

volatile int mic_time[5][SAMPLE_AVG_COUNT];					//2D array, first dimension holds timing values from mic and second is for the multiple samples int mic_adc[5];
unsigned int mic_check;										//BITN in mic_check is for MICN. Determines which MIC has a time value
unsigned int mic_check_check;
unsigned int mic_check_subcheck[5];
unsigned int mic_use;										//vector that determines mic being used

volatile int CD10_horz[SAMPLE_AVG_COUNT], CD21_horz[SAMPLE_AVG_COUNT];	//holy shit memory usage Batman!
volatile float B_horz[SAMPLE_AVG_COUNT];
volatile float R_horz[SAMPLE_AVG_COUNT];

volatile int CD10_vert[SAMPLE_AVG_COUNT], CD21_vert[SAMPLE_AVG_COUNT];
volatile float B_vert[SAMPLE_AVG_COUNT];
volatile float R_vert[SAMPLE_AVG_COUNT];

volatile float sound_speed,temperature_val,array_length;
volatile float R_horz_avg,B_horz_avg;
volatile float R_vert_avg,B_vert_avg;

//acos in degrees
const int acos_table[100] = {180,169,164,160,157,154,152,149,147,145,143,141,139,138,136,134,133,131,130,128,127,125,124,123,121,120,119,117,116,115,114,112,111,110,109,107,106,105,104,103,102,100,99,98,97,96,95,93,92,91,90,89,88,87,85,84,83,82,81,80,78,77,76,75,74,73,71,70,69,68,66,65,64,63,61,60,59,57,56,55,53,52,50,49,47,46,44,42,41,39,37,35,33,31,28,26,23,20,16,11};
//atan in millidegrees
const int atan_table[100] = {573,682,791,899,1008,1117,1226,1335,1444,1552,1661,1770,1879,1987,2096,2205,2313,2422,2531,2639,2748,2857,2965,3074,3182,3291,3399,3508,3616,3725,3833,3941,4050,4158,4266,4375,4483,4591,4699,4807,4915,5023,5131,5239,5347,5455,5563,5671,5779,5886,5994,6102,6209,6317,6424,6532,6639,6747,6854,6961,7069,7176,7283,7390,7497,7604,7711,7818,7925,8031,8138,8245,8351,8458,8564,8671,8777,8883,8990,9096,9202,9308,9414,9520,9626,9732,9837,9943,10049,10154,10259,10365,10470,10575,10681,10786,10891,10996,11100,11205};
//cos in milliradius
const int cos_table[25] = {1000,969,876,729,536,309,63,-187,-426,-637,-809,-930,-992,-992,-930,-809,-637,-426,-187,63,309,536,729,876,969};
//sin in milliradius
const int sin_table[25] = {0,249,482,685,844,951,998,982,905,771,588,368,125,-125,-368,-588,-771,-905,-982,-998,-951,-844,-685,-482,-249};

unsigned int circle_flag = 0,servo_low, servo_high_horz, servo_high_vert, servo_low_horz, servo_low_vert;


int time_index;
int r_ratio;
float arctan_const;

int main(void)
{
	unsigned int i,k;

    WDTCTL = WDTPW | WDTHOLD;						// Stop watchdog timer
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;
	_enable_interrupts();

	P2DIR |= SERVO_HORZ + SERVO_VERT;

	TACTL |= TASSEL_2 + TAIE;
	TACCR0 = 1000;

	//TACTL |= MC_1;						//set before the delay

	//Reset the servos
	for(i=0; i < SERVO_WAIT; i++)
	{
		P2OUT |= SERVO_HORZ + SERVO_VERT;

		__delay_cycles(SERVO_RST);

		servo_low = (20 - SERVO_RST/4000.0/4000.0)*16;

		P2OUT &= ~(SERVO_HORZ + SERVO_VERT);

		for(k=0; k < servo_low; k++)
		{
			__delay_cycles(1000);
		}
	}

	//Generates the mic_check vector for arbitrary samples
	for(i=0; i < SAMPLE_AVG_COUNT*5; i++)
	{
		mic_check_check |= 1 << i;
	}

	//generates the individual mic_check vectors for arbitrary samples
	for(i=0; i < SAMPLE_AVG_COUNT; i++)
	{
		mic_check_subcheck[i] |= SAMPLE_AVG_COUNT << SAMPLE_AVG_COUNT*i;
	}

	InitDS18B20();
	temperature_val = GetData();
	sound_speed = 0.606 * temperature_val + 331.3;
	array_length = ARRAY_LENGTH_ACTUAL * 4000 / sound_speed * 2000;

	TAR = 0x00;

	ADC10CTL1 |= ADC10SSEL_2;
	ADC10CTL0 |= ADC10IE /* + ADC10SHT_2 */;

    while(1)
    {
    	MIC0_sample_count = 0;
    	MIC1_sample_count = 0;
    	MIC2_sample_count = 0;
    	MIC3_sample_count = 0;
    	MIC4_sample_count = 0;

    	virgin_flag = 1;
		convert_flag = 1;
		mic_check = 0x00;
		ADC10CTL0 |= ADC10ON + ADC10IE;						//turn on the ADC and interrupts for it

		while( mic_check != mic_check_check  )				//when all bits in mic_check vector are set, exit while loop to continue
		{
			/**
			//insert servo circular code and determine optimal menacing frequency
			if(circle_flag)
			{
				//CIRCLE CIRCLE CIRCLE
			}
			**/

			//Checking MIC0
			if(~(mic_check & mic_check_subcheck[0] ))		//0x1F is 5 bits, when all 5 samples have been taken mic_check4:0 will be high
			{
				mic_use = 0;
				ADC10CTL0 |= ADC10SC+ENC;
				while(convert_flag);
				convert_flag = 1;
			}

			//Checking MIC1

			//HOLY FUCK HOW WILL WE DO THIS OSDFSKLF:SDF:LJDSL:SDFJ
			if(~(mic_check & mic_check_subcheck[1] ))
			{
				mic_use = 1;
				ADC10CTL1 |= INCH_1;
				ADC10CTL0 |= ADC10SC+ENC;;
				while(convert_flag);
				convert_flag = 1;
				ADC10CTL1 &= ~INCH_1;
			}

			//Checking MIC2
			if(~(mic_check & mic_check_subcheck[2] ))
			{
				mic_use = 2;
				ADC10CTL1 |= INCH_2;
				ADC10CTL0 |= ADC10SC+ENC;;
				while(convert_flag);
				convert_flag = 1;
				ADC10CTL1 &= ~INCH_2;
			}

			//Checking MIC3
			if(~(mic_check & mic_check_subcheck[3] ))
			{
				mic_use = 3;
				ADC10CTL1 |= INCH_3;
				ADC10CTL0 |= ADC10SC+ENC;;
				while(convert_flag);
				convert_flag = 1;
				ADC10CTL1 &= ~INCH_3;
			}

			//Checking MIC4
			if(~(mic_check & mic_check_subcheck[4] ))
			{
				mic_use = 4;
				ADC10CTL1 |= INCH_4;
				ADC10CTL0 |= ADC10SC+ENC;;
				while(convert_flag);
				convert_flag = 1;
				ADC10CTL1 &= ~INCH_4;
			}

		}

		ADC10CTL0 &= ~ADC10ON;					//turn off ADC when done filling with values
		TACTL &= ~MC_2;							//turn off timer and clear TAR
		TAR = 0x00;

		R_horz_avg = 0;
		B_horz_avg = 0;
		R_vert_avg = 0;
		B_vert_avg = 0;

		//INSERT LOOP TO CALCULATE RANGE AND BEARING FOR 5 SAMPLES
		for(i=0; i<SAMPLE_AVG_COUNT; i++)
		{
			CD10_horz[i] = (mic_time[1][i]-mic_time[0][i]);
			CD21_horz[i] = (mic_time[2][i]-mic_time[1][i]);

			R_horz[i] =  ARRAY_LENGTH_ACTUAL * ( 1 - ( (CD10_horz[i] / array_length ) * ( CD10_horz[i] / array_length ) ) );
			R_horz[i] += ARRAY_LENGTH_ACTUAL * ( 1 - ( (CD21_horz[i] / array_length ) * ( CD21_horz[i] / array_length ) ) );
			R_horz[i] = R_horz[i] / fabs( 2 * ( ( CD21_horz[i] / array_length ) - ( CD10_horz[i] / array_length ) ) );

			R_horz_avg += R_horz[i];

			B_horz[i] =( ARRAY_LENGTH_ACTUAL*ARRAY_LENGTH_ACTUAL - 2*R_horz[i]*CD21_horz[i]/4000*sound_speed/2000 - CD21_horz[i]/4000*343/2000*CD21_horz[i]/4000*sound_speed/2000) / ( (2*R_horz[i]*ARRAY_LENGTH_ACTUAL) );
			B_horz[i] = acos_table[ (int)( 100/2*( B_horz[i] + 1 ) )];

			B_horz_avg += B_horz[i];

			CD10_vert[i] = (mic_time[1][i]-mic_time[3][i]);
			CD21_vert[i] = (mic_time[4][i]-mic_time[1][i]);

			R_vert[i] =  ARRAY_LENGTH_ACTUAL * ( 1 - ( (CD10_vert[i] / array_length ) * ( CD10_vert[i] / array_length ) ) );
			R_vert[i] += ARRAY_LENGTH_ACTUAL * ( 1 - ( (CD21_vert[i] / array_length ) * ( CD21_vert[i] / array_length ) ) );
			R_vert[i] = R_vert[i] / fabs( 2 * ( ( CD21_vert[i] / array_length ) - ( CD10_vert[i] / array_length ) ) );

			R_vert_avg += R_vert[i];

			B_vert[i] =( ARRAY_LENGTH_ACTUAL*ARRAY_LENGTH_ACTUAL - 2*R_vert[i]*CD21_vert[i]/4000*sound_speed/2000 - CD21_vert[i]/4000*343/2000*CD21_vert[i]/4000*sound_speed/2000) / ( (2*R_vert[i]*ARRAY_LENGTH_ACTUAL) );
			B_vert[i] = acos_table[ (int)( 100/2*( B_vert[i] + 1 ) )];

			B_vert_avg += B_vert[i];

		}

		R_horz_avg = R_horz_avg / SAMPLE_AVG_COUNT;
		B_horz_avg = B_horz_avg / SAMPLE_AVG_COUNT;

		R_vert_avg = R_vert_avg / SAMPLE_AVG_COUNT;
		B_vert_avg = B_vert_avg / SAMPLE_AVG_COUNT;

		servo_high_horz = (B_horz_avg/100 + 0.6) * 16;
		servo_high_vert = (B_vert_avg/100 + 0.6) * 16;

		servo_low_horz = 320 - servo_high_horz;
		servo_low_vert = 320 - servo_high_vert;

		circle_flag = 1;

		//if the horz time is longer than vert, move vert first
		if(servo_high_horz > servo_high_vert)
		{
			//move both till horz needs to move more
			for(i=0; i < SERVO_WAIT; i++)
			{
				P2OUT |= SERVO_HORZ + SERVO_VERT;

				for(k=0; k < servo_high_vert; k++)
				{
					__delay_cycles(1000);
				}

				P2OUT &= ~(SERVO_HORZ + SERVO_VERT);

				for(k=0; k < servo_low_vert; k++)
				{
					__delay_cycles(1000);
				}
			}
			//move horz the additional distance
			for(i=0; i < SERVO_WAIT; i++)
			{
				P2OUT |= SERVO_HORZ;

				for(k=0; k < servo_high_horz; k++)
				{
					__delay_cycles(1000);
				}

				P2OUT &= ~(SERVO_HORZ);

				for(k=0; k < servo_low_horz; k++)
				{
					__delay_cycles(1000);
				}
			}
		}
		//opposite of above
		else
		{
			for(i=0; i < SERVO_WAIT; i++)
			{
				P2OUT |= SERVO_HORZ + SERVO_VERT;

				for(k=0; k < servo_high_horz; k++)
				{
					__delay_cycles(1000);
				}

				P2OUT &= ~(SERVO_HORZ + SERVO_VERT);

				for(k=0; k < servo_low_horz; k++)
				{
					__delay_cycles(1000);
				}
			}

			for(i=0; i < SERVO_WAIT; i++)
			{
				P2OUT |= SERVO_VERT;

				for(k=0; k < servo_high_vert; k++)
				{
					__delay_cycles(1000);
				}

				P2OUT &= ~(SERVO_VERT);

				for(k=0; k < servo_low_vert; k++)
				{
					__delay_cycles(1000);
				}
			}
		}

		P2OUT &= ~(SERVO_HORZ + SERVO_VERT);

		servo_high_horz = (B_horz_avg/100 + 0.6) * 16;
		servo_high_vert = (B_vert_avg/100 + 0.6) * 16;

		servo_low_horz = 320 - servo_high_horz;
		servo_low_vert = 320 - servo_high_vert;

		r_ratio = 50/R_horz_avg;
		arctan_const = atan_table[(int)(0.5263*(r_ratio - 10))] / 1000.0;

		while(1)
		{
			for( time_index = 0; time_index < 25; time_index++)
			{
				servo_high_horz = ( (B_horz_avg + (arctan_const * cos_table[time_index]/1000.0 ) )/100 + 0.6) * 16;
				servo_low_horz = 320 - servo_high_horz;

				servo_high_vert = ((B_vert_avg + (arctan_const * sin_table[time_index]/1000.0 ) )/100 + 0.6) * 16;
				servo_low_vert = 320 - servo_high_vert;

				for(i=0 ; i < 2 ; i++)
				{
					P2OUT |= SERVO_HORZ;

					for(k=0; k < servo_high_horz; k++)
					{
						__delay_cycles(1000);
					}

					P2OUT &= ~(SERVO_HORZ);

					for(k=0; k < servo_low_horz; k++)
					{
						__delay_cycles(1000);
					}
				}

				P2OUT &= ~SERVO_HORZ;

				for(i=0 ; i < 2 ; i++)
				{
					P2OUT |= SERVO_VERT;

					for(k=0; k < servo_high_vert; k++)
					{
						__delay_cycles(1000);
					}

					P2OUT &= ~(SERVO_VERT);

					for(k=0; k < servo_low_vert; k++)
					{
						__delay_cycles(1000);
					}
				}

				P2OUT &= ~SERVO_VERT;
			}
		}
    }

    return 0;
}

//ISR for Timer
#pragma vector=TIMER0_A1_vector
__interrupt void timer_isr (void)
{
	//set the servo change flag
	TACTL &= ~MC_1;
	TAR = 0;
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
			TACTL |= TASSEL_2 + MC_2 + ID_1;         // SMCLK, continous mode @ 16MHz

			virgin_flag = 0;						//turn off flag so program doesn't repeat branch

			//the switch checks to see what mic was triggered and use that as 0 time
			//if it aint broke dont fix it

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
					mic_check |= 1 << (MIC1_sample_count + SAMPLE_AVG_COUNT);
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
					mic_check |= 1 << (MIC2_sample_count + SAMPLE_AVG_COUNT*2);
					mic_time[mic_use][MIC2_sample_count] = 0;
					MIC2_sample_count++;
				}
			}
			else if(mic_use == 3)
			{
				if(MIC3_sample_count == SAMPLE_AVG_COUNT)
				{
				}
				else
				{
					mic_check |= 1 << (MIC3_sample_count + SAMPLE_AVG_COUNT*3);
					mic_time[mic_use][MIC3_sample_count] = 0;
					MIC3_sample_count++;
				}
			}
			else if(mic_use == 4)
			{
				if(MIC3_sample_count == SAMPLE_AVG_COUNT)
				{
				}
				else
				{
					mic_check |= 1 << (MIC4_sample_count + SAMPLE_AVG_COUNT*4);
					mic_time[mic_use][MIC4_sample_count] = 0;
					MIC4_sample_count++;
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
					mic_time[mic_use][MIC0_sample_count] = TAR;
					MIC0_sample_count++;
				}
			}
			else if(mic_use == 1)
			{
				if(MIC1_sample_count < SAMPLE_AVG_COUNT)
				{
					mic_check |= 1 << (MIC1_sample_count + SAMPLE_AVG_COUNT );
					mic_time[mic_use][MIC1_sample_count] = TAR;
					MIC1_sample_count++;
				}
			}
			else if(mic_use == 2)
			{
				if(MIC2_sample_count < SAMPLE_AVG_COUNT)
				{
					mic_check |= 1 << (MIC2_sample_count + SAMPLE_AVG_COUNT*2 );
					mic_time[mic_use][MIC2_sample_count] = TAR;
					MIC2_sample_count++;
				}
			}
			else if(mic_use == 3)
			{
				if(MIC3_sample_count < SAMPLE_AVG_COUNT)
				{
					mic_check |= 1 << (MIC3_sample_count + SAMPLE_AVG_COUNT*3 );
					mic_time[mic_use][MIC3_sample_count] = TAR;
					MIC3_sample_count++;
				}
			}
			else if(mic_use == 4)
			{
				if(MIC4_sample_count < SAMPLE_AVG_COUNT)
				{
					mic_check |= 1 << (MIC4_sample_count + SAMPLE_AVG_COUNT * 4);
					mic_time[mic_use][MIC4_sample_count] = TAR;
					MIC4_sample_count++;
				}
			}
		}
	}
}

