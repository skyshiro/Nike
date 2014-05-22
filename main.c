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

#define L10	.075
#define L21	.075

/*
 * P1.1 through P1.5 are the microphone inputs MIC0 through MIC4
 *
 * P1.0 will be the reference voltage
 *
 * ADC value of about 1020 will be rail
 *
 * CONSEQ_3 will convert selected channels continously
 *
 * ADC10ON = 1
 *
 * set INCH_1 so ADC starts at channel 1 and goes to channel 0
 *
 * ENC = 1
 * ADC10SC = 1
 */

int virgin_flag;
int convert_flag;									//determines if ADC has converted value

int mic_time[5];									//holds value of microphone
int mic_adc[5];
int mic_check;										//BITN in mic_check is for MICN. Determines which MIC has a time value
int mic_use;										//vector that determines mic being used

int mic_adc[5];

float test, CD10, CD21,R_horz,B_horz;

int main(void) {


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
    //while(!(mic_check & 0x1F))								//when all bits in mic_check vector are set, exit while loop to continue

    //for 2 mic setup
    while(mic_check != 0x7)
    {
    	if(~mic_check & MIC0)
    	{
    		mic_use = 0;
    		ADC10CTL0 |= ADC10SC+ENC;
    		while(convert_flag);
    		convert_flag = 1;
    	}
    	if(~mic_check & MIC1)
		{
    		mic_use = 1;
    		ADC10CTL1 |= INCH_1;
			ADC10CTL0 |= ADC10SC+ENC;;
			while(convert_flag);
			convert_flag = 1;
			ADC10CTL1 &= ~INCH_1;
		}
    	if(~mic_check & MIC2)
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

    CD10 = (mic_time[1]-mic_time[0])/4000.0*343/4000.0;
    CD21 = (mic_time[2]-mic_time[1])/4000.0*343/4000.0;

    R_horz = L10 * ( 1 - ( (CD10 / L10)*(CD10 / L10)) ) + L21 * ( 1 - ( (CD21 / L21 ) * (CD21 / L21) ));
    R_horz = R_horz / ( 2 * ( ( CD21 / L21 ) - ( CD10 / L10 ) ) );
    R_horz = R_horz * 100;	//convert to cm

    B_horz = acos(( L10*L10 - 2*R_horz*CD21 - CD21*CD21) / ( 2*R_horz*L10 ))*180/3.14159;

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
