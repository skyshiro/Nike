#include <msp430.h> 

#define MIC0 BIT0
#define MIC1 BIT1
#define MIC2 BIT2
#define MIC3 BIT3
#define MIC4 BIT4

#define MIC_CAL 20

#define WIN_TIME 5

/*
 * P1.1 through P1.5 are the microphone inputs MIC1 through MIC5
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

int mic_time[5];									//holds value of microphone
int mic_check;										//BITN in mic_check is for MICN+1. Determines which MIC has a time value
int mic_num;										//determine which MIC is being used
int mic_use;										//vector that determines mic being used

int mic_adc[5];

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;						// Stop watchdog timer
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;
	_enable_interrupts();

    virgin_flag = 1;
    mic_check = 0x00;

    ADC10CTL1 |= INCH_2 + ADC10SSEL_2 + CONSEQ_3;
    ADC10CTL0 |= ADC10IE + MSC + ADC10SHT_2;
    ADC10CTL0 |= ADC10ON;								//turn on the ADC and interrupts for it
    ADC10CTL0 |= ENC + ADC10SC;
    //sweep microphone input

    //for 5 mic setup
    //while(!(mic_check & 0x1F))								//when all bits in mic_check vector are set, exit while loop to continue

    //for 2 mic setup
    while(mic_check != 0x3)
    {

    }

    ADC10CTL0 &= ~ADC10ON;										//turn off ADC when done filling with values

    //mic_time should be around 4660 for 10cm
    while(1);
	
	return 0;
}

//ISR for ADC
#pragma vector=ADC10_VECTOR
__interrupt void adc_isr (void)
{
	//determine which mic was converted

	mic_num = 0xF & (ADC10CTL1 >> 12);					//determine which microphone was converted

	if(ADC10MEM > 1000)									//check is ADC value is close to rail
	{
		mic_check |= 1 << mic_num;

		if(virgin_flag)
		{
			//turn on the timer
			TACTL = TASSEL_2 + MC_2;                  // SMCLK, continous mode @ 16MHz

			virgin_flag = 0;
			mic_time[mic_num] = 0;
		}
		else
		{
			//mic_time[mic_use] = TAR-MIC_CAL;		//the value of the next microphone is the value of TAR
			mic_time[mic_use] = TAR;
		}
	}
}
