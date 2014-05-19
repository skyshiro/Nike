#include <msp430.h> 

#define MIC1 BIT1
#define MIC2 BIT2
#define MIC3 BIT3
#define MIC4 BIT4
#define MIC5 BIT5

#define MIC_CAL 20

#define VOLT_REF BIT0

#define WIN_TIME 8

/*
 * P1.1 through P1.5 are the microphone inputs MIC1 through MIC5
 *
 * P1.0 will be the reference voltage
 */

int virgin_flag;

int mic_time[5];									//holds value of microphone
int mic_check;										//BITN in mic_check is for MICN+1. Determines which MIC has a time value
int mic_num;										//determine which MIC is being used
int mic_use;										//vector that determines mic being used

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;						// Stop watchdog timer
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;
	_enable_interrupts();

    P1DIR |= MIC1 + MIC2 + MIC3 + MIC4 + MIC5 + VOLT_REF;		//configure microphone & reference voltage inputs

    virgin_flag = 1;
    mic_check = 0x00;

    CACTL2 |= P2CA0;											//select reference voltage
    CACTL2 &= ~(P2CA3 + P2CA2 + P2CA1);							//clears selection

    CACTL2 |= P2CA1;

    CACTL1 |= CAON;
    CACTL1 &= ~CAIFG;

    CACTL1 |= CAIES + CAIE;

    //sweep microphone input

    //for 5 mic setup
    //while(!(mic_check & 0x1F))									//when all bits in mic_check vector are set, exit while loop to continue

    //for 2 mic setup
    while(mic_check != 0x3)
    {
    	if(~mic_check & (MIC1 >> 1))
    	{
    		mic_use = 0;
    		CACTL2 |= P2CA1;										//selects CA1
    		__delay_cycles(WIN_TIME);
    		CACTL2 &= ~(P2CA1 + P2CA2 + P2CA3);						//clears control lines on multiplexer
    	}

    	if(~mic_check & (MIC2 >> 1))
		{
    		mic_use = 1;
    		CACTL2 |= P2CA2;										//selects CA2
    		__delay_cycles(WIN_TIME);
    		CACTL2 &= ~(P2CA1 + P2CA2 + P2CA3);						//clears control lines on multiplexer
		}
    }

    //mic_time should be around 4660 for 10cm
    while(1);
	
	return 0;
}

//ISR for comparator
#pragma vector=COMPARATORA_VECTOR
__interrupt void comp_isr (void)
{
	mic_check |= 1 << mic_use;

	if(virgin_flag)
	{
		//turn on the timer
		TACTL = TASSEL_2 + MC_2;                  // SMCLK, continous mode @ 16MHz

		virgin_flag = 0;
		mic_time[mic_num] = 0;
	}
	else
	{
		mic_time[mic_use] = TAR-MIC_CAL;		//the value of the next microphone is the value of TAR
	}

}
