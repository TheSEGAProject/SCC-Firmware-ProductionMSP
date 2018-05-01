/*
 * adc.c
 *
 *  Created on: Feb 15, 2016
 *      Author: jdk85
 */

#include "msp.h"
#include "adc.h"

volatile float Battery_ADC[Num_of_Results];
volatile float Solar_V_ADC[Num_of_Results];
volatile float Solar_I_ADC[Num_of_Results];
/**
 * ADC Samples Averaged Li-ion Battery Voltage
 */
volatile float fBatteryVoltage;
/**
 * ADC Samples Averaged Solar Panel Voltage
 */
volatile float fSolarVoltage;
/**
 * Instananeous current on the MAX9934
 */
volatile float fCurrentCurrent;
/**
 * Instananeous voltage on the MAX9934
 */
volatile float fCurrentVoltage;
/**
 * ADC Samples Averaged current of the solar panel
 */
volatile float fAverageCurrent;
/**
 * ADC Samples Averaged Power on Solar Panel
 */
volatile float fAveragePower;
/**
 * ADC Samples Averaged voltage of MAX 9934
 */
volatile float fAverageVoltage;
volatile float fVsense;


volatile uint8 index;
volatile uint8 ucADC_COMPLETE;
volatile uint8 ucFaultDetected = 0x00;

void vADC_Init(void){

	//Configure reference voltage
	//while(REF_A->CTL0 & REF_A_CTL0_GENBUSY); //Wait for ref to be ready
	REF_A->CTL0 |= REF_A_CTL0_VSEL_3 | REF_A_CTL0_ON; //2.5V and turn on reference
	while(REF_A->CTL0 & REF_A_CTL0_GENBUSY); //Wait for ref to be ready
	//__delay_cycles(300); //Let ref settle (~75uS)


	// Configure GPIO
	P5->SEL1 |= BIT4 | BIT3 | BIT0;       // Enable A/D channel A1,A2,A5
	P5->SEL0 |= BIT4 | BIT3 | BIT0;




	P5->DIR |= BIT5 | BIT1; //Set P5.1 and P5.5 to output
	P5->OUT |= BIT5 | BIT1; //Turn on MAX9934 and Battery ADC

	// Enable global interrupt
	__enable_irq();

	// Enable ADC interrupt in NVIC module
	NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31);




	ADC14->CTL0 = ADC14_CTL0_ON |  ADC14_CTL0_MSC | ADC14_CTL0_SHT0__192 | ADC14_CTL0_SHP | ADC14_CTL0_CONSEQ_3; // Turn on ADC14, extend sampling time to avoid overflow of results
	ADC14->CTL1 = ADC14_CTL1_RES_3; //14 bit resolution

	ADC14->MCTL[1] = ADC14_MCTLN_VRSEL_1 | ADC14_MCTLN_INCH_1;				// ref+=VREF, channel = A1, ADC = Battery_ADC
	ADC14->MCTL[2] = ADC14_MCTLN_VRSEL_1 | ADC14_MCTLN_INCH_2;				// ref+=VREF, channel = A2, ADC = SolarVin_ADC
	ADC14->MCTL[5] = ADC14_MCTLN_VRSEL_1 | ADC14_MCTLN_INCH_5 | ADC14_MCTLN_EOS;		// ref+=VREF, channel = A5, end seq, ADC = SolarIin_ADC
	ADC14->IER0 = ADC14_IER0_IE5;                     // Enable ADC14IFG.5
	__delay_cycles(600);//Give MAX9934 time to settle (~100uS)


}

void vADC_Shutdown(void){

	P5->OUT &= ~(BIT5 | BIT1); //Turn off MAX9934 and Battery ADC

	// Configure GPIO
	P5->SEL1 &= ~(BIT4 | BIT3 | BIT0);       // Disable A/D channel A5
	P5->SEL0 &= ~(BIT4 | BIT3 | BIT0);

	//Configure reference voltage
	REF_A->CTL0 &= ~REF_A_CTL0_ON; //Turn off reference

	ADC14->CTL0 &= ~(ADC14_CTL0_ON | ADC14_CTL0_ENC | ADC14_CTL0_SC); // Turn OFF ADC14
	ADC14->IER0 &= ~ADC14_IER0_IE5;  // Disable ADC14IFG.5
}

uint8 uiADC_Read(uint8 *ucParam){
	ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;        // Start conversion-software trigger

	while(!ucADC_COMPLETE){
		//do nothing
	}

		vADC_Calculate();

	if(fSolarVoltage > 0.25 && fAverageCurrent < 10.0 && fAverageVoltage < 0.25 ){
		ucFaultDetected = 0x01;
		__delay_cycles(4800000);
		vADC_Init();
		ucADC_COMPLETE = 0x00;//Set adc complete flag to 0
		ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;        // Start conversion-software trigger

		while(!ucADC_COMPLETE){
			//do nothing
		}

			vADC_Calculate();

	}

	return 0;







}



void vADC_Calculate(){
	unsigned char j;
	//Reset current and average values
	fBatteryVoltage = 0;
	fSolarVoltage = 0;
	fAverageVoltage = 0;
	fAverageCurrent = 0;
	fAveragePower = 0;


	//Convert the battery ADC
	for(j = 0; j < Num_of_Results; j++)
	{
		fBatteryVoltage += Battery_ADC[j];
		fSolarVoltage += Solar_V_ADC[j];
		fCurrentVoltage = Solar_I_ADC[j];
		fCurrentCurrent = fCurrentVoltage/11.25;
		fAverageVoltage += fCurrentVoltage;
		fAverageCurrent += fCurrentCurrent;
		fAveragePower += Solar_V_ADC[j]*fCurrentCurrent;

	}


	fBatteryVoltage = fBatteryVoltage/Num_of_Results;
	fSolarVoltage = fSolarVoltage/Num_of_Results;
	fAverageVoltage = fAverageVoltage/Num_of_Results;
	fAverageCurrent = fAverageCurrent/Num_of_Results;
	fAveragePower = fAveragePower/Num_of_Results;

}























