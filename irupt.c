/*
 * irupt.c
 *
 *  Created on: Mar 8, 2016
 *      Author: jdk85
 */




#include "msp.h"
#include "core/core.h"
#include "helpers/adc.h"
#include "helpers/led.h"
#include <stdio.h>


extern volatile uint8 g_ucaRXBuffer[MAXMSGLEN];
extern volatile uint8 g_ucRXBufferIndex;
extern volatile uint8 ucADC_COMPLETE;

uint8 ucSecondCounter = 1;
uint8_t rxData;

// Timer A0 interrupt service routine
void TA0_0_IRQHandler(void)
{

	TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;

	if(ucSecondCounter == 12){

		//TODO: figure out if there's a problem communicating with the CP
		//If the UART buffer got stuck somehow
		if(0){
			// Clear the RX buffer and reset index
			g_ucRXBufferIndex = MAXMSGLEN;
			while (g_ucRXBufferIndex) {
				g_ucRXBufferIndex--;
				g_ucaRXBuffer[g_ucRXBufferIndex] = 0xFF;
			}
			g_ucRXBufferIndex = 0x00;
		}



		SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;    // Wake up on exit from ISR
		// Ensures SLEEPONEXIT takes effect immediately
		__DSB();
		ucSecondCounter = 1;
	}
	else{
		SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;    // Do not wake up on exit from ISR
		// Ensures SLEEPONEXIT takes effect immediately
		__DSB();
		ucSecondCounter++;
	}



}


// UART interrupt service routine
void EUSCIA0_IRQHandler(void)
{

	if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG)
	{
		g_ucaRXBuffer[g_ucRXBufferIndex] = EUSCI_A0->RXBUF;
		g_ucRXBufferIndex++; // Increment index for next byte
		if(g_ucRXBufferIndex >= 64){
			//if we got a bad message size, reset the buffer
			vCOMM_ResetBuffer();
		}
		SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;    // Wake up on exit from ISR
		// Ensures SLEEPONEXIT takes effect immediately
		__DSB();

	}








}



// ADC14 interrupt service routine
void ADC14_IRQHandler(void)
{

	if (ADC14->IFGR0 & ADC14_IFGR0_IFG5)
	{
		Battery_ADC[index] = ADC14->MEM[1] * 0.0002670288;
		Solar_V_ADC[index] = ADC14->MEM[2] * 0.0001525879f;
		Solar_I_ADC[index] = ADC14->MEM[5] * 0.0001525879f;

		if(index == (Num_of_Results - 1))
		{
			index = 0;
			//Stop the conversion
			ADC14->CTL0 &= ~ADC14_CTL0_ENC & ~ADC14_CTL0_SC;
			//Shutdown the ADC
			vADC_Shutdown();
			//Alert that ADC readings are fully complete
			ucADC_COMPLETE = 0x01;

		}
		else
		{
			index++;
		}


		//Clear ADC flag
		ADC14->CLRIFGR0 = ADC14_CLRIFGR0_CLRIFG5;
	}

}
