/*
 * clock.c
 *
 *  Created on: Mar 8, 2016
 *      Author: jdk85
 */

#include "msp.h"
#include "clock.h"
#include "core.h"



void vMCLK_24MHz(void){
	P4->DIR |= BIT2 | BIT3;
	P4->SEL0 |= BIT2 | BIT3;                         // Output ACLK & MCLK
	P4->SEL1 &= ~(BIT2 | BIT3);

	CS->KEY = CS_KEY_VAL;                        // Unlock CS module for register access
	CS->CTL0 = 0;                            // Reset tuning parameters
	CS->CTL0 = CS_CTL0_DCORSEL_4;                   // Set DCO to 24MHz (nominal, center of 8-16MHz range)

	// Select ACLK = REFO, SMCLK = MCLK = DCO
	CS->CTL1 = CS_CTL1_SELA_2 | CS_CTL1_SELS_3 | CS_CTL1_SELM_3;
	CS->KEY = 0;                             // Lock CS module from unintended accesses
#if 1
	// Output ACLK HSMCLK MCLK clock signals for testing
	P4->DIR |= BIT2 | BIT3 | BIT4;
	P4->SEL0 |= BIT2 | BIT3 | BIT4;
	P4->SEL1 &= ~(BIT2 | BIT3 | BIT4);
#endif
}

void vMCLK_12MHz(void){
	P4->DIR |= BIT2 | BIT3;
	P4->SEL0 |= BIT2 | BIT3;                         // Output ACLK & MCLK
	P4->SEL1 &= ~(BIT2 | BIT3);

	CS->KEY = 0x695A;                        // Unlock CS module for register access
	CS->CTL0 = 0;                            // Reset tuning parameters
	CS->CTL0 = CS_CTL0_DCOEN | CS_CTL0_DCORSEL_3;                   // Set DCO to 12MHz (nominal, center of 8-16MHz range)

	// Select ACLK = REFO, SMCLK = MCLK = DCO
	CS->CTL1 = CS_CTL1_SELA_2 | CS_CTL1_SELS_3 | CS_CTL1_SELM_3;
	CS->KEY = 0;                             // Lock CS module from unintended accesses
#if 0
	// Output ACLK HSMCLK MCLK clock signals for testing
	P4->DIR |= BIT2 | BIT3 | BIT4;
	P4->SEL0 |= BIT2 | BIT3 | BIT4;
	P4->SEL1 &= ~(BIT2 | BIT3 | BIT4);
#endif
}
void vMCLK_3MHz(void){
	P4->DIR |= BIT2 | BIT3;
	P4->SEL0 |= BIT2 | BIT3;                         // Output ACLK & MCLK
	P4->SEL1 &= ~(BIT2 | BIT3);


	CS->KEY = CS_KEY_VAL;                        // Unlock CS module for register access
	// Select ACLK = REFO, SMCLK = MCLK = DCO
	CS->CTL1 = CS_CTL1_SELA_2 | CS_CTL1_SELS_3 | CS_CTL1_SELM_3;
	CS->KEY = 0;                             // Lock CS module from unintended accesses
#if 0
	// Output ACLK HSMCLK MCLK clock signals for testing
	P4->DIR |= BIT2 | BIT3 | BIT4;
	P4->SEL0 |= BIT2 | BIT3 | BIT4;
	P4->SEL1 &= ~(BIT2 | BIT3 | BIT4);
#endif

}


void vMCLK_16MHzCAL(void){

	float dcoConst;
	int32_t calVal,target;
	uint32_t centeredFreq;
	int16_t dcoTune;

	dcoConst = *((float *) &TLV->DCOER_CONSTK_RSEL04);
	calVal = TLV->DCOER_FCAL_RSEL04;

	target = 16000000;
	centeredFreq = 12000000;




	dcoTune = ((target - centeredFreq) * (1 + dcoConst * (768 - calVal))) / (target * dcoConst);

	if (dcoTune < 0)
	{
		dcoTune = (dcoTune & CS_CTL0_DCOTUNE_MASK) | 0x1000;
	}
	else
	{
		dcoTune &= CS_CTL0_DCOTUNE_MASK;
	}

	PJ->SEL0 |= BIT0 | BIT1;                    // set LFXT pin as second function


	CS->KEY = CS_KEY_VAL;                     // Unlock CS module for register access
	CS->CTL0 = 0;                            // Reset tuning parameters

	CS->CTL0 = CS_CTL0_DCOEN | CS_CTL0_DCORES | CS_CTL0_DCORSEL_3 | dcoTune;                   // Set DCO to 12MHz (nominal, center of 16-32MHz range, tune to 16MHz)

	CS->CTL2 |= CS_CTL2_LFXT_EN;                         // LFXT on
	CS->CLKEN = CS_CLKEN_SMCLK_EN | CS_CLKEN_MCLK_EN | CS_CLKEN_HSMCLK_EN | CS_CLKEN_ACLK_EN;


	do
	{
		// Clear XT2,XT1,DCO fault flags
		CS->CLRIFG |= CS_CLRIFG_CLR_DCOR_OPNIFG | CS_CLRIFG_CLR_HFXTIFG | CS_CLRIFG_CLR_LFXTIFG;
		SYSCTL->NMI_CTLSTAT &= ~ SYSCTL_NMI_CTLSTAT_CS_SRC;
	} while (SYSCTL->NMI_CTLSTAT & SYSCTL_NMI_CTLSTAT_CS_FLG);// Test oscillator fault flag

	// Select ACLK = LFXT, SMCLK = MCLK = DCO/3
	CS->CTL1 = CS_CTL1_DIVS__4 | CS_CTL1_DIVHS__4 | CS_CTL1_DIVA_0 | CS_CTL1_DIVM_0 | CS_CTL1_DIVHS_0 | CS_CTL1_SELA__LFXTCLK | CS_CTL1_SELS_3 | CS_CTL1_SELM_3;
	CS->KEY = 0;                             // Lock CS module from unintended accesses

#if 0
	// Output ACLK HSMCLK MCLK clock signals for testing
	P4->DIR |= BIT2 | BIT3 | BIT4;
	P4->SEL0 |= BIT2 | BIT3 | BIT4;
	P4->SEL1 &= ~(BIT2 | BIT3 | BIT4);
#endif

}
void vMCLK_16MHz(void){

	unsigned char ucDCO = 16;
	unsigned char ucNominalFreq = 24;
	float * pulDCO_ER_RSEL04_K = (float*)0x00201084L;
	unsigned long * pulER_Cal = (unsigned long *)0x0020106CL;

	float fDCOConstant = *pulDCO_ER_RSEL04_K;
	unsigned long ulDCOCal = *pulER_Cal;


	float fTempNum = (ucDCO - ucNominalFreq)*(((fDCOConstant * (768 - ulDCOCal)) + 1)*8);
	float fTempDen = (fDCOConstant * ucDCO);


	signed long lDCOTuneBits = fTempNum/fTempDen;
	signed int iDCOTune;
	// The DCO tune bits field is 13-bits wide and the value is two's complement,
	// therefore, we must do some bit manipulation.
	if(lDCOTuneBits < 0){
		iDCOTune = (signed int)(lDCOTuneBits & 0x1FFF) *-1;
		iDCOTune = (signed int)(iDCOTune ^ 0x1FFF)+1;
		iDCOTune &= 0x1FFF;
	}
	else{
		iDCOTune =	lDCOTuneBits & 0x00001FFF;
	}

	PJ->SEL0 |= BIT0 | BIT1;                    // set LFXT pin as second function


	CS->KEY = 0x695A;                        // Unlock CS module for register access
	CS->CTL0 = 0;                            // Reset tuning parameters

	/**
	 * NOTE: TODO: The CS_CTL0_DCORES external selection is unstable in the current release,
	 */
	//	CS->CTL0 = CS_CTL0_DCOEN | CS_CTL0_DCORES | CS_CTL0_DCORSEL_3 | iDCOTune;                   // Set DCO to 12MHz (nominal, center of 8-16MHz range)
	CS->CTL0 = CS_CTL0_DCOEN  | CS_CTL0_DCORSEL_4 | iDCOTune;
	CS->CTL2 |= CS_CTL2_LFXT_EN;                         // LFXT on
	CS->CLKEN = CS_CLKEN_SMCLK_EN | CS_CLKEN_MCLK_EN | CS_CLKEN_HSMCLK_EN | CS_CLKEN_ACLK_EN;
	do
	{
		// Clear XT2,XT1,DCO fault flags
		CS->CLRIFG |= CS_CLRIFG_CLR_DCOR_OPNIFG  | CS_CLRIFG_CLR_HFXTIFG | CS_CLRIFG_CLR_LFXTIFG;
		SYSCTL->NMI_CTLSTAT &= ~ SYSCTL_NMI_CTLSTAT_CS_SRC;
	} while (SYSCTL->NMI_CTLSTAT & SYSCTL_NMI_CTLSTAT_CS_FLG);// Test oscillator fault flag

	// Select ACLK = LFXT, SMCLK = MCLK = DCO/3
	CS->CTL1 = CS_CTL1_DIVS__4 | CS_CTL1_DIVHS__4 | CS_CTL1_DIVA_0 | CS_CTL1_DIVM_0 | CS_CTL1_DIVHS_0 | CS_CTL1_SELA__LFXTCLK | CS_CTL1_SELS_3 | CS_CTL1_SELM_3;
	CS->KEY = 0;                             // Lock CS module from unintended accesses

#if 0
	// Output ACLK HSMCLK MCLK clock signals for testing
	P4->DIR |= BIT2 | BIT3 | BIT4;
	P4->SEL0 |= BIT2 | BIT3 | BIT4;
	P4->SEL1 &= ~(BIT2 | BIT3 | BIT4);
#endif

}

void vMCLK_48MHz(void){
	PJ->SEL0 |= BIT2 | BIT3;                  // Configure PJ.2/3 for HFXT function
	PJ->SEL1 &= ~(BIT2 | BIT3);

	CS->KEY = CS_KEY_VAL;                      // Unlock CS module for register access
	CS->CTL2 |= CS_CTL2_HFXT_EN | CS_CTL2_HFXTFREQ_6 | CS_CTL2_HFXTDRIVE; //Enable HFXT, set freq range to 40-48MHz, drive when HFXTFREQ > 4MHz

	//Wait for both clock and HFXT interrupt flags to be set
	while(CS->IFG & CS_IFG_HFXTIFG)
		CS->CLRIFG |= CS_CLRIFG_CLR_HFXTIFG; //Clear the HFXT interrupt

	// Select MCLK & HSMCLK = HFXT, no divider
	//CS->CTL1 = CS->CTL1 & ~(CS_CTL1_SELM_MASK | CS_CTL1_DIVM_MASK | CS_CTL1_SELS_MASK | CS_CTL1_DIVHS_MASK) | CS_CTL1_SELM__HFXTCLK | CS_CTL1_SELS__HFXTCLK;
	//Set MCLK = HFXT, SMCLK=MCLK/4
	CS->CTL1 = CS_CTL1_DIVS_2 | CS_CTL1_SELM_5 | CS_CTL1_SELS_5;
	CS->KEY = 0;                             // Lock CS module from unintended accesses
#if 0
	// Output ACLK HSMCLK MCLK clock signals for testing
	P4->DIR |= BIT2 | BIT3 | BIT4;
	P4->SEL0 |= BIT2 | BIT3 | BIT4;
	P4->SEL1 &= ~(BIT2 | BIT3 | BIT4);
#endif
}

