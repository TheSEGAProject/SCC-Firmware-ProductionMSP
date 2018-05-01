/*
 * uart.c
 *
 *  Created on: Feb 15, 2016
 *      Author: jdk85
 */

#include "msp.h"
#include "adc.h"
#include <stdio.h>
#include <cstring>

volatile unsigned char i;
volatile unsigned char j;
volatile unsigned int tx_pointer;



void vUARTCOM_TXString(char* string, int len)
{

	for(tx_pointer = 0; tx_pointer < len; tx_pointer++)
	{
		EUSCI_A0->TXBUF = string[tx_pointer];
		while(!(EUSCI_A0->IFG&EUSCI_A_IFG_TXIFG));              // USCI_A1 TX buffer ready for next byte?
	}
}

void vUARTCOM_TXStringNoLen(char* string)
{

	for(tx_pointer = 0; tx_pointer < strlen(string); tx_pointer++)
	{
		EUSCI_A0->TXBUF = string[tx_pointer];
		while(!(EUSCI_A0->IFG&EUSCI_A_IFG_TXIFG));              // USCI_A0 TX buffer ready for next byte?
	}
}




