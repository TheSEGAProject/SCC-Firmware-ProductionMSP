///////////////////////////////////////////////////////////////////////////////
//! \file comm.c
//! \brief This modules implements a software I2C on any two Digital I/O pins
//!
//! This module uses Port 2 to implement a software I2C on any two defined
//! digital I/O pin. Note that the serial clock (SCL) pin must be interrupt capable.
//!
//! @addtogroup core
//! @{
//!
//! @addtogroup comm Software I2C
//! The software I2C module allows the user to define a I2C interface on
//! any two GPIO pins, provided that the SCL pin is interrupt capable.
//! @{
///////////////////////////////////////////////////////////////////////////////
//*****************************************************************************
// By: Kenji Yamamoto
//     Wireless Networks Research Lab
//     Dept. of Electrical Engineering, CEFNS
//     Northern Arizona University
//
// Edited By: Christopher Porter, JD Knapp
//*****************************************************************************

#include "msp.h"
#include "../core.h"
#include "comm.h"
#include "crc.h"
#include <stdio.h>


//******************  Control and Indication Variables  *********************//
//! @name Control and Indication Variables
//! These variables are used to indicate to the system the current status
//! of the \ref comm Module and to store the baud rate timer information.
//! @{
//! \var volatile uint8 g_ucCOMM_Flags
//! \brief This 8-bit field indicates the status of the COMM module.
volatile uint8 g_ucCOMM_Flags;

//! \var uint16 g_unCOMM_BaudRateControl
//! \brief This is the value used to control the baud rate.
//!
//! This value is the number of timer ticks corresponding to one bit period
//! for the baud rate. It should be set from the \ref comm_baud
//! "Baud Rate Defines".
uint16 g_unCOMM_BaudRateControl;

//! \var uint16 g_unCOMM_BaudRateDelayControl
//! \brief This is the value used to delay from the start bit
//!
//! This value is the number of timer ticks to wait from the beginning of the
//! start bit to the middle of the first data bit. It should be set from the
//! \ref comm_baud_delay "Baud Rate Start Delays".
uint16 g_unCOMM_BaudRateDelayControl;
//! @}

//******************  RX Variables  *****************************************//
//! @name Receive Variables
//! These variables are used in the receiving of data on the \ref comm Module.
//! @{
//! \var volatile uint8 g_ucaRXBuffer[MAXMSGLEN]
//! \brief The software UART RX Buffer
volatile uint8 g_ucaRXBuffer[MAXMSGLEN];

//! \var volatile uint8 g_ucRXBufferIndex
//! \brief This index into g_ucaRXBuffer showing the current write position.
volatile uint8 g_ucRXBufferIndex;

//! \var uint8 g_ucRXBitsLeft
//! \brief The number of bits left to be received for the current byte.
uint8 g_ucRXBitsLeft;

//! \var uint8 g_ucRXParityBit
//! \brief Even Parity for bit banging uart
uint8 ucRXParityBit;
//! @}

//******************  Functions  ********************************************//
///////////////////////////////////////////////////////////////////////////////
//! \brief This sets up the hardware resources for doing software UART
//!
//! Since we are doing UART without the USCI, we use TimerA and it's interrupt
//! to control the baud rate. The RX a pins are completely controllable
//! at compile time. The software UART expects 1 start bit, 8 data bits and
//! 1 stop bit.
//!
//! To ensure correct operation of the software UART, the \ref comm_pins
//! "Comm Pin Defines" must be set correctly.
//!   \param ucBaud The baud rate define to use
//!   \return None
//!   \sa vCOMM_SendByte(), TIMERA0_ISR()
///////////////////////////////////////////////////////////////////////////////
void vCOMM_Init()
{



	// Configure UART pins
	P1->SEL0 |= BIT2 | BIT3;                  // set 2-UART pin as second function




	NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31); // Enable eUSCIA0 interrupt in NVIC module

	// Configure UART
	EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST;
	EUSCI_A0->CTLW0 |= EUSCI_B_CTLW0_SSEL__SMCLK;             // Put eUSCI in reset


	//9600 @ 16MHz
	UCA0BR0 = 0x02;
	UCA0BR1 = 0x00;
	EUSCI_A0->MCTLW = 0xBB00 | 0x0020 | EUSCI_A_MCTLW_OS16;

	EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;                  // Initialize eUSCI

	EUSCI_A0->IE |= EUSCI_A_IE_RXIE;                       // Enable USCI_A0 RX interrupt

	vCOMM_ResetBuffer();

	g_ucCOMM_Flags = COMM_RUNNING;


}

///////////////////////////////////////////////////////////////////////////////
//! \brief Clears the RX message buffer
//!
//! Set the index to the end of the buffer and count down to zero while
//! setting the buffer to 0xFF and then resetting the index to 0
//!
//!   \param None
//!   \return None
///////////////////////////////////////////////////////////////////////////////
void vCOMM_ResetBuffer(){
	// Clear the RX buffer and reset index
	g_ucRXBufferIndex = MAXMSGLEN;
	while (g_ucRXBufferIndex) {
		g_ucRXBufferIndex--;
		g_ucaRXBuffer[g_ucRXBufferIndex] = 0xFF;
	}

	g_ucRXBufferIndex = 0x00;


}

///////////////////////////////////////////////////////////////////////////////
//! \brief Sends a byte via the software UART
//!
//!   \param ucTXChar The 8-bit value to send
//!   \return None
///////////////////////////////////////////////////////////////////////////////
uint8 ucCOMM_SendByte(uint8 ucTXChar)
{

	// If we are already busy, return
	if (g_ucCOMM_Flags & COMM_TX_BUSY)
		return COMM_ERROR;

	// Indicate in the status register that we are now busy
	g_ucCOMM_Flags |= COMM_TX_BUSY;

	//Wait for TX linewhile (!(EUSCI_A0->IFG&EUSCI_A_IFG_TXIFG));
	while (!(EUSCI_A0->IFG&EUSCI_A_IFG_TXIFG));
	//Transmit the byte
	EUSCI_A0->TXBUF = ucTXChar;

	//Unset the comm_tx_busy flag
	g_ucCOMM_Flags &= ~COMM_TX_BUSY;

	return COMM_OK;

}


///////////////////////////////////////////////////////////////////////////////
//! \brief Shuts off the software modules
//!
//! This shuts down TimerA and disables all of the interrupts used
//!   \param None
//!   \return None
//!   \sa vCOMM_Init()
///////////////////////////////////////////////////////////////////////////////
void vCOMM_Shutdown(void)
{
	//Update comm status
	g_ucCOMM_Flags &= ~COMM_RUNNING;

	// Turn off UART pins
	P1->SEL0 &= ~(BIT2 | BIT3);

	// Disable USCI_A0 RX interrupt
	EUSCI_A0->IE &= ~EUSCI_A_IE_RXIE;

}


///////////////////////////////////////////////////////////////////////////////
//! \brief Sends a data message on the serial port
//!
//! This function sends the data message pointed to by \e p_DataMessage on the
//! software UART line
//!   \param p_DataMessage Pointer to the message to send
//!   \return None
///////////////////////////////////////////////////////////////////////////////.
void vCOMM_SendMessage(volatile uint8 * pBuff, uint8 ucLength)
{
	uint8 ucLoopCount;
	uint8 ucErrorCount;

	// Clear error count
	ucErrorCount = 0;

	// add the CRC bytes to the length
	ucLength += CRC_SZ;

	// Compute the CRC of the message
	ucCRC16_compute_msg_CRC(CRC_FOR_MSG_TO_SEND, pBuff, ucLength);

	for (ucLoopCount = 0x00; ucLoopCount < ucLength; ucLoopCount++) {

		// Attempt to send a byte
		if (ucCOMM_SendByte(*pBuff++) != COMM_OK) {

			// If there is an error then increment the error count
			ucErrorCount++;

			// Decrement the loop count to attempt to resend the byte
			ucLoopCount--;

			// If the error count reaches 5 then consider this a failure
			if (ucErrorCount == 5)
				break;
		}

	}
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Grabs the raw chars from buffer and formats into a data message
//!
//! This function takes the characters from \e g_ucaRXBuffer and
//! stores them in the data message pointed to by \e message.
//!   \param message Pointer to the message
//!   \return The error code indicating the status after call
//!   \sa comm.h msg.h
///////////////////////////////////////////////////////////////////////////////
uint8 ucCOMM_GrabMessageFromBuffer(volatile uint8 * pucBuff)
{
	uint8 ucLoopCount;
	uint8 ucLength;
	if (g_ucRXBufferIndex < SP_HEADERSIZE){
		return COMM_BUFFER_UNDERFLOW;
	}

	// Read the length of the message
	ucLength = g_ucaRXBuffer[MSG_LEN_IDX];

	// If the message is too long return error
	if (ucLength > MAXMSGLEN){
		return COMM_BUFFER_OVERFLOW;
	}

	// Check the CRC of the message
	if (!ucCRC16_compute_msg_CRC(CRC_FOR_MSG_TO_REC, g_ucaRXBuffer, g_ucaRXBuffer[MSG_LEN_IDX] + CRC_SZ)){
		return COMM_ERROR;
	}

	for (ucLoopCount = 0x00; ucLoopCount < ucLength; ucLoopCount++){
		*pucBuff++ = g_ucaRXBuffer[ucLoopCount];
	}



	return COMM_OK;
}

//! @}
//! @}

