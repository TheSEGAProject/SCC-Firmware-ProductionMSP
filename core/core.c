///////////////////////////////////////////////////////////////////////////////
//! \file core.c
//! \brief This is the primary file for the SP Board core
//!
//! This file contains the heart of the SP Board core. The function pointer
//! and label tables are kept here and maintained by the core.
//!
//! @addtogroup core Core
//! The Core Module handles all of the communication to the CP board as well
//! as acts as the supervisor to all activities on the SP Board. The user
//! built wrapper should interface with the Core Module as documented.
//! @{
///////////////////////////////////////////////////////////////////////////////
//*****************************************************************************
// By: Kenji Yamamoto
//     Wireless Networks Research Lab
//     Dept of Electrical Engineering, CEFNS
//     Northern Arizona University
//
//	Modified by: Chris Porter
//
//*****************************************************************************

#include "msp.h"
#include "core.h"
#include "clock.h"
#include "comm/crc.h"
#include "comm/msg.h"
#include "flash.h"
#include "../helpers/led.h"
#include "driverlib.h"
#include <stdio.h>


#define HID_START 0x0003F000





//******************  Software version variables  ***************************//
//! @name Software Version Variables
//! These variables contain ID and version information.
//! @{
//! \var static const uint8 g_ucaCoreVersion[VERSION_LABEL_LEN]
//! \brief The name and version of the core
#define VERSION_LABEL "SCC-Core v1.00   "
//! @}

//! \var uiHID
//! \brief Variable holds the unique SCC ID as a byte array
uint8 uiHID[8];

extern volatile uint8 g_ucaRXBuffer[MAXMSGLEN];
extern volatile uint8 g_ucRXBufferIndex;
extern volatile float fBatteryVoltage;
extern volatile float fSolarVoltage;
extern volatile float fAverageCurrent;
extern volatile float fAverageVoltage;
extern volatile uint8 ucADC_COMPLETE;
extern volatile uint8 ucFaultDetected;

uint8 ucCommError = 0;
//******************  Functions  ********************************************//
///////////////////////////////////////////////////////////////////////////////
//! \brief This function starts up the Core and configures hardware & RAM
//!
//! All of the clock setup and initial port configuration is handled in this
//! function. At the end of the function, before the return, any additional
//! core initilization functions are called.
//!   \param None.
//!   \return None.
///////////////////////////////////////////////////////////////////////////////
void vCORE_Initialize(void)
{
	// First, stop the watchdog
	WDT_A->CTL = WDT_A_CTL_PW + WDT_A_CTL_HOLD;



	// Terminate all pins on the device
	P1->DIR  |= 0xFF; P1->OUT  = 0x00;
	P2->DIR  |= 0xFF; P2->OUT  = 0x00;
	P3->DIR  |= 0xFF; P3->OUT  = 0x00;
	P4->DIR  |= 0xFF; P4->OUT  = 0x00;
	P5->DIR  |= 0xFF; P5->OUT  = 0x00;
	P6->DIR  |= 0xFF; P6->OUT  = 0x00;
	P7->DIR  |= 0xFF; P7->OUT  = 0x00;
	P8->DIR  |= 0xFF; P8->OUT  = 0x00;
	P9->DIR  |= 0xFF; P9->OUT  = 0x00;
	P10->DIR |= 0xFF; P10->OUT = 0x00;


	//Configure the m clock for 4MHz
	vMCLK_16MHzCAL();

	// Get the SCCs serial number from flash
	ucFlash_GetHID(uiHID);

	//Init UART for SCC-WiSARD communication
	vCOMM_Init();

	SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;           //Resume operation after returning from interrupt


	// Ensures SLEEPONEXIT takes effect immediately
	__DSB();

	// Enable global interrupt
	__enable_irq();

	NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31);

	TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
	TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;                          // TACCR0 interrupt enabled
	TIMER_A0->CCR[0] = 32768;
	TIMER_A0->CTL |= TIMER_A_CTL_SSEL__ACLK | TIMER_A_CTL_MC__UP | TIMER_A_CTL_CLR;   // ACLK, UP mode

	/* Configure watch dog
	 * - ACLK as clock source
	 * - Watchdog mode
	 * - Clear WDT counter (initial value = 0)
	 * - Timer interval = 16s @ 32kHz ACLK
	 */
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_SSEL__ACLK | WDT_A_CTL_CNTCL | WDT_A_CTL_IS_3;






}

///////////////////////////////////////////////////////////////////////////////
//! \brief Measure the MSP430 supply voltage
//!
//! Uses the ADC12 to measure the input voltage. Uses the MEM15 register.
//!
//!   \param none
//!
//!   \return unsigned int Input voltage * 100
///////////////////////////////////////////////////////////////////////////////
/*
unsigned int unCORE_GetVoltage(void)
{
	int rt_volts;

	ADC12CTL0 &= ~(SHT10 + SHT12 + SHT13 + MSC + ADC12OVIE + ADC12TOVIE + ENC + ADC12SC); //ADC12CTL0 &= ~0xD08F = ~1101 0000 1000 1111 //Have to turn ENC Off first
	ADC12CTL0 |= (SHT11 + REF2_5V + REF_A_CTL0_ON + ADC12ON); //ADC12CTL0 |= 0x2070 = 0010 xxxx 0111 00(11)* - 16-Cycle Hold time + Single Conversion + 2.5V Ref + RefON + ADC ON + Interrupts off + (Enable + Start)
	ADC12CTL1 &= ~(SHS1 + SHS0 + ISSH + ADC12DIV2 + ADC12DIV1 + ADC12DIV0 + ADC12SSEL1 + ADC12SSEL0 + CONSEQ1 + CONSEQ0); //ADC12CTL1 &= ~0x0FDE = ~0000 1101 1111 1110
	ADC12MEM15 = 0;
	ADC12MCTL15 |= (SREF0 + INCH3 + INCH1 + INCH0); //ADC12MCTL15 |= 0x1B = x001 1011 - Reference Select + Input Select
	ADC12MCTL15 &= ~(SREF2 + SREF1 + INCH2); // ADC12MCTL15 &= ~0x64 = 0110 0100
	ADC12IE &= ~0x8000; //Turn off IE and clear IFG
	ADC12IFG &= ~0x8000;

	__delay_cycles(1000);

	ADC12CTL1 |= (CSTARTADD3 + CSTARTADD2 + CSTARTADD1 + CSTARTADD0 + SHP); //ADC12CTL1 |= 0xF200 = 1111 0010 0000 000x - MEM15 + Internal OSC CLK + Single-Channel, Single-conversion
	ADC12CTL0 |= ENC + ADC12SC; // Sampling and conversion start

	while (!(ADC12IFG & 0x8000)); //End when something is written in. Can't sleep because we wanted to keep interrupts for users (not in core)

	rt_volts = ADC12MEM15; //(0.5*Vin)/2.5V * 4095
	ADC12IFG &= ~0x8000; //Unset IFG Flag
	ADC12CTL0 &= ~ENC;
	ADC12CTL0 &= ~(REF_A_CTL0_ON + ADC12ON); // turn off A/D to save power

	rt_volts = (rt_volts * 5) / 41;
	return (rt_volts);
}
 */

///////////////////////////////////////////////////////////////////////////////
//! \brief Send the confirm packet
//!
//!  Confirm packet includes all the data received so CP Board can confirm it
//!  is correct.
//!
//!   \param none
//!   \sa core.h
///////////////////////////////////////////////////////////////////////////////
vCORE_Send_ConfirmPKT()
{
	uint8 ucaMsg_Buff[MAXMSGLEN];

	// Send confirm packet that we received message
	ucaMsg_Buff[MSG_TYP_IDX] = CONFIRM_COMMAND;
	ucaMsg_Buff[MSG_LEN_IDX] = SP_HEADERSIZE;
	ucaMsg_Buff[MSG_VER_IDX] = SP_DATAMESSAGE_VERSION;
	ucaMsg_Buff[MSG_FLAGS_IDX] = 0;
	// Send the message
	vCOMM_SendMessage(ucaMsg_Buff, ucaMsg_Buff[MSG_LEN_IDX]);
}

///////////////////////////////////////////////////////////////////////////////
//! \brief Send the confirm packet
//!
//!  Confirm packet includes all the data received so CP Board can confirm it
//!  is correct.
//!
//!   \param none
//!   \sa core.h
///////////////////////////////////////////////////////////////////////////////
void vCORE_Send_ErrorMsg(uint8 ucErrMsg)
{
	uint8 ucaMsg_Buff[MAXMSGLEN];

	// Send confirm packet that we received message
	ucaMsg_Buff[MSG_TYP_IDX] = REPORT_ERROR;
	ucaMsg_Buff[MSG_LEN_IDX] = 5;
	ucaMsg_Buff[MSG_VER_IDX] = SP_DATAMESSAGE_VERSION;
	ucaMsg_Buff[MSG_PAYLD_IDX] = ucErrMsg;
	ucaMsg_Buff[MSG_FLAGS_IDX] = 0;

	// Send the message
	vCOMM_SendMessage(ucaMsg_Buff, ucaMsg_Buff[MSG_LEN_IDX]);
}


void setP1_2Int(void)
{
	P1->SEL0 &= ~BIT2;//P1.2 as IO
	P1->DIR &= ~BIT2;//P1.2 as input
	P1->OUT |= BIT2;//P1.2 pull up
	P1->REN |= BIT2;//P1.2 resistor enable
	P1->IES |= BIT2;//High-to int
	P1->IFG &= ~BIT2;//clear flag
	P1->IE |= BIT2;//enable int

	// Enable Port1 interrupt in NVIC
	NVIC->ISER[1] = 1 << ((PORT1_IRQn) & 31);
}



///////////////////////////////////////////////////////////////////////////////
//! \brief This functions runs the core
//!
//! This function runs the core. This function does not return, so all of the
//! core setup and init must be done before the call to this function. The
//! function waits for a data packet from the CP Board, then parses and handles
//! it appropriately. A response packet is then sent and the core waits for
//! the next data packet.
//!   \param None.
//!   \return NEVER. This function never returns
//!   \sa msg.h
///////////////////////////////////////////////////////////////////////////////
void vCORE_Run(void)
{
	uint16 unTransducerReturn; //The return parameter from the transducer function
	uint8 ucaMsg_Buff[MAXMSGLEN];
	uint8 ucMsgBuffIdx;
	uint8 ucCmdTransNum;
	uint8 ucCmdParamLen;
	uint8 ucParamCount;
	uint8 ucParam[20];
	uint8 ucCommState;
	uint8 ucRXMessageSize = SP_HEADERSIZE;


	__no_operation();


	// The primary execution loop
	while(1)
	{

		//Pet the dog
		WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_SSEL__ACLK | WDT_A_CTL_CNTCL | WDT_A_CTL_IS_3;


		//Go to sleep
		__sleep();

		//UART RX interrupt or timer A woke us up
		//Check if full message was received
		if(g_ucRXBufferIndex == SP_HEADERSIZE){
			//set the message size to the message length in the packet and add the crc length
			ucRXMessageSize = g_ucaRXBuffer[MSG_LEN_IDX] + CRC_SZ;
			//check if the message size is invalid
			if(ucRXMessageSize > MAXMSGLEN){
				//if we got a bad message size, reset the buffer
				if(debug){
					printf("\r\nBuffer reset max message size\r\n");
				}
				vCOMM_ResetBuffer();

			}
		}
		else if(ucRXMessageSize != SP_HEADERSIZE && g_ucRXBufferIndex == ucRXMessageSize){
			//We RX'd the full message, get the state
			ucCommState = ucCOMM_GrabMessageFromBuffer(ucaMsg_Buff);
			// Reset the RX index
			g_ucRXBufferIndex = 0x00;
			// Pull the message from the RX buffer and load it into a local buffer
			if (ucCommState == COMM_OK)
			{
				//Switch based on the message type
				switch (ucaMsg_Buff[MSG_TYP_IDX])
				{
				//////////////////////////////////////////////////////////////////
				//
				//		COMMAND_PKT
				//
				//////////////////////////////////////////////////////////////////
				case COMMAND_PKT:

					// Send a confirmation packet
					vCORE_Send_ConfirmPKT();
					unTransducerReturn = 0; //default return value to 0

					// Read through the length of the message and execute commands as they are read
					for (ucMsgBuffIdx = MSG_PAYLD_IDX; ucMsgBuffIdx < ucaMsg_Buff[MSG_LEN_IDX];)
					{
						// Get the transducer number and the parameter length
						ucCmdTransNum = ucaMsg_Buff[ucMsgBuffIdx++];
						ucCmdParamLen = ucaMsg_Buff[ucMsgBuffIdx++];

						for (ucParamCount = 0; ucParamCount < ucCmdParamLen; ucParamCount++)
						{
							ucParam[ucParamCount] = ucaMsg_Buff[ucMsgBuffIdx++];
						}

						// Dispatch to perform the task, pass all values needed to populate the data
						unTransducerReturn |= uiMainDispatch(ucCmdTransNum, ucCmdParamLen, ucParam);
					}

					break; //END COMMAND_PKT

					//////////////////////////////////////////////////////////////////
					//
					//		REQUEST_DATA
					//
					//////////////////////////////////////////////////////////////////
				case REQUEST_DATA:
					// Stuff the header
					ucaMsg_Buff[MSG_FLAGS_IDX] = 0;
					ucaMsg_Buff[MSG_VER_IDX] = SP_DATAMESSAGE_VERSION;

					//Make sure the ADC readings are complete
					if (ucADC_COMPLETE == 1){
						ucaMsg_Buff[MSG_TYP_IDX] = REPORT_DATA;
						// Load the message buffer with data.  The fetch function returns length
						ucaMsg_Buff[MSG_LEN_IDX] = SP_HEADERSIZE + ucMain_FetchData(&ucaMsg_Buff[MSG_PAYLD_IDX]);

					}
					else{
						ucaMsg_Buff[MSG_TYP_IDX] = REPORT_ERROR;
						ucaMsg_Buff[MSG_LEN_IDX] = SP_HEADERSIZE;
					}
					// Send the message
					vCOMM_SendMessage(ucaMsg_Buff, ucaMsg_Buff[MSG_LEN_IDX]);
					if(debug){
						vLED_Blink_Delay(6,SHORT_DELAY);

						//Check for fault
						if(ucFaultDetected){
							printf("\r\n===== FAULT DETECTED =====\r\n");
						}
					}
					//Reset the fault detection flag
					ucFaultDetected = 0x00;
					if(debug){
						//If we're debugging, print the ADC readings
						printf("%f,",fBatteryVoltage); //Battery Voltage
						printf("%f,",fSolarVoltage); //Solar Vin Voltage
						//printf("%f,",fAverageVoltage);
						printf("%f\r\n",fAverageCurrent*1000); //Solar Vin Current
						if(ucCommError != 0){
							printf("Comm Error: %d\r\n",ucCommError);
						}

						debug_counter++;
						if(debug_counter > 24){ //two minutes at 5 sec samples
							printf("\r\nDebug completed\r\n");
							debug = 0;
						}
					}



					break; //END REQUEST_DATA
					//////////////////////////////////////////////////////////////////
					//
					//		REQUEST_LABEL
					//
					//////////////////////////////////////////////////////////////////
				case REQUEST_LABEL:
					//					// Format first part of return message
					//					ucaMsg_Buff[MSG_TYP_IDX] = REPORT_LABEL;
					//					ucaMsg_Buff[MSG_LEN_IDX] = SP_HEADERSIZE;// + TRANSDUCER_LABEL_LEN;
					//					ucaMsg_Buff[MSG_VER_IDX] = SP_LABELMESSAGE_VERSION;
					//
					//					// Make call to main for the trans. labels.  This way the core is not constrained to a fixed number of transducers
					//					vMain_FetchLabel(ucaMsg_Buff[3], &ucaMsg_Buff[3]);
					//
					//					// Send the label message
					//					vCOMM_SendMessage(ucaMsg_Buff, ucaMsg_Buff[MSG_LEN_IDX]);
					break; //END REQUEST_LABEL
					//////////////////////////////////////////////////////////////////
					//
					//		INTERROGATE
					//
					//////////////////////////////////////////////////////////////////
				case INTERROGATE:
					//Load the return message buffer
					ucaMsg_Buff[MSG_TYP_IDX] = INTERROGATE;
					ucaMsg_Buff[MSG_LEN_IDX] = 17;  //header and ID packet length
					ucaMsg_Buff[MSG_VER_IDX] = SP_DATAMESSAGE_VERSION;
					ucaMsg_Buff[MSG_FLAGS_IDX] = 0;
					ucMsgBuffIdx = MSG_PAYLD_IDX;
					ucaMsg_Buff[ucMsgBuffIdx++] = 0x02; 	//Two "transducers"
					ucaMsg_Buff[ucMsgBuffIdx++] = 0x53; 	//Sensor - ADC readings
					ucaMsg_Buff[ucMsgBuffIdx++] = 0x02; 	//2 second duration
					ucaMsg_Buff[ucMsgBuffIdx++] = 0x41;	//Actuator -forecast data
					ucaMsg_Buff[ucMsgBuffIdx++] = 0x00;	//Less than a second duration

					// Load the board name into the message buffer
					//NOTE: This is not the HID
					ucaMsg_Buff[ucMsgBuffIdx++] = ID_PKT_HI_BYTE1;
					ucaMsg_Buff[ucMsgBuffIdx++] = ID_PKT_LO_BYTE1;
					ucaMsg_Buff[ucMsgBuffIdx++] = ID_PKT_HI_BYTE2;
					ucaMsg_Buff[ucMsgBuffIdx++] = ID_PKT_LO_BYTE2;
					ucaMsg_Buff[ucMsgBuffIdx++] = ID_PKT_HI_BYTE3;
					ucaMsg_Buff[ucMsgBuffIdx++] = ID_PKT_LO_BYTE3;
					ucaMsg_Buff[ucMsgBuffIdx++] = ID_PKT_HI_BYTE4;
					ucaMsg_Buff[ucMsgBuffIdx  ] = ID_PKT_LO_BYTE4;

					// Send the message
					vCOMM_SendMessage(ucaMsg_Buff, ucaMsg_Buff[MSG_LEN_IDX]);
					break;
					//////////////////////////////////////////////////////////////////
					//
					//		SET_SERIALNUM
					//
					//////////////////////////////////////////////////////////////////
				case SET_SERIALNUM:

					//Load the new HID into memory
					ucMsgBuffIdx = MSG_PAYLD_IDX;
					uiHID[0] = ucaMsg_Buff[ucMsgBuffIdx++];
					uiHID[1] = ucaMsg_Buff[ucMsgBuffIdx++];

					uiHID[2] = ucaMsg_Buff[ucMsgBuffIdx++];
					uiHID[3] = ucaMsg_Buff[ucMsgBuffIdx++];

					uiHID[4] = ucaMsg_Buff[ucMsgBuffIdx++];
					uiHID[5] = ucaMsg_Buff[ucMsgBuffIdx++];

					uiHID[6] = ucaMsg_Buff[ucMsgBuffIdx++];
					uiHID[7] = ucaMsg_Buff[ucMsgBuffIdx++];

					// Write the return message header assuming success
					ucaMsg_Buff[MSG_TYP_IDX] = SET_SERIALNUM;
					ucaMsg_Buff[MSG_LEN_IDX] = 12;
					ucaMsg_Buff[MSG_VER_IDX] = SP_DATAMESSAGE_VERSION;


					// Write the new HID to flash
					if (ucFlash_SetHID(uiHID))
					{
						// Report an error if the write was unsuccessful
						ucaMsg_Buff[MSG_TYP_IDX] = REPORT_ERROR;
						ucaMsg_Buff[MSG_LEN_IDX] = 3;
					}
					else
					{
						// Get the SCCs serial number from flash
						ucFlash_GetHID(uiHID);

						ucMsgBuffIdx = MSG_PAYLD_IDX;
						// Write the new HID to the message buffer
						ucaMsg_Buff[ucMsgBuffIdx++] = uiHID[0];
						ucaMsg_Buff[ucMsgBuffIdx++] = uiHID[1];
						ucaMsg_Buff[ucMsgBuffIdx++] = uiHID[2];
						ucaMsg_Buff[ucMsgBuffIdx++] = uiHID[3];
						ucaMsg_Buff[ucMsgBuffIdx++] = uiHID[4];
						ucaMsg_Buff[ucMsgBuffIdx++] = uiHID[5];
						ucaMsg_Buff[ucMsgBuffIdx++] = uiHID[6];
						ucaMsg_Buff[ucMsgBuffIdx  ] = uiHID[7];
					}

					// Send the message
					vCOMM_SendMessage(ucaMsg_Buff, ucaMsg_Buff[MSG_LEN_IDX]);
					break;

					//////////////////////////////////////////////////////////////////
					//
					//		COMMAND_SENSOR_TYPE
					//
					//////////////////////////////////////////////////////////////////
				case COMMAND_SENSOR_TYPE:
					//				{
					//					uint8 ucChannel;
					//
					//					// loop through each channel, get sample, and assign type
					//					for(ucChannel = 1; ucChannel < 5; ucChannel++)
					//					{
					//						// command the retrieval of sensor type
					//						vMAIN_RequestSensorType(ucChannel);
					//					}
					//				}
					break;

					// The CP requests the sensor types from the SPs
					//////////////////////////////////////////////////////////////////
					//
					//		REQUEST_SENSOR_TYPE
					//
					//////////////////////////////////////////////////////////////////
				case REQUEST_SENSOR_TYPE:
					//				{
					//					uint8 retVal = 0;
					//					uint8 ucSensorTypes[4];
					//					uint8 ucSensorCount = 0;
					//
					//					// Format first part of return message
					//					ucaMsg_Buff[MSG_TYP_IDX] = 0x0D;
					//					ucaMsg_Buff[MSG_LEN_IDX] = SP_HEADERSIZE + 2;
					//					ucaMsg_Buff[MSG_VER_IDX] = SP_DATAMESSAGE_VERSION;
					//
					//					// Loop through sensors and place types on msg buffer
					//					for(ucSensorCount = 1; ucSensorCount < 5; ucSensorCount++)
					//					{
					//						// get sensor type for a specific channel
					//						retVal = ucMAIN_ReturnSensorType(ucSensorCount);
					//
					//						// otherwise, register type
					//						ucSensorTypes[ucSensorCount - 1] = retVal;
					//					}
					//
					//					// put sensor types on buffer
					//					for(ucSensorCount = 0; ucSensorCount < 4; ucSensorCount++)
					//					{
					//						ucaMsg_Buff[ucSensorCount + 3] = ucSensorTypes[ucSensorCount];
					//					}
					//
					//					// Send the sensor types message
					//					vCOMM_SendMessage(ucaMsg_Buff, ucaMsg_Buff[MSG_LEN_IDX]);
					//				}
					break;

				default:
					//Load a report message into the buffer
					ucaMsg_Buff[MSG_TYP_IDX] = REPORT_ERROR;
					ucaMsg_Buff[MSG_LEN_IDX] = SP_HEADERSIZE;
					ucaMsg_Buff[MSG_VER_IDX] = SP_DATAMESSAGE_VERSION;
					ucaMsg_Buff[MSG_FLAGS_IDX] = 0;

					// Send the message
					vCOMM_SendMessage(ucaMsg_Buff, ucaMsg_Buff[MSG_LEN_IDX]);

					break; //END default
				} // END: switch

				ucCommError = 0;

			}// END: if(comm is ok)
			else
			{
				vCOMM_ResetBuffer();
				//Send an error message to alert the CP there was a comm error
				vCORE_Send_ErrorMsg(ucCommState);
				ucCommError = ucCommState;
				vCOMM_ResetBuffer();

				if(debug){
					debug_counter++;
					if(debug_counter > 24){ //two minutes at 5 sec samples
						debug = 0;
					}
				}



			} // END: if(ucCOMM_GrabMessageFromBuffer)


			ucRXMessageSize = SP_HEADERSIZE; //Reset ucRXMessageSize


		}// END: if(rx'd full message)



	} // END: while(TRUE)
}

//! @}
