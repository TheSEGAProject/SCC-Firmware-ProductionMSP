////////////////////////////////////////////////////////////////////////
//!	\file flash.c
//! \addtogroup HAL
//! @{
//!	\brief Used to read and write to flash memory
//!
//! The procedure for reading and writing to flash is as follows:
//! 1. Read the segment being written to into a local variable
//! 2. Edit the local copy of the segment
//! 3. Erase the segment in flash
//! 4. Write local copy of the segment back to flash
//!
////////////////////////////////////////////////////////////////////////

#include "msp.h"
#include "core.h"
#include "flash.h"
#include "driverlib.h"




////////////////////////////////////////////////////////////////////////////////
//! \fn ucFlash_SetHID
//! \brief Sets the hardware ID in flash.  The hardware ID is unique for every board and
//! is set before deployment.
//!
//! \param ucHID
//! \return 0 - success
//!			1 - error
////////////////////////////////////////////////////////////////////////////////
uint8 ucFlash_SetHID(uint8 *uiHID)
{

		//Set the flash wait state
		MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);


		/* Unprotecting Info Bank 0, Sector 0  */
		MAP_FlashCtl_unprotectSector(FLASH_MAIN_MEMORY_SPACE_BANK1,FLASH_SECTOR31);

		/* Trying to erase the sector. Within this function, the API will
			automatically try to erase the maximum number of tries. If it fails,
			 trap in an infinite loop */
		if(!MAP_FlashCtl_eraseSector(HID_ADDRESS))
		{
			return 1;
		}

		/* Trying to program the memory. Within this function, the API will
			automatically try to program the maximum number of tries. If it fails,
			trap inside an infinite loop */
		if(!MAP_FlashCtl_programMemory(uiHID,(void*) HID_ADDRESS, 8))
		{
			return 1;
		}

		/* Setting the sector back to protected  */
		MAP_FlashCtl_protectSector(FLASH_MAIN_MEMORY_SPACE_BANK1,FLASH_SECTOR31);

		__no_operation();



		return 0;






	/*
	ulong ulSegmentData[32];
	uint uiIndex;
	ulong ulTemp;

	vFlash_Read_Segment(ulSegmentData, HID_SECTOR);

	// Write the HID to the local variable
	for(uiIndex = HID_ADDRESS; uiIndex < (HID_ADDRESS + 2); uiIndex++)
	{
		ulTemp = *uiHID++;
		ulTemp = ulTemp << 16;
		ulTemp |= *uiHID++;
		ulSegmentData[uiIndex] = ulTemp;
	}

	// Write the variable to flash
	vFlash_Write_Segment(ulSegmentData, HID_SECTOR);
	*/

}


////////////////////////////////////////////////////////////////////////////////
//! \fn vFlash_GetHID
//! \brief Reads the hardware ID from flash.
//!
//! \param *uiHID
//! \return none
////////////////////////////////////////////////////////////////////////////////
void  ucFlash_GetHID(uint8 *uiHID)
{
	uint8 ucIndex;
	// Loop through flash and get the
	for(ucIndex = 0; ucIndex < 8; ucIndex++){
		uiHID[ucIndex] = *(uint8*)(ucIndex+HID_ADDRESS);
	}

	/*
	ulong ulSegmentData[32];
	uint uiIndex;

	vFlash_Read_Segment(ulSegmentData, HID_SECTOR);

	// Read the HID from the segment
	for(uiIndex = HID_ADDRESS; uiIndex < (HID_ADDRESS + 2); uiIndex++)
	{
		*uiHID++ = (uint)(ulSegmentData[uiIndex] >> 16);
		*uiHID++ = (uint)ulSegmentData[uiIndex];
	}
	*/
}


//! @}


