///////////////////////////////////////////////////////////////////////////////
//! \file flash.h
//! \brief Header file for the flash module
//!
//!

#ifndef FLASH_H_
#define FLASH_H_


//! \def HID_ADDRESS
//! \brief Location within the HID sector that the first byte of the HID is stored
#define HID_ADDRESS		0x0003F000


uint8 ucFlash_SetHID(uint8 *uiHID);
void ucFlash_GetHID(uint8 *uiHID);
#endif /* FLASH_H_ */

