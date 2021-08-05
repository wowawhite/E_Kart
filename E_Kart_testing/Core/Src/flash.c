/*
 * flash.c
 *
 *  Created on: 29.11.2018
 *      Author: melf_
 */

#include "flash.h"
#include "Parameter.h"

// variables here
uint16_t Flash_Memory_List[NumberOfSavedParameter];
uint16_t Flash_New_Parameters_List[NumberOfSavedParameter];

uint16_t *Pointer_GasProzent=&Flash_Memory_List[Anfaenger_GasProzent];	// Parameter dient zur Touch-Eingabe, Wertbereich (0-100)=(0%-100%), dessen Initialwert ist 100%
															// als der Beiwert f�r Einstellung des Wirkungsbereichs von Gaspedal(bei Drehzahlvorgabe_mode )
                              // z.B wenn "Pointer_GasProzent=100", Gaspedal volldurchgedr�ckt, die max.Drehzahl=3000 wird durch CAN-BUS vorgegeben
                              // wenn "Pointer_GasProzent=50", volldurchgedr�ckt, dann wird nur die Drehzahl=1500 vorgegeben.
															// der Wirkungsbereich halbiert sich

uint16_t *Pointer_BeschlProzent=&Flash_Memory_List[Anfaenger_BeschlProzent];   // Parameter dient zur Touch-Eingabe, Wertbereich (0-100)=(0%-100%), dessen Initialwert ist 100%					--------------------------------------
uint16_t *Pointer_Rueckwaert=&Flash_Memory_List[Anfaenger_Rueckwaert];

//uint16_t test1;

uint32_t FirstSector = 0, NbOfSectors = 0, Address = 0;
uint32_t SectorError = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;
static FLASH_EraseInitTypeDef EraseInitStruct;
//static FLASH_ErrorTypeDef errorcode;

uint32_t *FlashMemoryPointer=0;

// function prototypes here
static uint32_t GetSector(uint32_t Address);
static uint32_t GetSectorSize(uint32_t Sector);

// function definitions here
void flash_init(void)
{
	Flash_New_Parameters_List[ChosenDrivingMode]=AnfaengerModeButton;
	  Flash_New_Parameters_List[VorgabeMomentDrehzahl]=VorgabeMoment;

	  Flash_New_Parameters_List[Anfaenger_GasProzent]=40;
	  Flash_New_Parameters_List[Anfaenger_BeschlProzent]=45;
	  Flash_New_Parameters_List[Anfaenger_Rueckwaert]=30;

	  Flash_New_Parameters_List[Standard_GasProzent]=70;
	  Flash_New_Parameters_List[Standard_BeschlProzent]=50;
	  Flash_New_Parameters_List[Standard_Rueckwaert]=40;

	  Flash_New_Parameters_List[Profi_GasProzent]=100;
	  Flash_New_Parameters_List[Profi_BeschlProzent]=100;
	  Flash_New_Parameters_List[Profi_Rueckwaert]=50;

	  Flash_New_Parameters_List[Test_GasProzent]=100;
	  Flash_New_Parameters_List[Test_BeschlProzent]=100;
	  Flash_New_Parameters_List[Test_Rueckwaert]=50;

	  // die letzten Cursor-Positionen
	  Flash_New_Parameters_List[10]=170;
	  Flash_New_Parameters_List[11]=180;
	  Flash_New_Parameters_List[12]=230;
	  Flash_New_Parameters_List[13]=190;
	  Flash_New_Parameters_List[14]=290;
	  Flash_New_Parameters_List[15]=290;
	  Flash_New_Parameters_List[16]=250;
	  Flash_New_Parameters_List[17]=250;
	  Flash_New_Parameters_List[26]=150;
	  Flash_New_Parameters_List[27]=170;
	  Flash_New_Parameters_List[28]=190;
	  Flash_New_Parameters_List[29]=190;

	  Flash_New_Parameters_List[Motor_SDO]=0;
	  Flash_New_Parameters_List[Strom]=400;
	  Flash_New_Parameters_List[21]=290;
	  Flash_New_Parameters_List[INIT]=2345;

//	  /* Get the 1st sector to erase */
//	  FirstSector = GetSector(FLASH_USER_START_ADDR);
//	  /* Get the number of sector to erase from 1st sector*/
//	  NbOfSectors = GetSector(FLASH_USER_END_ADDR) - FirstSector + 1;
//
//	  /* Fill EraseInit structure*/
//	  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
//	  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
//	  EraseInitStruct.Sector = FirstSector;
//	  EraseInitStruct.NbSectors = NbOfSectors;
//	  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
//	//	  __HAL_FLASH_DATA_CACHE_DISABLE();
//	//	  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
//	//  HAL_FLASH_Unlock();
//	//  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
//	//  {
//	//    /*
//	//      Error occurred while sector erase.
//	//      User can add here some code to deal with this error.
//	//      SectorError will contain the faulty sector and then to know the code error on this sector,
//	//      user can call function 'HAL_FLASH_GetError()'
//	//    */
//	//    /*
//	//      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
//	//    */
//	//    Error_Handler();
//	//  }
//	//
//	//  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
//	//     you have to make sure that these data are rewritten before they are accessed during code
//	//     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
//	//     DCRST and ICRST bits in the FLASH_CR register. */
//	////  __HAL_FLASH_DATA_CACHE_DISABLE();
//	////  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
//	////
//	////  __HAL_FLASH_DATA_CACHE_RESET();
//	////  __HAL_FLASH_INSTRUCTION_CACHE_RESET();
//	////
//	////  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
//	////  __HAL_FLASH_DATA_CACHE_ENABLE();
//	//
//	//  /* Program the user Flash area word by word
//	//    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
//	//
//	//  Address = FLASH_USER_START_ADDR;
//	//
//	//  y=0;
//	////  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
//	//
//	//  while ((Address < FLASH_USER_END_ADDR) && (y<=NumberOfSavedParameter))
//	//  {
//	//
//	//    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address, Flash_Memory_List[y]) == HAL_OK)
//	//    {
//	//    	Address = Address + 2;
//	//    	y++;
//	//    }
//	//    else
//	//    {
//	//      /* Error occurred while writing data in Flash memory.
//	//         User can add here some code to deal with this error */
//	//      while (1)
//	//      {
//	////        BSP_LED_On(LED3);
//	//      }
//	//    }
//	//  }
//	////	  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
//	////	  __HAL_FLASH_DATA_CACHE_ENABLE();
//	//  HAL_FLASH_Lock();
//
//	  test1=*(uint32_t *)(FLASH_USER_START_ADDR);
//
//	  return test1;
}

void TransferListElements(int8_t x)
{
	int8_t y;
  if(x==1)
	{
		for(y=0; y<NumberOfSavedParameter; y++)
		{	Flash_Memory_List[y]=Flash_New_Parameters_List[y];		}
	}
	if(x==0)
	{
		for(y=0; y<NumberOfSavedParameter; y++)
		{	Flash_New_Parameters_List[y]=Flash_Memory_List[y];		}
	}
}

static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_SECTOR_8;
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_SECTOR_9;
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_SECTOR_10;
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
  {
    sector = FLASH_SECTOR_11;
  }

  return sector;
}

/**
  * @brief  Gets sector Size
  * @param  uint32_t Input target sector
  * @retval The size of a given sector
  */
static uint32_t GetSectorSize(uint32_t Sector)
{
  uint32_t sectorsize = 0x00;

  if((Sector == FLASH_SECTOR_0) || (Sector == FLASH_SECTOR_1) || (Sector == FLASH_SECTOR_2) || (Sector == FLASH_SECTOR_3))
  {
    sectorsize = 16 * 1024;
  }
  else if(Sector == FLASH_SECTOR_4)
  {
    sectorsize = 64 * 1024;
  }
  else
  {
    sectorsize = 128 * 1024;
  }
  return sectorsize;
}

void SavetoFlash(void)
{
	volatile uint32_t i=0;

	  /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();
	  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	  __HAL_FLASH_DATA_CACHE_DISABLE();
	  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

	  /* Erase the user Flash area
	    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	  /* Get the 1st sector to erase */
	  FirstSector = GetSector(FLASH_USER_START_ADDR);
	  /* Get the number of sector to erase from 1st sector*/
	  NbOfSectors = GetSector(FLASH_USER_END_ADDR) - FirstSector + 1;

	  /* Fill EraseInit structure*/
	  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	  EraseInitStruct.Sector = FirstSector;
	  EraseInitStruct.NbSectors = NbOfSectors;
	  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
	  {
	    /*
	      Error occurred while sector erase.
	      User can add here some code to deal with this error.
	      SectorError will contain the faulty sector and then to know the code error on this sector,
	      user can call function 'HAL_FLASH_GetError()'
	    */
//	    while (1)
//	    {
//	    }
	  }

//	  __HAL_FLASH_DATA_CACHE_DISABLE();
//	  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
//
//	  __HAL_FLASH_DATA_CACHE_RESET();
//	  __HAL_FLASH_INSTRUCTION_CACHE_RESET();
//
//	  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
//	  __HAL_FLASH_DATA_CACHE_ENABLE();

	Address=FLASH_USER_START_ADDR;
	while(Address < (FLASH_USER_START_ADDR+NumberOfSavedParameter*FlashPageNextPosition-1))
	{
	    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address, Flash_Memory_List[i]) == HAL_OK)
	    {
	      Address = Address + FlashPageNextPosition;
	      i++;
	    }
	    else
	    {
	      /* Error occurred while writing data in Flash memory.
	         User can add here some code to deal with this error */
//	      while (1)
//	      {
//	      }
	    }

	}
	  /* Lock the Flash to disable the flash control register access (recommended
	     to protect the FLASH memory against possible unwanted operation) *********/
	__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
	__HAL_FLASH_DATA_CACHE_ENABLE();
	  HAL_FLASH_Lock();
}

void LoadfromFlash(void) {
	volatile uint32_t i = 0;

	Address = FLASH_USER_START_ADDR;

	while(Address < (FLASH_USER_START_ADDR+NumberOfSavedParameter*FlashPageNextPosition-1))
	{
		Flash_New_Parameters_List[i] = *(__IO uint16_t*)Address;
		Address = Address + FlashPageNextPosition;
		i++;
	}

	if (Flash_New_Parameters_List[INIT] != 2345) {
		flash_init();
		TransferListElements(1);
		SavetoFlash();
	}

	TransferListElements(1);
	// pr�fe Flash_New_Parameters_List[INIT] == 2345
	// ja, dann alles ok
	// nein
	// dann init Flash_New_Parameters_List[ChosenDrivingMode]=2
	// Flash_New_Parameters_List[INIT] = 2345
	// und dann ins Flash speichern
}
