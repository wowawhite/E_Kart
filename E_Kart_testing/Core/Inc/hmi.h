/*
 * hmi.h - one line comments only!
 *  Created on: 05.11.2018
 *  Updated on: 2021.07 - Added RCP-Mode
 */

#ifndef HMI_H_
#define HMI_H_

#include "main.h"
#include "ssd1289_fsmc.h"
#include "xpt2046.h"
#include <string.h>
#include <stdlib.h>

// function prototypes here
char * getErrorString(uint8_t errorcode);
char * getStatusString(uint8_t statuscode);
void WriteButton(uint_fast8_t Menuenummer, uint8_t Buttonnummer);
void MarkChosenButton(uint_fast8_t Menuenummer, uint8_t Buttonnummer);
void UnmarkChosenButton(uint_fast8_t Menuenummer, uint8_t Buttonnummer);
void BlinkChosenButton(uint_fast8_t Menuenummer, uint8_t Buttonnummer);
void AnzeigeSlider_Init(uint_fast8_t Menuenummer, uint_fast8_t slidernummer);
void TouchSlider(uint8_t menuenummer, uint8_t slidernummer);
void ZeitAnzeige_Init(void);
void ZeitAnzeige(void);
void Menu_Oben(void);
void Menu_Unten(void);
void PasswordAnzeige_Init(uint8_t menuebene);
void PasswordAnzeige(void);
void GeschwindigkeitsAnzeige_Init(void);
void GasAnzeige_Init(void);
void BremsAnzeige_Init(void);
void LenkwinkelAnzeige_Init(uint16_t Startwert);
void BatterieAnzeige_Init(void);
void RCPstatusAnzeige_Init(void);
void StateofChargeAnzeige(void);
void GasAnzeige(void);
void BremsAnzeige(void);
void Geschwindigkeitsanzeige(void);
void LenkwinkelAnzeige(void);
void GPIOAnzeige (void);
void RCP_show_connect(uint8_t msg_switch);
void RCP_show_status(uint8_t RCP_Mode_status);
void EKartZustand(void);
void Anzeige_Init(uint8_t menuebene);
void getTouch(void);
uint8_t TouchAction(uint8_t menuebene);
void Menuestruktur(void);
void Anzeige(uint_fast8_t menuebene);



#endif /* HMI_H_ */
