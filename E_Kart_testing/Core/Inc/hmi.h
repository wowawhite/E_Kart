/*
 * hmi.h
 *
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
void WriteButton(uint_fast8_t Menuenummer, uint8_t Buttonnummer);
void MarkChosenButton(uint_fast8_t Menuenummer, uint8_t Buttonnummer);
void UnmarkChosenButton(uint_fast8_t Menuenummer, uint8_t Buttonnummer);
void BlinkChosenButton(uint_fast8_t Menuenummer, uint8_t Buttonnummer);

void Menu_Oben(void);
void Menu_Unten(void);
void Password_Init(uint8_t menuebene);
void PasswordAnzeige(void);
void GeschwindigkeitsAnzeige_Init(void);
void GasAnzeige_Init(void);
void BremsAnzeige_Init(void);
void LenkwinkelAnzeige_Init(uint16_t Startwert);
void BatterieAnzeige_Init(void);
void Batterie_Init(void);
void GasAnzeige(void);
void BremsAnzeige(void);
void Geschwindigkeitsanzeige(void);
void StateofChargeAnzeige(void);
void LenkwinkelAnzeige(void);
void ZeitAnzeige_Init(void);
void ZeitAnzeige(void);
void GPIOAnzeige (void);
void EKartZustand(void);
void Anzeige_Init(uint8_t menuebene);
uint8_t TouchAction(uint8_t menuebene);
void Menuestruktur(void);
void getTouch(void);
void Anzeige(uint_fast8_t menuebene);
void AnzeigeSlider_Init(uint_fast8_t Menuenummer, uint_fast8_t slidernummer);
void TouchSlider(uint8_t menuenummer, uint8_t slidernummer);

#endif /* HMI_H_ */
