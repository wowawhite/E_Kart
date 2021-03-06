/*
 * hmi.c - one line comments only!
 *  Created on: 05.11.2018
 *  Updated on: 2021.07 - Added RCP-Mode.
 */
// Includes and defines here:
#include "hmi.h"
#include "flash.h"
#include "Parameter.h"
#include "messages.h"
#include "fonts/Font_8_Retro.h"

#define _8_Retro               &Font_8_Retro

// Variables here:
extern int16_t  Lenkradwinkel;            // Lenkradwinkelwert in Grad*10 nach LW-Sensor �ber CAN-It, -7800 .. 0 .. +7800
extern uint32_t Geschwindigkeit_kmh;      // Geschwindigkeit des E-Karts in kmh, 0 .. umgerechneter Wert von DREHZAHL_MAX
extern uint8_t  ReverseGear;              // R�ckw�rtsgang 0 - aus, 1 - ein = R�ckw�rtsfahren
extern uint8_t  Hauptrelais;              // Schaltsignal-Hauptrelias,  1 - alles i.O. , 0 - Fehler
//extern uint32_t Merker_aktuelle_Drehzahl; // Merker f�r Berechnung der Beschleunigung in stm32f10x_it.c   Funktion: AccelerationCalc()
//extern int32_t  Motor_Drehzahl_r;		// Motordrehzahl rechts in rpm nach Speed-Sensor �ber CAN-It -DREHZAHL_MAX  .. 0 .. +DREHZAHL_MAX
//extern int32_t  Motor_Drehzahl_l;		// Motordrehzahl links in rpm nach Speed-Sensor �ber CAN-It -DREHZAHL_MAX  .. 0 .. +DREHZAHL_MAX
extern int16_t  Gas;				              // Wert des Gaspedals, der �ber PWM-Sensor ermittelt wurde, Wertebereich: 0 .. 100 = Volldurchgedr�ckt
extern int16_t  Bremse;				            // Bremswert, der �ber PWM-Sensor ermittelt wurde, Wertebereich: 0 .. 100 = Volldurchgedr�ckt
extern uint8_t StateofCharge;
char buf[4];
char buf1[4];
uint8_t PasswordCode[4]={1,2,3,4};
uint8_t PasswordCodeUser[4];
uint8_t sCode;                 // 0-4, eingegebene Zeichen des Password
char Asci_SliderWert[11];  		// Puffer-f�r uintToasci-Wandelung

extern uint8_t Sp_Min;
extern uint8_t Sp_Sek;
extern uint8_t Sp_Stu;

extern uint8_t Temperatur_Zellen_l[8];
extern uint8_t Temperatur_Zellen_r[8];
extern uint16_t Spannung_Zellen_l[8];
extern uint16_t Spannung_Zellen_r[8];

uint16_t touchX = 0, touchY = 0;  // LCD is 320x240 pixels

extern uint16_t Flash_Memory_List[NumberOfSavedParameter];
extern uint16_t Flash_New_Parameters_List[NumberOfSavedParameter];

extern uint16_t *Pointer_GasProzent;	// Parameter dient zur Touch-Eingabe, Wertbereich (0-100)=(0%-100%), dessen Initialwert ist 100%
															// als der Beiwert f�r Einstellung des Wirkungsbereichs von Gaspedal(bei Drehzahlvorgabe_mode )
                              // z.B wenn "Pointer_GasProzent=100", Gaspedal volldurchgedr�ckt, die max.Drehzahl=3000 wird durch CAN-BUS vorgegeben
                              // wenn "Pointer_GasProzent=50", volldurchgedr�ckt, dann wird nur die Drehzahl=1500 vorgegeben.
															// der Wirkungsbereich halbiert sich

extern uint16_t *Pointer_BeschlProzent;   // Parameter dient zur Touch-Eingabe, Wertbereich (0-100)=(0%-100%), dessen Initialwert ist 100%					--------------------------------------
extern uint16_t *Pointer_Rueckwaert;

// RCP-Mode global communication variables
extern uint8_t RCP_Mode_status;  // actual rcp status: FALSE == 0 == disconnected, TRUE == 1 == connencted
extern uint8_t RCP_Mode_selected;  // indicates if user wants connect/disconnect
extern uint8_t RCP_Mode_pending;  // indicates connection/disconnection event ongoing
extern uint8_t RCP_Mode_errorcode;  // contaims gĺobal RCP errocodes
uint8_t RCP_status_renew = 0;
uint8_t RCP_error_renew = 0;
// Typedefs here:
struct SwitchButton
{
	uint16_t   	x1; 							// Startkoordinate x P1
	uint8_t    	y1;					  		// Startkoordinate Y P1
	uint16_t   	x2;					  		// Startkoordinate x P2
	uint8_t    	y2;					  		// Startkoordinate Y P2
	uint8_t	  	Menuverlinkung;	// Auf welches Men� verlinkt wird beim klicken
	uint8_t    	Wert;             // Werte f�r Code
	char		*Text;				    // Buttontext
	uint8_t    	Textlaenge;			  // Buttontext L�nge
	uint8_t    	bottonsanzahl;    // Anzahl aller Buttons in einer Ebene
} S_Button[max_menuebenen][max_buttons];

volatile struct GleitBlock
{
	uint16_t  x; 		 		// Grundkoordinate von Gleitblock
	uint8_t   y;
	uint8_t   Laenge; 		// geometrische Gr��e von Gleitblock
	uint8_t   Breite;
	uint16_t  Bahn_x1; 	// Anzeige der Gleitbahn
	uint8_t   Bahn_y1;
	uint16_t  Bahn_x2;
	uint8_t   Bahn_y2;
	uint16_t  Touch_x1; 	// Touch-Reaktionsflaeche
	uint8_t   Touch_y1;
   	uint16_t  Touch_x2;
   	uint8_t   Touch_y2;
	uint32_t  SliderWert; // der Wert von Gleitblock
	uint32_t  max_wert;	 // Begrenzung fuer Wertebereich
	uint32_t  min_wert;
	char	  *Text;		// Beschriftung
	uint16_t  Farbe;	 		 // Farbe
} Slider[max_menuebenen][max_sliders];

// functions here
void WriteButton(uint_fast8_t menunummer, uint8_t Buttonnummer)
{
     uint16_t textposi_x, textposi_y;
     // Aufzeichnung des Buttons
     LCD_Rect_Fill(S_Button[menunummer][Buttonnummer].x1,S_Button[menunummer][Buttonnummer].y1,S_Button[menunummer][Buttonnummer].x2-S_Button[menunummer][Buttonnummer].x1,S_Button[menunummer][Buttonnummer].y2-S_Button[menunummer][Buttonnummer].y1, WHITE);
     // Mit der Annahme das ein Buchstabe = 8 x 14 pixel gro� ist
	 // Berechnung fuer die Position der Beschriftung
     textposi_x=((S_Button[menunummer][Buttonnummer].x2+S_Button[menunummer][Buttonnummer].x1)/2)-S_Button[menunummer][Buttonnummer].Textlaenge*8/2;
     textposi_y=((S_Button[menunummer][Buttonnummer].y2+S_Button[menunummer][Buttonnummer].y1)/2+14/2);
	 // Ausgabe der Beschriftung auf Display
	 LCD_Font(textposi_x, textposi_y, S_Button[menunummer][Buttonnummer].Text, _8_Retro, 1, BLACK);
}

char * getErrorString(uint8_t errorcode)
{
	char * tmpptr;
	switch(errorcode)
	 {
		case 0: tmpptr = NO_ERROR_MSG; break;
		case 1: tmpptr = RCP_CONNECTIONERR_MSG; break;
		case 2: tmpptr = NO_RCP_HEARTBEAT_MSG; break;
		case 3: tmpptr = NO_MOTOR_HEARTBEAT_MSG; break;
		case 4: tmpptr = RCP_MOVING_ERROR_MSG; break;
		default: tmpptr=NO_ERROR_MSG; break;
	}
	return tmpptr;
}
char * getStatusString(uint8_t statuscode)
{
	char *tmpptr;
	switch(statuscode)
	 {
		case 0: tmpptr = RCP_OFF_MSG; break;
		case 1: tmpptr = RCP_ON_MSG; break;
		default: tmpptr=RCP_OFF_MSG; break;
	}
	return tmpptr;
}


void MarkChosenButton(uint_fast8_t menunummer, uint8_t Buttonnummer)
{
	LCD_Rect(S_Button[menunummer][Buttonnummer].x1,S_Button[menunummer][Buttonnummer].y1,S_Button[menunummer][Buttonnummer].x2-S_Button[menunummer][Buttonnummer].x1,S_Button[menunummer][Buttonnummer].y2-S_Button[menunummer][Buttonnummer].y1,1,RED);
}

void UnmarkChosenButton(uint_fast8_t menunummer, uint8_t Buttonnummer)
{
	LCD_Rect(S_Button[menunummer][Buttonnummer].x1,S_Button[menunummer][Buttonnummer].y1,S_Button[menunummer][Buttonnummer].x2-S_Button[menunummer][Buttonnummer].x1,S_Button[menunummer][Buttonnummer].y2-S_Button[menunummer][Buttonnummer].y1,1,BLACK);
}

void BlinkChosenButton(uint_fast8_t menunummer, uint8_t Buttonnummer)
{
	if (Sp_Sek%2){
		MarkChosenButton(menunummer, Buttonnummer);
	} else {
		UnmarkChosenButton(menunummer, Buttonnummer);
	}
}

/*******************************************************************************
* Function Name  :  AnzeigeSliderInit
* Description    :  Initiale Anzeige des Gleitblocksystems,(Gleitbahn, Grenzwert, Beschriftung)
* Input          :  menunummer: welche menubene 1,2,3,4,5,6
                    slidernummer: welche Gleitblock 0,1,2
* Output         :  None
* Return         :  None
*******************************************************************************/
void AnzeigeSlider_Init(uint_fast8_t menunummer, uint_fast8_t slidernummer)
{
	uint16_t delta;
	char Asci_max_wert[11];         // Puffer-f�r uintToasci-Wandelung
	char Asci_min_wert[11];         // Puffer-f�r uintToasci-Wandelung
	// Grenzwert des Gleitblocksystems in Asci-Zeichen wandeln
	utoa( Slider[menunummer][slidernummer].max_wert, Asci_max_wert ,10);
	utoa( Slider[menunummer][slidernummer].min_wert, Asci_min_wert ,10);

	// Ausgabe der Asci-Zeichen von Grenzwert auf Display
	LCD_Font(Slider[menunummer][slidernummer].Bahn_x1-5, Slider[menunummer][slidernummer].Bahn_y2+21, Asci_min_wert, _8_Retro, 1, WHITE);
	LCD_Font(Slider[menunummer][slidernummer].Bahn_x2-5,Slider[menunummer][slidernummer].Bahn_y2+21,Asci_max_wert,_8_Retro,1,WHITE);
	// Ausgabe der Beschriftung
	LCD_Font(1,Slider[menunummer][slidernummer].Bahn_y2+5,Slider[menunummer][slidernummer].Text,_8_Retro,1,Slider[menunummer][slidernummer].Farbe);
	// Aufzeichnen der Gleitbahn
	LCD_Rect(Slider[menunummer][slidernummer].Bahn_x1,Slider[menunummer][slidernummer].Bahn_y1,Slider[menunummer][slidernummer].Bahn_x2-Slider[menunummer][slidernummer].Bahn_x1,Slider[menunummer][slidernummer].Bahn_y2-Slider[menunummer][slidernummer].Bahn_y1, 1, GRAY);

	//untere Begrenzung
	if(Slider[menunummer][slidernummer].x<=Slider[menunummer][slidernummer].Bahn_x1)
	{
		Slider[menunummer][slidernummer].x=Slider[menunummer][slidernummer].Bahn_x1;
	}
	// obere Begrenzung
	if(Slider[menunummer][slidernummer].x>(Slider[menunummer][slidernummer].Bahn_x2))
	{
		Slider[menunummer][slidernummer].x=Slider[menunummer][slidernummer].Bahn_x2;
	}
	// Berechnung des Anpassungsfaktors, Delta=(max-min)*1000/(laenge der Bahn)
	delta=((Slider[menunummer][slidernummer].max_wert-Slider[menunummer][slidernummer].min_wert)*1000)/(Slider[menunummer][slidernummer].Bahn_x2-Slider[menunummer][slidernummer].Bahn_x1);
	// neuer Wert des Gleitblock
	Slider[menunummer][slidernummer].SliderWert=(((Slider[menunummer][slidernummer].x-Slider[menunummer][slidernummer].Bahn_x1)*delta)/1000)+Slider[menunummer][slidernummer].min_wert;

	LCD_Rect_Fill(Slider[menunummer][slidernummer].x,Slider[menunummer][slidernummer].y,Slider[menunummer][slidernummer].Breite,Slider[menunummer][slidernummer].Laenge,GREEN);
	utoa(Slider[menunummer][slidernummer].SliderWert,Asci_SliderWert,10);
	LCD_Font(Slider[menunummer][slidernummer].x-5, Slider[menunummer][slidernummer].y-1, Asci_SliderWert, _8_Retro, 1, Slider[menunummer][slidernummer].Farbe);
}

void TouchSlider(uint8_t menunummer, uint8_t slidernummer)
{
	uint16_t delta;
	LCD_Rect_Fill(Slider[menunummer][slidernummer].x,Slider[menunummer][slidernummer].y,Slider[menunummer][slidernummer].Breite,Slider[menunummer][slidernummer].Laenge,BLACK);
	LCD_Rect_Fill(Slider[menunummer][slidernummer].x-5,Slider[menunummer][slidernummer].y-16,8*4,16,BLACK);
	LCD_Rect(Slider[menunummer][slidernummer].Bahn_x1,Slider[menunummer][slidernummer].Bahn_y1,Slider[menunummer][slidernummer].Bahn_x2-Slider[menunummer][slidernummer].Bahn_x1,Slider[menunummer][slidernummer].Bahn_y2-Slider[menunummer][slidernummer].Bahn_y1, 1, GRAY);
	Slider[menunummer][slidernummer].x=touchX;
	//untere Begrenzung
	if(Slider[menunummer][slidernummer].x<=Slider[menunummer][slidernummer].Bahn_x1)
	{
		Slider[menunummer][slidernummer].x=Slider[menunummer][slidernummer].Bahn_x1;
	}
	// obere Begrenzung
	if(Slider[menunummer][slidernummer].x>(Slider[menunummer][slidernummer].Bahn_x2))
	{
		Slider[menunummer][slidernummer].x=Slider[menunummer][slidernummer].Bahn_x2;
	}
	// Berechnung des Anpassungsfaktors, Delta=(max-min)*1000/(laenge der Bahn)
	delta=((Slider[menunummer][slidernummer].max_wert-Slider[menunummer][slidernummer].min_wert)*1000)/(Slider[menunummer][slidernummer].Bahn_x2-Slider[menunummer][slidernummer].Bahn_x1);
	// neuer Wert des Gleitblock
	Slider[menunummer][slidernummer].SliderWert=(((Slider[menunummer][slidernummer].x-Slider[menunummer][slidernummer].Bahn_x1)*delta)/1000)+Slider[menunummer][slidernummer].min_wert;

	LCD_Rect_Fill(Slider[menunummer][slidernummer].x,Slider[menunummer][slidernummer].y,Slider[menunummer][slidernummer].Breite,Slider[menunummer][slidernummer].Laenge,GREEN);
	utoa(Slider[menunummer][slidernummer].SliderWert,Asci_SliderWert,10);
	LCD_Font(Slider[menunummer][slidernummer].x-5, Slider[menunummer][slidernummer].y-1, Asci_SliderWert, _8_Retro, 1, Slider[menunummer][slidernummer].Farbe);
	//LCD_Rect(Slider[menunummer][0].Bahn_x1,Slider[menunummer][0].Bahn_y1,Slider[menunummer][0].Bahn_x2-Slider[menunummer][0].Bahn_x1,Slider[menunummer][0].Bahn_y2-Slider[menunummer][0].Bahn_y1, 1, GRAY);

}

void ZeitAnzeige_Init(void)
{
	char Asci_Time[2];
	LCD_Font(250,17,"00:00:00",_8_Retro,1,WHITE);
	if (Sp_Sek < 10)
	{
		LCD_Rect_Fill(298,1,16,16,BLACK);
		LCD_Font(298,17,"0",_8_Retro,1,WHITE);
		LCD_Font(306, 17, itoa(Sp_Sek,Asci_Time,10), _8_Retro, 1, WHITE);
	}
	else
	{
		LCD_Rect_Fill(298,1,16,16,BLACK);
		LCD_Font(298,17,itoa(Sp_Sek,Asci_Time,10),_8_Retro,1,WHITE);
	}

	if (Sp_Min < 10)
	{
		LCD_Rect_Fill(274,1,16,16,BLACK);
		LCD_Font(274,17,"0",_8_Retro,1,WHITE);
		LCD_Font(282,17,itoa(Sp_Min,Asci_Time,10),_8_Retro,1,WHITE);
	}
	else
	{
		LCD_Rect_Fill(274,1,16,16,BLACK);
		LCD_Font(274,17,itoa(Sp_Min,Asci_Time,10),_8_Retro,1,WHITE);
	}


	if (Sp_Stu<10)
	{
		LCD_Rect_Fill(250,1,16,16,BLACK);
		LCD_Font(250,17,"0",_8_Retro,1,WHITE);
		LCD_Font(258, 17, itoa(Sp_Stu,Asci_Time,10), _8_Retro, 1, WHITE);
	}
	else
	{
		LCD_Rect_Fill(250,1,16,16,BLACK);
		LCD_Font(250, 17, itoa(Sp_Stu,Asci_Time,10), _8_Retro, 1, WHITE);
	}

	if ( (Sp_Stu >= 99) && (Sp_Min >= 59) && (Sp_Sek >= 59))
	{
		LCD_Font(250,17,"00:00:00",_8_Retro,1,WHITE);
	}
}

void ZeitAnzeige(void)
{
	static uint8_t Sp_Sek_Merker;				  // Merker, ob entsprechende Zeit vorangeschritten ist
    static uint8_t Sp_Min_Merker;				  // Merker, ob entsprechende Zeit vorangeschritten ist
    static uint8_t Sp_Stu_Merker;				  // Merker, ob entsprechende Zeit vorangeschritten ist
    char Asci_Time[3];
    // Initialisierung mit Menu_Oben() bewirkte Ausgabe: "00:00:00"
    // GUI_Text(250,1,"00:00:00",White,Black);
    // Zeichen sind 8 Pixel breit und 16 hoch
    // Trennzeichen ":" wird nur beim R�cksetzen neu geschrieben
    // Sekunden,  Ausgabe, wenn Wert sich vergr��ert hat -> eine Sekunde sp�ter, so muss Wert nicht jedesmal geschrieben werden
	// oder wenn Minute eins weiter gez�hlt hat, neue "00"
	if ( (Sp_Sek != Sp_Sek_Merker) || (Sp_Min != Sp_Min_Merker) || (Sp_Stu != Sp_Stu_Merker))
	{
		if (Sp_Sek < 10)
		{
			LCD_Rect_Fill(298,1,16,16,BLACK);
			LCD_Font(298,17,"0",_8_Retro,1,WHITE);
			LCD_Font(306, 17, itoa(Sp_Sek,Asci_Time,10), _8_Retro, 1, WHITE);

		}
		else
		{
			LCD_Rect_Fill(298,1,16,16,BLACK);
			LCD_Font(298,17,itoa(Sp_Sek,Asci_Time,10),_8_Retro,1,WHITE);
		}

	    Sp_Sek_Merker = Sp_Sek;				 // neuen Merker-Wert setzen

		if (Sp_Min < 10)
		{
			LCD_Rect_Fill(274,1,16,16,BLACK);
			LCD_Font(274,17,"0",_8_Retro,1,WHITE);
			LCD_Font(282,17,itoa(Sp_Min,Asci_Time,10),_8_Retro,1,WHITE);
		}
		else
		{
			LCD_Rect_Fill(274,1,16,16,BLACK);
			LCD_Font(274,17,itoa(Sp_Min,Asci_Time,10),_8_Retro,1,WHITE);
		}
	    Sp_Min_Merker = Sp_Min;


		if (Sp_Stu<10)
		{
			LCD_Rect_Fill(250,1,16,16,BLACK);
			LCD_Font(250,17,"0",_8_Retro,1,WHITE);
			LCD_Font(258, 17, itoa(Sp_Stu,Asci_Time,10), _8_Retro, 1, WHITE);
		}
		else
		{
			LCD_Rect_Fill(250,1,16,16,BLACK);
			LCD_Font(250, 17, itoa(Sp_Stu,Asci_Time,10), _8_Retro, 1, WHITE);
		}
	    Sp_Stu_Merker = Sp_Stu;

		if ( (Sp_Stu >= 99) && (Sp_Min >= 59) && (Sp_Sek >= 59))
		{
			Sp_Stu_Merker = 0;
			Sp_Min_Merker = 0;
			Sp_Sek_Merker = 0;
			LCD_Font(250,17,"00:00:00",_8_Retro,1,WHITE);
		}

	}
}


void Menu_Oben(void)
{
	LCD_Rect(0,0,320,16,1,BLACK);
	LCD_Font(3, 17, SOFTWAREVERSION, _8_Retro, 1, WHITE); // software version number
	LCD_Font(99, 17, "Fahrzeugrechner", _8_Retro, 1, WHITE);
	ZeitAnzeige_Init();
	LCD_Line(0,17,320,17,1,WHITE);
}

void Menu_Unten(void)
{
	//Pfeil links
	LCD_Rect_Fill(1,210,95,27,WHITE);
	LCD_Triangle(10,223,80,212,80,234,1,BLACK);

	//Button Mitte
	LCD_Rect_Fill(100,210,120,27,WHITE);
	LCD_Font(140, 232, "Menu", _8_Retro,1,BLACK);

	//Pfeil rechts
	LCD_Rect_Fill(225,210,95,27,WHITE);
	LCD_Triangle(239,212,239,234,309,223,1,BLACK);
}

void PasswordAnzeige_Init(uint8_t menubene)
{

	//Anzeigesbereich des Paswort
	LCD_Rect(89, 6, 14, 17, 1, WHITE);
	LCD_Rect(109, 6, 14, 17, 1, WHITE);
	LCD_Rect(129, 6, 14, 17, 1, WHITE);
	LCD_Rect(149, 6, 14, 17, 1, WHITE);


	// 13 Buttons
	WriteButton(menubene, 0);
	WriteButton(menubene, 1);
	WriteButton(menubene, 2);
	WriteButton(menubene, 3);
	WriteButton(menubene, 4);
	WriteButton(menubene, 5);
	WriteButton(menubene, 6);
	WriteButton(menubene, 7);
	WriteButton(menubene, 8);
	WriteButton(menubene, 9);
	WriteButton(menubene, 10);		//Abbruch
	WriteButton(menubene, 11);		//Best�tigen
	WriteButton(menubene, 12);		//L�schen

}

void PasswordAnzeige(void)
{
	if(sCode >= 1)
	{
		LCD_Font(91, 23, itoa(PasswordCodeUser[0],buf,10), _8_Retro, 1, WHITE);
	}
	else
	{
		LCD_Rect_Fill(90, 7, 13, 16, BLACK);
	}

	if(sCode >= 2)
	{
		LCD_Font(111, 23, itoa(PasswordCodeUser[1],buf,10), _8_Retro, 1, WHITE);
	}
	else
	{
		LCD_Rect_Fill(110, 7, 13, 16, BLACK);
	}

	if(sCode >= 3)
	{
		LCD_Font(131, 23, itoa(PasswordCodeUser[2],buf,10), _8_Retro, 1, WHITE);
	}
	else
	{
		LCD_Rect_Fill(130, 7, 13, 16, BLACK);
	}

	if(sCode >= 4)
	{
		LCD_Font(151, 23, itoa(PasswordCodeUser[3],buf,10), _8_Retro, 1, WHITE);
	}
	else
	{
		LCD_Rect_Fill(150, 7, 13, 16, BLACK);
	}
}

void GeschwindigkeitsAnzeige_Init(void)
{
	LCD_Line(8,120,309,120,1,WHITE);	//Rahmen Strich oben
	LCD_Line(8,150,309,150,1,WHITE);	//Rahmen Strich unten
	LCD_Font(60,168 , "Geschwindigkeit:", _8_Retro,1,WHITE);
	LCD_Font(212,168 , "km/h", _8_Retro,1,WHITE);
	LCD_Font(204,168,"0",_8_Retro,1,WHITE);

	LCD_Line(8,117,8,127,1,WHITE);
	LCD_Line(8,144,8,150,1,WHITE);
	LCD_Font(4,116,"0",_8_Retro,1,WHITE);

	LCD_Line(58,117,58,127,1,WHITE);
	LCD_Line(58,144,58,150,1,WHITE);
	LCD_Font(50,116,"10",_8_Retro,1,WHITE);

	LCD_Line(108,117,108,127,1,WHITE);
	LCD_Line(108,144,108,150,1,WHITE);
	LCD_Font(100,116,"20",_8_Retro,1,WHITE);

	LCD_Line(158,117,158,127,1,WHITE);
	LCD_Line(158,144,158,150,1,WHITE);
	LCD_Font(150,116,"30",_8_Retro,1,WHITE);

	LCD_Line(208,117,208,127,1,WHITE);
	LCD_Line(208,144,208,150,1,WHITE);
	LCD_Font(200,116,"40",_8_Retro,1,WHITE);

	LCD_Line(258,117,258,127,1,WHITE);
	LCD_Line(258,144,258,150,1,WHITE);
	LCD_Font(250,116,"50",_8_Retro,1,WHITE);

	LCD_Line(308,117,308,127,1,WHITE);
	LCD_Line(308,144,308,150,1,WHITE);
	LCD_Font(300,116,"60",_8_Retro,1,WHITE);

	LCD_Line(33,120,33,127,1,WHITE);
	LCD_Line(33,144,33,150,1,WHITE);

	LCD_Line(83,120,83,127,1,WHITE);
	LCD_Line(83,144,83,150,1,WHITE);

	LCD_Line(133,120,133,127,1,WHITE);
	LCD_Line(133,144,133,150,1,WHITE);

	LCD_Line(183,120,183,127,1,WHITE);
	LCD_Line(183,144,183,150,1,WHITE);

	LCD_Line(233,120,233,127,1,WHITE);
	LCD_Line(233,144,233,150,1,WHITE);

	LCD_Line(283,120,283,127,1,WHITE);
	LCD_Line(283,144,283,150,1,WHITE);
}

void GasAnzeige_Init(void)
{
	LCD_Rect(279,29,21,52,1,WHITE);
	LCD_Font(277,100,"Gas",_8_Retro,1,WHITE);
}

void BremsAnzeige_Init(void)
{
	LCD_Rect(20,29,21,52,1,WHITE);
	LCD_Font(5,100,"Bremse",_8_Retro,1,WHITE);
}

void LenkwinkelAnzeige_Init(uint16_t Startwert)
{
	LCD_Rect(109,50,102,22,1,WHITE);
	LCD_Rect_Fill(160,51,1,21,YELLOW);
	LCD_Font(120,90,"Lenkwinkel",_8_Retro,1,WHITE);
}

void BatterieMenu_Init(void)
{
	LCD_Rect_Fill(0, 0, 320, 240, BLACK);

	Menu_Oben();					            // �berschrift, Fahrzeugrechner
	Menu_Unten();						        // Menu-Zeile unten, Optionen, Pfeile

	LCD_Font(0,35,"Batterie - Spannung", _8_Retro,1,WHITE);

	//Spannungsanzeige F�r Zelle 1
	LCD_Font(0,60,"U 1:", _8_Retro,1,WHITE);
	utoa(Spannung_Zellen_l[0],buf,10);
	buf1[0]=buf[0];
	buf1[1]=',';
	buf1[2]=buf[1];
	buf1[3]=buf[2];
	LCD_Font(32,60,buf1, _8_Retro,1,WHITE);
	LCD_Font(64,60,"V", _8_Retro,1,WHITE);

	//Spannungsanzeige F�r Zelle 2
	LCD_Font(0,80,"U 2:", _8_Retro,1,WHITE);
	utoa(Spannung_Zellen_l[1],buf,10);
	buf1[0]=buf[0];
	buf1[1]=',';
	buf1[2]=buf[1];
	buf1[3]=buf[2];
	LCD_Font(32,80,buf1, _8_Retro,1,WHITE);
	LCD_Font(64,80,"V", _8_Retro,1,WHITE);

	//Spannungsanzeige F�r Zelle 3
	LCD_Font(0,100,"U 3:", _8_Retro,1,WHITE);
	utoa(Spannung_Zellen_l[2],buf,10);
	buf1[0]=buf[0];
	buf1[1]=',';
	buf1[2]=buf[1];
	buf1[3]=buf[2];
	LCD_Font(32,100,buf1, _8_Retro,1,WHITE);
	LCD_Font(64,100,"V", _8_Retro,1,WHITE);

	//Spannungsanzeige F�r Zelle 4
	LCD_Font(0,120,"U 4:", _8_Retro,1,WHITE);
	utoa(Spannung_Zellen_l[3],buf,10);
	buf1[0]=buf[0];
	buf1[1]=',';
	buf1[2]=buf[1];
	buf1[3]=buf[2];
	LCD_Font(32,120,buf1, _8_Retro,1,WHITE);
	LCD_Font(64,120,"V", _8_Retro,1,WHITE);

	//Spannungsanzeige F�r Zelle 5
	LCD_Font(0,140,"U 5:", _8_Retro,1,WHITE);
	utoa(Spannung_Zellen_l[4],buf,10);
	buf1[0]=buf[0];
	buf1[1]=',';
	buf1[2]=buf[1];
	buf1[3]=buf[2];
	LCD_Font(32,140,buf1, _8_Retro,1,WHITE);
	LCD_Font(64,140,"V", _8_Retro,1,WHITE);

	//Spannungsanzeige F�r Zelle 6
	LCD_Font(0,160,"U 6:", _8_Retro,1,WHITE);
	utoa(Spannung_Zellen_l[5],buf,10);
	buf1[0]=buf[0];
	buf1[1]=',';
	buf1[2]=buf[1];
	buf1[3]=buf[2];
	LCD_Font(32,160,buf1, _8_Retro,1,WHITE);
	LCD_Font(64,160,"V", _8_Retro,1,WHITE);

	//Spannungsanzeige F�r Zelle 7
	LCD_Font(0,180,"U 7:", _8_Retro,1,WHITE);
	utoa(Spannung_Zellen_l[6],buf,10);
	buf1[0]=buf[0];
	buf1[1]=',';
	buf1[2]=buf[1];
	buf1[3]=buf[2];
	LCD_Font(32,180,buf1, _8_Retro,1,WHITE);
	LCD_Font(64,180,"V", _8_Retro,1,WHITE);

	//Spannungsanzeige F�r Zelle 8
	LCD_Font(0,200,"U 8:", _8_Retro,1,WHITE);
	utoa(Spannung_Zellen_l[7],buf,10);
	buf1[0]=buf[0];
	buf1[1]=',';
	buf1[2]=buf[1];
	buf1[3]=buf[2];
	LCD_Font(32,200,buf1, _8_Retro,1,WHITE);
	LCD_Font(64,200,"V", _8_Retro,1,WHITE);

	//Spannungsanzeige F�r Zelle 9
	LCD_Font(90,60,"U 9:", _8_Retro,1,WHITE);
	utoa(Spannung_Zellen_r[0],buf,10);
	buf1[0]=buf[0];
	buf1[1]=',';
	buf1[2]=buf[1];
	buf1[3]=buf[2];
	LCD_Font(122,60,buf1, _8_Retro,1,WHITE);
	LCD_Font(154,60,"V", _8_Retro,1,WHITE);

	//Spannungsanzeige F�r Zelle 10
	LCD_Font(90,80,"U10:", _8_Retro,1,WHITE);
	utoa(Spannung_Zellen_r[1],buf,10);
	buf1[0]=buf[0];
	buf1[1]=',';
	buf1[2]=buf[1];
	buf1[3]=buf[2];
	LCD_Font(122,80,buf1, _8_Retro,1,WHITE);
	LCD_Font(154,80,"V", _8_Retro,1,WHITE);

	//Spannungsanzeige F�r Zelle 11
	LCD_Font(90,100,"U11:", _8_Retro,1,WHITE);
	utoa(Spannung_Zellen_r[2],buf,10);
	buf1[0]=buf[0];
	buf1[1]=',';
	buf1[2]=buf[1];
	buf1[3]=buf[2];
	LCD_Font(122,100,buf1, _8_Retro,1,WHITE);
	LCD_Font(154,100,"V", _8_Retro,1,WHITE);

	//Spannungsanzeige F�r Zelle 12
	LCD_Font(90,120,"U12:", _8_Retro,1,WHITE);
	utoa(Spannung_Zellen_r[3],buf,10);
	buf1[0]=buf[0];
	buf1[1]=',';
	buf1[2]=buf[1];
	buf1[3]=buf[2];
	LCD_Font(122,120,buf1, _8_Retro,1,WHITE);
	LCD_Font(154,120,"V", _8_Retro,1,WHITE);

	//Spannungsanzeige F�r Zelle 13
	LCD_Font(90,140,"U13:", _8_Retro,1,WHITE);
	utoa(Spannung_Zellen_r[4],buf,10);
	buf1[0]=buf[0];
	buf1[1]=',';
	buf1[2]=buf[1];
	buf1[3]=buf[2];
	LCD_Font(122,140,buf1, _8_Retro,1,WHITE);
	LCD_Font(154,140,"V", _8_Retro,1,WHITE);

	//Spannungsanzeige F�r Zelle 14
	LCD_Font(90,160,"U14:", _8_Retro,1,WHITE);
	utoa(Spannung_Zellen_r[5],buf,10);
	buf1[0]=buf[0];
	buf1[1]=',';
	buf1[2]=buf[1];
	buf1[3]=buf[2];
	LCD_Font(122,160,buf1, _8_Retro,1,WHITE);
	LCD_Font(154,160,"V", _8_Retro,1,WHITE);

	//Spannungsanzeige F�r Zelle 15
	LCD_Font(90,180,"U15:", _8_Retro,1,WHITE);
	utoa(Spannung_Zellen_r[6],buf,10);
	buf1[0]=buf[0];
	buf1[1]=',';
	buf1[2]=buf[1];
	buf1[3]=buf[2];
	LCD_Font(122,180,buf1, _8_Retro,1,WHITE);
	LCD_Font(154,180,"V", _8_Retro,1,WHITE);

	//Spannungsanzeige F�r Zelle 16
	LCD_Font(90,200,"U16:", _8_Retro,1,WHITE);
	utoa(Spannung_Zellen_r[7],buf,10);
	buf1[0]=buf[0];
	buf1[1]=',';
	buf1[2]=buf[1];
	buf1[3]=buf[2];
	LCD_Font(122,200,buf1, _8_Retro,1,WHITE);
	LCD_Font(154,200,"V", _8_Retro,1,WHITE);



	LCD_Font(178,35,"Batterie - Temp", _8_Retro,1,WHITE);

	//Temperaturanzeige F�r Zelle 1
	LCD_Font(178,60,"T 1:", _8_Retro,1,WHITE);
	utoa(Temperatur_Zellen_l[0],buf,10);
	LCD_Font(210,60,buf, _8_Retro,1,WHITE);
	LCD_Font(226,60,"C", _8_Retro,1,WHITE);

	//Temperaturanzeige F�r Zelle 2
	LCD_Font(178,80,"T 2:", _8_Retro,1,WHITE);
	utoa(Temperatur_Zellen_l[1],buf,10);
	LCD_Font(210,80,buf, _8_Retro,1,WHITE);
	LCD_Font(226,80,"C", _8_Retro,1,WHITE);

	//Temperaturanzeige F�r Zelle 3
	LCD_Font(178,100,"T 3:", _8_Retro,1,WHITE);
	utoa(Temperatur_Zellen_l[2],buf,10);
	LCD_Font(210,100,buf, _8_Retro,1,WHITE);
	LCD_Font(226,100,"C", _8_Retro,1,WHITE);

	//Temperaturanzeige F�r Zelle 4
	LCD_Font(178,120,"T 4:", _8_Retro,1,WHITE);
	utoa(Temperatur_Zellen_l[3],buf,10);
	LCD_Font(210,120,buf, _8_Retro,1,WHITE);
	LCD_Font(226,120,"C", _8_Retro,1,WHITE);

	//Temperaturanzeige F�r Zelle 5
	LCD_Font(178,140,"T 5:", _8_Retro,1,WHITE);
	utoa(Temperatur_Zellen_l[4],buf,10);
	LCD_Font(210,140,buf, _8_Retro,1,WHITE);
	LCD_Font(226,140,"C", _8_Retro,1,WHITE);

	//Temperaturanzeige F�r Zelle 6
	LCD_Font(178,160,"T 6:", _8_Retro,1,WHITE);
	utoa(Temperatur_Zellen_l[5],buf,10);
	LCD_Font(210,160,buf, _8_Retro,1,WHITE);
	LCD_Font(226,160,"C", _8_Retro,1,WHITE);

	//Temperaturanzeige F�r Zelle 7
	LCD_Font(178,180,"T 7:", _8_Retro,1,WHITE);
	utoa(Temperatur_Zellen_l[6],buf,10);
	LCD_Font(210,180,buf, _8_Retro,1,WHITE);
	LCD_Font(226,180,"C", _8_Retro,1,WHITE);

	//Temperaturanzeige F�r Zelle 8
	LCD_Font(178,200,"T 8:", _8_Retro,1,WHITE);
	utoa(Temperatur_Zellen_l[7],buf,10);
	LCD_Font(210,200,buf, _8_Retro,1,WHITE);
	LCD_Font(226,200,"C", _8_Retro,1,WHITE);



	//Temperaturanzeige F�r Zelle 9
	LCD_Font(244,60,"T 9:", _8_Retro,1,WHITE);
	utoa(Temperatur_Zellen_r[0],buf,10);
	LCD_Font(276,60,buf, _8_Retro,1,WHITE);
	LCD_Font(292,60,"C", _8_Retro,1,WHITE);

	//Temperaturanzeige F�r Zelle 10
	LCD_Font(244,80,"T10:", _8_Retro,1,WHITE);
	utoa(Temperatur_Zellen_r[1],buf,10);
	LCD_Font(276,80,buf, _8_Retro,1,WHITE);
	LCD_Font(292,80,"C", _8_Retro,1,WHITE);

	//Temperaturanzeige F�r Zelle 11
	LCD_Font(244,100,"T11:", _8_Retro,1,WHITE);
	utoa(Temperatur_Zellen_r[2],buf,10);
	LCD_Font(276,100,buf, _8_Retro,1,WHITE);
	LCD_Font(292,100,"C", _8_Retro,1,WHITE);

	//Temperaturanzeige F�r Zelle 12
	LCD_Font(244,120,"T12:", _8_Retro,1,WHITE);
	utoa(Temperatur_Zellen_r[3],buf,10);
	LCD_Font(276,120,buf, _8_Retro,1,WHITE);
	LCD_Font(292,120,"C", _8_Retro,1,WHITE);

	//Temperaturanzeige F�r Zelle 13
	LCD_Font(244,140,"T13:", _8_Retro,1,WHITE);
	utoa(Temperatur_Zellen_r[4],buf,10);
	LCD_Font(276,140,buf, _8_Retro,1,WHITE);
	LCD_Font(292,140,"C", _8_Retro,1,WHITE);

	//Temperaturanzeige F�r Zelle 14
	LCD_Font(244,160,"T14:", _8_Retro,1,WHITE);
	utoa(Temperatur_Zellen_r[5],buf,10);
	LCD_Font(276,160,buf, _8_Retro,1,WHITE);
	LCD_Font(292,160,"C", _8_Retro,1,WHITE);

	//Temperaturanzeige F�r Zelle 15
	LCD_Font(244,180,"T15:", _8_Retro,1,WHITE);
	utoa(Temperatur_Zellen_r[6],buf,10);
	LCD_Font(276,180,buf, _8_Retro,1,WHITE);
	LCD_Font(292,180,"C", _8_Retro,1,WHITE);

	//Temperaturanzeige F�r Zelle 16
	LCD_Font(244,200,"T16:", _8_Retro,1,WHITE);
	utoa(Temperatur_Zellen_r[7],buf,10);
	LCD_Font(276,200,buf, _8_Retro,1,WHITE);
	LCD_Font(292,200,"C", _8_Retro,1,WHITE);

}

void BatterieAnzeige_Init(void)
{
	LCD_Rect(19,170,52,15,1,WHITE);
	LCD_Font(10,204,"AKKU Stat:",_8_Retro,1,WHITE);
	LCD_Font(110,185,"%",_8_Retro,1,WHITE);
	uint8_t StateofChargeAnzeige=0;

	LCD_Rect_Fill(80,170,16,16,BLACK);
	LCD_Font(80,186,itoa(StateofCharge,buf,10),_8_Retro,1,WHITE);

	//Balkenanzeige
	StateofChargeAnzeige=StateofCharge>>1;
	LCD_Rect_Fill(20, 171,StateofChargeAnzeige,14,GREEN);	// Farb-Rechteck von linker unteren Ecke nach entspr Pos. oben rechts, variable Balkenh�he

}

void RCPstatusAnzeige_Init(void)
{
	static uint8_t RCP_flag = 0;
	const uint8_t xposition = 200;
	const uint8_t yposition = 185;
	LCD_Font(xposition,yposition,"RCP Mode:",_8_Retro,1,WHITE);
	LCD_Rect_Fill(xposition,yposition,200,16,BLACK);
	LCD_Font(xposition+80,yposition,getStatusString(RCP_Mode_status),_8_Retro,1,WHITE);
	if(RCP_flag != RCP_Mode_status){
		RCP_flag = RCP_Mode_status;
		LCD_Rect_Fill(xposition,yposition,200,16,BLACK);
		LCD_Font(xposition+80,yposition,getStatusString(RCP_Mode_status),_8_Retro,1,WHITE);
	}

}

void StateofChargeAnzeige(void)
{
	uint8_t StateofChargeAnzeige=0;
	static uint8_t StateofCharge_old=0;
	static uint8_t MerkerStateofChargeAnzeige;
	if(StateofCharge!=StateofCharge_old)
	{
		//Ausgabe Wert
		StateofCharge_old=StateofCharge;
		LCD_Rect_Fill(80,170,16,16,BLACK);
		LCD_Font(80,186,itoa(StateofCharge_old,buf,10),_8_Retro,1,WHITE);

		//Balkenanzeige
		StateofChargeAnzeige=StateofCharge>>1;
		if (StateofChargeAnzeige >= MerkerStateofChargeAnzeige )
		{
			LCD_Rect_Fill(20, 171,StateofChargeAnzeige,14,GREEN);	// Farb-Rechteck von linker unteren Ecke nach entspr Pos. oben rechts, variable Balkenh�he
		}
		else if (StateofChargeAnzeige < MerkerStateofChargeAnzeige)
		{
			// alte Zeile(n) l�schen
			LCD_Rect_Fill(20+StateofChargeAnzeige,171,50-StateofChargeAnzeige,14, BLACK);
			// schwarzes Rechteck von linker oberer Ecke nach entspr. Pos. unten rechts, variable Balkentiefe
			// sonst keine neue Anzeige, da alter und neuer Wert gleich sind
		}
		MerkerStateofChargeAnzeige=StateofChargeAnzeige; // neuen Wert der Anzeige f�r n�chsten Durchlauf �bernehmen
	}
}


void GasAnzeige(void)
{
	uint8_t  GasAnzeige=0;	  		          // Wert des Gaspedals f�r Anzeige, Wertebereich 0 .. 50 (Pixelh�he)
	static uint8_t MerkerGasAnzeige=0;      // Merker der letzten Gasanzeige
	GasAnzeige=Gas>>1;				              // Anpassung der Anzeige an Pixelh�he = 50, Schiebeoperation um eins nach rechts = Division durch 2
	if (GasAnzeige >= MerkerGasAnzeige )
	{
		LCD_Rect_Fill(280,80-GasAnzeige,20,GasAnzeige,GREEN);	// Farb-Rechteck von linker unteren Ecke nach entspr Pos. oben rechts, variable Balkenh�he
	}
	else if (GasAnzeige < MerkerGasAnzeige)
	{
		// alte Zeile(n) l�schen
		LCD_Rect_Fill(280,30,20,50-GasAnzeige,BLACK);
		// schwarzes Rechteck von linker oberer Ecke nach entspr. Pos. unten rechts, variable Balkentiefe
		// sonst keine neue Anzeige, da alter und neuer Wert gleich sind
	}
	MerkerGasAnzeige=GasAnzeige; // neuen Wert der Anzeige f�r n�chsten Durchlauf �bernehmen
}

void BremsAnzeige(void)
{
	uint8_t  BremsAnzeige=0;	  		          // Wert des Gaspedals f�r Anzeige, Wertebereich 0 .. 50 (Pixelh�he)
	static uint8_t MerkerBremsAnzeige=0;      // Merker der letzten Gasanzeige
	BremsAnzeige=Bremse>>1;				              // Anpassung der Anzeige an Pixelh�he = 50, Schiebeoperation um eins nach rechts = Division durch 2
	if (BremsAnzeige >= MerkerBremsAnzeige )
	{
		LCD_Rect_Fill(21,80-BremsAnzeige,20,BremsAnzeige,RED);	// Farb-Rechteck von linker unteren Ecke nach entspr Pos. oben rechts, variable Balkenh�he
	}
	else if (BremsAnzeige < MerkerBremsAnzeige)
	{
		// alte Zeile(n) l�schen
		LCD_Rect_Fill(21,30,20,50-BremsAnzeige,BLACK);
		// schwarzes Rechteck von linker oberer Ecke nach entspr. Pos. unten rechts, variable Balkentiefe
		// sonst keine neue Anzeige, da alter und neuer Wert gleich sind
	}
	MerkerBremsAnzeige=BremsAnzeige; // neuen Wert der Anzeige f�r n�chsten Durchlauf �bernehmen

}

void Geschwindigkeitsanzeige(void)
{
	uint16_t GeschwindigkeitAnzeige=0;      // Geschwindigkeit des E-karts, Wertebereich 0 .. 300 (Pixelbreite)
	static uint16_t MerkerGeschwindigkeitAnzeige=0;  // Merker der letzten Geschwindigkeitsanzeige
	if(Geschwindigkeit_kmh>60)GeschwindigkeitAnzeige=60*5;
	else GeschwindigkeitAnzeige=Geschwindigkeit_kmh*5;			// Anpassung der Anzeige an Pixelbreite,  60 km/h = 300 Pixel
	// analog
	if (GeschwindigkeitAnzeige != MerkerGeschwindigkeitAnzeige)
	{
		if (GeschwindigkeitAnzeige >= MerkerGeschwindigkeitAnzeige )
		{
			LCD_Rect_Fill(GeschwindigkeitAnzeige+7,127,2,16,GREEN); // neuer Wert	rechts
		    LCD_Rect_Fill(0,127,GeschwindigkeitAnzeige+6,16,BLACK); // Feld links l�schen
		}
	    else if (GeschwindigkeitAnzeige < MerkerGeschwindigkeitAnzeige)
		{
	    	LCD_Rect_Fill(GeschwindigkeitAnzeige+7,127,2,16,GREEN); // neuer Wert links
	    	LCD_Rect_Fill(GeschwindigkeitAnzeige+7+2,127,300-GeschwindigkeitAnzeige,16,BLACK); // Feld rechts l�schen
		}


		MerkerGeschwindigkeitAnzeige=GeschwindigkeitAnzeige; // neuen Wert der Anzeige f�r n�chsten Durchlauf �bernehmen
	}
	LCD_Rect_Fill(188,152,24,16,BLACK);
	LCD_Font(188,168,itoa(Geschwindigkeit_kmh,buf,10),_8_Retro,1,WHITE);

}

void LenkwinkelAnzeige(void)
{
	int8_t LenkwinkelAnzeige=0;
	static uint8_t MerkerLenkwinkelAnzeige=0;
	LenkwinkelAnzeige = (int32_t) Lenkradwinkel*100/(LW_R-LW_L) + 50;
	if ( LenkwinkelAnzeige > 100 ) LenkwinkelAnzeige = 100 ;	 // obere Begrenzung
	if ( LenkwinkelAnzeige < 0) LenkwinkelAnzeige = 0 ;	     // untere Begrenzung

	if (LenkwinkelAnzeige>= 50)
	{
		LCD_Rect_Fill(110,51,49,21,BLACK);	     // Zeilen links von Balken l�schen, bis Mitte ran
		if (LenkwinkelAnzeige >= MerkerLenkwinkelAnzeige && MerkerLenkwinkelAnzeige!=0)
		{
			LCD_Rect_Fill(160,51,LenkwinkelAnzeige-50,21,YELLOW);
		}
		else if (LenkwinkelAnzeige < MerkerLenkwinkelAnzeige )
		{
			LCD_Rect_Fill(160+LenkwinkelAnzeige-50,51,100-LenkwinkelAnzeige,21,BLACK);	// Zeilen rechts von Balken l�schen, variabel
		}
	}
	if(LenkwinkelAnzeige < 50)
	{
		LCD_Rect_Fill(161,51,49,21,BLACK);	     // Zeilen Rechts von Balken l�schen, bis Mitte ran
		if (LenkwinkelAnzeige >= MerkerLenkwinkelAnzeige && MerkerLenkwinkelAnzeige!=0)
		{
			LCD_Rect_Fill(110,51,LenkwinkelAnzeige,21,BLACK);	// Zeilen rechts von Balken l�schen, variabel
		}
		else if (LenkwinkelAnzeige < MerkerLenkwinkelAnzeige )
		{
			LCD_Rect_Fill(110+LenkwinkelAnzeige,51,50-LenkwinkelAnzeige,21,YELLOW);
		}
	}
	MerkerLenkwinkelAnzeige=LenkwinkelAnzeige; // neuen Wert der Anzeige f�r n�chsten Durchlauf �bernehmen
}


void GPIOAnzeige (void)
{
	static uint8_t MerkerHauptrelais = 3;
	static uint8_t MerkerReverseGear = 3;
	if(Hauptrelais)
	{
		if((ReverseGear != MerkerReverseGear) && (ReverseGear))
		{
			LCD_Rect_Fill(56,29,208,18,BLUE);
			LCD_Font(56,45,"Rueckwaertsgang aktiviert",_8_Retro,1,WHITE);
		}
		else if((ReverseGear != MerkerReverseGear) && (ReverseGear == 0))
		{
			LCD_Rect_Fill(56,29,208,18,BLACK);
		}
	}
	else if(Hauptrelais != MerkerHauptrelais)
	{
		LCD_Rect_Fill(56,29,208,18,RED);
		LCD_Font(56,45,"   Fehler - Hauptrelais !  ",_8_Retro,1,WHITE);
	}
	MerkerHauptrelais = Hauptrelais;
	MerkerReverseGear = ReverseGear;
}

void RCP_show_connect(uint8_t pending)
{
	if(pending)
	{
		LCD_Rect_Fill(56,29,208,18,RED);
		LCD_Font(56,45,RCP_CONNECTING,_8_Retro,1,WHITE);
	} else {
		LCD_Rect_Fill(56,29,208,18,BLACK);
	}
}

void RCP_show_status(uint8_t RCP_Mode_status, uint8_t RCP_Mode_error)
{
	if(RCP_Mode_status != RCP_status_renew || RCP_Mode_error != RCP_error_renew)
	{
		LCD_Rect_Fill(150,60,150,80,BLACK);
		LCD_Font(150,80,getStatusString(RCP_Mode_status),_8_Retro,1,WHITE);
		LCD_Font(150,105,getErrorString(RCP_Mode_error),_8_Retro,1,WHITE);
		RCP_status_renew = RCP_Mode_status;
		RCP_error_renew = RCP_Mode_error;
	}

}


void EKartZustand(void)
{
	ZeitAnzeige();
	GPIOAnzeige();
	GasAnzeige();

	//Brems Anzeige
	BremsAnzeige();

	//Geschwindigkeitsanzeige
	Geschwindigkeitsanzeige();
	LenkwinkelAnzeige();

	StateofChargeAnzeige();
}


void Anzeige_Init(uint8_t menubene)
{
	switch(menubene)
	{
	case 1:		//E-Kart Zustand
	{
		LCD_Rect_Fill(0, 0, 320, 240, BLACK);
		Menu_Oben();					            // �berschrift, Fahrzeugrechner
		Menu_Unten();						        // Menu-Zeile unten, Optionen, Pfeile
		GeschwindigkeitsAnzeige_Init();
		BremsAnzeige_Init();
		GasAnzeige_Init();
		LenkwinkelAnzeige_Init(50);
		BatterieAnzeige_Init();
		RCPstatusAnzeige_Init();
	}
	break;

	case 2:		//Hauptmenu
	{
		LCD_Rect_Fill(0, 0, 320, 240, BLACK);
		Menu_Oben();					            // �berschrift, Fahrzeugrechner
		//Menu_Unten();						        // Menu-Zeile unten, Optionen, Pfeile
		LCD_Font(0,35,"Hauptmenu", _8_Retro,1,WHITE);
		WriteButton(menubene, 0);
		WriteButton(menubene, 1);
		WriteButton(menubene, 2);
		WriteButton(menubene, 3);
		WriteButton(menubene, 4);
//		WriteButton(menubene, 5);
	}
	break;

	case 3:		//Batterie
	{
		LCD_Rect_Fill(0, 0, 320, 240, BLACK);
		LCD_Font(0,35,"Batterie", _8_Retro,1,WHITE);
		BatterieMenu_Init();
		Menu_Oben();					            // �berschrift, Fahrzeugrechner
		Menu_Unten();						        // Menu-Zeile unten, Optionen, Pfeile
	}
	break;

	case 4:		//Fahrmodi �bersicht
	{
		LCD_Rect_Fill(0, 0, 320, 240, BLACK);
		LCD_Font(10,66,"Differenzial", _8_Retro,1,WHITE);
		LCD_Font(90,101,"FahrModi", _8_Retro,1,WHITE);
//		GUI_Text(10,50,"Differenzial:",Black,DarkGrey);
//		GUI_Text(90,85,"FahrModi:",Black,DarkGrey);
		Menu_Oben();					            // �berschrift, Fahrzeugrechner

		WriteButton(4,VorgabeDrehzahl);	           // Switch-Button ParameterAuswahl "Moment"
		WriteButton(4,VorgabeMoment);               // Switch-Button ParameterAuswahl "Drehzahl"
	    WriteButton(4,AnfaengerModeButton);         // Switch-Button "FahrModi Anfaenger"verlinkt 	6.menubene
	    WriteButton(4,StandardModeButton);          // Switch-Button "FahrModi Standard" verlinkt 	7.menubene
	    WriteButton(4,ProfiModeButton);             // Switch-Button "FahrModi Profi" 		verlinkt 	8.menubene
		WriteButton(4,TestModeButton);              // Switch-Button "FahrModi Test" 		verlinkt 	9.menubene
		WriteButton(4,6);   // Return to Men� (Ebene 4)
		switch (Flash_New_Parameters_List[VorgabeMomentDrehzahl]) {
			case VorgabeDrehzahl:
				MarkChosenButton(4,VorgabeDrehzahl);  // Vorgabedrehzahl nicht implementiert
				break;
			case VorgabeMoment:
				MarkChosenButton(4,VorgabeMoment);
				break;
			default:
				break;
		}
		switch (Flash_New_Parameters_List[ChosenDrivingMode]) {
					case AnfaengerModeButton:
						MarkChosenButton(4,AnfaengerModeButton);
						break;
					case StandardModeButton:
						MarkChosenButton(4,StandardModeButton);
						break;
					case ProfiModeButton:
						MarkChosenButton(4,ProfiModeButton);
						break;
					case TestModeButton:
						MarkChosenButton(4,TestModeButton);
						break;
					default:
						break;
				}
	}
	break;

    case 5:		//Fahrmodus Anf�nger
    {
    	LCD_Rect_Fill(0, 0, 320, 240, BLACK);
    	LCD_Font(0,35,"Modus:Anfaenger", _8_Retro,1,GREEN);
//    	LCD_Rect(220, 30, 90, 50, 1, WHITE);
    	WriteButton(5,0);	           // Switch-Button ParameterAuswahl "Moment"
    	WriteButton(5,1);
//    	LCD_Font(235,61,"FahrModi", _8_Retro,1,WHITE);
        Menu_Oben();					           	// �berschrift, Fahrzeugrechner
//		Menu_Fahrmodi();									// Menu-Zeile unten: Fahrmodi| E-Kart
		AnzeigeSlider_Init(5,0);          // initiale Anzeige 1.Gleiblocksystem
        AnzeigeSlider_Init(5,1);          // initiale Anzeige 2.Gleiblocksystem
		AnzeigeSlider_Init(5,2);          // initiale Anzeige 3.Gleiblocksystem
	}
    break;

    case 6:		//Fahrmodus Standart
    {
    	LCD_Rect_Fill(0, 0, 320, 240, BLACK);
    	LCD_Font(0,35,"Modus:Standart", _8_Retro,1,GREEN);

//    	LCD_Rect(220, 30, 90, 50, 1, WHITE);
//    	LCD_Font(235,61,"FahrModi", _8_Retro,1,WHITE);

    	WriteButton(6,0);	           // Switch-Button ParameterAuswahl "Moment"
    	WriteButton(6,1);

        Menu_Oben();					           	// �berschrift, Fahrzeugrechner
//		Menu_Fahrmodi();									// Menu-Zeile unten: Fahrmodi| E-Kart
		AnzeigeSlider_Init(6,0);          // initiale Anzeige 1.Gleiblocksystem
        AnzeigeSlider_Init(6,1);          // initiale Anzeige 2.Gleiblocksystem
		AnzeigeSlider_Init(6,2);          // initiale Anzeige 3.Gleiblocksystem
	}
    break;

    case 7:		//Fahrmodus Profi
    {
    	LCD_Rect_Fill(0, 0, 320, 240, BLACK);
    	LCD_Font(0,35,"Modus:Profi", _8_Retro,1,GREEN);

//    	LCD_Rect(220, 30, 90, 50, 1, WHITE);
//    	LCD_Font(235,61,"FahrModi", _8_Retro,1,WHITE);

    	WriteButton(7,0);	           // Switch-Button ParameterAuswahl "Moment"
    	WriteButton(7,1);

        Menu_Oben();					           	// �berschrift, Fahrzeugrechner
//		Menu_Fahrmodi();									// Menu-Zeile unten: Fahrmodi| E-Kart
		AnzeigeSlider_Init(7,0);          // initiale Anzeige 1.Gleiblocksystem
        AnzeigeSlider_Init(7,1);          // initiale Anzeige 2.Gleiblocksystem
		AnzeigeSlider_Init(7,2);          // initiale Anzeige 3.Gleiblocksystem
	}
    break;

    case 8:		//Fahrmodus Test
    {
    	LCD_Rect_Fill(0, 0, 320, 240, BLACK);
    	LCD_Font(0,35,"Modus:Test", _8_Retro,1,GREEN);

//    	LCD_Rect(220, 30, 90, 50, 1, WHITE);
//    	LCD_Font(235,61,"FahrModi", _8_Retro,1,WHITE);

    	WriteButton(8,0);	           // Switch-Button ParameterAuswahl "Moment"
    	WriteButton(8,1);

    	Menu_Oben();					           	// �berschrift, Fahrzeugrechner
//		Menu_Fahrmodi();									// Menu-Zeile unten: Fahrmodi| E-Kart
		AnzeigeSlider_Init(8,0);          // initiale Anzeige 1.Gleiblocksystem
        AnzeigeSlider_Init(8,1);          // initiale Anzeige 2.Gleiblocksystem
		AnzeigeSlider_Init(8,2);          // initiale Anzeige 3.Gleiblocksystem
	}
    break;

	case 9:		//Strom Einstellung
	{
    	LCD_Rect_Fill(0, 0, 320, 240, BLACK);
    	LCD_Font(0,35,"Strom-Ansteuerung", _8_Retro,1,GREEN);
		Menu_Oben();                     // �berschrift, Fahrzeugrechner
//		LCD_Rect(100, 210, 120, 27, 1, WHITE);
//		LCD_Font(140,231,"Menu", _8_Retro,1,WHITE);
    	WriteButton(9,0);	           // Switch-Button ParameterAuswahl "Moment"
    	WriteButton(9,1);
		AnzeigeSlider_Init(9,0);          // initiale Anzeige 1.Gleiblocksystem
	}
	break;

	case 10:	//Password
	{
    	LCD_Rect_Fill(0, 0, 320, 240, BLACK);
    	LCD_Font(45,23,"Code:", _8_Retro,1,WHITE);
    	ZeitAnzeige_Init();
		PasswordAnzeige_Init(menubene);
		//AnzeigeSliderInit(13,0);          // initiale Anzeige 1.Gleiblocksystem
	}
	break;

	case 11:	// RCP-Menu handling
	{

    	LCD_Rect_Fill(0, 0, 320, 240, BLACK);  // clean previous screen

    	LCD_Font(0,80,"RCP-Mode Status:", _8_Retro,1,WHITE);
    	LCD_Font(0,105,"RCP-Mode Errorcode:", _8_Retro,1,WHITE);
		LCD_Font(150,80,getStatusString(RCP_Mode_status),_8_Retro,1,WHITE);
		LCD_Font(150,105,getErrorString(RCP_Mode_errorcode),_8_Retro,1,WHITE);
		Menu_Oben();                     // �berschrift, Fahrzeugrechner
    	WriteButton(11,0);	           // Switch-Button zurueck zum "Hauptmenu"
    	WriteButton(11,1);			// Switch-Button RCP Connect

	}
	break;

    default: break;

	}
}
void getTouch(void)
{
	touchX = getX();
	touchY = getY();
}

uint8_t TouchAction(uint8_t menunummer)
{
	static uint8_t ret=1;
	static uint8_t touch=0;
	getTouch();
	if ((touchX && touchY) && (touch == 0))
	{
		touch=1;
		switch(menunummer)
		{
		case 1:			//Buttons in der E-Kart Anzeige
		{
			if (S_Button[menunummer][0].x1<touchX&&S_Button[menunummer][0].x2>touchX&&S_Button[menunummer][0].y1<touchY&&S_Button[menunummer][0].y2>touchY)
			{
				Anzeige_Init(S_Button[menunummer][0].Menuverlinkung);
				ret = S_Button[menunummer][0].Menuverlinkung;
			}
			if (S_Button[menunummer][1].x1<touchX&&S_Button[menunummer][1].x2>touchX&&S_Button[menunummer][1].y1<touchY&&S_Button[menunummer][1].y2>touchY)
			{
				Anzeige_Init(S_Button[menunummer][1].Menuverlinkung);
				ret = S_Button[menunummer][1].Menuverlinkung;
			}
			if (S_Button[menunummer][2].x1<touchX&&S_Button[menunummer][2].x2>touchX&&S_Button[menunummer][2].y1<touchY&&S_Button[menunummer][2].y2>touchY)
			{
				Anzeige_Init(S_Button[menunummer][2].Menuverlinkung);
				ret = S_Button[menunummer][2].Menuverlinkung;
			}
		}
		break;

		case 2:			//Buttons im Hauptmenu
		{
			if (S_Button[menunummer][4].x1<touchX&&S_Button[menunummer][4].x2>touchX&&S_Button[menunummer][4].y1<touchY&&S_Button[menunummer][4].y2>touchY)
			{
				Anzeige_Init(S_Button[menunummer][4].Menuverlinkung);
				ret = S_Button[menunummer][4].Menuverlinkung;
			}
			if (S_Button[menunummer][1].x1<touchX&&S_Button[menunummer][1].x2>touchX&&S_Button[menunummer][1].y1<touchY&&S_Button[menunummer][1].y2>touchY)
			{
				Anzeige_Init(S_Button[menunummer][1].Menuverlinkung);
				ret = S_Button[menunummer][1].Menuverlinkung;
			}
			if (S_Button[menunummer][2].x1<touchX&&S_Button[menunummer][2].x2>touchX&&S_Button[menunummer][2].y1<touchY&&S_Button[menunummer][2].y2>touchY)
			{
				Anzeige_Init(S_Button[menunummer][2].Menuverlinkung);
				ret = S_Button[menunummer][2].Menuverlinkung;
			}
			if (S_Button[menunummer][3].x1<touchX&&S_Button[menunummer][3].x2>touchX&&S_Button[menunummer][3].y1<touchY&&S_Button[menunummer][3].y2>touchY)
			{
				Anzeige_Init(S_Button[menunummer][3].Menuverlinkung);
				ret = S_Button[menunummer][3].Menuverlinkung;
			}

			if (S_Button[menunummer][0].x1<touchX&&S_Button[menunummer][0].x2>touchX&&S_Button[menunummer][0].y1<touchY&&S_Button[menunummer][0].y2>touchY)
			{
				Anzeige_Init(S_Button[menunummer][0].Menuverlinkung);
				ret = S_Button[menunummer][0].Menuverlinkung;
			}
		}
		break;

		case 3:			//Buttons in der Batterieanzeige
		{
			if (S_Button[menunummer][0].x1<touchX&&S_Button[menunummer][0].x2>touchX&&S_Button[menunummer][0].y1<touchY&&S_Button[menunummer][0].y2>touchY)
			{
				Anzeige_Init(S_Button[menunummer][0].Menuverlinkung);
				ret = S_Button[menunummer][0].Menuverlinkung;
			}
			if (S_Button[menunummer][1].x1<touchX&&S_Button[menunummer][1].x2>touchX&&S_Button[menunummer][1].y1<touchY&&S_Button[menunummer][1].y2>touchY)
			{
				Anzeige_Init(S_Button[menunummer][1].Menuverlinkung);
				ret = S_Button[menunummer][1].Menuverlinkung;
			}
			if (S_Button[menunummer][2].x1<touchX&&S_Button[menunummer][2].x2>touchX&&S_Button[menunummer][2].y1<touchY&&S_Button[menunummer][2].y2>touchY)
			{
				Anzeige_Init(S_Button[menunummer][2].Menuverlinkung);
				ret = S_Button[menunummer][2].Menuverlinkung;
			}
		}
		break;

		case 4:			//Buttons im Fahrmodus �bersicht Menu
		{
			if (S_Button[menunummer][0].x1<touchX&&S_Button[menunummer][0].x2>touchX&&S_Button[menunummer][0].y1<touchY&&S_Button[menunummer][0].y2>touchY)
			{
//				Flash_New_Parameters_List[VorgabeMomentDrehzahl]=VorgabeDrehzahl;

				Anzeige_Init(S_Button[menunummer][0].Menuverlinkung);
				ret = S_Button[menunummer][0].Menuverlinkung;
			}
			if (S_Button[menunummer][1].x1<touchX&&S_Button[menunummer][1].x2>touchX&&S_Button[menunummer][1].y1<touchY&&S_Button[menunummer][1].y2>touchY)
			{
				Flash_New_Parameters_List[VorgabeMomentDrehzahl]=VorgabeMoment;

				Anzeige_Init(S_Button[menunummer][1].Menuverlinkung);
				ret = S_Button[menunummer][1].Menuverlinkung;
			}
			if (S_Button[menunummer][2].x1<touchX&&S_Button[menunummer][2].x2>touchX&&S_Button[menunummer][2].y1<touchY&&S_Button[menunummer][2].y2>touchY)
			{
				Flash_New_Parameters_List[ChosenDrivingMode]=AnfaengerModeButton;

				Pointer_GasProzent=&Flash_New_Parameters_List[Anfaenger_GasProzent];				// POINTER nehme von diesem Listenelement-Speicherort den Wert an
				Pointer_BeschlProzent=&Flash_New_Parameters_List[Anfaenger_BeschlProzent];	// f�r �nderungen in stm32f10x_it.c
				Pointer_Rueckwaert=&Flash_New_Parameters_List[Anfaenger_Rueckwaert];

				Anzeige_Init(S_Button[menunummer][2].Menuverlinkung);
				ret = S_Button[menunummer][2].Menuverlinkung;
			}
			if (S_Button[menunummer][3].x1<touchX&&S_Button[menunummer][3].x2>touchX&&S_Button[menunummer][3].y1<touchY&&S_Button[menunummer][3].y2>touchY)
			{
				Flash_New_Parameters_List[ChosenDrivingMode]=StandardModeButton;

				Pointer_GasProzent=&Flash_New_Parameters_List[Standard_GasProzent];					// ___POINTER_____
				Pointer_BeschlProzent=&Flash_New_Parameters_List[Standard_BeschlProzent];
				Pointer_Rueckwaert=&Flash_New_Parameters_List[Standard_Rueckwaert];

				Anzeige_Init(S_Button[menunummer][3].Menuverlinkung);
				ret = S_Button[menunummer][3].Menuverlinkung;
			}
			if (S_Button[menunummer][4].x1<touchX&&S_Button[menunummer][4].x2>touchX&&S_Button[menunummer][4].y1<touchY&&S_Button[menunummer][4].y2>touchY)
			{
				Flash_New_Parameters_List[ChosenDrivingMode]=ProfiModeButton;

				Pointer_GasProzent=&Flash_New_Parameters_List[Profi_GasProzent]; 						// ___POINTER_____
				Pointer_BeschlProzent=&Flash_New_Parameters_List[Profi_BeschlProzent];
				Pointer_Rueckwaert=&Flash_New_Parameters_List[Profi_Rueckwaert];

				Anzeige_Init(S_Button[menunummer][4].Menuverlinkung);
				ret = S_Button[menunummer][4].Menuverlinkung;
			}
			if (S_Button[menunummer][5].x1<touchX&&S_Button[menunummer][5].x2>touchX&&S_Button[menunummer][5].y1<touchY&&S_Button[menunummer][5].y2>touchY)
			{
				Flash_New_Parameters_List[ChosenDrivingMode]=TestModeButton;

				Pointer_GasProzent=&Flash_New_Parameters_List[Test_GasProzent];  						// ___POINTER_____
				Pointer_BeschlProzent=&Flash_New_Parameters_List[Test_BeschlProzent];
				Pointer_Rueckwaert=&Flash_New_Parameters_List[Test_Rueckwaert];

				Anzeige_Init(S_Button[menunummer][5].Menuverlinkung);
				ret = S_Button[menunummer][5].Menuverlinkung;
			}
			if (S_Button[menunummer][6].x1<touchX&&S_Button[menunummer][6].x2>touchX&&S_Button[menunummer][6].y1<touchY&&S_Button[menunummer][6].y2>touchY)
			{
				Anzeige_Init(S_Button[menunummer][6].Menuverlinkung);
				ret = S_Button[menunummer][6].Menuverlinkung;
			}
		}
		break;

		case 5:			//Buttons im Anfaenger Fahrmodus Menu
		{
			if (S_Button[menunummer][0].x1<touchX&&S_Button[menunummer][0].x2>touchX&&S_Button[menunummer][0].y1<touchY&&S_Button[menunummer][0].y2>touchY)
			{
//				TransferListElements(0);

//				Flash_New_Parameters_List[Anfaenger_GasProzent]=Slider[menunummer][0].SliderWert;
//				Flash_New_Parameters_List[10]=Slider[menunummer][0].x;
//				Flash_New_Parameters_List[Anfaenger_BeschlProzent]=Slider[menunummer][1].SliderWert;
//				Flash_New_Parameters_List[11]=Slider[menunummer][1].x;
//				Flash_New_Parameters_List[Anfaenger_Rueckwaert]=Slider[menunummer][2].SliderWert;
//				Flash_New_Parameters_List[26]=Slider[menunummer][2].x;
				Anzeige_Init(S_Button[menunummer][0].Menuverlinkung);
				ret = S_Button[menunummer][0].Menuverlinkung;
			}
			if (S_Button[menunummer][1].x1<touchX&&S_Button[menunummer][1].x2>touchX&&S_Button[menunummer][1].y1<touchY&&S_Button[menunummer][1].y2>touchY)
			{
//				Flash_New_Parameters_List[Anfaenger_GasProzent]=Slider[menunummer][0].SliderWert;
//				Flash_New_Parameters_List[10]=Slider[menunummer][0].x;
//				Flash_New_Parameters_List[Anfaenger_BeschlProzent]=Slider[menunummer][1].SliderWert;
//				Flash_New_Parameters_List[11]=Slider[menunummer][1].x;
//				Flash_New_Parameters_List[Anfaenger_Rueckwaert]=Slider[menunummer][2].SliderWert;
//				Flash_New_Parameters_List[26]=Slider[menunummer][2].x;
				TransferListElements(1);
				SavetoFlash();
				Anzeige_Init(S_Button[menunummer][0].Menuverlinkung);
				ret = S_Button[menunummer][0].Menuverlinkung;
			}
		}
		break;

		case 6:			//Buttons im Standart Fahrmodus Menu
		{
			if (S_Button[menunummer][0].x1<touchX&&S_Button[menunummer][0].x2>touchX&&S_Button[menunummer][0].y1<touchY&&S_Button[menunummer][0].y2>touchY)
			{
//				TransferListElements(0);

//				Flash_New_Parameters_List[Standard_GasProzent]=Slider[menunummer][0].SliderWert;
//				Flash_New_Parameters_List[12]=Slider[menunummer][0].x;
//				Flash_New_Parameters_List[Standard_BeschlProzent]=Slider[menunummer][1].SliderWert;
//				Flash_New_Parameters_List[13]=Slider[menunummer][1].x;
//				Flash_New_Parameters_List[Standard_Rueckwaert]=Slider[menunummer][2].SliderWert;
//				Flash_New_Parameters_List[27]=Slider[menunummer][2].x;
				Anzeige_Init(S_Button[menunummer][0].Menuverlinkung);
				ret = S_Button[menunummer][0].Menuverlinkung;
			}
			if (S_Button[menunummer][1].x1<touchX&&S_Button[menunummer][1].x2>touchX&&S_Button[menunummer][1].y1<touchY&&S_Button[menunummer][1].y2>touchY)
			{
//				Flash_New_Parameters_List[Standard_GasProzent]=Slider[menunummer][0].SliderWert;
//				Flash_New_Parameters_List[12]=Slider[menunummer][0].x;
//				Flash_New_Parameters_List[Standard_BeschlProzent]=Slider[menunummer][1].SliderWert;
//				Flash_New_Parameters_List[13]=Slider[menunummer][1].x;
//				Flash_New_Parameters_List[Standard_Rueckwaert]=Slider[menunummer][2].SliderWert;
//				Flash_New_Parameters_List[27]=Slider[menunummer][2].x;
				TransferListElements(1);
				SavetoFlash();
				Anzeige_Init(S_Button[menunummer][0].Menuverlinkung);
				ret = S_Button[menunummer][0].Menuverlinkung;
			}
		}
		break;

		case 7:			//Buttons im Profi Fahrmodus Menu
		{
			if (S_Button[menunummer][0].x1<touchX&&S_Button[menunummer][0].x2>touchX&&S_Button[menunummer][0].y1<touchY&&S_Button[menunummer][0].y2>touchY)
			{
//				TransferListElements(0);

//				Flash_New_Parameters_List[Profi_GasProzent]=Slider[menunummer][0].SliderWert;
//				Flash_New_Parameters_List[14]=Slider[menunummer][0].x;
//				Flash_New_Parameters_List[Profi_BeschlProzent]=Slider[menunummer][1].SliderWert;
//				Flash_New_Parameters_List[15]=Slider[menunummer][1].x;
//				Flash_New_Parameters_List[Profi_Rueckwaert]=Slider[menunummer][2].SliderWert;
//				Flash_New_Parameters_List[28]=Slider[menunummer][2].x;
				Anzeige_Init(S_Button[menunummer][0].Menuverlinkung);
				ret = S_Button[menunummer][0].Menuverlinkung;
			}
			if (S_Button[menunummer][1].x1<touchX&&S_Button[menunummer][1].x2>touchX&&S_Button[menunummer][1].y1<touchY&&S_Button[menunummer][1].y2>touchY)
			{
//				Flash_New_Parameters_List[Profi_GasProzent]=Slider[menunummer][0].SliderWert;
//				Flash_New_Parameters_List[14]=Slider[menunummer][0].x;
//				Flash_New_Parameters_List[Profi_BeschlProzent]=Slider[menunummer][1].SliderWert;
//				Flash_New_Parameters_List[15]=Slider[menunummer][1].x;
//				Flash_New_Parameters_List[Profi_Rueckwaert]=Slider[menunummer][2].SliderWert;
//				Flash_New_Parameters_List[28]=Slider[menunummer][2].x;
				TransferListElements(1);
				SavetoFlash();
				Anzeige_Init(S_Button[menunummer][0].Menuverlinkung);
				ret = S_Button[menunummer][0].Menuverlinkung;
			}
		}
		break;

		case 8:			//Buttons im Test Fahrmodus Menu
		{
			if (S_Button[menunummer][0].x1<touchX&&S_Button[menunummer][0].x2>touchX&&S_Button[menunummer][0].y1<touchY&&S_Button[menunummer][0].y2>touchY)
			{
//				TransferListElements(0);

//				Flash_New_Parameters_List[Test_GasProzent]=Slider[menunummer][0].SliderWert;
//				Flash_New_Parameters_List[16]=Slider[menunummer][0].x;
//				Flash_New_Parameters_List[Test_BeschlProzent]=Slider[menunummer][1].SliderWert;
//				Flash_New_Parameters_List[17]=Slider[menunummer][1].x;
//				Flash_New_Parameters_List[Test_Rueckwaert]=Slider[menunummer][2].SliderWert;
//				Flash_New_Parameters_List[29]=Slider[menunummer][2].x;
				Anzeige_Init(S_Button[menunummer][0].Menuverlinkung);
				ret = S_Button[menunummer][0].Menuverlinkung;
			}
			if (S_Button[menunummer][1].x1<touchX&&S_Button[menunummer][1].x2>touchX&&S_Button[menunummer][1].y1<touchY&&S_Button[menunummer][1].y2>touchY)
			{
//				Flash_New_Parameters_List[Test_GasProzent]=Slider[menunummer][0].SliderWert;
//				Flash_New_Parameters_List[16]=Slider[menunummer][0].x;
//				Flash_New_Parameters_List[Test_BeschlProzent]=Slider[menunummer][1].SliderWert;
//				Flash_New_Parameters_List[17]=Slider[menunummer][1].x;
//				Flash_New_Parameters_List[Test_Rueckwaert]=Slider[menunummer][2].SliderWert;
//				Flash_New_Parameters_List[29]=Slider[menunummer][2].x;
				TransferListElements(1);
				SavetoFlash();
				Anzeige_Init(S_Button[menunummer][0].Menuverlinkung);
				ret = S_Button[menunummer][0].Menuverlinkung;
			}
		}
		break;

		case 9:			//Buttons in Strom Einstellung Menu
		{
			if (S_Button[menunummer][0].x1<touchX&&S_Button[menunummer][0].x2>touchX&&S_Button[menunummer][0].y1<touchY&&S_Button[menunummer][0].y2>touchY)
			{
//				TransferListElements(0);

				Anzeige_Init(S_Button[menunummer][0].Menuverlinkung);
				ret = S_Button[menunummer][0].Menuverlinkung;
			}
			if (S_Button[menunummer][1].x1<touchX&&S_Button[menunummer][1].x2>touchX&&S_Button[menunummer][1].y1<touchY&&S_Button[menunummer][1].y2>touchY)
			{
				TransferListElements(1);
				SavetoFlash();
				Anzeige_Init(S_Button[menunummer][0].Menuverlinkung);
				ret = S_Button[menunummer][0].Menuverlinkung;
			}
		}
		break;

		case 10:		//Buttons im Password Menu
		{
			if (S_Button[menunummer][0].x1<touchX&&S_Button[menunummer][0].x2>touchX&&S_Button[menunummer][0].y1<touchY&&S_Button[menunummer][0].y2>touchY)
			{
				if(sCode <= 3)
				{
					sCode++;
					PasswordCodeUser[sCode-1] = S_Button[menunummer][0].Wert;
				}
				ret = S_Button[menunummer][0].Menuverlinkung;
			}
			if (S_Button[menunummer][1].x1<touchX&&S_Button[menunummer][1].x2>touchX&&S_Button[menunummer][1].y1<touchY&&S_Button[menunummer][1].y2>touchY)
			{
				if(sCode <= 3)
				{
					sCode++;
					PasswordCodeUser[sCode-1] = S_Button[menunummer][1].Wert;
				}
				ret = S_Button[menunummer][1].Menuverlinkung;
			}
			if (S_Button[menunummer][2].x1<touchX&&S_Button[menunummer][2].x2>touchX&&S_Button[menunummer][2].y1<touchY&&S_Button[menunummer][2].y2>touchY)
			{
				if(sCode <= 3)
				{
					sCode++;
					PasswordCodeUser[sCode-1] = S_Button[menunummer][2].Wert;
				}
				ret = S_Button[menunummer][2].Menuverlinkung;
			}
			if (S_Button[menunummer][3].x1<touchX&&S_Button[menunummer][3].x2>touchX&&S_Button[menunummer][3].y1<touchY&&S_Button[menunummer][3].y2>touchY)
			{
				if(sCode <= 3)
				{
					sCode++;
					PasswordCodeUser[sCode-1] = S_Button[menunummer][3].Wert;
				}
				ret = S_Button[menunummer][3].Menuverlinkung;
			}
			if (S_Button[menunummer][4].x1<touchX&&S_Button[menunummer][4].x2>touchX&&S_Button[menunummer][4].y1<touchY&&S_Button[menunummer][4].y2>touchY)
			{
				if(sCode <= 3)
				{
					sCode++;
					PasswordCodeUser[sCode-1] = S_Button[menunummer][4].Wert;
				}
				ret = S_Button[menunummer][4].Menuverlinkung;
			}
			if (S_Button[menunummer][5].x1<touchX&&S_Button[menunummer][5].x2>touchX&&S_Button[menunummer][5].y1<touchY&&S_Button[menunummer][5].y2>touchY)
			{
				if(sCode <= 3)
				{
					sCode++;
					PasswordCodeUser[sCode-1] = S_Button[menunummer][5].Wert;
				}
				ret = S_Button[menunummer][5].Menuverlinkung;
			}
			if (S_Button[menunummer][6].x1<touchX&&S_Button[menunummer][6].x2>touchX&&S_Button[menunummer][6].y1<touchY&&S_Button[menunummer][6].y2>touchY)
			{
				if(sCode <= 3)
				{
					sCode++;
					PasswordCodeUser[sCode-1] = S_Button[menunummer][6].Wert;
				}
				ret = S_Button[menunummer][6].Menuverlinkung;
			}
			if (S_Button[menunummer][7].x1<touchX&&S_Button[menunummer][7].x2>touchX&&S_Button[menunummer][7].y1<touchY&&S_Button[menunummer][7].y2>touchY)
			{
				if(sCode <= 3)
				{
					sCode++;
					PasswordCodeUser[sCode-1] = S_Button[menunummer][7].Wert;
				}
				ret = S_Button[menunummer][7].Menuverlinkung;
			}
			if (S_Button[menunummer][8].x1<touchX&&S_Button[menunummer][8].x2>touchX&&S_Button[menunummer][8].y1<touchY&&S_Button[menunummer][8].y2>touchY)
			{
				if(sCode <= 3)
				{
					sCode++;
					PasswordCodeUser[sCode-1] = S_Button[menunummer][8].Wert;;
				}
				ret = S_Button[menunummer][8].Menuverlinkung;
			}
			if (S_Button[menunummer][9].x1<touchX&&S_Button[menunummer][9].x2>touchX&&S_Button[menunummer][9].y1<touchY&&S_Button[menunummer][9].y2>touchY)
			{
				if(sCode <= 3)
				{
					sCode++;
					PasswordCodeUser[sCode-1] = S_Button[menunummer][9].Wert;
				}
				ret = S_Button[menunummer][9].Menuverlinkung;

			}
			if (S_Button[menunummer][10].x1<touchX&&S_Button[menunummer][10].x2>touchX&&S_Button[menunummer][10].y1<touchY&&S_Button[menunummer][10].y2>touchY)
			{
				PasswordCodeUser[0]=0;
				PasswordCodeUser[1]=0;
				PasswordCodeUser[2]=0;
				PasswordCodeUser[3]=0;
				sCode=0;
				Anzeige_Init(S_Button[menunummer][10].Menuverlinkung);
				ret = S_Button[menunummer][10].Menuverlinkung;
			}
			if (S_Button[menunummer][11].x1<touchX&&S_Button[menunummer][11].x2>touchX&&S_Button[menunummer][11].y1<touchY&&S_Button[menunummer][11].y2>touchY)
			{
				if((sCode == 4) && (PasswordCode[0] == PasswordCodeUser[0]) && (PasswordCode[1] == PasswordCodeUser[1]) && (PasswordCode[2] == PasswordCodeUser[2]) && (PasswordCode[3] == PasswordCodeUser[3]))
				{
					PasswordCodeUser[0]=0;
					PasswordCodeUser[1]=0;
					PasswordCodeUser[2]=0;
					PasswordCodeUser[3]=0;
					sCode=0;
					Anzeige_Init(S_Button[menunummer][11].Menuverlinkung);
					ret = S_Button[menunummer][11].Menuverlinkung;
				}
				else
				{
					ret = menunummer;
				}
			}
			if (S_Button[menunummer][12].x1<touchX&&S_Button[menunummer][12].x2>touchX&&S_Button[menunummer][12].y1<touchY&&S_Button[menunummer][12].y2>touchY)
			{
				if(sCode >= 1)
				{
					sCode--;
				}
				ret = S_Button[menunummer][12].Menuverlinkung;
			}

		}
		break;
		case 11:			//Buttons in "RCP-Mode" menu
		{
			if (S_Button[menunummer][0].x1<touchX&&S_Button[menunummer][0].x2>touchX&&S_Button[menunummer][0].y1<touchY&&S_Button[menunummer][0].y2>touchY)
			{
				// button Just go back to "Main Menu"
				Anzeige_Init(S_Button[menunummer][0].Menuverlinkung);
				ret = S_Button[menunummer][0].Menuverlinkung;
			}
			if (S_Button[menunummer][1].x1<touchX&&S_Button[menunummer][1].x2>touchX&&S_Button[menunummer][1].y1<touchY&&S_Button[menunummer][1].y2>touchY)
			{
				// button enable rcp mode
				RCP_Mode_selected = !RCP_Mode_selected;
				RCP_Mode_pending = TRUE;

				ret = S_Button[menunummer][1].Menuverlinkung;
			}
		}
		break;
		default:break;

		}
	}
	else if ((touchX && touchY) && (touch == 1))
	{
		switch(menunummer)
		{
		case 5:			//Slider im Anfaenger Fahrmodus Menu
		{
			if((touchX>Slider[menunummer][0].Touch_x1) && (touchX<Slider[menunummer][0].Touch_x2) && (touchY>Slider[menunummer][0].Touch_y1) && (touchY<Slider[menunummer][0].Touch_y2))
			{
				TouchSlider(menunummer,0);
				Flash_New_Parameters_List[Anfaenger_GasProzent]=Slider[menunummer][0].SliderWert;
				Flash_New_Parameters_List[10]=Slider[menunummer][0].x;
			}
			if((touchX>Slider[menunummer][1].Touch_x1) && (touchX<Slider[menunummer][1].Touch_x2) && (touchY>Slider[menunummer][1].Touch_y1) && (touchY<Slider[menunummer][1].Touch_y2))
			{
				TouchSlider(menunummer,1);
				Flash_New_Parameters_List[Anfaenger_BeschlProzent]=Slider[menunummer][1].SliderWert;
				Flash_New_Parameters_List[11]=Slider[menunummer][1].x;
			}
			if((touchX>Slider[menunummer][2].Touch_x1) && (touchX<Slider[menunummer][2].Touch_x2) && (touchY>Slider[menunummer][2].Touch_y1) && (touchY<Slider[menunummer][2].Touch_y2))
			{
				TouchSlider(menunummer,2);
				Flash_New_Parameters_List[Anfaenger_Rueckwaert]=Slider[menunummer][2].SliderWert;
				Flash_New_Parameters_List[26]=Slider[menunummer][2].x;
			}
		}
		break;

		case 6:			//Slider im Standart Fahrmodus Menu
		{
			if((touchX>Slider[menunummer][0].Touch_x1) && (touchX<Slider[menunummer][0].Touch_x2) && (touchY>Slider[menunummer][0].Touch_y1) && (touchY<Slider[menunummer][0].Touch_y2))
			{
				TouchSlider(menunummer,0);
				Flash_New_Parameters_List[Standard_GasProzent]=Slider[menunummer][0].SliderWert;
				Flash_New_Parameters_List[12]=Slider[menunummer][0].x;
			}
			if((touchX>Slider[menunummer][1].Touch_x1) && (touchX<Slider[menunummer][1].Touch_x2) && (touchY>Slider[menunummer][1].Touch_y1) && (touchY<Slider[menunummer][1].Touch_y2))
			{
				TouchSlider(menunummer,1);
				Flash_New_Parameters_List[Standard_BeschlProzent]=Slider[menunummer][1].SliderWert;
				Flash_New_Parameters_List[13]=Slider[menunummer][1].x;
			}
			if((touchX>Slider[menunummer][2].Touch_x1) && (touchX<Slider[menunummer][2].Touch_x2) && (touchY>Slider[menunummer][2].Touch_y1) && (touchY<Slider[menunummer][2].Touch_y2))
			{
				TouchSlider(menunummer,2);
				Flash_New_Parameters_List[Standard_Rueckwaert]=Slider[menunummer][2].SliderWert;
				Flash_New_Parameters_List[27]=Slider[menunummer][2].x;
			}
		}
		break;

		case 7:			//Slider im Profi Fahrmodus Menu
		{
			if((touchX>Slider[menunummer][0].Touch_x1) && (touchX<Slider[menunummer][0].Touch_x2) && (touchY>Slider[menunummer][0].Touch_y1) && (touchY<Slider[menunummer][0].Touch_y2))
			{
				TouchSlider(menunummer,0);
				Flash_New_Parameters_List[Profi_GasProzent]=Slider[menunummer][0].SliderWert;
				Flash_New_Parameters_List[14]=Slider[menunummer][0].x;
			}
			if((touchX>Slider[menunummer][1].Touch_x1) && (touchX<Slider[menunummer][1].Touch_x2) && (touchY>Slider[menunummer][1].Touch_y1) && (touchY<Slider[menunummer][1].Touch_y2))
			{
				TouchSlider(menunummer,1);
				Flash_New_Parameters_List[Profi_BeschlProzent]=Slider[menunummer][1].SliderWert;
				Flash_New_Parameters_List[15]=Slider[menunummer][1].x;
			}
			if((touchX>Slider[menunummer][2].Touch_x1) && (touchX<Slider[menunummer][2].Touch_x2) && (touchY>Slider[menunummer][2].Touch_y1) && (touchY<Slider[menunummer][2].Touch_y2))
			{
				TouchSlider(menunummer,2);
				Flash_New_Parameters_List[Profi_Rueckwaert]=Slider[menunummer][2].SliderWert;
				Flash_New_Parameters_List[28]=Slider[menunummer][2].x;
			}
		}
		break;

		case 8:			//Slider im Test Fahrmodus Menu
		{
			if((touchX>Slider[menunummer][0].Touch_x1) && (touchX<Slider[menunummer][0].Touch_x2) && (touchY>Slider[menunummer][0].Touch_y1) && (touchY<Slider[menunummer][0].Touch_y2))
			{
				TouchSlider(menunummer,0);
				Flash_New_Parameters_List[Test_GasProzent]=Slider[menunummer][0].SliderWert;
				Flash_New_Parameters_List[16]=Slider[menunummer][0].x;
			}
			if((touchX>Slider[menunummer][1].Touch_x1) && (touchX<Slider[menunummer][1].Touch_x2) && (touchY>Slider[menunummer][1].Touch_y1) && (touchY<Slider[menunummer][1].Touch_y2))
			{
				TouchSlider(menunummer,1);
				Flash_New_Parameters_List[Test_BeschlProzent]=Slider[menunummer][1].SliderWert;
				Flash_New_Parameters_List[17]=Slider[menunummer][1].x;
			}
			if((touchX>Slider[menunummer][2].Touch_x1) && (touchX<Slider[menunummer][2].Touch_x2) && (touchY>Slider[menunummer][2].Touch_y1) && (touchY<Slider[menunummer][2].Touch_y2))
			{
				TouchSlider(menunummer,2);
				Flash_New_Parameters_List[Test_Rueckwaert]=Slider[menunummer][2].SliderWert;
				Flash_New_Parameters_List[29]=Slider[menunummer][2].x;
			}
		}
		break;

		case 9:			//Slider im Strom Einstellung Menu
		{
			if((touchX>Slider[menunummer][0].Touch_x1) && (touchX<Slider[menunummer][0].Touch_x2) && (touchY>Slider[menunummer][0].Touch_y1) && (touchY<Slider[menunummer][0].Touch_y2))
			{
				TouchSlider(menunummer,0);
				Flash_New_Parameters_List[Strom]=Slider[menunummer][0].SliderWert;
			}
		}
		break;

		default: ret = menunummer;
		}
	}
	else if (!(touchX && touchY) && (touch == 1)) touch=0;

	return ret;
}

/*******************************************************************************
* Function Name  :  WriteButton
* Description    :  Aufzeichnung des Buttons
* Input          :  menunummer: welche menubene 1,2,3,4,5,6,7,8,9,10
                    Buttonnummer: welcher Button 0,1,2,3,4,5
* Output         :  None
* Return         :  None
*******************************************************************************/

/* Menuebenen
 * 1. Zustand-Anzeige
 * 2."Menu"
 * 3. Batteriestatus
 * 4. "Fahrmodi - �bersicht"
 * 5. "FahrModi-Anfaenger"
 * 6. "FahrModi-Anfaenger"
 * 7. "FahrModi-Profi"
 * 8. "FahrModi-Test"
 * 9. Strom
 * 10. Password
 * 11. Menu "RCP-Mode"
 */




void Menustruktur(void)
{
	uint8_t menunummer;
	uint8_t buttonnummer;
	uint8_t slidernummer;

	// menubene 1
	// Zustand-Anzeige f�r EKART
	menunummer=1;

	buttonnummer=0;
	S_Button[menunummer][buttonnummer].x1=1;
	S_Button[menunummer][buttonnummer].y1=210;
	S_Button[menunummer][buttonnummer].x2=95;
	S_Button[menunummer][buttonnummer].y2=237;
	S_Button[menunummer][buttonnummer].Menuverlinkung=3;
	S_Button[menunummer][buttonnummer].bottonsanzahl=3;


	buttonnummer=1;
	S_Button[menunummer][buttonnummer].x1=100;
	S_Button[menunummer][buttonnummer].y1=210;
	S_Button[menunummer][buttonnummer].x2=220;
	S_Button[menunummer][buttonnummer].y2=237;
	S_Button[menunummer][buttonnummer].Menuverlinkung=10;
	S_Button[menunummer][buttonnummer].bottonsanzahl=3;


	buttonnummer=2;
	S_Button[menunummer][buttonnummer].x1=225;
	S_Button[menunummer][buttonnummer].y1=210;
	S_Button[menunummer][buttonnummer].x2=318;
	S_Button[menunummer][buttonnummer].y2=237;
	S_Button[menunummer][buttonnummer].Menuverlinkung=3;
	S_Button[menunummer][buttonnummer].bottonsanzahl=3;

	// menubene 2
	// "Menu"
	menunummer=2;

	buttonnummer=0;
	S_Button[menunummer][buttonnummer].x1=10;
	S_Button[menunummer][buttonnummer].y1=50;
	S_Button[menunummer][buttonnummer].x2=155;
	S_Button[menunummer][buttonnummer].y2=100;
	S_Button[menunummer][buttonnummer].Menuverlinkung=11;
	S_Button[menunummer][buttonnummer].Text="RCP-Mode";
	S_Button[menunummer][buttonnummer].Textlaenge=8;
	S_Button[menunummer][buttonnummer].bottonsanzahl=5;

	buttonnummer=1;
	S_Button[menunummer][buttonnummer].x1=165;
	S_Button[menunummer][buttonnummer].y1=50;
	S_Button[menunummer][buttonnummer].x2=310;
	S_Button[menunummer][buttonnummer].y2=100;
	S_Button[menunummer][buttonnummer].Menuverlinkung=3;
	S_Button[menunummer][buttonnummer].Text="Batterie";
	S_Button[menunummer][buttonnummer].Textlaenge=8;
	S_Button[menunummer][buttonnummer].bottonsanzahl=5;

	buttonnummer=2;
	S_Button[menunummer][buttonnummer].x1=10;
	S_Button[menunummer][buttonnummer].y1=110;
	S_Button[menunummer][buttonnummer].x2=155;
	S_Button[menunummer][buttonnummer].y2=160;
	S_Button[menunummer][buttonnummer].Menuverlinkung=4;
	S_Button[menunummer][buttonnummer].Text="FahrModi";
	S_Button[menunummer][buttonnummer].Textlaenge=8;
	S_Button[menunummer][buttonnummer].bottonsanzahl=5;

	buttonnummer=3;
	S_Button[menunummer][buttonnummer].x1=165;
	S_Button[menunummer][buttonnummer].y1=110;
	S_Button[menunummer][buttonnummer].x2=310;
	S_Button[menunummer][buttonnummer].y2=160;
	S_Button[menunummer][buttonnummer].Menuverlinkung=9;
	S_Button[menunummer][buttonnummer].Text="Strom";
	S_Button[menunummer][buttonnummer].Textlaenge=5;
	S_Button[menunummer][buttonnummer].bottonsanzahl=5;


	buttonnummer=4;
	S_Button[menunummer][buttonnummer].x1=10;
	S_Button[menunummer][buttonnummer].y1=170;
	S_Button[menunummer][buttonnummer].x2=310;
	S_Button[menunummer][buttonnummer].y2=220;
	S_Button[menunummer][buttonnummer].Menuverlinkung=1;
	S_Button[menunummer][buttonnummer].Text="Aktueller Zustand vom EKART";
	S_Button[menunummer][buttonnummer].Textlaenge=27;
	S_Button[menunummer][buttonnummer].bottonsanzahl=5;

	// menubene 3
	// Batteriestatus
	menunummer=3;

	buttonnummer=0;
	S_Button[menunummer][buttonnummer].x1=1;
	S_Button[menunummer][buttonnummer].y1=210;
	S_Button[menunummer][buttonnummer].x2=95;
	S_Button[menunummer][buttonnummer].y2=237;
	S_Button[menunummer][buttonnummer].Menuverlinkung=1;
	S_Button[menunummer][buttonnummer].bottonsanzahl=3;


	buttonnummer=1;
	S_Button[menunummer][buttonnummer].x1=100;
	S_Button[menunummer][buttonnummer].y1=210;
	S_Button[menunummer][buttonnummer].x2=220;
	S_Button[menunummer][buttonnummer].y2=237;
	S_Button[menunummer][buttonnummer].Menuverlinkung=10;
	S_Button[menunummer][buttonnummer].bottonsanzahl=3;


	buttonnummer=2;
	S_Button[menunummer][buttonnummer].x1=225;
	S_Button[menunummer][buttonnummer].y1=210;
	S_Button[menunummer][buttonnummer].x2=318;
	S_Button[menunummer][buttonnummer].y2=237;
	S_Button[menunummer][buttonnummer].Menuverlinkung=1;
	S_Button[menunummer][buttonnummer].bottonsanzahl=3;

	// menubene 4
	// "Fahrmodi - �bersicht"
	menunummer=4;

	buttonnummer=VorgabeDrehzahl;
	S_Button[menunummer][buttonnummer].x1=135;
	S_Button[menunummer][buttonnummer].y1=30;
	S_Button[menunummer][buttonnummer].x2=215;
	S_Button[menunummer][buttonnummer].y2=80;
	S_Button[menunummer][buttonnummer].Menuverlinkung=4;
	S_Button[menunummer][buttonnummer].Text="Drehzahl";
	S_Button[menunummer][buttonnummer].Textlaenge=8;
	S_Button[menunummer][buttonnummer].bottonsanzahl=7;


	buttonnummer=VorgabeMoment;
	S_Button[menunummer][buttonnummer].x1=230;
	S_Button[menunummer][buttonnummer].y1=30;
	S_Button[menunummer][buttonnummer].x2=310;
	S_Button[menunummer][buttonnummer].y2=80;
	S_Button[menunummer][buttonnummer].Menuverlinkung=4;
	S_Button[menunummer][buttonnummer].Text="Moment";
	S_Button[menunummer][buttonnummer].Textlaenge=6;
	S_Button[menunummer][buttonnummer].bottonsanzahl=7;


	buttonnummer=AnfaengerModeButton;
	S_Button[menunummer][buttonnummer].x1=90;
	S_Button[menunummer][buttonnummer].y1=105;
	S_Button[menunummer][buttonnummer].x2=190;
	S_Button[menunummer][buttonnummer].y2=155;
	S_Button[menunummer][buttonnummer].Menuverlinkung=5;
	S_Button[menunummer][buttonnummer].Text="Anfaenger";
	S_Button[menunummer][buttonnummer].Textlaenge=9;
	S_Button[menunummer][buttonnummer].bottonsanzahl=7;


	buttonnummer=StandardModeButton;
	S_Button[menunummer][buttonnummer].x1=210;
	S_Button[menunummer][buttonnummer].y1=105;
	S_Button[menunummer][buttonnummer].x2=310;
	S_Button[menunummer][buttonnummer].y2=155;
	S_Button[menunummer][buttonnummer].Menuverlinkung=6;
	S_Button[menunummer][buttonnummer].Text="Standard";
	S_Button[menunummer][buttonnummer].Textlaenge=8;
	S_Button[menunummer][buttonnummer].bottonsanzahl=7;

	buttonnummer=ProfiModeButton;
	S_Button[menunummer][buttonnummer].x1=90;
	S_Button[menunummer][buttonnummer].y1=175;
	S_Button[menunummer][buttonnummer].x2=190;								// max Wert: 320 (mit 10 mm von rechts zum Rand => 310)
	S_Button[menunummer][buttonnummer].y2=225;
	S_Button[menunummer][buttonnummer].Menuverlinkung=7;
	S_Button[menunummer][buttonnummer].Text="Profi";
	S_Button[menunummer][buttonnummer].Textlaenge=5;
	S_Button[menunummer][buttonnummer].bottonsanzahl=7;



	buttonnummer=TestModeButton;
	S_Button[menunummer][buttonnummer].x1=210;
	S_Button[menunummer][buttonnummer].y1=175;
	S_Button[menunummer][buttonnummer].x2=310;
	S_Button[menunummer][buttonnummer].y2=225;
	S_Button[menunummer][buttonnummer].Menuverlinkung=8;
	S_Button[menunummer][buttonnummer].Text="Test";
	S_Button[menunummer][buttonnummer].Textlaenge=4;
	S_Button[menunummer][buttonnummer].bottonsanzahl=7;

	buttonnummer=6;																						// Return zu Men� - Ebene
	S_Button[menunummer][buttonnummer].x1=10;
	S_Button[menunummer][buttonnummer].y1=105;
	S_Button[menunummer][buttonnummer].x2=70;
	S_Button[menunummer][buttonnummer].y2=225;
	S_Button[menunummer][buttonnummer].Menuverlinkung=2;
	S_Button[menunummer][buttonnummer].Text="Hauptmenu";
	S_Button[menunummer][buttonnummer].Textlaenge=5;
	S_Button[menunummer][buttonnummer].bottonsanzahl=7;

	// menubene 5
	// "FahrModi-Anfaenger"
	menunummer=5;


	//---------> die Buttons f�r "Menu_Fahrmodi()"
	buttonnummer=0;
	S_Button[menunummer][buttonnummer].x1=240;
	S_Button[menunummer][buttonnummer].y1=30;
	S_Button[menunummer][buttonnummer].x2=310;
	S_Button[menunummer][buttonnummer].y2=80;
	S_Button[menunummer][buttonnummer].Menuverlinkung=4;					// "Zur�ck" verlinkt zur "Fahrmodi"
	S_Button[menunummer][buttonnummer].Text="Zurueck";
	S_Button[menunummer][buttonnummer].Textlaenge=7;
	S_Button[menunummer][buttonnummer].bottonsanzahl=2;

	buttonnummer=1;                                  //Button f�r Best�tigen
	S_Button[menunummer][buttonnummer].x1=120;
	S_Button[menunummer][buttonnummer].y1=30;
	S_Button[menunummer][buttonnummer].x2=230;
	S_Button[menunummer][buttonnummer].y2=80;
	S_Button[menunummer][buttonnummer].Menuverlinkung=4;
	S_Button[menunummer][buttonnummer].bottonsanzahl=2;
	S_Button[menunummer][buttonnummer].Text="Flashen";
	S_Button[menunummer][buttonnummer].Textlaenge=7;



	slidernummer=0;
	Slider[menunummer][slidernummer].x=Flash_New_Parameters_List[10];
	Slider[menunummer][slidernummer].y=100;
	Slider[menunummer][slidernummer].Laenge=20;
	Slider[menunummer][slidernummer].Breite=10;
	Slider[menunummer][slidernummer].Bahn_x1=90;
	Slider[menunummer][slidernummer].Bahn_y1=105;
	Slider[menunummer][slidernummer].Bahn_x2=290;
	Slider[menunummer][slidernummer].Bahn_y2=115;
	Slider[menunummer][slidernummer].Touch_x1=Slider[menunummer][slidernummer].Bahn_x1-20; //70
	Slider[menunummer][slidernummer].Touch_y1=Slider[menunummer][slidernummer].Bahn_y1-5;  //100
	Slider[menunummer][slidernummer].Touch_x2=Slider[menunummer][slidernummer].Bahn_x2+20; //310
	Slider[menunummer][slidernummer].Touch_y2=Slider[menunummer][slidernummer].Bahn_y2+25; //140
	Slider[menunummer][slidernummer].max_wert=100;
	Slider[menunummer][slidernummer].min_wert=0;
	Slider[menunummer][slidernummer].SliderWert=Flash_New_Parameters_List[Anfaenger_GasProzent];
	Slider[menunummer][slidernummer].Text="GasProzent:";
	Slider[menunummer][slidernummer].Farbe=GREEN;



	slidernummer=1;
	Slider[menunummer][slidernummer].x=Flash_New_Parameters_List[11];
	Slider[menunummer][slidernummer].y=150;
	Slider[menunummer][slidernummer].Laenge=20;
	Slider[menunummer][slidernummer].Breite=10;
	Slider[menunummer][slidernummer].Bahn_x1=90;
	Slider[menunummer][slidernummer].Bahn_y1=155;
	Slider[menunummer][slidernummer].Bahn_x2=290;
	Slider[menunummer][slidernummer].Bahn_y2=165;
	Slider[menunummer][slidernummer].Touch_x1=Slider[menunummer][slidernummer].Bahn_x1-20; //70
	Slider[menunummer][slidernummer].Touch_y1=Slider[menunummer][slidernummer].Bahn_y1-5;  //150
	Slider[menunummer][slidernummer].Touch_x2=Slider[menunummer][slidernummer].Bahn_x2+20; //310
	Slider[menunummer][slidernummer].Touch_y2=Slider[menunummer][slidernummer].Bahn_y2+25; //190
	Slider[menunummer][slidernummer].max_wert=100;
	Slider[menunummer][slidernummer].min_wert=0;
	Slider[menunummer][slidernummer].SliderWert=Flash_New_Parameters_List[Anfaenger_BeschlProzent];
	Slider[menunummer][slidernummer].Text="Beschl.:";
	Slider[menunummer][slidernummer].Farbe=GREEN;



	slidernummer=2;
	Slider[menunummer][slidernummer].x=Flash_New_Parameters_List[26];
	Slider[menunummer][slidernummer].y=200;
	Slider[menunummer][slidernummer].Laenge=20;
	Slider[menunummer][slidernummer].Breite=10;
	Slider[menunummer][slidernummer].Bahn_x1=90;
	Slider[menunummer][slidernummer].Bahn_y1=205;
	Slider[menunummer][slidernummer].Bahn_x2=290;
	Slider[menunummer][slidernummer].Bahn_y2=215;
	Slider[menunummer][slidernummer].Touch_x1=Slider[menunummer][slidernummer].Bahn_x1-20; //70
	Slider[menunummer][slidernummer].Touch_y1=Slider[menunummer][slidernummer].Bahn_y1-5;  //200
	Slider[menunummer][slidernummer].Touch_x2=Slider[menunummer][slidernummer].Bahn_x2+20; //310
	Slider[menunummer][slidernummer].Touch_y2=Slider[menunummer][slidernummer].Bahn_y2+25; //240
	Slider[menunummer][slidernummer].max_wert=100;
	Slider[menunummer][slidernummer].min_wert=0;
	Slider[menunummer][slidernummer].SliderWert=Flash_New_Parameters_List[Anfaenger_Rueckwaert];
	Slider[menunummer][slidernummer].Text="Rueckwaert:";
	Slider[menunummer][slidernummer].Farbe=GREEN;

	// menubene 6
	// "FahrModi-Standart"
	menunummer=6;


	//---------> die Buttons f�r "Menu_Fahrmodi()"
	buttonnummer=0;
	S_Button[menunummer][buttonnummer].x1=240;
	S_Button[menunummer][buttonnummer].y1=30;
	S_Button[menunummer][buttonnummer].x2=310;
	S_Button[menunummer][buttonnummer].y2=80;
	S_Button[menunummer][buttonnummer].Menuverlinkung=4;					// "Zur�ck" verlinkt zur "Fahrmodi"
	S_Button[menunummer][buttonnummer].Text="Zurueck";
	S_Button[menunummer][buttonnummer].Textlaenge=7;
	S_Button[menunummer][buttonnummer].bottonsanzahl=2;

	buttonnummer=1;                                  //Button f�r Best�tigen
	S_Button[menunummer][buttonnummer].x1=120;
	S_Button[menunummer][buttonnummer].y1=30;
	S_Button[menunummer][buttonnummer].x2=230;
	S_Button[menunummer][buttonnummer].y2=80;
	S_Button[menunummer][buttonnummer].Menuverlinkung=4;
	S_Button[menunummer][buttonnummer].bottonsanzahl=2;
	S_Button[menunummer][buttonnummer].Text="Flashen";
	S_Button[menunummer][buttonnummer].Textlaenge=7;



	slidernummer=0;
	Slider[menunummer][slidernummer].x=Flash_New_Parameters_List[12];
	Slider[menunummer][slidernummer].y=100;
	Slider[menunummer][slidernummer].Laenge=20;
	Slider[menunummer][slidernummer].Breite=10;
	Slider[menunummer][slidernummer].Bahn_x1=90;
	Slider[menunummer][slidernummer].Bahn_y1=105;
	Slider[menunummer][slidernummer].Bahn_x2=290;
	Slider[menunummer][slidernummer].Bahn_y2=115;
	Slider[menunummer][slidernummer].Touch_x1=Slider[menunummer][slidernummer].Bahn_x1-20; //70
	Slider[menunummer][slidernummer].Touch_y1=Slider[menunummer][slidernummer].Bahn_y1-5;  //100
	Slider[menunummer][slidernummer].Touch_x2=Slider[menunummer][slidernummer].Bahn_x2+20; //310
	Slider[menunummer][slidernummer].Touch_y2=Slider[menunummer][slidernummer].Bahn_y2+25; //140
	Slider[menunummer][slidernummer].max_wert=100;
	Slider[menunummer][slidernummer].min_wert=0;
	Slider[menunummer][slidernummer].SliderWert=Flash_New_Parameters_List[Standard_GasProzent];
	Slider[menunummer][slidernummer].Text="GasProzent:";
	Slider[menunummer][slidernummer].Farbe=GREEN;



	slidernummer=1;
	Slider[menunummer][slidernummer].x=Flash_New_Parameters_List[13];
	Slider[menunummer][slidernummer].y=150;
	Slider[menunummer][slidernummer].Laenge=20;
	Slider[menunummer][slidernummer].Breite=10;
	Slider[menunummer][slidernummer].Bahn_x1=90;
	Slider[menunummer][slidernummer].Bahn_y1=155;
	Slider[menunummer][slidernummer].Bahn_x2=290;
	Slider[menunummer][slidernummer].Bahn_y2=165;
	Slider[menunummer][slidernummer].Touch_x1=Slider[menunummer][slidernummer].Bahn_x1-20; //70
	Slider[menunummer][slidernummer].Touch_y1=Slider[menunummer][slidernummer].Bahn_y1-5;  //150
	Slider[menunummer][slidernummer].Touch_x2=Slider[menunummer][slidernummer].Bahn_x2+20; //310
	Slider[menunummer][slidernummer].Touch_y2=Slider[menunummer][slidernummer].Bahn_y2+25; //190
	Slider[menunummer][slidernummer].max_wert=100;
	Slider[menunummer][slidernummer].min_wert=0;
	Slider[menunummer][slidernummer].SliderWert=Flash_New_Parameters_List[Standard_BeschlProzent];
	Slider[menunummer][slidernummer].Text="Beschl.:";
	Slider[menunummer][slidernummer].Farbe=GREEN;



	slidernummer=2;
	Slider[menunummer][slidernummer].x=Flash_New_Parameters_List[27];
	Slider[menunummer][slidernummer].y=200;
	Slider[menunummer][slidernummer].Laenge=20;
	Slider[menunummer][slidernummer].Breite=10;
	Slider[menunummer][slidernummer].Bahn_x1=90;
	Slider[menunummer][slidernummer].Bahn_y1=205;
	Slider[menunummer][slidernummer].Bahn_x2=290;
	Slider[menunummer][slidernummer].Bahn_y2=215;
	Slider[menunummer][slidernummer].Touch_x1=Slider[menunummer][slidernummer].Bahn_x1-20; //70
	Slider[menunummer][slidernummer].Touch_y1=Slider[menunummer][slidernummer].Bahn_y1-5;  //200
	Slider[menunummer][slidernummer].Touch_x2=Slider[menunummer][slidernummer].Bahn_x2+20; //310
	Slider[menunummer][slidernummer].Touch_y2=Slider[menunummer][slidernummer].Bahn_y2+25; //240
	Slider[menunummer][slidernummer].max_wert=100;
	Slider[menunummer][slidernummer].min_wert=0;
	Slider[menunummer][slidernummer].SliderWert=Flash_New_Parameters_List[Standard_Rueckwaert];
	Slider[menunummer][slidernummer].Text="Rueckwaert:";
	Slider[menunummer][slidernummer].Farbe=GREEN;

	// menubene 7
	// "FahrModi-Profi"
	menunummer=7;


	//---------> die Buttons f�r "Menu_Fahrmodi()"
	buttonnummer=0;
	S_Button[menunummer][buttonnummer].x1=240;
	S_Button[menunummer][buttonnummer].y1=30;
	S_Button[menunummer][buttonnummer].x2=310;
	S_Button[menunummer][buttonnummer].y2=80;
	S_Button[menunummer][buttonnummer].Menuverlinkung=4;					// "Zur�ck" verlinkt zur "Fahrmodi"
	S_Button[menunummer][buttonnummer].Text="Zurueck";
	S_Button[menunummer][buttonnummer].Textlaenge=7;
	S_Button[menunummer][buttonnummer].bottonsanzahl=2;

	buttonnummer=1;                                  //Button f�r Best�tigen
	S_Button[menunummer][buttonnummer].x1=120;
	S_Button[menunummer][buttonnummer].y1=30;
	S_Button[menunummer][buttonnummer].x2=230;
	S_Button[menunummer][buttonnummer].y2=80;
	S_Button[menunummer][buttonnummer].Menuverlinkung=4;
	S_Button[menunummer][buttonnummer].bottonsanzahl=2;
	S_Button[menunummer][buttonnummer].Text="Flashen";
	S_Button[menunummer][buttonnummer].Textlaenge=7;



	slidernummer=0;
	Slider[menunummer][slidernummer].x=Flash_New_Parameters_List[14];
	Slider[menunummer][slidernummer].y=100;
	Slider[menunummer][slidernummer].Laenge=20;
	Slider[menunummer][slidernummer].Breite=10;
	Slider[menunummer][slidernummer].Bahn_x1=90;
	Slider[menunummer][slidernummer].Bahn_y1=105;
	Slider[menunummer][slidernummer].Bahn_x2=290;
	Slider[menunummer][slidernummer].Bahn_y2=115;
	Slider[menunummer][slidernummer].Touch_x1=Slider[menunummer][slidernummer].Bahn_x1-20; //70
	Slider[menunummer][slidernummer].Touch_y1=Slider[menunummer][slidernummer].Bahn_y1-5;  //100
	Slider[menunummer][slidernummer].Touch_x2=Slider[menunummer][slidernummer].Bahn_x2+20; //310
	Slider[menunummer][slidernummer].Touch_y2=Slider[menunummer][slidernummer].Bahn_y2+25; //140
	Slider[menunummer][slidernummer].max_wert=100;
	Slider[menunummer][slidernummer].min_wert=0;
	Slider[menunummer][slidernummer].SliderWert=Flash_New_Parameters_List[Profi_GasProzent];
	Slider[menunummer][slidernummer].Text="GasProzent:";
	Slider[menunummer][slidernummer].Farbe=GREEN;



	slidernummer=1;
	Slider[menunummer][slidernummer].x=Flash_New_Parameters_List[15];
	Slider[menunummer][slidernummer].y=150;
	Slider[menunummer][slidernummer].Laenge=20;
	Slider[menunummer][slidernummer].Breite=10;
	Slider[menunummer][slidernummer].Bahn_x1=90;
	Slider[menunummer][slidernummer].Bahn_y1=155;
	Slider[menunummer][slidernummer].Bahn_x2=290;
	Slider[menunummer][slidernummer].Bahn_y2=165;
	Slider[menunummer][slidernummer].Touch_x1=Slider[menunummer][slidernummer].Bahn_x1-20; //70
	Slider[menunummer][slidernummer].Touch_y1=Slider[menunummer][slidernummer].Bahn_y1-5;  //150
	Slider[menunummer][slidernummer].Touch_x2=Slider[menunummer][slidernummer].Bahn_x2+20; //310
	Slider[menunummer][slidernummer].Touch_y2=Slider[menunummer][slidernummer].Bahn_y2+25; //190
	Slider[menunummer][slidernummer].max_wert=100;
	Slider[menunummer][slidernummer].min_wert=0;
	Slider[menunummer][slidernummer].SliderWert=Flash_New_Parameters_List[Profi_BeschlProzent];
	Slider[menunummer][slidernummer].Text="Beschl.:";
	Slider[menunummer][slidernummer].Farbe=GREEN;



	slidernummer=2;
	Slider[menunummer][slidernummer].x=Flash_New_Parameters_List[28];
	Slider[menunummer][slidernummer].y=200;
	Slider[menunummer][slidernummer].Laenge=20;
	Slider[menunummer][slidernummer].Breite=10;
	Slider[menunummer][slidernummer].Bahn_x1=90;
	Slider[menunummer][slidernummer].Bahn_y1=205;
	Slider[menunummer][slidernummer].Bahn_x2=290;
	Slider[menunummer][slidernummer].Bahn_y2=215;
	Slider[menunummer][slidernummer].Touch_x1=Slider[menunummer][slidernummer].Bahn_x1-20; //70
	Slider[menunummer][slidernummer].Touch_y1=Slider[menunummer][slidernummer].Bahn_y1-5;  //200
	Slider[menunummer][slidernummer].Touch_x2=Slider[menunummer][slidernummer].Bahn_x2+20; //310
	Slider[menunummer][slidernummer].Touch_y2=Slider[menunummer][slidernummer].Bahn_y2+25; //240
	Slider[menunummer][slidernummer].max_wert=100;
	Slider[menunummer][slidernummer].min_wert=0;
	Slider[menunummer][slidernummer].SliderWert=Flash_New_Parameters_List[Profi_Rueckwaert];
	Slider[menunummer][slidernummer].Text="Rueckwaert:";
	Slider[menunummer][slidernummer].Farbe=GREEN;

	// menubene 8
	// "FahrModi-Test"
	menunummer=8;


	//---------> die Buttons f�r "Menu_Fahrmodi()"
	buttonnummer=0;
	S_Button[menunummer][buttonnummer].x1=240;
	S_Button[menunummer][buttonnummer].y1=30;
	S_Button[menunummer][buttonnummer].x2=310;
	S_Button[menunummer][buttonnummer].y2=80;
	S_Button[menunummer][buttonnummer].Menuverlinkung=4;					// "Zur�ck" verlinkt zur "Fahrmodi"
	S_Button[menunummer][buttonnummer].Text="Zurueck";
	S_Button[menunummer][buttonnummer].Textlaenge=7;
	S_Button[menunummer][buttonnummer].bottonsanzahl=2;

	buttonnummer=1;                                  //Button f�r Best�tigen
	S_Button[menunummer][buttonnummer].x1=120;
	S_Button[menunummer][buttonnummer].y1=30;
	S_Button[menunummer][buttonnummer].x2=230;
	S_Button[menunummer][buttonnummer].y2=80;
	S_Button[menunummer][buttonnummer].Menuverlinkung=4;
	S_Button[menunummer][buttonnummer].bottonsanzahl=2;
	S_Button[menunummer][buttonnummer].Text="Flashen";
	S_Button[menunummer][buttonnummer].Textlaenge=7;



	slidernummer=0;
	Slider[menunummer][slidernummer].x=Flash_New_Parameters_List[16];
	Slider[menunummer][slidernummer].y=100;
	Slider[menunummer][slidernummer].Laenge=20;
	Slider[menunummer][slidernummer].Breite=10;
	Slider[menunummer][slidernummer].Bahn_x1=90;
	Slider[menunummer][slidernummer].Bahn_y1=105;
	Slider[menunummer][slidernummer].Bahn_x2=290;
	Slider[menunummer][slidernummer].Bahn_y2=115;
	Slider[menunummer][slidernummer].Touch_x1=Slider[menunummer][slidernummer].Bahn_x1-20; //70
	Slider[menunummer][slidernummer].Touch_y1=Slider[menunummer][slidernummer].Bahn_y1-5;  //100
	Slider[menunummer][slidernummer].Touch_x2=Slider[menunummer][slidernummer].Bahn_x2+20; //310
	Slider[menunummer][slidernummer].Touch_y2=Slider[menunummer][slidernummer].Bahn_y2+25; //140
	Slider[menunummer][slidernummer].max_wert=100;
	Slider[menunummer][slidernummer].min_wert=0;
	Slider[menunummer][slidernummer].SliderWert=Flash_New_Parameters_List[Test_GasProzent];
	Slider[menunummer][slidernummer].Text="GasProzent:";
	Slider[menunummer][slidernummer].Farbe=GREEN;



	slidernummer=1;
	Slider[menunummer][slidernummer].x=Flash_New_Parameters_List[17];
	Slider[menunummer][slidernummer].y=150;
	Slider[menunummer][slidernummer].Laenge=20;
	Slider[menunummer][slidernummer].Breite=10;
	Slider[menunummer][slidernummer].Bahn_x1=90;
	Slider[menunummer][slidernummer].Bahn_y1=155;
	Slider[menunummer][slidernummer].Bahn_x2=290;
	Slider[menunummer][slidernummer].Bahn_y2=165;
	Slider[menunummer][slidernummer].Touch_x1=Slider[menunummer][slidernummer].Bahn_x1-20; //70
	Slider[menunummer][slidernummer].Touch_y1=Slider[menunummer][slidernummer].Bahn_y1-5;  //150
	Slider[menunummer][slidernummer].Touch_x2=Slider[menunummer][slidernummer].Bahn_x2+20; //310
	Slider[menunummer][slidernummer].Touch_y2=Slider[menunummer][slidernummer].Bahn_y2+25; //190
	Slider[menunummer][slidernummer].max_wert=100;
	Slider[menunummer][slidernummer].min_wert=0;
	Slider[menunummer][slidernummer].SliderWert=Flash_New_Parameters_List[Test_BeschlProzent];
	Slider[menunummer][slidernummer].Text="Beschl.:";
	Slider[menunummer][slidernummer].Farbe=GREEN;



	slidernummer=2;
	Slider[menunummer][slidernummer].x=Flash_New_Parameters_List[29];
	Slider[menunummer][slidernummer].y=200;
	Slider[menunummer][slidernummer].Laenge=20;
	Slider[menunummer][slidernummer].Breite=10;
	Slider[menunummer][slidernummer].Bahn_x1=90;
	Slider[menunummer][slidernummer].Bahn_y1=205;
	Slider[menunummer][slidernummer].Bahn_x2=290;
	Slider[menunummer][slidernummer].Bahn_y2=215;
	Slider[menunummer][slidernummer].Touch_x1=Slider[menunummer][slidernummer].Bahn_x1-20; //70
	Slider[menunummer][slidernummer].Touch_y1=Slider[menunummer][slidernummer].Bahn_y1-5;  //200
	Slider[menunummer][slidernummer].Touch_x2=Slider[menunummer][slidernummer].Bahn_x2+20; //310
	Slider[menunummer][slidernummer].Touch_y2=Slider[menunummer][slidernummer].Bahn_y2+25; //240
	Slider[menunummer][slidernummer].max_wert=100;
	Slider[menunummer][slidernummer].min_wert=0;
	Slider[menunummer][slidernummer].SliderWert=Flash_New_Parameters_List[Test_Rueckwaert];
	Slider[menunummer][slidernummer].Text="Rueckwaert:";
	Slider[menunummer][slidernummer].Farbe=GREEN;

	// menubene 9
	// Strom
	menunummer=9;


	buttonnummer=0;
	S_Button[menunummer][buttonnummer].x1=240;
	S_Button[menunummer][buttonnummer].y1=50;
	S_Button[menunummer][buttonnummer].x2=310;
	S_Button[menunummer][buttonnummer].y2=100;
	S_Button[menunummer][buttonnummer].Menuverlinkung=2;					// "Zur�ck" verlinkt zur "Fahrmodi"
	S_Button[menunummer][buttonnummer].Text="Zurueck";
	S_Button[menunummer][buttonnummer].Textlaenge=7;
	S_Button[menunummer][buttonnummer].bottonsanzahl=2;

	buttonnummer=1;                                  //Button f�r Best�tigen
	S_Button[menunummer][buttonnummer].x1=120;
	S_Button[menunummer][buttonnummer].y1=50;
	S_Button[menunummer][buttonnummer].x2=230;
	S_Button[menunummer][buttonnummer].y2=100;
	S_Button[menunummer][buttonnummer].Menuverlinkung=2;
	S_Button[menunummer][buttonnummer].bottonsanzahl=2;
	S_Button[menunummer][buttonnummer].Text="Flashen";
	S_Button[menunummer][buttonnummer].Textlaenge=7;


	slidernummer=0;
	Slider[menunummer][slidernummer].x=Flash_New_Parameters_List[Strom_Cursorpositionen];
	Slider[menunummer][slidernummer].y=120;
	Slider[menunummer][slidernummer].Laenge=20;
	Slider[menunummer][slidernummer].Breite=10;
	Slider[menunummer][slidernummer].Bahn_x1=90;
	Slider[menunummer][slidernummer].Bahn_y1=125;
	Slider[menunummer][slidernummer].Bahn_x2=290;
	Slider[menunummer][slidernummer].Bahn_y2=135;
	Slider[menunummer][slidernummer].Touch_x1=Slider[menunummer][slidernummer].Bahn_x1-20; //70
	Slider[menunummer][slidernummer].Touch_y1=Slider[menunummer][slidernummer].Bahn_y1-5;  //100
	Slider[menunummer][slidernummer].Touch_x2=Slider[menunummer][slidernummer].Bahn_x2+20; //310
	Slider[menunummer][slidernummer].Touch_y2=Slider[menunummer][slidernummer].Bahn_y2+25; //140
	Slider[menunummer][slidernummer].max_wert=400;
	Slider[menunummer][slidernummer].min_wert=200;
	Slider[menunummer][slidernummer].SliderWert=Flash_New_Parameters_List[Strom];
	Slider[menunummer][slidernummer].Text="   Strom:";
	Slider[menunummer][slidernummer].Farbe=GREEN;

	// menubene 10
	// Password
	menunummer=10;

	buttonnummer=0;
	S_Button[menunummer][buttonnummer].x1=220;
	S_Button[menunummer][buttonnummer].y1=123;
	S_Button[menunummer][buttonnummer].x2=315;
	S_Button[menunummer][buttonnummer].y2=163;
	S_Button[menunummer][buttonnummer].Wert=0;
	S_Button[menunummer][buttonnummer].Menuverlinkung=10;
	S_Button[menunummer][buttonnummer].bottonsanzahl=13;
	S_Button[menunummer][buttonnummer].Text="0";
	S_Button[menunummer][buttonnummer].Textlaenge=1;

	buttonnummer=1;
	S_Button[menunummer][buttonnummer].x1=10;
	S_Button[menunummer][buttonnummer].y1=60;
	S_Button[menunummer][buttonnummer].x2=70;
	S_Button[menunummer][buttonnummer].y2=100;
	S_Button[menunummer][buttonnummer].Wert=1;
	S_Button[menunummer][buttonnummer].Menuverlinkung=10;
	S_Button[menunummer][buttonnummer].bottonsanzahl=13;
	S_Button[menunummer][buttonnummer].Text="1";
	S_Button[menunummer][buttonnummer].Textlaenge=1;

	buttonnummer=2;
	S_Button[menunummer][buttonnummer].x1=80;
	S_Button[menunummer][buttonnummer].y1=60;
	S_Button[menunummer][buttonnummer].x2=140;
	S_Button[menunummer][buttonnummer].y2=100;
	S_Button[menunummer][buttonnummer].Wert=2;
	S_Button[menunummer][buttonnummer].Menuverlinkung=10;
	S_Button[menunummer][buttonnummer].bottonsanzahl=13;
	S_Button[menunummer][buttonnummer].Text="2";
	S_Button[menunummer][buttonnummer].Textlaenge=1;

	buttonnummer=3;
	S_Button[menunummer][buttonnummer].x1=150;
	S_Button[menunummer][buttonnummer].y1=60;
	S_Button[menunummer][buttonnummer].x2=210;
	S_Button[menunummer][buttonnummer].y2=100;
	S_Button[menunummer][buttonnummer].Wert=3;
	S_Button[menunummer][buttonnummer].Menuverlinkung=10;
	S_Button[menunummer][buttonnummer].bottonsanzahl=13;
	S_Button[menunummer][buttonnummer].Text="3";
	S_Button[menunummer][buttonnummer].Textlaenge=1;

	buttonnummer=4;
	S_Button[menunummer][buttonnummer].x1=10;
	S_Button[menunummer][buttonnummer].y1=123;
	S_Button[menunummer][buttonnummer].x2=70;
	S_Button[menunummer][buttonnummer].y2=163;
	S_Button[menunummer][buttonnummer].Wert=4;
	S_Button[menunummer][buttonnummer].Menuverlinkung=10;
	S_Button[menunummer][buttonnummer].bottonsanzahl=13;
	S_Button[menunummer][buttonnummer].Text="4";
	S_Button[menunummer][buttonnummer].Textlaenge=1;

	buttonnummer=5;
	S_Button[menunummer][buttonnummer].x1=80;
	S_Button[menunummer][buttonnummer].y1=123;
	S_Button[menunummer][buttonnummer].x2=140;
	S_Button[menunummer][buttonnummer].y2=163;
	S_Button[menunummer][buttonnummer].Wert=5;
	S_Button[menunummer][buttonnummer].Menuverlinkung=10;
	S_Button[menunummer][buttonnummer].bottonsanzahl=13;
	S_Button[menunummer][buttonnummer].Text="5";
	S_Button[menunummer][buttonnummer].Textlaenge=1;

	buttonnummer=6;
	S_Button[menunummer][buttonnummer].x1=150;
	S_Button[menunummer][buttonnummer].y1=123;
	S_Button[menunummer][buttonnummer].x2=210;
	S_Button[menunummer][buttonnummer].y2=163;
	S_Button[menunummer][buttonnummer].Wert=6;
	S_Button[menunummer][buttonnummer].Menuverlinkung=10;
	S_Button[menunummer][buttonnummer].bottonsanzahl=13;
	S_Button[menunummer][buttonnummer].Text="6";
	S_Button[menunummer][buttonnummer].Textlaenge=1;

	buttonnummer=7;
	S_Button[menunummer][buttonnummer].x1=10;
	S_Button[menunummer][buttonnummer].y1=186;
	S_Button[menunummer][buttonnummer].x2=70;
	S_Button[menunummer][buttonnummer].y2=226;
	S_Button[menunummer][buttonnummer].Wert=7;
	S_Button[menunummer][buttonnummer].Menuverlinkung=10;
	S_Button[menunummer][buttonnummer].bottonsanzahl=13;
	S_Button[menunummer][buttonnummer].Text="7";
	S_Button[menunummer][buttonnummer].Textlaenge=1;

	buttonnummer=8;
	S_Button[menunummer][buttonnummer].x1=80;
	S_Button[menunummer][buttonnummer].y1=186;
	S_Button[menunummer][buttonnummer].x2=140;
	S_Button[menunummer][buttonnummer].y2=226;
	S_Button[menunummer][buttonnummer].Wert=8;
	S_Button[menunummer][buttonnummer].Menuverlinkung=10;
	S_Button[menunummer][buttonnummer].bottonsanzahl=13;
	S_Button[menunummer][buttonnummer].Text="8";
	S_Button[menunummer][buttonnummer].Textlaenge=1;

	buttonnummer=9;
	S_Button[menunummer][buttonnummer].x1=150;
	S_Button[menunummer][buttonnummer].y1=186;
	S_Button[menunummer][buttonnummer].x2=210;
	S_Button[menunummer][buttonnummer].y2=226;
	S_Button[menunummer][buttonnummer].Wert=9;
	S_Button[menunummer][buttonnummer].Menuverlinkung=10;
	S_Button[menunummer][buttonnummer].bottonsanzahl=13;
	S_Button[menunummer][buttonnummer].Text="9";
	S_Button[menunummer][buttonnummer].Textlaenge=1;

	buttonnummer=10;                                  //Button f�r Abbruch
	S_Button[menunummer][buttonnummer].x1=200;
	S_Button[menunummer][buttonnummer].y1=23;
	S_Button[menunummer][buttonnummer].x2=280;
	S_Button[menunummer][buttonnummer].y2=45;
	S_Button[menunummer][buttonnummer].Wert=10;
	S_Button[menunummer][buttonnummer].Menuverlinkung=1;
	S_Button[menunummer][buttonnummer].bottonsanzahl=13;
	S_Button[menunummer][buttonnummer].Text="Abbruch";
	S_Button[menunummer][buttonnummer].Textlaenge=7;

	buttonnummer=11;                                  //Button f�r Best�tigen
	S_Button[menunummer][buttonnummer].x1=220;
	S_Button[menunummer][buttonnummer].y1=60;
	S_Button[menunummer][buttonnummer].x2=315;
	S_Button[menunummer][buttonnummer].y2=100;
	S_Button[menunummer][buttonnummer].Wert=11;
	S_Button[menunummer][buttonnummer].Menuverlinkung=2;
	S_Button[menunummer][buttonnummer].bottonsanzahl=13;
	S_Button[menunummer][buttonnummer].Text="Bestaetigen";
	S_Button[menunummer][buttonnummer].Textlaenge=11;

	buttonnummer=12;                                  //Button f�r L�schen
	S_Button[menunummer][buttonnummer].x1=220;
	S_Button[menunummer][buttonnummer].y1=186;
	S_Button[menunummer][buttonnummer].x2=315;
	S_Button[menunummer][buttonnummer].y2=226;
	S_Button[menunummer][buttonnummer].Wert=12;
	S_Button[menunummer][buttonnummer].Menuverlinkung=10;
	S_Button[menunummer][buttonnummer].bottonsanzahl=13;
	S_Button[menunummer][buttonnummer].Text="Loeschen";
	S_Button[menunummer][buttonnummer].Textlaenge=8;

	// menubene 11
	// Menu "RCP-Mode"
	menunummer=11;

	buttonnummer=0;
	S_Button[menunummer][buttonnummer].x1=220;
	S_Button[menunummer][buttonnummer].y1=190;
	S_Button[menunummer][buttonnummer].x2=320;
	S_Button[menunummer][buttonnummer].y2=240;
	S_Button[menunummer][buttonnummer].Menuverlinkung=2; // "Zur�ck" verlinkt zur "Hauptmenu"
	S_Button[menunummer][buttonnummer].Text="Zurueck";
	S_Button[menunummer][buttonnummer].Textlaenge=7;
	S_Button[menunummer][buttonnummer].bottonsanzahl=2;

	buttonnummer=1;                                  //Button sent RCP connection request
	S_Button[menunummer][buttonnummer].x1=110;
	S_Button[menunummer][buttonnummer].y1=190;
	S_Button[menunummer][buttonnummer].x2=215;
	S_Button[menunummer][buttonnummer].y2=240;
	S_Button[menunummer][buttonnummer].Menuverlinkung=11;
	S_Button[menunummer][buttonnummer].bottonsanzahl=2;
	S_Button[menunummer][buttonnummer].Text="RCP-Connect";
	S_Button[menunummer][buttonnummer].Textlaenge=11;

}

// Diese Funktion sorgt für ein Update der tatsaechlich angezeigten GUI-Struktur
void Anzeige(uint_fast8_t menubene)
{
	switch(menubene)
	{
		case 1:
		{
			EKartZustand();
		}
		break;

		case 2:
		{
			ZeitAnzeige();
		}
		break;

		case 3:
		{
			ZeitAnzeige();
		}
		break;

		case 4:
		{
			ZeitAnzeige();
		}
		break;

		case 5:
		{
			ZeitAnzeige();
		}
		break;

		case 6:
		{
			ZeitAnzeige();
		}
		break;

		case 7:
		{
			ZeitAnzeige();
		}
		break;

		case 8:
		{
			ZeitAnzeige();
		}
		break;

		case 9:
		{
			ZeitAnzeige();
		}
		break;

		case 10:
		{
			ZeitAnzeige();
			PasswordAnzeige();
		}
		break;
		case 11:
		{
			ZeitAnzeige();
			RCP_show_connect(RCP_Mode_pending);
			RCP_show_status(RCP_Mode_status, RCP_Mode_errorcode);
		}
		break;
		default: break;
	}
}









