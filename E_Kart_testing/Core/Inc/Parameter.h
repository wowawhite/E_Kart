/*******************************************************************************
* File Name          : Parameter.h
* Author             : Ostfalia Hochschule f�r angewandte Wissenschaften
                       Michael Scheele
* Modification		 : Tina K�hling
* Date First Issued  : 2012-05-15
* Updated 	         : 2021-07  - added RCP mode
* Description        : Parameter des E-Karts
*******************************************************************************/

#ifndef __PARAMETER_H
#define __PARAMETER_H
/* Includes ----------------------------------------------------------*/
/* Exported types ----------------------------------------------------*/
/* Exported constants ------------------------------------------------*/                             
/* Macros ------------------------------------------------------------*/

#define SOFTWAREVERSION "V2021.08" // dont forget to increment if changed

#define TRUE 1
#define FALSE 0
#define ON 1
#define OFF 0

// Errorcodes RCP-Mode
#define NO_ERROR 0
#define RCP_TIMEOUT 1
#define NO_RCP_HEARTBEAT 2
#define NO_MOTOR_HEARTBEAT 3
#define RCP_MOVING_ERROR 4


#define max_menuebenen 19       // insgesamt 19 Menuebenen, 0.Menuebene wird nicht genutzt
#define max_buttons 13           // in jeder Menuebene kann man maximal 14(13) Switch-Bottons(0.Buttons z�hlt dazu) auslegen.
// SwitchButton ist ein Button auf Display angezeigt und der verlinkt zu anderen Menuebenen
// In "struct" [TouchFunktion.c] sind die Eigenschaft von SwitchButton definiert

#define max_sliders 3           // in jeder Menuebene kann man maximal (2+1) Gleitblock auslegen [0. bis 2. Slider]
// Gleitblock ist fuer Eingabe von Parameter ausgelegt, der Wert von Gleitblock kann man durch Touch-Funktion ver�ndern. 
// In "struct" [TouchFunktion.c] sind die Eigenschaft von Slidern definiert

#define TOUCH_AKTIV					  	// wenn definiert, Touch-Aktiv, durch auskommentieren, Touch-Inaktiv
//#define TEMPORARY_INVERTTOUCH	// TODO: define this to invert touch on the testing Board LCD because the touch coordinates there is flipped
/* -------------------------------- FFR ----------------------------------------------  Motor-Controller-Daten --------------------------------*/
#define VorgabeDrehzahl				0 // Speed mode is not implemented. Torque control mode is default
#define VorgabeMoment				1

#define	AnfaengerModeButton			2
#define	StandardModeButton			3
#define	ProfiModeButton				4
#define	TestModeButton				5

/* -------------------------------- FLASH Definitionen -----------------------------------------------------------*/
#define NumberOfSavedParameter 	((uint32_t)0x0000001E)//30 Elemente sind in den Listen f�r die Flash-Speicherung

#define ChosenDrivingMode			0
#define VorgabeMomentDrehzahl		1

#define Anfaenger_GasProzent		2
#define Standard_GasProzent			3
#define Profi_GasProzent			4
#define Test_GasProzent				5

#define Anfaenger_BeschlProzent		6
#define Standard_BeschlProzent		7
#define Profi_BeschlProzent			8
#define Test_BeschlProzent			9
#define INIT						18
#define Motor_SDO					19
#define Strom						20
#define Strom_Cursorpositionen 		21

#define Anfaenger_Rueckwaert		22
#define Standard_Rueckwaert			23
#define Profi_Rueckwaert			24
#define Test_Rueckwaert				25
// die restlichen Elemente 10-17 sind die letzten Cursor-Positionen und werden in Menuestruktur() [TouchFunktion.c] verwendet f�r die Initial.Position
// Element 18 ist Initialisierung
// Element 19 ist die Markierung f�r SDO-Parameters, 0 - SDO nicht senden; 1 - SDO werde dann gesendet; 2 - SDO von Rechtsmotor schon gesendet, aber Linksmotor noch nicht;
// Manual Reference, Seite 50, Tabelle 4, "High density": Auf Sektor 4 ist 64 Kbyte mit 0x0801 0000 - 0x0801 FFFF
#define FlashMemoryAddressSTART		((uint32_t)0x08010000)
#define FlashPageNextPosition		((uint32_t)0x00000010)		// auf einem Sektor stehen 16*512 Bits = 2x512 Bytes = 16'383 Bits zur Verf�gung
#define FlashMemoryAddressEND		((uint32_t)0x0801FFFF)  	// Darf nicht �berschirtten werden!

/* -------------------------------- Fahrzeugdaten --------------------------------*/
#define DREHZAHL_MAX 				5000 		// Maximale Motordrehzahl in rpm, Motor PERM PMS120, zur Sicherheit weniger angenommen, Hersteller: n_max = 6000 rpm
#define DREHZAHL_GRENZ				500
#define R_BEGRENZUNG				4 		// Begrenzungsdivisor der Motordrehzahl/-moment f�r R�ckw�rtsgang, n_R�ckw�rts_max = Drehzahl_Max/R_Begrenzung, 3000rpm/4 = 750 rpm
#define REIFENRADIUS 			 	13 		// Reifenradius in cm, f�r Geschwindigkeitsberechnung
#define MOTOR_RAD_RATIO	    	  	25 		// 25 = 2.5, Motor zu Rad �bersetzung = Motor_Drehzahl / Rad_Drehzahl

/* -------------------------------- Differenzial ---------------------------*/
//#define Lenkuebersetzung			15 		// Lenkradwinkel/Lenkwinkel = 90�/6� = 15

/* -------------------------------- Batteriemanagement ---------------------------*/
//#define BM	 					    0		// Batteriemanagement, 0 - nicht vorhanden, 1 - vorhanden

/* -------------------------------- Gas und Bremspedal-Daten ---------------------*/
#define DUTY_CYCLE_GAS_MIN			84 		// Tastverh�ltnis in Grundstellung in 10*%, 0 .. 100,0 % = 0 .. 1000
#define DUTY_CYCLE_GAS_MAX 			205 	// Tastverh�ltnis bei voller Bet�tigung in 10*%, 0 .. 100,0 % = 0 .. 1000

#define DUTY_CYCLE_BREMSE_MIN  		114 	// Tastverh�ltnis in Grundstellung in 10*%, 0 .. 100,0 % = 0 .. 1000
#define DUTY_CYCLE_BREMSE_MAX   	236 	// Tastverh�ltnis bei voller Bet�tigung in 10*%, 0 .. 100,0 % = 0 .. 1000

#define Norm_100                	100 	// Normwert f�r 100 %
#define Norm_0                    	0 	// Normwert f�r 0 %

#define Bremse_SchwelleVerGas	   	30 	// Schwelle in Prozent das Bremspedals, ab der Gasbet�tigung verriegelt wird

/* -------------------------------- Lenkwinkel-Daten ----------------------------*/
#define LW_R					 	-920 // -900= -90,0� - Lenkradwinkel bei Vollausschlag nach rechts in �/10
#define LW_L 					 	+920 // +900= +90,0� - Lenkradwinkel bei Vollausschlag nach links in �/10

/* -------------------------------- CAN-Daten -----------------------------------*/
#define CANx                     	CAN1 	// CAN1 oder CAN2, bei Verwendung von CAN2 Konfiguration entsprechend �perpr�fen

// Senden
#define ID_MC_rechts 				0x205 	// Nachrichten-ID f�r Motorcontroller rechts
#define ID_MC_links 				0x206 	// Nachrichten-ID f�r Motorcontroller links
#define Motor_EIN_rechts          	1 		// 2. Datenbyte der Nachricht auf ID_MC_rechts
#define Motor_EIN_links           	1 		// 2. Datenbyte der Nachricht auf ID_MC_links
#define Motor_Moment_REV     		5 		// 5-dez = 101 - negatives Moment vorgeben, R�ckw�rts
#define Motor_Moment_FWD      		6 		// 6-dez = 110 - positives Moment vorgeben, Vorw�rts

#define ID_Heartbeat_FFR			0x701 	// Nachrichten-ID f�r Heartbeat des FFRs, FFR ist bereit
#define Msg_Heartbeat_FFR		    5 		// Message-Inhalt des Heartbeats vom FFR
#define ID_Synchronisation     		0x80 	// Nachrichten-ID der Synchronisation

#define ID_TxPDO1 					0x181	// Nachrichten-ID TxPDO1

#define ID_NMT     					0x00 	// Nachrichten-ID der NMT


// Empfangen
#define ID_Lenkradwinkelsensor		0x184 // 0x600 Nachrichten-ID vom Lenkradwinkelsensor im E-Kart, 0xC0 vom Testsensor
#define DL_Lenkradwinkelsensor		5 	// Datenl�nge in Bytes der Lenkradwinkelsensor-Nachricht, 0-8 Bytes

#define ID_Heartbeat_MC_rechts  	0x705 // Nachrichten-ID des Heartbeats des Motorcontrollers rechts
#define Msg_Heartbeat_MC_rechts	    5 // Message-Inhalt des Heartbeats, nicht Null
#define DL_Heartbeat_MC_rechts	    1 // Datenl�nge in Bytes der Nachricht, 0-8 Bytes

#define ID_Heartbeat_MC_links   	0x706 // Nachrichten-ID des Heartbeats des Motorcontrollers links
#define Msg_Heartbeat_MC_links	    5 // Message-Inhalt des Heartbeats
#define DL_Heartbeat_MC_links	    1 // Datenl�nge in Bytes der Nachricht, 0-8 Bytes

#define ID_Motor_Drehzahl_r	    	0x185 // Nachrichten-ID vom Speed-Sensor rechts
#define DL_Motor_Drehzahl_r		   	4 // Datenl�nge in Bytes der Nachricht, 0-8 Bytes

#define ID_Motor_Drehzahl_l	    	0x186 // Nachrichten-ID vom Speed-Sensor links
#define DL_Motor_Drehzahl_l		    4 // Datenl�nge in Bytes der Nachricht, 0-8 Bytes

#define ID_Status_Batteriesteuerung	0x1BA // Nachrichten-ID vom Speed-Sensor rechts
#define DL_Status_Batteriesteuerung	8 // Datenl�nge in Bytes der Nachricht, 0-8 Bytes

#define ID_Temperatur_Zellen_r		0x1F5 // Nachrichten-ID vom BMS f�r die Zellen rechts
#define DL_Temperatur_Zellen_r		8 // Datenl�nge in Bytes der Nachricht, 0-8 Bytes

#define ID_Temperatur_Zellen_l		0x1F6 // Nachrichten-ID vom BMS f�r die Zellen links
#define DL_Temperatur_Zellen_l		8 // Datenl�nge in Bytes der Nachricht, 0-8 Bytes

#define ID_Spannung_Zellen_r		0x1D5 // Nachrichten-ID vom BMS f�r die Zellen rechts
#define DL_Spannung_Zellen_r		8 // Datenl�nge in Bytes der Nachricht, 0-8 Bytes

#define ID_Spannung_Zellen_l		0x1D6 // Nachrichten-ID vom BMS f�r die Zellen links
#define DL_Spannung_Zellen_l		8 // Datenl�nge in Bytes der Nachricht, 0-8 Bytes

#define ID_Heartbeat_RCP			0x70A
#define Msg_Heartbeat_RCP	    	5 // Message-Inhalt des Heartbeats

// SDO-Parameters senden & Empfangen
#define ID_SDO_MC_rechts			0x606 // Nachrichten-ID f�r SDO-Parameters von Motorcontroller rechts, Senden
#define ID_SDO_MC_links 			0x605 // Nachrichten-ID f�r SDO-Parameters von Motorcontroller links, Senden
#define ID_SDO_MC_rechts_E			0x586 // Nachrichten-ID f�r SDO-Parameters von Motorcontroller rechts, Empfangen
#define ID_SDO_MC_links_E			0x585 // Nachrichten-ID f�r SDO-Parameters von Motorcontroller links, Empfangen

#define ID_SDO_RCP_Rx				0x60A
#define ID_SDO_RCP_Tx				0x58A
#define Msg_SDO_RCP_ack_OK			0x60

#define Motor_links					5
#define Motor_rechts				6

// Definition der SDO-Parameters f�r SDO_Parameter_transmit(uint16_t SDO_Merker)
#define User_ID						0			// 0,1,2 f�r Pre-Operational; 0,1,3 f�r Operational
#define Password					1
#define Pre_Operational				2
#define Operational					3
#define setStrom					4
#define Drehzahl					5
#define Beschleunigung				6
// Speed mode is not implemented. Torque control mode is default
//#define Speed_mode					7			// 7-8 f�r Speedmode, Vorgabedrehzahl
//#define Traction_control_mode1		8
//#define Torque_mode					9			// 9-11 f�r Torquemode, Vorgabemoment
//#define Traction_control_mode0  	10
//#define Maximum_torque				11

#define Throttle_Flags0				12		// 12,14,16,18 f�r Torque-mode; 13,15,17,19 f�r Speed-mode
#define Throttle_Flags1				13
#define Throttle_Start_Voltage0		14
#define Throttle_Start_Voltage1		15
#define Throttle_End_Voltage0		16
#define Throttle_End_Voltage1		17
#define Transmission_Type0			18
#define Transmission_Type1			19

#define MaxTorque_A0				20		// 20,22,24 f�r Torque-Mode; 21,23,25 f�r Speed-Mode
#define MaxTorque_A1				21
#define MaxTorque_B0				22
#define MaxTorque_B1				23
#define MaxTorque_C0				24
#define MaxTorque_C1				25

#define Transmissiontype_RS			26		// RS f�r Rechtsmotor_Speedmode, RT f�r Rechtsmotor_Torquemode
#define Transmissiontype_RT			27
#define Eventtimer_RS				28
#define Eventtimer_RT				29
#define Encoder_Offset_RS			30
#define Encoder_Offset_RT			31
#define SinMin_Voltage_RS			32
#define SinMin_Voltage_RT			33
#define SinMax_Voltage_RS			34
#define SinMax_Voltage_RT			35
#define CosMin_Voltage_RS			36
#define CosMin_Voltage_RT			37
#define CosMax_Voltage_RS			38
#define CosMax_Voltage_RT			39

#define Num_Entries_A0				40		// 40,42,44,46,48,50,52,54 f�r Torque-Mode; 41,43,45,47,49,51,53,55 f�r Speed-Mode; 56 f�r Beide
#define Num_Entries_A1				41
#define Num_Entries_B0				42
#define Num_Entries_B1				43
#define Analog_InputA0				44
#define Analog_InputA1				45
#define Analog_InputB0				46
#define Analog_InputB1				47
#define Num_Mapped_Objects_0		48
#define Num_Mapped_Objects_1		49
#define Map_A0						50
#define Map_A1						51
#define Map_B0						52
#define Map_B1						53
#define Map_C0						54
#define Map_C1						55
#define Num_Mapped_Objects_On		56

#define Test_MaxTorque				57
#define Test_MaxSpeed				58

/* -------------------------------- mathematische Konstanten --------------------*/
#define PI							314 // Pi mit 100 multipliziert, da keine Festkommazahlarithmetik

/* -------------------------------- Sonstiges ----------------------------------*/

// Aus Par_Fahrzeugmodel.m - Matlab
// lh=0.325;
// lv=0.71;        %Abstand zum Schwerpunkt von VA in m
// l=lv+lh;
// sw=1;           %Spurweite
// i_s=15;         %Lenk�bersetzung
// d= 0.26;        %Reifendurchmesser
// r=d/2;
// J=1.6;          %Massentraegheitsmoment der R�der einer Achse
// c1=0.686;       %Werte f�r Schlupfkurve
// c2=33.82;
// c3=0.36;
// hs=0.1;         %Hoehe des Schwerpunktes
// v0=0.001/3.6 ;  %m/s Anfangsgeschwindigkeit 0.001 km/h

// m=250;          %Masse des E-Karts in kg
// fR=0.015;       %Rollwiderstandsbeiwert
// rho=1.2;        %Luftdichte
// cW=0.62;        %Luftwiderstandsbeiwert
// A=0.5;          %Querschnittsflaeche
// g=9.81;         %Erdbeschleunigung

// v_ch oder folgende Parameter:
// cav=90000;      %Schraeglaufsteifigkeiten
// cah=90000;

/* Exported functions --------------------------------------------------------*/

#endif /*__PARAMETER_H */
/************************END OF FILE*******************************************/
