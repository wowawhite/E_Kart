/*
 * control.c
 *
 *  Created on: 28.11.2018
 *      Author: melf_
 */

#include "control.h"
#include "can.h"

CAN_RxHeaderTypeDef RxMessage;			    // CAN-Receive
CAN_TxHeaderTypeDef TxMessage;	            // CAN-Transmit
#define ABS(x) ((x) > 0 ? (x) : -(x))
//#define m_Gas      (( Norm_100 - Norm_0) * 100) / (DUTY_CYCLE_GAS_MAX - DUTY_CYCLE_GAS_MIN)         // 2 Nachkommastellen im Zwischenergebnis
//int32_t n_Gas    =  ( (Norm_0 * 100) - ( m_Gas * DUTY_CYCLE_GAS_MIN ) ) ;							                // 2 Nachkommastellen im Zwischenergebnis
//#define m_Bremse   (( Norm_100 - Norm_0) * 100) / (DUTY_CYCLE_BREMSE_MAX - DUTY_CYCLE_BREMSE_MIN)   // 2 Nachkommastellen im Zwischenergebnis
//int32_t n_Bremse =  ( (Norm_0 * 100) - m_Bremse * DUTY_CYCLE_BREMSE_MIN ) ;  						              // 2 Nachkommastellen im Zwischenergebnis

extern uint16_t *Pointer_GasProzent;	// Parameter dient zur Touch-Eingabe, Wertbereich (0-100)=(0%-100%), dessen Initialwert ist 100%
															// als der Beiwert f�r Einstellung des Wirkungsbereichs von Gaspedal(bei Drehzahlvorgabe_mode )
                              // z.B wenn "Pointer_GasProzent=100", Gaspedal volldurchgedr�ckt, die max.Drehzahl=3000 wird durch CAN-BUS vorgegeben
                              // wenn "Pointer_GasProzent=50", volldurchgedr�ckt, dann wird nur die Drehzahl=1500 vorgegeben.
															// der Wirkungsbereich halbiert sich

extern uint16_t *Pointer_BeschlProzent;   // Parameter dient zur Touch-Eingabe, Wertbereich (0-100)=(0%-100%), dessen Initialwert ist 100%					--------------------------------------
extern uint16_t *Pointer_Rueckwaert;


uint8_t txData[8];
uint32_t Mailbox=0;
volatile uint8_t  Sp_mSek=0;               // Milli-Sekunden, 0-9
uint8_t  Sp_mSek_mul10=0;         // 10er-Vielfache von Millisekunden, 0-9 = 0 .. 90 Millisekunden
uint8_t  Sp_mSek_mul100=0;        // 100er-Vielfache von Millisekunden, 0-9 = 0 .. 900 Millisekunden
uint8_t  Sp_Sek=0;                // Sekunden, 0-9
uint8_t  Sp_Min=0;                // Minuten, 0-59
uint8_t  Sp_Stu=0;                // Stunden, 0-59
uint32_t DutyCycleGas;         // DutyCycle des Gassensors als absoluter Wert, 0-1000 = 0-100,0%
uint32_t DutyCycleBremse;	   	 // DutyCycle des Bremssensors als absoluter Wert, 0-1000 = 0-100,0%
uint32_t FrequencyGas;	       // PWM-Frequenz des Gassensors in Hz
uint32_t FrequencyBremse;      // PWM-Frequenz des Bremssensors in Hz

uint32_t Motor_Drehzahl=0;	   // Motordrehzahl in rpm, 0 .. DREHZAHL_MAX

int16_t  Vorgabe_Moment=0;           // 0-1000 = 0-100,0%, Vorgabewert des Moments
int16_t  Vorgabe_Moment_rechts=0;    // 0-1000 = 0-100,0%, Vorgabewert des Moments in %*10 f�r den Motorcontroller rechts
int16_t  Vorgabe_Moment_links=0;     // 0-1000 = 0-100,0%, Vorgabewert des Moments in %*10 f�r den Motorcontroller links
uint8_t  Drehrichtung_Moment_rechts=0; // Drehrichtung f�r Momentenvorgabe, Motor-Controller rechts
uint8_t  Drehrichtung_Moment_links=0; // Drehrichtung f�r Momentenvorgabe, Motor-Controller links
uint16_t Rueckwaert=0;
int16_t  Lenkradwinkel;         // Lenkradwinkelwert in Grad*10 nach LW-Sensor �ber CAN-It, -7800 .. 0 .. +7800
uint32_t Geschwindigkeit_kmh;   // Geschwindigkeit des E-Karts in kmh, 0 .. umgerechneter Wert von DREHZAHL_MAX
uint32_t Merker_aktuelle_Drehzahl; // Merker f�r Berechnung der Beschleunigung in stm32f10x_it.c   Funktion: AccelerationCalc()

uint8_t  ReverseGear;           // R�ckw�rtsgang 0 - aus, 1 - ein = R�ckw�rtsfahren
uint8_t  Hauptrelais;           // Schaltsignal-Hauptrelias,  1 - alles i.O. , 0 - Fehler
int32_t  Motor_Drehzahl_r;
int32_t  Motor_Drehzahl_l;
extern int16_t  Gas;				              // Wert des Gaspedals, der �ber PWM-Sensor ermittelt wurde, Wertebereich: 0 .. 100 = Volldurchgedr�ckt
extern int16_t  Bremse;				            // Bremswert, der �ber PWM-Sensor ermittelt wurde, Wertebereich: 0 .. 100 = Volldurchgedr�ckt
uint8_t  Heartbeat_MC_rechts;   // Heartbeat Motorcontroller rechts
uint8_t  Heartbeat_MC_links;    // Heartbeat Motorcontroller links

uint8_t StateofCharge=0;
uint8_t Temperatur_Zellen_l[8];
uint8_t Temperatur_Zellen_r[8];
uint16_t Spannung_Zellen_l[8];
uint16_t Spannung_Zellen_r[8];

uint8_t touchIRQ = 0;
// RCP-Mode global communication variables
uint8_t RCP_Mode_status= 0;
uint8_t RCP_Mode_selected = 0;
uint8_t RCP_Mode_errorcode = 0;
uint8_t Heartbeat_RCP = 0;
uint8_t SDOack = 0;

void HAL_SYSTICK_Callback(void)
{
	Sp_mSek++;

	if ( Sp_mSek == 10 )
	{
		Sp_mSek=0;
	  	Sp_mSek_mul10++;
	}
	if ( Sp_mSek_mul10 == 10 )
	{
		Sp_mSek_mul10=0;
		Sp_mSek_mul100++;
	}
	if ( Sp_mSek_mul100 == 10 )
	{
		Sp_mSek_mul100=0;
		Sp_Sek++;
	}

	if (Sp_Sek == 60)		   // Anzeige bis 59
	{
		Sp_Sek=0;
		Sp_Min++;
	}

	if (Sp_Min == 60)			   // Anzeige bis 59
	{
		Sp_Min=0;
		Sp_Stu++;
	}
	if (Sp_Stu == 99)		      // Anzeige bis 99
	{
		Sp_Stu=0;
	}

	// Motor_Drehzahl_r wird �ber CAN-Interrupt bereitgestellt
	// Motor_Drehzahl_l wird �ber CAN-Interrupt bereitgestellt
	Motor_Drehzahl_r = ABS(Motor_Drehzahl_r);
	Motor_Drehzahl_l = ABS(Motor_Drehzahl_l);
	// Mittlewert berechnen, Schiebeoperation um eins nach rechts = Division durch 2
	Motor_Drehzahl = ( Motor_Drehzahl_r + Motor_Drehzahl_l )>>1 ;

	Merker_aktuelle_Drehzahl= Motor_Drehzahl;
	// Zwischenspeicherung des aktuellen Wertes an den R�dern

	// Geschwindigkeit berechnen
	// v = 2*PI*n_reifen*r_reifen , n_reifen = n_motor / �bersetzung
	Geschwindigkeit_kmh = (2*PI*Motor_Drehzahl*REIFENRADIUS*36)/(100*60*100*MOTOR_RAD_RATIO);
	//if (Geschwindigkeit_kmh>60) (Geschwindigkeit_kmh=60);	// Begrenzung der Geschwindigkeit, notwendig f�r Anzeige

	// R�ckwertsgang einlesen, PE6
	//Schaltsignal-Hauptrelais einlesen, PE4
	ReverseGear = HAL_GPIO_ReadPin(Reverse_Switch_Input_GPIO_Port,Reverse_Switch_Input_Pin);
	Hauptrelais = HAL_GPIO_ReadPin(Main_Relay_Input_GPIO_Port, Main_Relay_Input_Pin);

	/*Sync Message*/
	if(Sp_mSek==1)
	{
		TxMessage.StdId = ID_Synchronisation ;    // Standart Identifier
		TxMessage.ExtId = ID_Synchronisation;     // extended Identifier
		TxMessage.RTR = CAN_RTR_DATA;             // Data frame
		TxMessage.IDE = CAN_ID_STD;               // use Standart Identifier
		TxMessage.DLC = 0;                        // length of the frame in Bytes
		txData[0] = 0;
		HAL_CAN_AddTxMessage(&hcan1,&TxMessage,txData,(uint32_t *)Mailbox);
	}

	/*Heartbeat FFR*/
	if((Sp_mSek==5) && (Sp_mSek_mul10==0))
	{
		TxMessage.StdId = ID_Heartbeat_FFR ;    // Standart Identifier
		TxMessage.ExtId = ID_Heartbeat_FFR;     // extended Identifier
		TxMessage.RTR = CAN_RTR_DATA;             // Data frame
		TxMessage.IDE = CAN_ID_STD;               // use Standart Identifier
		TxMessage.DLC = 1;                        // length of the frame in Bytes
		txData[0] = Msg_Heartbeat_FFR;
		HAL_CAN_AddTxMessage(&hcan1,&TxMessage,txData,(uint32_t *)Mailbox);
	}

	/*Send TxPDO1: Gas, Brake and direction*/
	if(Sp_mSek == 4)
	{
		TxMessage.StdId = ID_TxPDO1;	// Standart Identifier
		TxMessage.ExtId = ID_TxPDO1;	// extended Identifier
		TxMessage.RTR = CAN_RTR_DATA;   // Data frame
		TxMessage.IDE = CAN_ID_STD;     // use Standart Identifier
		TxMessage.DLC = 3;              // length of the frame in Bytes
		txData[0] = (uint8_t) Gas;
		txData[1] = (uint8_t) Bremse;
		txData[2] = ReverseGear;

		HAL_CAN_AddTxMessage(&hcan1,&TxMessage,txData,(uint32_t *)Mailbox);
	}
	if(Sp_mSek == 3)
	{
	    /*NMT*/
		TxMessage.StdId = ID_NMT ;     // Standart Identifier
	    TxMessage.ExtId = ID_NMT ;     // extended Identifier
	    TxMessage.RTR = CAN_RTR_DATA;       // Data frame
	    TxMessage.IDE = CAN_ID_STD;         // use Standart Identifier
	    TxMessage.DLC = 2;                  // length of the frame in Bytes
	    txData[0] = 0x01;  		// 0. Byte  , Little Endian
	    txData[1] = 0x00;  		// 0. Byte  , Little Endian

	    HAL_CAN_AddTxMessage(&hcan1, &TxMessage,txData,(uint32_t *)Mailbox);     // Message �bertragen
	}

	/*Turn RCP-Mode on or off*/
	if(RCP_Mode_selected != RCP_Mode_status)
	{
		if(Motor_Drehzahl <= 1 && Vorgabe_Moment == 0)
		{
			TxMessage.StdId = ID_SDO_RCP_Rx;	// Standart Identifier
			TxMessage.ExtId = ID_SDO_RCP_Rx;	// extended Identifier
			TxMessage.RTR = CAN_RTR_DATA;   // Data frame
			TxMessage.IDE = CAN_ID_STD;     // use Standart Identifier
			TxMessage.DLC = 8;              // length of the frame in Bytes
			txData[0] = 0x2F;
			txData[1] = 0x00;
			txData[2] = 0x20;
			txData[3] = 0x01;
			txData[4] = RCP_Mode_selected;

			HAL_CAN_AddTxMessage(&hcan1,&TxMessage,txData,(uint32_t *)Mailbox);
			RCP_Mode_errorcode = WAITING_RESPOND;
		}
	}

	if (( Sp_mSek==3 ) || ( Sp_mSek==8 ))
	{
		if ( (Heartbeat_MC_rechts==Msg_Heartbeat_MC_rechts) &&	(Heartbeat_MC_links==Msg_Heartbeat_MC_links) )
		{
			/*************************************************************************************************************************/
			/*****************************************************RCP-Mode************************************************************/
			/*************************************************************************************************************************/
			if (RCP_Mode_errorcode == NO_MOTOR_HEARTBEAT)
			{
				RCP_Mode_errorcode = NO_ERROR;
			}

			if(RCP_Mode_status== 1)
			{
				if(Heartbeat_RCP == 0)
				{
					//kein Heartbeat
					Emergency_Stop();
					RCP_Mode_errorcode = NO_RCP_HEARTBEAT;
				}
			}
			/*************************************************************************************************************************/

			else
			{
				if (Bremse>=Bremse_SchwelleVerGas)
				{
					Gas=0;
				}
				Vorgabe_Moment = (Gas) * 10	;  // Vorgabe des Moments, 0 - 1000 = 0-100,0% - Aufl�sungsfehler, Gas nur ca. 200 Werte
				Vorgabe_Moment =  (Vorgabe_Moment*(*Pointer_BeschlProzent))/100; 		// Touch-Parameter "Pointer_BeschlProzent"
				Rueckwaert=	(Vorgabe_Moment*(*Pointer_Rueckwaert))/100;

				if(Motor_Drehzahl>(DREHZAHL_MAX*(*Pointer_GasProzent))/100)
				{
					Vorgabe_Moment = Vorgabe_Moment-Vorgabe_Moment*((Motor_Drehzahl-(DREHZAHL_MAX*(*Pointer_GasProzent))/100)*100)/(DREHZAHL_GRENZ)/100;

					if(Vorgabe_Moment<0)
					{
						Vorgabe_Moment=0;
					}
				}
				if(ReverseGear)
				{
					Vorgabe_Moment_rechts=Rueckwaert*(-1);
					Vorgabe_Moment_links=Rueckwaert;
					Drehrichtung_Moment_links = Motor_Moment_REV;// Nachrichteninhalt �ber define setzen
					Drehrichtung_Moment_rechts = Motor_Moment_FWD; // Nachrichteninhalt �ber define setzen
				}

				else
				{
					Vorgabe_Moment_rechts=Vorgabe_Moment;
					Vorgabe_Moment_links=Vorgabe_Moment*(-1);
					Drehrichtung_Moment_links = Motor_Moment_FWD;// Nachrichteninhalt �ber define setzen
					Drehrichtung_Moment_rechts = Motor_Moment_REV; // Nachrichteninhalt �ber define setzen
				}


				TxMessage.StdId = ID_MC_rechts ;    // Standart Identifier
				TxMessage.ExtId = ID_MC_rechts;     // extended Identifier
				TxMessage.RTR = CAN_RTR_DATA;       // Data frame
				TxMessage.IDE = CAN_ID_STD;         // use Standart Identifier
				TxMessage.DLC = 8;                  // length of the frame in Bytes

				txData[0] = (uint8_t) ( Vorgabe_Moment_rechts );  		// 0. Byte  , Little Endian
				txData[1] = (uint8_t) ( Vorgabe_Moment_rechts>>8 ); 	// 1. Byte
				txData[2] = Drehrichtung_Moment_rechts;               // 2. Byte

				HAL_CAN_AddTxMessage(&hcan1, &TxMessage,txData,(uint32_t *)Mailbox);     // Message �bertragen

				TxMessage.StdId = ID_MC_links ;     // Standart Identifier
				TxMessage.ExtId = ID_MC_links ;     // extended Identifier
				TxMessage.RTR = CAN_RTR_DATA;       // Data frame
				TxMessage.IDE = CAN_ID_STD;         // use Standart Identifier
				TxMessage.DLC = 8;                  // length of the frame in Bytes
				txData[0] = (uint8_t) ( Vorgabe_Moment_links );  		// 0. Byte  , Little Endian
				txData[1] = (uint8_t) ( Vorgabe_Moment_links>>8 ); 	// 1. Byte
				txData[2] = Drehrichtung_Moment_links;              // 2. Byte

				HAL_CAN_AddTxMessage(&hcan1, &TxMessage,txData,(uint32_t *)Mailbox);     // Message �bertragen
			}

		}
		/*Heartbeat Motorcontroller*/
		else
		{
			Emergency_Stop();
			RCP_Mode_errorcode = NO_MOTOR_HEARTBEAT;
		}
	}

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	// Variablendeklaration
	CAN_RxHeaderTypeDef sCanRxHeader;			// Header empfangener Nachrichten
	uint8_t RxMessage[8];		// Zwischenspeicher f�r empfangene Daten
	// Nachricht abrufen
	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &sCanRxHeader, RxMessage) != HAL_OK)
	{
		/* Reception Error */
//		Error_Handler();
	}

	//Lenkwinkel und Lenkgeschwindigkeit
	if ((sCanRxHeader.StdId == ID_Lenkradwinkelsensor)&&(sCanRxHeader.IDE == CAN_ID_STD) && (sCanRxHeader.DLC == DL_Lenkradwinkelsensor))
	{
		Lenkradwinkel = RxMessage[1];       // erste 8 Bits
		Lenkradwinkel = (Lenkradwinkel<<8);
		Lenkradwinkel = Lenkradwinkel | RxMessage[0];
	}

	/*Heartbeat RCP*/
	if ((sCanRxHeader.StdId == ID_Heartbeat_RCP)&&(sCanRxHeader.IDE == CAN_ID_STD) && (sCanRxHeader.DLC == 1))
	{
		Heartbeat_RCP = RxMessage[0];
	}

	/*SDO Tx RCP*/
	if ((sCanRxHeader.StdId == ID_SDO_RCP_Tx)&&(sCanRxHeader.IDE == CAN_ID_STD) && (sCanRxHeader.DLC == 8))
	{
		SDOack= RxMessage[0];

		if(SDOack == 0x60)
		{
			RCP_Mode_status = RCP_Mode_selected;
		}else{
			RCP_Mode_errorcode = NO_CAN_RESPOND;
		}
	}


	// Heartbeat engine right
	if ((sCanRxHeader.StdId == ID_Heartbeat_MC_rechts)&&(sCanRxHeader.IDE == CAN_ID_STD) && (sCanRxHeader.DLC == DL_Heartbeat_MC_rechts))
	{
		Heartbeat_MC_rechts = RxMessage[0];       // erste 8 Bits
	}
	// Heartbeat engine left
	if ((sCanRxHeader.StdId == ID_Heartbeat_MC_links)&&(sCanRxHeader.IDE == CAN_ID_STD) && (sCanRxHeader.DLC == DL_Heartbeat_MC_links))
	{
		Heartbeat_MC_links = RxMessage[0];       // erste 8 Bits
	}
	// engine rpm right
	if ((sCanRxHeader.StdId == ID_Motor_Drehzahl_r)&&(sCanRxHeader.IDE == CAN_ID_STD) && (sCanRxHeader.DLC == DL_Motor_Drehzahl_r))
	{
		Motor_Drehzahl_r = RxMessage[3];                     // h�chsten Bits, 16-Bit
		Motor_Drehzahl_r =  ( Motor_Drehzahl_r<<8 );              // h�chsten Bits schieben
	    Motor_Drehzahl_r = Motor_Drehzahl_r | RxMessage[2];  // Bits dazu setzen
	    Motor_Drehzahl_r =  ( Motor_Drehzahl_r<<8 );              // Bits schieben
	    Motor_Drehzahl_r = Motor_Drehzahl_r | RxMessage[1];  // Bits dazu setzen
	    Motor_Drehzahl_r =  ( Motor_Drehzahl_r<<8 );              // Bits schieben
	    Motor_Drehzahl_r = Motor_Drehzahl_r | RxMessage[0];  // letzten Bits dazu setzen
	}
	// engine rpm left
	if ((sCanRxHeader.StdId == ID_Motor_Drehzahl_l)&&(sCanRxHeader.IDE == CAN_ID_STD) && (sCanRxHeader.DLC == DL_Motor_Drehzahl_l))
	{
	    Motor_Drehzahl_l = RxMessage[3];                     // h�chsten Bits, 16-Bit
		Motor_Drehzahl_l =  ( Motor_Drehzahl_l<<8 );              // h�chsten Bits schieben
	    Motor_Drehzahl_l = Motor_Drehzahl_l | RxMessage[2];  // Bits dazu setzen
	    Motor_Drehzahl_l =  ( Motor_Drehzahl_l<<8 );              // Bits schieben
	    Motor_Drehzahl_l = Motor_Drehzahl_l | RxMessage[1];  // Bits dazu setzen
	    Motor_Drehzahl_l =  ( Motor_Drehzahl_l<<8 );              // Bits schieben
	    Motor_Drehzahl_l = Motor_Drehzahl_l | RxMessage[0];  // letzten Bits dazu setzen
	}

	// State of charge
	if ((sCanRxHeader.StdId == ID_Status_Batteriesteuerung)&&(sCanRxHeader.IDE == CAN_ID_STD) && (sCanRxHeader.DLC == DL_Status_Batteriesteuerung))
	{
		StateofCharge = RxMessage[6];                     // h�chsten Bits, 16-Bit
	}

	// Cell temperature
	if ((sCanRxHeader.StdId == ID_Temperatur_Zellen_l)&&(sCanRxHeader.IDE == CAN_ID_STD) && (sCanRxHeader.DLC == DL_Temperatur_Zellen_l))
	{
		Temperatur_Zellen_l[0] = RxMessage[0]-100;                     // h�chsten Bits, 16-Bit
		Temperatur_Zellen_l[1] = RxMessage[1]-100;                     // h�chsten Bits, 16-Bit
		Temperatur_Zellen_l[2] = RxMessage[2]-100;                     // h�chsten Bits, 16-Bit
		Temperatur_Zellen_l[3] = RxMessage[3]-100;                     // h�chsten Bits, 16-Bit
		Temperatur_Zellen_l[4] = RxMessage[4]-100;                     // h�chsten Bits, 16-Bit
		Temperatur_Zellen_l[5] = RxMessage[5]-100;                     // h�chsten Bits, 16-Bit
		Temperatur_Zellen_l[6] = RxMessage[6]-100;                     // h�chsten Bits, 16-Bit
		Temperatur_Zellen_l[7] = RxMessage[7]-100;                     // h�chsten Bits, 16-Bit

	}

	// Cell temperatur
	if ((sCanRxHeader.StdId == ID_Temperatur_Zellen_r)&&(sCanRxHeader.IDE == CAN_ID_STD) && (sCanRxHeader.DLC == DL_Temperatur_Zellen_r))
	{
		Temperatur_Zellen_r[0] = RxMessage[0]-100;                     // h�chsten Bits, 16-Bit
		Temperatur_Zellen_r[1] = RxMessage[1]-100;                     // h�chsten Bits, 16-Bit
		Temperatur_Zellen_r[2] = RxMessage[2]-100;                     // h�chsten Bits, 16-Bit
		Temperatur_Zellen_r[3] = RxMessage[3]-100;                     // h�chsten Bits, 16-Bit
		Temperatur_Zellen_r[4] = RxMessage[4]-100;                     // h�chsten Bits, 16-Bit
		Temperatur_Zellen_r[5] = RxMessage[5]-100;                     // h�chsten Bits, 16-Bit
		Temperatur_Zellen_r[6] = RxMessage[6]-100;                     // h�chsten Bits, 16-Bit
		Temperatur_Zellen_r[7] = RxMessage[7]-100;                     // h�chsten Bits, 16-Bit
	}

	// Cell voltage
	if ((sCanRxHeader.StdId == ID_Spannung_Zellen_l)&&(sCanRxHeader.IDE == CAN_ID_STD) && (sCanRxHeader.DLC == DL_Spannung_Zellen_l))
	{
		Spannung_Zellen_l[0] = RxMessage[0]+200;                     // h�chsten Bits, 16-Bit
		Spannung_Zellen_l[1] = RxMessage[1]+200;                     // h�chsten Bits, 16-Bit
		Spannung_Zellen_l[2] = RxMessage[2]+200;                     // h�chsten Bits, 16-Bit
		Spannung_Zellen_l[3] = RxMessage[3]+200;                     // h�chsten Bits, 16-Bit
		Spannung_Zellen_l[4] = RxMessage[4]+200;                     // h�chsten Bits, 16-Bit
		Spannung_Zellen_l[5] = RxMessage[5]+200;                     // h�chsten Bits, 16-Bit
		Spannung_Zellen_l[6] = RxMessage[6]+200;                     // h�chsten Bits, 16-Bit
		Spannung_Zellen_l[7] = RxMessage[7]+200;                     // h�chsten Bits, 16-Bit
	}

	// Cell voltage
	if ((sCanRxHeader.StdId == ID_Spannung_Zellen_r)&&(sCanRxHeader.IDE == CAN_ID_STD) && (sCanRxHeader.DLC == DL_Spannung_Zellen_r))
	{
		Spannung_Zellen_r[0] = RxMessage[0]+200;                     // h�chsten Bits, 16-Bit
		Spannung_Zellen_r[1] = RxMessage[1]+200;                     // h�chsten Bits, 16-Bit
		Spannung_Zellen_r[2] = RxMessage[2]+200;                     // h�chsten Bits, 16-Bit
		Spannung_Zellen_r[3] = RxMessage[3]+200;                     // h�chsten Bits, 16-Bit
		Spannung_Zellen_r[4] = RxMessage[4]+200;                     // h�chsten Bits, 16-Bit
		Spannung_Zellen_r[5] = RxMessage[5]+200;                     // h�chsten Bits, 16-Bit
		Spannung_Zellen_r[6] = RxMessage[6]+200;                     // h�chsten Bits, 16-Bit
		Spannung_Zellen_r[7] = RxMessage[7]+200;                     // h�chsten Bits, 16-Bit
	}

}

void Motor_Init(void)
{
    HAL_Delay(3000);

    /*NMT*/
	TxMessage.StdId = ID_NMT ;     // Standart Identifier
    TxMessage.ExtId = ID_NMT ;     // extended Identifier
    TxMessage.RTR = CAN_RTR_DATA;       // Data frame
    TxMessage.IDE = CAN_ID_STD;         // use Standart Identifier
    TxMessage.DLC = 2;                  // length of the frame in Bytes
    txData[0] = 0x01;  		// 0. Byte  , Little Endian
    txData[1] = 0x00;  		// 0. Byte  , Little Endian

    HAL_CAN_AddTxMessage(&hcan1, &TxMessage,txData,(uint32_t *)Mailbox);     // Message �bertragen

    HAL_Delay(50);

    /*Analog Output value 1 left*/
    TxMessage.StdId = ID_SDO_MC_links ;     // Standart Identifier
    TxMessage.ExtId = ID_SDO_MC_links ;     // extended Identifier
    TxMessage.RTR = CAN_RTR_DATA;       // Data frame
    TxMessage.IDE = CAN_ID_STD;         // use Standart Identifier
    TxMessage.DLC = 8;                  // length of the frame in Bytes
    txData[0] = 0x2B;
    txData[1] = 0x11;
    txData[2] = 0x6C;
    txData[3] = 0x01;
    txData[4] = 0xFF;
    txData[5] = 0x7F;
    txData[6] = 0x00;
    txData[7] = 0x00;

    HAL_CAN_AddTxMessage(&hcan1, &TxMessage,txData,(uint32_t *)Mailbox);     // Message �bertragen

    /*Analog Output value 1 right*/
    HAL_Delay(50);
    TxMessage.StdId = ID_SDO_MC_rechts ;     // Standart Identifier
    TxMessage.ExtId = ID_SDO_MC_rechts ;     // extended Identifier
    TxMessage.RTR = CAN_RTR_DATA;       // Data frame
    TxMessage.IDE = CAN_ID_STD;         // use Standart Identifier
    TxMessage.DLC = 8;                  // length of the frame in Bytes
    txData[0] = 0x2B;
    txData[1] = 0x11;
    txData[2] = 0x6C;
    txData[3] = 0x01;
    txData[4] = 0xFF;
    txData[5] = 0x7F;
    txData[6] = 0x00;
    txData[7] = 0x00;

    HAL_CAN_AddTxMessage(&hcan1, &TxMessage,txData,(uint32_t *)Mailbox);     // Message �bertragen

    HAL_Delay(50);

    /*control word left*/
    TxMessage.StdId = ID_SDO_MC_links ;     // Standart Identifier
    TxMessage.ExtId = ID_SDO_MC_links ;     // extended Identifier
    TxMessage.RTR = CAN_RTR_DATA;       // Data frame
    TxMessage.IDE = CAN_ID_STD;         // use Standart Identifier
    TxMessage.DLC = 8;                  // length of the frame in Bytes
    txData[0] = 0x2B;
    txData[1] = 0x40;
    txData[2] = 0x60;
    txData[3] = 0x00;
    txData[4] = 0x06;
    txData[5] = 0x00;
    txData[6] = 0x00;
    txData[7] = 0x00;

    HAL_CAN_AddTxMessage(&hcan1, &TxMessage,txData,(uint32_t *)Mailbox);     // Message �bertragen

    HAL_Delay(50);

    /*control word left*/
    TxMessage.StdId = ID_SDO_MC_links ;     // Standart Identifier
    TxMessage.ExtId = ID_SDO_MC_links ;     // extended Identifier
    TxMessage.RTR = CAN_RTR_DATA;       // Data frame
    TxMessage.IDE = CAN_ID_STD;         // use Standart Identifier
    TxMessage.DLC = 8;                  // length of the frame in Bytes
    txData[0] = 0x2B;
    txData[1] = 0x40;
    txData[2] = 0x60;
    txData[3] = 0x00;
    txData[4] = 0x07;
    txData[5] = 0x00;
    txData[6] = 0x00;
    txData[7] = 0x00;

    HAL_CAN_AddTxMessage(&hcan1, &TxMessage,txData,(uint32_t *)Mailbox);     // Message �bertragen

    HAL_Delay(50);
    /*control word left*/
    TxMessage.StdId = ID_SDO_MC_links ;     // Standart Identifier
    TxMessage.ExtId = ID_SDO_MC_links ;     // extended Identifier
    TxMessage.RTR = CAN_RTR_DATA;       // Data frame
    TxMessage.IDE = CAN_ID_STD;         // use Standart Identifier
    TxMessage.DLC = 8;                  // length of the frame in Bytes
    txData[0] = 0x2B;
    txData[1] = 0x40;
    txData[2] = 0x60;
    txData[3] = 0x00;
    txData[4] = 0x0f;
    txData[5] = 0x00;
    txData[6] = 0x00;
    txData[7] = 0x00;

    HAL_CAN_AddTxMessage(&hcan1, &TxMessage,txData,(uint32_t *)Mailbox);     // Message �bertragen

    HAL_Delay(50);
    /*control word right*/
    TxMessage.StdId = ID_SDO_MC_rechts ;     // Standart Identifier
    TxMessage.ExtId = ID_SDO_MC_rechts ;     // extended Identifier
    TxMessage.RTR = CAN_RTR_DATA;       // Data frame
    TxMessage.IDE = CAN_ID_STD;         // use Standart Identifier
    TxMessage.DLC = 8;                  // length of the frame in Bytes
    txData[0] = 0x2B;
    txData[1] = 0x40;
    txData[2] = 0x60;
    txData[3] = 0x00;
    txData[4] = 0x06;
    txData[5] = 0x00;
    txData[6] = 0x00;
    txData[7] = 0x00;

    HAL_CAN_AddTxMessage(&hcan1, &TxMessage,txData,(uint32_t *)Mailbox);     // Message �bertragen

    HAL_Delay(50);
    /*control word right*/
    TxMessage.StdId = ID_SDO_MC_rechts ;     // Standart Identifier
    TxMessage.ExtId = ID_SDO_MC_rechts ;     // extended Identifier
    TxMessage.RTR = CAN_RTR_DATA;       // Data frame
    TxMessage.IDE = CAN_ID_STD;         // use Standart Identifier
    TxMessage.DLC = 8;                  // length of the frame in Bytes
    txData[0] = 0x2B;
    txData[1] = 0x40;
    txData[2] = 0x60;
    txData[3] = 0x00;
    txData[4] = 0x07;
    txData[5] = 0x00;
    txData[6] = 0x00;
    txData[7] = 0x00;

    HAL_CAN_AddTxMessage(&hcan1, &TxMessage,txData,(uint32_t *)Mailbox);     // Message �bertragen

    HAL_Delay(50);
    /*control word right*/
    TxMessage.StdId = ID_SDO_MC_rechts ;     // Standart Identifier
    TxMessage.ExtId = ID_SDO_MC_rechts ;     // extended Identifier
    TxMessage.RTR = CAN_RTR_DATA;       // Data frame
    TxMessage.IDE = CAN_ID_STD;         // use Standart Identifier
    TxMessage.DLC = 8;                  // length of the frame in Bytes
    txData[0] = 0x2B;
    txData[1] = 0x40;
    txData[2] = 0x60;
    txData[3] = 0x00;
    txData[4] = 0x0f;
    txData[5] = 0x00;
    txData[6] = 0x00;
    txData[7] = 0x00;

    HAL_CAN_AddTxMessage(&hcan1, &TxMessage,txData,(uint32_t *)Mailbox);     // Message �bertragen
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_6) touchIRQ = 1;
	else { __NOP(); }
}

void Emergency_Stop()
{
	Vorgabe_Moment=0;

	TxMessage.StdId = ID_MC_rechts ;    // Standart Identifier
	TxMessage.ExtId = ID_MC_rechts;     // extended Identifier
	TxMessage.RTR = CAN_RTR_DATA;       // Data frame
	TxMessage.IDE = CAN_ID_STD;         // use Standart Identifier
	TxMessage.DLC = 8;                  // length of the frame in Bytes

	txData[0] = (uint8_t) Vorgabe_Moment;
	txData[1] = (uint8_t) Vorgabe_Moment;
	txData[2] = Drehrichtung_Moment_rechts;

	HAL_CAN_AddTxMessage(&hcan1, &TxMessage,txData,(uint32_t *)Mailbox);     // Message �bertragen

	TxMessage.StdId = ID_MC_links ;     // Standart Identifier
	TxMessage.ExtId = ID_MC_links ;     // extended Identifier
	TxMessage.RTR = CAN_RTR_DATA;       // Data frame
	TxMessage.IDE = CAN_ID_STD;         // use Standart Identifier
	TxMessage.DLC = 8;                  // length of the frame in Bytes
	txData[0] = (uint8_t) Vorgabe_Moment;
	txData[1] = (uint8_t) Vorgabe_Moment;
	txData[2] = Drehrichtung_Moment_links;

	HAL_CAN_AddTxMessage(&hcan1, &TxMessage,txData,(uint32_t *)Mailbox);     // Message �bertragen
}
