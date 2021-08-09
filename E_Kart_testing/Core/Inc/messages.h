/*
 * messages.h
 *	Created on: 22.07.2021 - file contains all hard coded string messages to display on LCD
 *
 */

#ifndef INC_MESSAGES_H_
#define INC_MESSAGES_H_


// RCP messages

char RCP_ON_MSG[] = "ON";
char RCP_OFF_MSG[] = "OFF";
char RCP_CONNECTING[] = "BREMSEN!Verbinde..."; //19 char + null terminator, so 20 max
// RCP error strings
char NO_ERROR_MSG[] = "No Error";
char RCP_CONNECTIONERR_MSG[] = "Connection Error";
char NO_RCP_HEARTBEAT_MSG[] = "No RCP Heartbeat";
char NO_MOTOR_HEARTBEAT_MSG[] = "No Motor Heatbeat"; //17+1 char max
char RCP_MOVING_ERROR_MSG[] = "Ekart moving!"; //17+1 char max



#endif /* INC_MESSAGES_H_ */
