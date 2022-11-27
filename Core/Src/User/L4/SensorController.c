/*
 * SensorController.c
 *
 *  Created on: Oct 24, 2022
 *      Author: kadh1
 */


#include <stdio.h>

#include "main.h"
#include "User/L2/Comm_Datalink.h"
#include "User/L3/AcousticSensor.h"
#include "User/L3/DepthSensor.h"
#include "User/L4/SensorPlatform.h"
#include "User/L4/SensorController.h"
#include "User/util.h"

//Required FreeRTOS header files
#include "FreeRTOS.h"
#include "Timers.h"
#include "semphr.h"

QueueHandle_t Queue_Sensor_Data;
QueueHandle_t Queue_HostPC_Data;


static void ResetMessageStruct(struct CommMessage* currentRxMessage){

	static const struct CommMessage EmptyMessage = {0};
	*currentRxMessage = EmptyMessage;
}

/******************************************************************************
This task is created from the main.
******************************************************************************/
void SensorControllerTask(void *params)
{
	enum AckTypes current_Sensor_Message = 0;
	enum HostPCCommands current_HostPC_Message = PC_Command_NONE;

	do {
		xQueueReceive( Queue_HostPC_Data, &current_HostPC_Message, 0 );

		if (current_HostPC_Message != PC_Command_NONE)
		{
			if (current_HostPC_Message == PC_Command_START)
			{
				send_sensorEnable_message(Acoustic,0);
				xQueueReceive( Queue_Sensor_Data, &current_Sensor_Message, 0 );
				while (current_Sensor_Message != AcousticSensorEnable)
				{
					xQueueReceive( Queue_Sensor_Data, &current_Sensor_Message, 0 );
					vTaskDelay(1000 / portTICK_RATE_MS);
				}

				send_sensorEnable_message(Depth,0);
				xQueueReceive( Queue_Sensor_Data, &current_Sensor_Message, 0 );
				while (current_Sensor_Message != DepthSensorEnable)
				{
					xQueueReceive( Queue_Sensor_Data, &current_Sensor_Message, 0 );
					vTaskDelay(1000 / portTICK_RATE_MS);
				}
			}
			else if (current_HostPC_Message == PC_Command_RESET)
			{
				send_sensorReset_message();
				xQueueReceive( Queue_Sensor_Data, &current_Sensor_Message, 0 );
				while (current_Sensor_Message != RemoteSensingPlatformReset)
				{
					xQueueReceive( Queue_Sensor_Data, &current_Sensor_Message, 0 );
					vTaskDelay(1000 / portTICK_RATE_MS);
				}
			}
		}

		vTaskDelay(1000 / portTICK_RATE_MS);
	} while(1);
}




/*
 * This task reads the queue of characters from the Sensor Platform when available
 * It then sends the processed data to the Sensor Controller Task
 */
void SensorPlatform_RX_Task(){
	struct CommMessage currentRxMessage = {0};
	Queue_Sensor_Data = xQueueCreate(80, sizeof(struct CommMessage));

	request_sensor_read();  // requests a usart read (through the callback)

	while(1){
		parse_sensor_message(&currentRxMessage);

		if(currentRxMessage.IsMessageReady == true && currentRxMessage.IsCheckSumValid == true){

			xQueueSendToBack(Queue_Sensor_Data, &currentRxMessage, 0);
			ResetMessageStruct(&currentRxMessage);
		}
	}
}



/*
 * This task reads the queue of characters from the Host PC when available
 * It then sends the processed data to the Sensor Controller Task
 */
void HostPC_RX_Task(){

	enum HostPCCommands HostPCCommand = PC_Command_NONE;

	Queue_HostPC_Data = xQueueCreate(80, sizeof(enum HostPCCommands));

	request_hostPC_read();

	while(1){
		HostPCCommand = parse_hostPC_message();

		if (HostPCCommand != PC_Command_NONE){
			xQueueSendToBack(Queue_HostPC_Data, &HostPCCommand, 0);
		}

	}
}
