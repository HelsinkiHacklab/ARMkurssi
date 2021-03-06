/*
 * ui.cpp
 *
 *  Created on: Mar 30, 2020
 *      Author: martti
 */

#include <stdio.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "main.h"
#include <lcd.h>

extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;

static Lcd_PortType dataPorts[] = { D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port };
static Lcd_PinType dataPins[] = {D4_Pin, D5_Pin, D6_Pin, D7_Pin};

using namespace lcd;

LCD display( dataPorts, dataPins, RS_GPIO_Port, RS_Pin, RW_GPIO_Port, RW_Pin, EN1_GPIO_Port, EN1_Pin, LCD_4_BIT_MODE);

void UI( void *pvParameters ) {
	QueueHandle_t *lcdQ;
	LCDMessage *pMsg;

	lcdQ = (QueueHandle_t *)pvParameters;

    display.begin();
    display.clear();
    vTaskDelay(1000);
    display.move_cursor(0,0);
    display.write_string((char *)"UITask ohjaa n\xe1ytt\xef\xe1");
	while ( 1 ) {
		xQueueReceive(*lcdQ, &pMsg, portMAX_DELAY );
		display.move_cursor(pMsg->x, pMsg->y);
		switch ( pMsg->T ) {
			case u8: {
				display.write_int(pMsg->Len, pMsg->v.U8);
				break;
			}
			case i8: {
				display.write_int(pMsg->Len, pMsg->v.I8);
				break;
			}
			case s: {
				display.write_string(pMsg->v.S);
				break;
			}
			default: display.write_int(pMsg->Len, pMsg->v.U8);
		}
		delete pMsg;
	}
}

