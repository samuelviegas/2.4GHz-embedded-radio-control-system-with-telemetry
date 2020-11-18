// 2.4GHz embedded radio control system with telemetry
// Writen by Samuel Viegas & Hugo Aquino
#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include <LiquidCrystal.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
// Radio nRF24 -> pin CE=4 , pin SCK=2
RF24 radio(4,2);
// LiquidCrystal lcd(rs,en,db4,db5,db6,db7);
LiquidCrystal lcd(25,26,32,33,13,14);
// Variables used in the ESC/Servos(joystick) control and button switches
#define Servo1_ADC 34
#define Servo2_ADC 12
#define Servo3_ADC 36
#define ESC_ADC 27
#define SW1_Pin 38
#define SW2_Pin 37
#define ADC_resolution 12
// Addresses for the radio communication
const uint64_t addr[2]={0xE8E8E0E0E1LL,0xE0E0E8E8E2LL};
// Serial monitor writing function, protected with a mutex
void vPrintString( const char *pcString );
void vPrintStringAndNumber( const char *pcString, unsigned ulValue );
// Tasks being created
void rf_com(void *pvParameters);
void adc(void *pvParameters);
void print_data(void *pvParameters);
// Interrupt functions
static void vLCD_InterruptHandler_1(void);
static void vLCD_InterruptHandler_2(void);
// Mutex declaration, in order to create the critical section
static portMUX_TYPE my_mutex = portMUX_INITIALIZER_UNLOCKED;;
// Declare a variable of type QueueHandle_t
// This is used to store the queues that are accessed by the tasks
QueueHandle_t xQueueData;
QueueHandle_t xQueueMotors;
QueueHandle_t xQueueLCD;
// Declare a variable of type SemaphoreHandle_t
// This is used to reference the semaphore that is used to synchronize a task with an interrupt
SemaphoreHandle_t xLCDSemaphore;
// Structure variables to store the data being transmitted
typedef struct
{
	int servo1,servo2,servo3,esc;
}xMotors;
typedef struct
{
	float temp,hum,pres,alt,speed,roll,pitch,acc;
}xData;
typedef struct
{
	float temp=0,hum=0,pres=0,alt=0,speed=0,roll=0,pitch=0,acc=0;
}xClean;
void setup(void)
{
	Serial.begin(921600);
	// nRF24 radio module configuration
	radio.begin();
	radio.setPALevel(RF24_PA_MAX);
	radio.openWritingPipe(addr[1]);
	radio.openReadingPipe(1,addr[0]);
	// ESC/Servos(joystick) ADC Pin Configuration
	pinMode(Servo1_ADC,INPUT);
	pinMode(Servo2_ADC,INPUT);
	pinMode(Servo3_ADC,INPUT);
	pinMode(ESC_ADC,INPUT);
	pinMode(SW1_Pin,INPUT);
	pinMode(SW2_Pin,INPUT);
	// Interrupts Configuration
	attachInterrupt(digitalPinToInterrupt(SW1_Pin),vLCD_InterruptHandler_1,FALLING);
	attachInterrupt(digitalPinToInterrupt(SW2_Pin),vLCD_InterruptHandler_2,FALLING);
	// ADC resolution Configuration
	analogReadResolution(ADC_resolution);
	// LCD configuration and initial message
	lcd.begin(16,4);
	lcd.cursor();
	lcd.setCursor(1,0);
	lcd.print("2.4GHz RC");
	lcd.setCursor(1,1);
	lcd.print("EMBEDDED");
	lcd.setCursor(1,2);
	lcd.print("SYSTEM");
	// The queues are created
    xQueueData=xQueueCreate(1,sizeof(xData));
    xQueueMotors=xQueueCreate(1,sizeof(xMotors));
    xQueueLCD=xQueueCreate(1,sizeof(char));
    // The binary semaphore is created
    vSemaphoreCreateBinary(xLCDSemaphore);
    if(xQueueData!=NULL && xQueueMotors!=NULL && xQueueLCD!=NULL && xLCDSemaphore!=NULL)
    {
    	// The tasks are created, only if the queues and semaphore aren't NULL
    	// Task adc uses the core 0 of the ESP32 microprocessor
    	// Tasks rf_com & print_data use the core 1 of the ESP32 microprocessor
    	xTaskCreatePinnedToCore(rf_com,"rf_com",1024,xQueueData,2,NULL,1);
    	xTaskCreatePinnedToCore(print_data,"Print_Data",1024,NULL,3,NULL,0);
    	xTaskCreatePinnedToCore(adc,"ADC",700,xQueueMotors,2,NULL,0);
    }
    else
    	vPrintString("Error! The queues and the tasks could not be created!\r\n");
    // If all is well we will never reach here as the scheduler will now be
    // running the tasks. If we do reach here then it is likely that there was
    // insufficient heap memory available for a resource to be created.
    for(;;);
}
void rf_com(void *pvParameters)
{
	xMotors xReceivedValues;
	xData xDataValues;
	xClean xDataClean;
	TickType_t xLastWakeTime;
	xLastWakeTime=xTaskGetTickCount();
	const TickType_t xTicksToWait=10/portTICK_PERIOD_MS;
	// The nRF24 radio module is configured as receiver
	radio.startListening();
	for(;;)
	{
		// If the radio receives a valid packet, the radio.available() function will output a '1'
		if(radio.available())
			// Since the received packet is valid, we can "read" it and store the data on the structure
			radio.read(&xDataValues,sizeof(xDataValues));
		// The xQueueSendToBack function sends data to the back of a queue, it returns "pdPASS" if the data was sent successfully
		// xQueueData is the handle of the queue to which the data is being sent (written)
		// &xDataValues is the pointer to the data to be copied into the queue
		// 0 is the block time
		if(xQueueSendToBack(xQueueData,&xDataValues,0)!=pdPASS)
			vPrintString("Error! Could not send to the QueueData!\r\n");
		// xQueueReceive reads an item from a queue, and removes the item from the queue,  it returns "pdPASS" if the data was read successfully
		// xQueueMotors is the handle of the queue from which data is to be read
		// &xReceivedValues is a pointer to the structure into which the data read from the queue will be copied
		// xTicksToWait is the block time, in this case: 10ms
		if(xQueueReceive(xQueueMotors,&xReceivedValues,xTicksToWait)==pdPASS)
		{
			// The nRF24 radio module is configured as transmitter
			radio.stopListening();
			// The data structure is sent to the airplane
			radio.write(&xReceivedValues,sizeof(xReceivedValues));
			// The nRF24 radio module is configured as receiver once again
			radio.startListening();
		}
		// With vTaskDelayUntil() function we achieve a constant execution frequency of 20ms
		vTaskDelayUntil(&xLastWakeTime,(20/portTICK_PERIOD_MS));
	}
}
void adc(void *pvParameters)
{
	xMotors xSendValues;
	portBASE_TYPE xQueue_Motors;
	TickType_t xLastWakeTime;
	xLastWakeTime=xTaskGetTickCount();
	int esc_motor;
	for(;;)
	{
		// Here we do an analog-digital conversion on the joystick pins
		// The ADC values are then convert to duty-cycle values for the ledcwrite function using the map function
		// It is implemented a "digital filter" so that the variations on the joystick ADC converted values are not to abrupt
		// The "filter" is implemented by the multiplications by 0.5
		xSendValues.servo1=xSendValues.servo1*0.5+map(analogRead(Servo1_ADC),300,3200,7500,2300)*0.5;
		xSendValues.servo2=xSendValues.servo2*0.5+map(analogRead(Servo2_ADC),750,3500,7500,2300)*0.5;
		xSendValues.servo3=xSendValues.servo3*0.5+map(analogRead(Servo3_ADC),330,3100,7500,2300)*0.5;
		// The duty-cycle value sent to the ESC is processed the same way but returns slightly different values
		// because we only want to use the joystick ADC values from the central position of the joystick to the top position
		esc_motor=map(analogRead(ESC_ADC),384,3500,3500,384);
		// If the joystick is positioned between the bottom and center position, the value sent is the minimum
		if(esc_motor<=2000)
			esc_motor=1634;
		else
			esc_motor=map(esc_motor,384,3500,1634,7500);
		xSendValues.esc=esc_motor;
		// The xQueueSendToBack function sends data to the back of a queue, it returns "pdPASS" if the data was sent successfully
		// xQueueMotors is the handle of the queue to which the data is being sent (written)
		// &xSendValues is the pointer to the data to be copied into the queue
		// 0 is the block time
		if(xQueueSendToBack(xQueueMotors,&xSendValues,0)!=pdPASS)
			vPrintString("Error! Could not send to the QueueMotors!\r\n");
		// With vTaskDelayUntil() function we achieve a constant execution frequency of 15ms
		vTaskDelayUntil(&xLastWakeTime,(15/portTICK_PERIOD_MS));
	}
}
void print_data(void *pvParameters)
{
	xSemaphoreTake(xLCDSemaphore,0);
	xData xDataValues;
	char val;
	const TickType_t xTicksToWait=10/portTICK_PERIOD_MS;
	static portBASE_TYPE xHigherPriorityTaskWoken;
	for(;;)
	{
		// Use the semaphore to wait for the event. The semaphore was created before the scheduler was started,
		// so before this task ran for the first time. The task blocks indefinitely meaning this function call will
		// only return once the semaphore has been successfully obtained.
		xSemaphoreTake(xLCDSemaphore,portMAX_DELAY);
		// To get here the event (interrupt) must have occurred.
		// xQueueReceive reads an item from a queue, and removes the item from the queue,  it returns "pdPASS" if the data was read successfully
		// xQueueData is the handle of the queue from which data is to be read
		// &xDataValues is a pointer to the structure into which the data read from the queue will be copied
		// xTicksToWait is the block time, in this case: 10ms
		if(xQueueReceive(xQueueData,&xDataValues,xTicksToWait)!=pdPASS)
			vPrintString("Error! Could not receive from the QueueData!\r\n");
		// xQueueReceiveFromISR function in a version of xQueueReceive() that can be called from an ISR
		// xQueueLCD is the handle of the queue from which the data is being received (read)
		// this queue (xQueueLCD) was sent from one of the interrupts
		// &val is a pointer to the variable into which the received data will be copied
		if(xQueueReceiveFromISR(xQueueLCD,&val,(BaseType_t*)&xHigherPriorityTaskWoken)==pdPASS)
		{
			switch(val)
			{
				// In order to avoid data corruption or other similar error, the LCD writing is placed inside a critical section
				portENTER_CRITICAL(&my_mutex);
				case 1:
					lcd.clear();
					lcd.setCursor(0,0);
					lcd.print("Temperature:");
					lcd.setCursor(12,0);
					lcd.print(xDataValues.temp,2);
					lcd.setCursor(0,1);
					lcd.print("Humidity:");
					lcd.setCursor(9,1);
					lcd.print(xDataValues.hum,2);
					lcd.setCursor(0,2);
					lcd.print("Pressure:");
					lcd.setCursor(9,2);
					lcd.print(xDataValues.pres/100,2);
					lcd.setCursor(0,3);
					lcd.print("Altitude:");
					lcd.setCursor(9,3);
					lcd.print(xDataValues.alt,2);
					break;
				case 2:
					lcd.clear();
					lcd.setCursor(0,0);
					lcd.print("Speed:");
					lcd.setCursor(7,0);
					lcd.print(xDataValues.speed,2);
					lcd.setCursor(0,1);
					lcd.print("Acceleration:");
					lcd.setCursor(13,1);
					lcd.print(xDataValues.acc,2);
					lcd.setCursor(0,2);
					lcd.print("Roll:");
					lcd.setCursor(5,2);
					lcd.print(xDataValues.roll,2);
					lcd.setCursor(0,3);
					lcd.print("Pitch:");
					lcd.setCursor(7,3);
					lcd.print(xDataValues.pitch,2);
					break;
				default:
					lcd.clear();
					lcd.setCursor(1,0);
					lcd.print("2.4GHz RC");
					lcd.setCursor(1,1);
					lcd.print("EMBEDDED");
					lcd.setCursor(1,2);
					lcd.print("SYSTEM");
				// Exits the critical section
				portEXIT_CRITICAL(&my_mutex);
			}
		}
		else
			vPrintString("Error! Could not receive from the QueueFromISR!\r\n");
	}
}
// Interrupts
// The first interrupt happens when the button switch of the joystick 1 is pressed
// The queue sent from this interrupt gives the LCD writing function information about what to print
static void vLCD_InterruptHandler_1(void)
{
	static signed portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken=pdFALSE;
	char val=1;
	xQueueSendToBackFromISR(xQueueLCD,&val,0);
	// By giving this semaphore it will unlock the print_data task
	xSemaphoreGiveFromISR(xLCDSemaphore,(signed portBASE_TYPE*)&xHigherPriorityTaskWoken);
	if(xHigherPriorityTaskWoken==pdTRUE)
		portYIELD_FROM_ISR();
}
// The second interrupt happens when the button switch of the joystick 2 is pressed
// The queue sent from this interrupt gives the LCD writing function information about what to print
static void vLCD_InterruptHandler_2(void)
{
	static signed portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken=pdFALSE;
	char val=2;
	xQueueSendToBackFromISR(xQueueLCD,&val,0);
	// By giving this semaphore it will unlock the print_data task
	xSemaphoreGiveFromISR(xLCDSemaphore,(signed portBASE_TYPE*)&xHigherPriorityTaskWoken);
	if(xHigherPriorityTaskWoken==pdTRUE)
		portYIELD_FROM_ISR();
}
void loop()
{
	// Start the scheduler so the tasks start executing
	vTaskStartScheduler();
	// Deletes the void loop() task
	vTaskDelete(NULL);
}
// Printing functions protected with a critical section
void vPrintString (const char *pcString )
{
	// In order to avoid data corruption or other similar error, this writing function is placed inside a critical section
	portENTER_CRITICAL(&my_mutex);
	{
		Serial.print(pcString);
		Serial.flush();
	}
	portEXIT_CRITICAL(&my_mutex);
}
void vPrintStringAndNumber( const char *pcString, unsigned ulValue )
{
	// In order to avoid data corruption or other similar error, this writing function is placed inside a critical section
	portENTER_CRITICAL(&my_mutex);
	{
      Serial.print(pcString);
      Serial.write(' ');
      Serial.println(ulValue);
      Serial.flush();
	}
	portEXIT_CRITICAL(&my_mutex);
}
