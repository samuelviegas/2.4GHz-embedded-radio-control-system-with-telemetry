// 2.4GHz embedded radio control system with telemetry
// Writen by Samuel Viegas & Hugo Aquino
#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include <SensorHub.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
SensorHub SensorHub;
TinyGPSPlus gps;
File data_logger;
HardwareSerial gpsSerial(2);
// Radio nRF24 -> pin CE=4 , pin SCK=2
RF24 radio(4,2);
// Variables used in the ESC/Servos control
#define Servo1_Pin 13
#define Servo2_Pin 12
#define Servo3_Pin 14
#define ESC_Pin 27
#define freq 50
#define resolution 16
#define servo1_Channel 1
#define servo2_Channel 2
#define servo3_Channel 3
#define esc_Channel 4
// Addresses for the radio communication
const uint64_t addr[2]={0xE8E8E0E0E1LL,0xE0E0E8E8E2LL};
// Clock frequency for the I2C bus
#define I2Cfreq 400000
// HardwareSerial used for serial communication using USART2 on the ESP32
// GPS_TX->GPIO16_ESP32(16=esp_rx)     GPS_RX->GPIO17_ESP32(17=esp_tx)
#define RX2 16
#define TX2 17
// ChipSelect pin for the microSD module
#define CS_pin 25
// Serial monitor writing function, protected with a mutex
void vPrintString(const char *pcString);
// Tasks being created
void rf_com(void *pvParameters);
void motors(void *pvParameters);
void sensors(void *pvParameters);
void save_data(void *pvParameters);
// Mutex declaration, in order to create the critical section
static portMUX_TYPE my_mutex=portMUX_INITIALIZER_UNLOCKED;;
// Declare a variable of type QueueHandle_t
// This is used to store the queues that are accessed by the tasks
QueueHandle_t xQueueSensors;
QueueHandle_t xQueueMotors;
// Structure variables to store the data being transmitted
typedef struct
{
	int servo1,servo2,servo3,esc;
}xMotors;
typedef struct
{
	char gps_error;
	int hr,min,sec;
	float temp,hum,pres,alt,speed,lat,lng,acc,roll,pitch;
}xData;
typedef struct
{
	float temp,hum,pres,alt,speed,roll,pitch,acc;
}xData_Send;
void setup(void)
{
	Serial.begin(921600);
	// nRF24 radio module configuration
	radio.begin();
	radio.setPALevel(RF24_PA_MAX);
	radio.openWritingPipe(addr[0]);
	radio.openReadingPipe(1,addr[1]);
	if(radio.isChipConnected())
		vPrintString("Radio successfully connected!\r\n");
	// Initializes the I2C communication with the sensor hub
	SensorHub.begin(I2Cfreq);
	// Initializes the serial communication with the GPS
	gpsSerial.begin(9600,SERIAL_8N1,RX2,TX2);
	// Initializes the microSD card module
	if(!SD.begin(CS_pin))
		vPrintString("An error occur while initializing the microSD card module!\r\n");
	else
		vPrintString("microSD card module successfully initialized!\r\n");
	data_logger=SD.open("data_logger.txt",FILE_WRITE);
	if(data_logger)
		vPrintString("data_logger.txt file successfully created!\r\n");
	else
		vPrintString("An error occur while creating data_logger.txt file!\r\n");
	data_logger.close();
	// ESC/Servos Pin Configuration
	pinMode(Servo1_Pin,OUTPUT);
	pinMode(Servo2_Pin,OUTPUT);
	pinMode(Servo3_Pin,OUTPUT);
	pinMode(ESC_Pin,OUTPUT);
	// ESC/Servos PWM control frequency and resolution configuration
	ledcSetup(servo1_Channel,freq,resolution);
	ledcSetup(servo2_Channel,freq,resolution);
	ledcSetup(servo3_Channel,freq,resolution);
	ledcSetup(esc_Channel,freq,resolution);
	// ESC/Servos channels attributed to their respective pins
	ledcAttachPin(Servo1_Pin,servo1_Channel);
	ledcAttachPin(Servo2_Pin,servo2_Channel);
	ledcAttachPin(Servo3_Pin,servo3_Channel);
	ledcAttachPin(ESC_Pin,esc_Channel);
	// The queues are created
    xQueueSensors=xQueueCreate(1,sizeof(xData));
    xQueueMotors=xQueueCreate(1,sizeof(xMotors));
    if(xQueueSensors!=NULL && xQueueMotors!=NULL)
    {
    	// The tasks are created, only if the queues aren't NULL
    	// Tasks rf_com & motors use the core 0 of the ESP32 microprocessor
    	// Tasks sensors & save_data use the core 1 of the ESP32 microprocessor
    	xTaskCreatePinnedToCore(rf_com,"rf_com",900,xQueueMotors,2,NULL,0);
    	xTaskCreatePinnedToCore(motors,"Motors",800,NULL,2,NULL,0);
    	xTaskCreatePinnedToCore(sensors,"Sensors",1024,xQueueSensors,2,NULL,1);
    	xTaskCreatePinnedToCore(save_data,"Save_Data",1024,NULL,1,NULL,1);
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
	xMotors xSendValues;
	xData xReceivedValues;
	xData_Send xSendData;
	const TickType_t xTicksToWait=(10/portTICK_PERIOD_MS);
	unsigned portBASE_TYPE uxPriority;
	uxPriority=uxTaskPriorityGet(NULL);
	TickType_t xLastWakeTime;
	xLastWakeTime=xTaskGetTickCount();
	// The nRF24 radio module is configured as receiver
	radio.startListening();
	for(;;)
	{
		// If the radio receives a valid packet, the radio.available() function will output a '1'
		if(radio.available())
			// Since the received packet is valid, we can "read" it and store the data on the structure
			radio.read(&xSendValues,sizeof(xSendValues));
		// The xQueueSendToBack function sends data to the back of a queue, it returns "pdPASS" if the data was sent successfully
		// xQueueMotors is the handle of the queue to which the data is being sent (written)
		// &xSendValues is the pointer to the data to be copied into the queue
		// 0 is the block time
		if(xQueueSendToBack(xQueueMotors,&xSendValues,0)!=pdPASS)
			vPrintString("Error! Could not send to the QueueMotors!\r\n");
		// xQueuePeek reads an item from a queue, but without removing the item from the queue,  it returns "pdPASS" if the data was read successfully
		// xQueueSensors is the handle of the queue from which data is to be read
		// &xReceivedValues is a pointer to the structure into which the data read from the queue will be copied
		// xTicksToWait is the block time, in this case: 10ms
		if(xQueuePeek(xQueueSensors,&xReceivedValues,xTicksToWait)==pdPASS)
		{
			// The nRF24 radio module is configured as transmitter
			radio.stopListening();
			// The data received in the queue is stored into the structure
			xSendData.temp=xReceivedValues.temp;
			xSendData.hum=xReceivedValues.hum;
			xSendData.pres=xReceivedValues.pres;
			xSendData.alt=xReceivedValues.alt;
			xSendData.speed=xReceivedValues.speed;
			xSendData.acc=xReceivedValues.acc;
			xSendData.roll=xReceivedValues.roll;
			xSendData.pitch=xReceivedValues.pitch;
			// The data structure is then sent to the controller
			radio.write(&xSendData,sizeof(xSendData));
			// The nRF24 radio module is configured as receiver once again
			radio.startListening();
		}
		if(uxPriority>2)
		{
			if(xSendValues.servo1!=0 && xSendValues.servo2!=0 && xSendValues.servo3!=0 && xSendValues.esc!=0)
				vTaskPrioritySet(NULL,(uxPriority-1));
		}
		// With vTaskDelayUntil() function we achieve a constant execution frequency of 20ms
		vTaskDelayUntil(&xLastWakeTime,(20/portTICK_PERIOD_MS));
	}
}
void motors(void *pvParameters)
{
	xMotors xReceivedValues;
	const TickType_t xTicksToWait=(10/portTICK_PERIOD_MS);
	TickType_t xLastWakeTime;
	xLastWakeTime=xTaskGetTickCount();
	for(;;)
	{
		// xQueueReceive reads an item from a queue, and removes the item from the queue,  it returns "pdPASS" if the data was read successfully
		// xQueueMotors is the handle of the queue from which data is to be read
		// &xReceivedValues is a pointer to the structure into which the data read from the queue will be copied
		// xTicksToWait is the block time, in this case: 10ms
		if(xQueueReceive(xQueueMotors,&xReceivedValues,xTicksToWait)==pdPASS)
		{
			// If the data is successfully received, then the received values are used to control the ESC/Servos
			// The ledcWrite function outputs a PWM signal for the ESC/Servos based on the received duty-cycle value
			ledcWrite(servo1_Channel,xReceivedValues.servo1);
			ledcWrite(servo2_Channel,xReceivedValues.servo2);
			ledcWrite(servo3_Channel,xReceivedValues.servo3);
			ledcWrite(esc_Channel,xReceivedValues.esc);
		}
		else
			vPrintString("Error! Could not receive from the QueueMotors!\r\n");
		// With vTaskDelayUntil() function we achieve a constant execution frequency of 25ms
		vTaskDelayUntil(&xLastWakeTime,(25/portTICK_PERIOD_MS));
	}
}
void sensors(void *pvParameters)
{
	xData xSendValues;
	TickType_t xLastWakeTime;
	xLastWakeTime=xTaskGetTickCount();
	for(;;)
	{
		xSendValues.gps_error=0;
		if(gpsSerial.available()>0)
		{
			// If there is serial communication with the GPS, the received data is stored in a structure
			gps.encode(gpsSerial.read());
			xSendValues.hr=gps.time.hour();
			xSendValues.min=gps.time.minute();
			xSendValues.sec=gps.time.second();
			xSendValues.lat=gps.location.lat();
			xSendValues.lng=gps.location.lng();
			xSendValues.speed=gps.speed.kmph();
		}
		else
		{
			vPrintString("Error! There is no serial communication with the GPS!\r\n");
			// If there is no communication with the GPS, the variable gps.error is set to '1'
			// so if we receive gps_error=1 we know that there is no serial communication with the GPS
			xSendValues.gps_error++;
		}
		if(!(gps.location.isValid()))
		{
			// If the GPS isn't receiving signal from the satellites a 2 is added to the gps_error variable
			// so if we receive gps_error=2 we know that there is no GPS signal
			xSendValues.gps_error+=2;
			vPrintString("Error! No GPS signal!\r\n");
		}
		// The values from the sensors are received from the queue and stored in s structure
		xSendValues.temp=SensorHub.getTemperature();
		xSendValues.hum=SensorHub.getHumidity();
		xSendValues.pres=SensorHub.getPressure();
		xSendValues.alt=SensorHub.getAltitude();
		xSendValues.acc=SensorHub.getAccY();
		xSendValues.roll=SensorHub.getRoll();
		xSendValues.pitch=SensorHub.getPitch();
		// The xQueueSendToBack function sends data to the back of a queue, it returns "pdPASS" if the data was sent successfully
		// xQueueSensors is the handle of the queue to which the data is being sent (written)
		// &xSendValues is the pointer to the data to be copied into the queue
		// 0 is the block time
		if(xQueueSendToBack(xQueueSensors,&xSendValues,0)!=pdPASS)
			vPrintString("Error! Could not send to the QueueSensors!\r\n");
		// With vTaskDelayUntil() function we achieve a constant execution frequency of 750ms
		vTaskDelayUntil(&xLastWakeTime,(750/portTICK_PERIOD_MS));
	}
}
void save_data(void *pvParameters)
{
	xData xReceivedValues;
	const TickType_t xTicksToWait=100/portTICK_PERIOD_MS;
	TickType_t xLastWakeTime;
	xLastWakeTime=xTaskGetTickCount();
	char hrs[9];
	for(;;)
	{
		// xQueueReceive reads an item from a queue, and removes the item from the queue,  it returns "pdPASS" if the data was read successfully
		// xQueueSensors is the handle of the queue from which data is to be read
		// &xReceivedValues is a pointer to the structure into which the data read from the queue will be copied
		// xTicksToWait is the block time, in this case: 100ms
		if(xQueueReceive(xQueueSensors,&xReceivedValues,xTicksToWait)==pdPASS)
		{
			// Opens the data_logger.txt file previously created on the microSD card
			data_logger=SD.open("data_logger.txt",FILE_WRITE);
			if(SD.open("data_logger.txt",FILE_WRITE))
			{
				// Writes the received sensors data in the txt file
			    snprintf(hrs,sizeof(hrs),"%02d:%02d:%02d",xReceivedValues.hr,xReceivedValues.min,xReceivedValues.sec);
			    // In order to avoid data corruption or other similar error, the microSD card writing function is placed inside a critical section
			    portENTER_CRITICAL(&my_mutex);
			    data_logger.println(hrs);
			    data_logger.print("Temperature:\t");
			    data_logger.print(xReceivedValues.temp,2);
			    data_logger.println(" °C");
			    data_logger.print("Humidity:\t");
			    data_logger.print(xReceivedValues.hum,2);
			    data_logger.println(" %");
			    data_logger.print("Pressure:\t");
			    data_logger.print(xReceivedValues.pres/100,2);
			    data_logger.println(" hPa");
			    data_logger.print("Altitude:\t");
			    data_logger.print(xReceivedValues.alt,2);
			    data_logger.println(" m");
			    data_logger.print("Acceleration:\t");
			    data_logger.print(xReceivedValues.acc,2);
			    data_logger.println(" m/s²");
			    data_logger.print("Roll:\t");
			    data_logger.print(xReceivedValues.roll,2);
			    data_logger.println(" °");
			    data_logger.print("Pitch:\t");
			    data_logger.print(xReceivedValues.pitch,2);
			    data_logger.println(" °");
			    // It only stores the latitude, longitude and speed if there is serial communication with the GPS or if there is GPS signal
				switch(xReceivedValues.gps_error)
				{
					case 0:
						data_logger.print("Latitude:\t");
						data_logger.println(xReceivedValues.lat,6);
						data_logger.print("Longitude:\t");
						data_logger.println(xReceivedValues.lng,6);
						data_logger.print("Speed:\t\t");
						data_logger.print(xReceivedValues.speed,2);
						data_logger.println(" Km/h");
						break;
					case 2:
						data_logger.println("No GPS signal!");
						break;
					default:
						data_logger.println("No serial communication with the GPS!");
				}
				data_logger.println();
				// Exits the critical section
				portEXIT_CRITICAL(&my_mutex);
			}
			else
				vPrintString("Error while trying to open data_logger.txt file!\r\n");
			// Closes the data_logger file
			data_logger.close();
		}
		else
			vPrintString("Error! Could not receive from the QueueSensors!\r\n");
		// With vTaskDelayUntil() function we achieve a constant execution frequency of 1s
		vTaskDelayUntil(&xLastWakeTime,(1000/portTICK_PERIOD_MS));
	}
}
void loop()
{
	// Start the scheduler so the tasks start executing
	vTaskStartScheduler();
	// Deletes the void loop() task
	vTaskDelete(NULL);
}
// vPrintString function sends a string over serial
void vPrintString(const char *pcString)
{
	// In order to avoid data corruption or other similar error, this writing function is placed inside a critical section
	portENTER_CRITICAL(&my_mutex);
	{
		Serial.print(pcString);
		Serial.flush();
	}
	portEXIT_CRITICAL(&my_mutex);
}
