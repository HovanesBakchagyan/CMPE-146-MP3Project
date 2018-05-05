/*
 + *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */
/**
 * @file
 * @brief This is the application entry point.
 *          FreeRTOS and stdio printf is pre-configured to use uart0_min.h before main() enters.
 *          @see L0_LowLevel/lpc_sys.h if you wish to override printf/scanf functions.
 *
 */
#include "LPC17xx.h"
#include "tasks.hpp"
#include "uart0_min.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "io.hpp"
#include "printf_lib.h"
#include "utilities.h"
#include "handlers.hpp"
#include "command_handler.hpp"
#include "gpio.hpp"
#include "i2c_base.hpp"
#include "i2c2.hpp"
#include "ff.h"
#include "lpc_sys.h"
#include "event_groups.h"
#include "time.h"
#include "storage.hpp"
#include "stdio.h"
#include <stdint.h>
#include <stdlib.h>
#include"string.h"
#include "periodic_scheduler/periodic_callback.h"
#include "lpc_pwm.hpp"
#include "uart2.hpp"
#include "uart3.hpp"
#include "ssp0.h"
#include "ssp1.h"
#include "eint.h"
#include <vector>
#include "LabGPIOInterrupt.hpp"
using namespace std;
// this is the register address for the decoder
#define SCI_MODE 0x00
#define SCI_STATUS 0x01
#define SCI_BASS 0x02
#define SCI_CLOCKF 0x03
#define SCI_DECODE_TIME 0x04
#define SCI_AUDATA 0x05
#define SCI_WRAM 0x06
#define SCI_WRAMADDR 0x07
#define SCI_HDAT0 0x08
#define SCI_HDAT1 0x09
#define SCI_AIADDR 0x0A
#define SCI_VOL 0x0B
#define SCI_AICTRL0 0x0C
#define SCI_AICTRL1 0x0D
#define SCI_AICTRL2 0x0E
#define SCI_AICTRL3 0x0F
#define writeDateOpcode 0x02
#define Readtime 512
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//this is define the address for the lcd
#define lcd_a 0x61
#define lcdAddress 0x4E


#define BLOCK_SIZE          512
#define MUSIC_QUEUE_SIZE    4
#define MAX_SONGS           100
#define MAX_SONG_LENGTH     13
typedef enum {
	volumnQueue, song_queue //Add by Khiem
} sharedHandleId_t;

SemaphoreHandle_t resumeButtonSemaphore = NULL;
//SemaphoreHandle_t displayScreenSemaphore = NULL;

//Add by Khiem
//SemaphoreHandle_t changeMusicSemaphore = NULL;

QueueHandle_t mysongQueue = xQueueCreate(1, sizeof(int));

uint8_t currentSong = 0;
uint8_t totalSong = 15;
uint8_t currentVolumn = 100;       //0 is max, fffe is muted
uint8_t volumnDisplay = 9;
bool pausedFlag = true;     // the pausedFlage is used to display the lcd status
bool reStartFlag = false;

vector<char*> musicList;       // to declare a 2d vector to store the music name

class controlPanel: public scheduler_task {
public:
	controlPanel(uint8_t priority) :
			scheduler_task("controlPanel", 4096, priority) {
		QueueHandle_t myVolumnQueue = xQueueCreate(3, sizeof(int));
		addSharedObject(volumnQueue, myVolumnQueue);

		//Add by Khiem
        addSharedObject(song_queue,mysongQueue);

	}

	void PauseMusic() {
		pausedFlag = true;
		reStartFlag = false;
		//long pauseSem = 0;
		scheduler_task *musicTask = scheduler_task::getTaskPtrByName(
				"musicPlayer");
		vTaskSuspend(musicTask->getTaskHandle());

//        scheduler_task *LcdTask = scheduler_task::getTaskPtrByName("LcdDisplay");
//        vTaskResume(LcdTask->getTaskHandle());
//         xSemaphoreGiveFromISR(displayScreenSemaphore, &pauseSem);
//        if(pauseSem){
//            portYIELD_FROM_ISR(pauseSem);
//        }
	}

	void ResumePlayMusic() {
		pausedFlag = false;
		reStartFlag = false;
		scheduler_task *musicTask = scheduler_task::getTaskPtrByName("musicPlayer");
		vTaskResume(musicTask->getTaskHandle());     // resume the task

//        scheduler_task *LcdTask = scheduler_task::getTaskPtrByName("LcdDisplay");
//        vTaskResume(LcdTask->getTaskHandle());
//        long startPlay = 0;
//        xSemaphoreGiveFromISR(displayScreenSemaphore, &startPlay);
//        if(startPlay){
//            portYIELD_FROM_ISR(startPlay);
//        }
	}
	void startMusic() {
		//  reStartFlag = false;

		// pausedFlag = false;

		scheduler_task *Musictask = scheduler_task::getTaskPtrByName("musicPlayer");
		vTaskResume(Musictask->getTaskHandle());

//        xQueueSend(getSharedObject(song_queue), &currentSong,0);//add by Khiem
//		long startPlay = 0;
//		xSemaphoreGiveFromISR(resumeButtonSemaphore, &startPlay);
//
//		if (startPlay) {
//			portYIELD_FROM_ISR(startPlay);
//		}
	}
	void selectNextSong() {     //not work yet

//        PauseMusic();
//        pausedFlag = true;
		// reStartFlag = false;


//		scheduler_task *sendmusicTask = scheduler_task::getTaskPtrByName("sendMusicTask");
//		vTaskSuspend(sendmusicTask->getTaskHandle());
//	  xQueueReset(getSharedObject(song_queue));

		currentSong++;
//		vTaskResume(sendmusicTask->getTaskHandle());


//u0_dbg_printf("current song number is %d\t in selectSong\n",currentSong);//Add by Khiem
//		long startPlay = 0;
//scheduler_task *Musictask = scheduler_task::getTaskPtrByName("musicPlayer");
//      vTaskResume(Musictask->getTaskHandle());

//		xSemaphoreGiveFromISR(resumeButtonSemaphore, &startPlay);
//		if (startPlay) {
//			portYIELD_FROM_ISR(startPlay);
//			u0_dbg_printf("Song has been incremented");
//		}

	}

	void selectPrevSong() {       // not work yet

//		reStartFlag = true;
//		pausedFlag = true;
//		PauseMusic();
		if (currentSong<=1)
			printf("Invalid Song");
		else
			currentSong--;

	}
	void volumnUp() {
		printf("Current volumn is: %d", currentVolumn);
		currentVolumn = currentVolumn - 5;      //0 is max volumn
		//volumnDisplay++;
		//  long volumnUpSem = 0;
		xQueueSend(getSharedObject(volumnQueue), &currentVolumn, 100);
//        scheduler_task *task = scheduler_task::getTaskPtrByName("LcdDisplay");
//        vTaskResume(task->getTaskHandle());
//
//        xSemaphoreGiveFromISR(displayScreenSemaphore, &volumnUpSem);
//        if(volumnUpSem){
//            portYIELD_FROM_ISR(volumnUpSem);
//        }
	}
	void volumnDown() {
		printf("Current volumn is: %d", currentVolumn);
		currentVolumn = currentVolumn + 5;      // + volumn is to silence
		// volumnDisplay--;
		// long volumnDownSem = 0 ;
		xQueueSend(getSharedObject(volumnQueue), &currentVolumn, 100);
//        scheduler_task *task = scheduler_task::getTaskPtrByName("LcdDisplay");
//        vTaskResume(task->getTaskHandle());
//        xSemaphoreGiveFromISR(displayScreenSemaphore, &volumnDownSem);
//        if(volumnDownSem){
//            portYIELD_FROM_ISR(volumnDownSem);
//        }
	}

	void upDataSongNameList() {
		char* a = "music1.mp3";
		char* b = "music2.mp3";
		char* c = "music3.mp3";
		char* d = "music4.mp3";
		char* e = "music5.mp3";
		char* f = "music6.mp3";
		char *g = "music7.mp3";
		char* h = "music8.mp3";
		char* i = "music9.mp3";
		char* j = "music10.mp3";
		char* k = "music11.mp3";
		char* l = "music12.mp3";
		char* s = "music13.mp3";
		char* t = "music14.mp3";
		char* u = "music15.mp3";
		char* v = "music16.mp3";
		musicList.push_back(a);
		musicList.push_back(b);
		musicList.push_back(c);
		musicList.push_back(d);
		musicList.push_back(e);
		musicList.push_back(f);
		musicList.push_back(g);
		musicList.push_back(h);
		musicList.push_back(i);
		musicList.push_back(j);
		musicList.push_back(k);
		musicList.push_back(l);
		musicList.push_back(s);
		musicList.push_back(t);
		musicList.push_back(u);
		musicList.push_back(v);

	}
	bool init(void) {
		//configure the pin p0.0. p0.1 as gpio
		LPC_PINCON->PINSEL0 &= ~(3 << 0);   // set the pin p0.0 as gpio
		LPC_PINCON->PINSEL0 &= ~(3 << 2);   // set the pin p0.1 as gpio

		LPC_GPIO0->FIODIR &= ~(1 << 0); // set the port p0.0 as input
		LPC_GPIO0->FIODIR &= ~(1 << 1); // set the port p0.1 as input

		LPC_PINCON->PINSEL4 &= ~(3 << 0);   // set the pin p2.0 as gpio
		LPC_PINCON->PINSEL4 &= ~(3 << 2);   // set the pin p2.1 as gpio

		LPC_GPIO2->FIODIR &= ~(1 << 0); // set the port p2.0 as input
		LPC_GPIO2->FIODIR &= ~(1 << 1); // set the port p2.1 as input

		//use to clear all the interrupts
		LPC_GPIOINT->IO0IntClr = 0xFFFFFFFF;
//		NVIC_EnableIRQ(EINT3_IRQn);
		// isr_register(EINT3_IRQn, EINT3_IRQHandler);
		resumeButtonSemaphore = xSemaphoreCreateBinary(); // to create the semaphore
//        displayScreenSemaphore = xSemaphoreCreateBinary();

		upDataSongNameList();
		return true;
	}
//    bool nextSongSensor(){      // accessration sensor use to play next song and previous song
//        int tilt_x = AS.getX();
//        if(tilt_x < -750){
//            return true;
//        }
//        else{
//            return false;
//        }
//    }
//    bool prevSongSensor(){      //accessration sensor use to play prev song and previous song
//        int tilt_x = AS.getX();
//        if(tilt_x > 750){
//            return true;
//        }
//        else{
//            return false;
//        }
//    }

	// p0.1 to left
	// p0.0 to right
	// p2.0 to volumn down
	//p2.1 to volumn up

	bool run(void *p) {

		if (SW.getSwitch(4)) {        // Button 1 for play and resume the music
//            volumnDown();
//			selectNextSong();
//        	long startPlay=0;
//        	xSemaphoreGiveFromISR(resumeButtonSemaphore, &startPlay);
//			currentSong++;

        	//xSemaphoreGive(resumeButtonSemaphore);
        	selectNextSong();

//        	        if(startPlay)
//        	        {
//        	        portYIELD_FROM_ISR(startPlay);
//        	        }

		}
		else if (SW.getSwitch(2)) {  //Button 2 for pause the music
			PauseMusic();
		}
		else if(SW.getSwitch(3))
		{
			volumnUp();
		}
		else if(SW.getSwitch(1)) {
			startMusic();
//            long startPlay = 0;
            xSemaphoreGive(resumeButtonSemaphore);

//            xSemaphoreGiveFromISR(resumeButtonSemaphore, &startPlay);
//            if (startPlay)
//                    	        {
//                    	        portYIELD_FROM_ISR(startPlay);
//                    	        }

		}
		else if((LPC_GPIO0->FIOPIN & (1 << 0))) { // p0.0 -->select the next song
			selectNextSong();

		}
		else if(!(LPC_GPIO0->FIOPIN & (1 << 1))) { // p0.1--> select to previous song
			selectPrevSong();
		}
		else if(!(LPC_GPIO2->FIOPIN & (1 << 0))) {  // p2.0, to set volumn up
			volumnUp();

		}
//		else if(!(LPC_GPIO2->FIOPIN & (1 << 1))) {   // p2.1, to set volumn down
//			volumnDown();
			//startMusic();

//		}
		////Add by Khiem
//		 if (xSemaphoreTake(resumeButtonSemaphore, portMAX_DELAY))

//			else if (LPC_GPIO0->FIOPIN & 1<<0)
//			{
//				xSemaphoreTake(resumeButtonSemaphore, portMAX_DELAY);
//				{	u0_dbg_printf("Interrupt from port 0.0");
//				xQueueReset(getSharedObject(song_queue));
//				currentSong++;
//				 xQueueSend(getSharedObject(song_queue), &currentSong,portMAX_DELAY);
////					startMusic();
//				}
//			}
				else if((LPC_GPIO2->FIOPIN & (1 << 1)))
				{
					u0_dbg_printf("Interrupt from port 2.1");
//					xSemaphoreGive(resumeButtonSemaphore);

					selectPrevSong();

				}
		vTaskDelay(100);
		return true;
	}
};

//class mp3Project: public scheduler_task // this task in to produce the sensor value and then send it to queue
//{
//public:
//	mp3Project(uint8_t priority) :
//			scheduler_task("musicPlayer", 5000, priority) {
//
//	}
//	bool init(void) {
//		SPI0_Init();
//		writeRegister(SCI_MODE, 0x0800);
//		writeRegister(SCI_BASS, 0x7A00);
//		writeRegister(SCI_CLOCKF, 0x2000);
//		writeRegister(SCI_AUDATA, 0xAC45);
//		writeRegister(SCI_VOL, 0x1010);
//		//  writeRegister(SCI_AUDATA, 0xAC45);
//		//   setClock(0x2000);
//		setVolum(100);
//		ssp0_set_max_clock(1);
//		return true;
//	}
//	void SPI0_Init()        //done in lab lecture
//	{
//		LPC_SC->PCONP |= (1 << 21);     // SPI0 Power Enable
//		LPC_SC->PCLKSEL1 &= ~(3 << 10); // Clear clock Bits
//		LPC_SC->PCLKSEL1 |= (1 << 10); // 01: CLK / 1. ||  00: /4    || 10: /2
//		// Select MISO, MOSI, and SCK pin-select functionality
//		LPC_PINCON->PINSEL0 &= ~((3 << 30));                   // clear the sck0
//		LPC_PINCON->PINSEL1 &= ~((3 << 2) | (3 << 4)); //  clear the miso0 and mosi0
//		LPC_PINCON->PINSEL0 |= (2 << 30);       // set set the function for sck0
//		LPC_PINCON->PINSEL1 |= ((2 << 2) | (2 << 4)); //and then set the function respectively
//		LPC_SSP0->CR0 = 7;          // 8-bit mode
//		LPC_SSP0->CR1 = (1 << 1);   // Enable SSP as Master
//		LPC_SSP0->CPSR = 1;         // SCK speed = CPU / 1
//		LPC_SSP0->CR1 &= ~(1 << 2); // to set the spi1 as master
//		// gpio initlization, configure the pin as gpio first
//		LPC_PINCON->PINSEL3 &= ~(3 << 12);      // set p1.22 as gpio
//		LPC_PINCON->PINSEL3 &= ~(3 << 24);      // set p1.28 as gpio
//		LPC_PINCON->PINSEL3 &= ~(3 << 28);      //set p1.30  as gpio
//		LPC_PINCON->PINSEL3 &= ~(3 << 6);      //set p1.19  as gpio
//		//   LPC_PINCON->PINSEL0 &= ~(3 << 0);           // this is for sd car, dont real need it
//		LPC_GPIO1->FIODIR |= (1 << 22);   //set pin P1.22 as output -- CS signal
//		LPC_GPIO1->FIODIR |= (1 << 28); //set pin P1.28 as output -- RESET signal
//		LPC_GPIO1->FIODIR |= (1 << 30);  //set pin P1.30 as output -- DCS signal
//		LPC_GPIO1->FIODIR &= ~(1 << 19); //set pin P1.19 as input -- DREQ signal
//		LPC_GPIO1->FIOSET = (1 << 28);    //set pin P1.28 high initially
//		LPC_GPIO1->FIOSET = (1 << 30);    //set pin P1.30 high initially
//		LPC_GPIO1->FIOSET = (1 << 22);    //set pin P1.22 high initially
//		// ssp0_set_max_clock(1);
//	}
//	uint8_t sendDataToRegister(uint8_t data) {
//		LPC_SSP0->DR = data; //Send the data Out
//		while (LPC_SSP0->SR & (1 << 4))
//			; // Wait until SSP is busy
//		return LPC_SSP0->DR;
//	}
//	void setClock(uint16_t clockData) {
//		writeRegister(SCI_CLOCKF, clockData);
//	}
//	bool dreqStatus() {
//		if (LPC_GPIO1->FIOPIN & (1 << 19)) {
//			return true;
//		} else {
//			return false;
//		}
//	}
//	uint16_t readRegister(uint8_t readReg) {
//		while (!(LPC_GPIO1->FIOPIN & (1 << 19)))
//			; //0 = not ready, wait till ready
//		setCS();      //Send Low on CS to enable VS1053, active low
//		sendDataToRegister(0x3);         //Send Read Opcode
//		sendDataToRegister(readReg);      //Send Address to read from
//		char firstByte = sendDataToRegister(0x99);
//		while (!(LPC_GPIO1->FIOPIN & (1 << 19)))
//			;
//		char secondByte = sendDataToRegister(0x98);
//		while (!(LPC_GPIO1->FIOPIN & (1 << 19)))
//			;
//		resetCS();     //Send High on CS to disable VS1053
//		uint16_t data = firstByte << 8;
//		data = data | secondByte;
//		return data;
//	}
//	// this function tells the music data can be transfer to the decoder
//	void setDataCS() {
//		ssp0_set_max_clock(1);
//		LPC_GPIO1->FIOCLR |= (1 << 30);
//	}
//	//clean the data cs
//	void resetDataCS() {
//		LPC_GPIO1->FIOSET |= (1 << 30);
//	}
//	// this function tells that the chip select is select the decoder and sending the data
//	void setCS() {
//		// ssp0_set_max_clock(12);
//		LPC_GPIO1->FIOCLR |= (1 << 22);
//	}
//	// this tells that the chip select is not been selected
//	void resetCS() {
//		LPC_GPIO1->FIOSET |= (1 << 22);
//	}
//	// when writing to the register, need to enable the CS for decoder
//	void writeRegister(uint8_t address, uint16_t data) {
//		uint8_t lowByteData = data & 0x00FF;
//		uint8_t highByteData = data >> 8;
//		// lowByteData = (uint8_t)data;        // to get the low byte of data
//		// highByteData = (uint8_t)(data>>8);  //to get the high byte of data
//		resetDataCS();          // set the pin to high
//		setCS();             //enable the chip select, the cs is active low
//		while (!(LPC_GPIO1->FIOPIN & (1 << 19)))
//			;
//		sendDataToRegister(0x02); //send the opcode fist, this is operation of write data
//		sendDataToRegister(address);             // send the address
//		sendDataToRegister(highByteData);
//		sendDataToRegister(lowByteData);
//		//setDataCS();
//		while (!(LPC_GPIO1->FIOPIN & (1 << 19)))
//			; // wait unitl the dreq is low
//		resetCS();
//	}
//	void setVolum(uint8_t volumnData) {
//		uint16_t volumn16Bit = (volumnData << 8) | volumnData;
//		writeRegister(SCI_VOL, volumn16Bit);
//	}
//	void mp3_SDI_Write_32(char *data) {
//		resetCS();       //disable CS
//		setDataCS();     //enable DCS
//		while (dreqStatus() == 0)
//			;  //wait until DREQ pin goes high
//		//    ssp0_exchange_byte(0x00);
//		for (int i = 0; i < 32; i++) {
//			ssp0_exchange_byte(data[i]);
//		}
//		resetDataCS();    //disable DCS
//	}
//	bool updataSong() { // this is function use to updata song when user select next and prev song from the list
////    	 uint8_t songNumberChange;//add by Khiem
////        QueueHandle_t current_song_control=getSharedObject(song_queue);
////            if(xQueueReceive(current_song_control, &songNumberChange,15))
////            {
////            	currentSong=songNumberChange;
////
////            }
//		return true;
//	}
//	bool run(void *p) {
//
//		//add by Khiem
//
//		//uint8_t songNumberChange;
////		if (xSemaphoreTake(resumeButtonSemaphore, portMAX_DELAY))
//		 while (xQueueReceive(getSharedObject(song_queue), &currentSong,portMAX_DELAY))
//
//	{
//			// currentSong=songNumberChange;
//
//
//		{
//			long int lSize;
//			char mp3_buffer[512];
//			FILE * mpFile;
//			int i, y = 0;
//			uint16_t volumnChange;
//			QueueHandle_t volumControl = getSharedObject(volumnQueue);
//
//			char prefix[] = "1:";
//			char* songName = musicList[currentSong];
//			char* fileName = (char*) malloc(strlen(prefix) + strlen(songName) + 1);
//			strcpy(fileName, prefix);
//			strcat(fileName, songName);
//
//			mpFile = fopen(fileName, "r");
////			if (!mpFile)        // to make sure the file is open success
////			{
////				fputs("File error", stderr);
////				exit(1);
////			}
//
//			fseek(mpFile, 0, SEEK_END);    // obtain file size:
//			lSize = ftell(mpFile);     // get the pointer of the file
//			rewind(mpFile);             // go the begining of the file
//////
////			if (mp3_buffer == NULL) {
////				printf("mp3 data memory allocate fail");
////				exit(1);
////			}
//
//			 u0_dbg_printf("mo dc file trong play %s\n",&mpFile);
//			while (mpFile) {
////                if( reStartFlag == true){
////                    u0_dbg_printf(" the data send to mp3 is true \n");
////                    vTaskDelay(1000);
////                    reStartFlag = false;
////                    break;
////                }
//
//				for (i = 0; i < lSize; i++) {
//					fseek(mpFile, 512 * i, SEEK_SET); //re direction the reading file
//					int readResult = fread(mp3_buffer, 1, 512, mpFile); // since cpu only can store at most 512k
//					// vTaskDelay(100);
//					while (y < readResult) {
//						// u0_dbg_printf(" the data send to mp3 is %x \n", mp3_buffer[x]);
//						while (dreqStatus() == 0) {
//							if (xQueueReceive(volumControl, &volumnChange,15)) {
//								setVolum(volumnChange);
//
//							}
//
//						}
//						if (dreqStatus()) {
//							resetCS();
//							setDataCS();
//							for (int j = 0; j < 32; j++) {
//								ssp0_exchange_byte(mp3_buffer[y]);
//								//  u0_dbg_printf(" the data send to mp3 is %x \n", mp3_buffer[y]);
//								// vTaskDelay(100);
//								y++;
//							}
//							// u0_dbg_printf(" the data send to mp3 is %x \n", *mp3_buffer);
//							//  while(!(LPC_GPIO1->FIOPIN & (1 << 19)));
//							resetDataCS();
//
//						}
//						vTaskDelay(0);
//						if (LPC_GPIO0->FIOPIN & 1<<0)
//									{
//										xSemaphoreTake(resumeButtonSemaphore, portMAX_DELAY);
//										{	u0_dbg_printf("Interrupt from port 0.0");
//										xQueueReset(getSharedObject(song_queue));
//										currentSong++;
//										 xQueueSend(getSharedObject(song_queue), &currentSong,portMAX_DELAY);
//						//					startMusic();
//										}
//									}
//					}
//					y = 0;
//
//				}
//				// u0_dbg_printf(" the buffer size is %i ", sizeof(buffer));
//
//			}
//
//			// terminate
//			fclose(mpFile);
//			free(mp3_buffer);
//
//		}
//
//   } //add by Khiem
//
//		return true;
//	}
//};



//Add interrupr by Khiem

LabGPIOInterrupt GPIOInt_instance;
  void eint3_IRQHandle(void)//Handle Interrupt
  {
	  GPIOInt_instance.handle_interrupt();
  }

	void callback_from_interrupt(void)//Call back function from interrupt
	{

		long yield = 0;
		{
		  xSemaphoreGiveFromISR(resumeButtonSemaphore, &yield);
		   if(yield)
		   {
			   u0_dbg_printf("YIELD from ISR\n");

		   portYIELD_FROM_ISR(yield);
		   }
		}

	}


//Add by Khiem another sendMusic task

//class sendMusic : public scheduler_task

//{
//    public:
//        sendMusic(uint8_t priority): scheduler_task("sendMusicTask", 2000, priority){}
//        bool init(void){
//            return true;
//        }
//
//        bool run(void *p)
//        {
//        		if	( xSemaphoreTake(resumeButtonSemaphore, portMAX_DELAY))
//        		{
////        		long int lSize;
////        				char mp3_buffer[512];
////        				FILE * mpFile;
////        				int i, y = 0;
////        				char prefix[] = "1:";
////        				char* songName = musicList[currentSong];
////        				char* fileName = (char*) malloc(strlen(prefix) + strlen(songName) + 1);
////        				strcpy(fileName, prefix);
////        				strcat(fileName, songName);
////
////        				mpFile = fopen(fileName, "r");
////        				if (!mpFile)        // to make sure the file is open success
////        				{
////        					fputs("File error", stderr);
////        					exit(1);
////        				}
//////        				fseek(mpFile, 0, SEEK_END);    // obtain file size:
//////        				lSize = ftell(mpFile);     // get the pointer of the file
//////        				rewind(mpFile);             // go the begining of the file
////
////        				if (mp3_buffer == NULL) {
////        					printf("mp3 data memory allocate fail");
////        					exit(1);
////        				}
////        				u0_dbg_printf("mo dc file trong sendMusic%s\n",mpFile);
////        				u0_dbg_printf("current song=%s\n",musicList[currentSong]);
////        			 xQueueSend(getSharedObject(song_queue), &mpFile,portMAX_DELAY);//add by Khiem
////        			fclose(mpFile);
////        		    free(mp3_buffer);
////        		}
////
//        		}
//        }
//
//};


	///Add on Stanley code
	//MP3 decoder interface
	typedef struct {
	    FILE* file;
	    uint8_t data[BLOCK_SIZE];
	    uint32_t fileSize;
	    uint32_t position;
	} musicBlock_t;
	class sendMusic : public scheduler_task
	{
	    public:
	        sendMusic(uint8_t priority): scheduler_task("sendMusicTask", 2000, priority){}
	        bool init(void){
	            return true;
	        }

	        bool run(void *p){

	            //wait for semaphore

	        	xSemaphoreTake(resumeButtonSemaphore, portMAX_DELAY);


	            char prefix[] = "1:";
	            char* songName = musicList[currentSong];
	            u0_dbg_printf("currently playing: %s\n", songName);
	            char* fileName = (char*)malloc(strlen(prefix)+strlen(songName)+1);
	            strcpy(fileName, prefix);
	            strcat(fileName, songName);

	            FILE* f = fopen( fileName, "r");
	            if(f == NULL)
	                return true;

	            //get filesize
	            fseek(f, 0, SEEK_END);
	            int fileSize = ftell(f);
	            fseek(f, 0, SEEK_SET);

	            //read in chunks of 512 bytes and send to queue
	            musicBlock_t m;
	            musicBlock_t* mPtr;
	            for(int i = 0; i < fileSize/BLOCK_SIZE+1; i++)
	            {if (LPC_GPIO0->FIOPIN & (1 << 0) ||(LPC_GPIO2->FIOPIN & (1 << 1)))
	            	break;

	                m.file = f;
	                m.fileSize = fileSize;
	                m.position = 0;
	                fread(m.data, 1, BLOCK_SIZE, m.file);
	                mPtr = &m;
	                xQueueSend(getSharedObject(song_queue), &mPtr, portMAX_DELAY);
	            }
	            fclose(f);



	            return true;
	        }

	};

	class mp3Project : public scheduler_task
	{
	    public:
		mp3Project(uint8_t priority): scheduler_task("musicPlayer", 2000, priority){}
	    	bool init(void) {
	    		SPI0_Init();
	    		writeRegister(SCI_MODE, 0x0800);
	    		writeRegister(SCI_BASS, 0x7A00);
	    		writeRegister(SCI_CLOCKF, 0x2000);
	    		writeRegister(SCI_AUDATA, 0xAC45);
	    		writeRegister(SCI_VOL, 0x1010);
	    		//  writeRegister(SCI_AUDATA, 0xAC45);
	    		//   setClock(0x2000);
	    		setVolum(100);
	    		ssp0_set_max_clock(1);
	    		return true;
	    	}
	    	void SPI0_Init()        //done in lab lecture
	    	{
	    		LPC_SC->PCONP |= (1 << 21);     // SPI0 Power Enable
	    		LPC_SC->PCLKSEL1 &= ~(3 << 10); // Clear clock Bits
	    		LPC_SC->PCLKSEL1 |= (1 << 10); // 01: CLK / 1. ||  00: /4    || 10: /2
	    		// Select MISO, MOSI, and SCK pin-select functionality
	    		LPC_PINCON->PINSEL0 &= ~((3 << 30));                   // clear the sck0
	    		LPC_PINCON->PINSEL1 &= ~((3 << 2) | (3 << 4)); //  clear the miso0 and mosi0
	    		LPC_PINCON->PINSEL0 |= (2 << 30);       // set set the function for sck0
	    		LPC_PINCON->PINSEL1 |= ((2 << 2) | (2 << 4)); //and then set the function respectively
	    		LPC_SSP0->CR0 = 7;          // 8-bit mode
	    		LPC_SSP0->CR1 = (1 << 1);   // Enable SSP as Master
	    		LPC_SSP0->CPSR = 1;         // SCK speed = CPU / 1
	    		LPC_SSP0->CR1 &= ~(1 << 2); // to set the spi1 as master
	    		// gpio initlization, configure the pin as gpio first
	    		LPC_PINCON->PINSEL3 &= ~(3 << 12);      // set p1.22 as gpio
	    		LPC_PINCON->PINSEL3 &= ~(3 << 24);      // set p1.28 as gpio
	    		LPC_PINCON->PINSEL3 &= ~(3 << 28);      //set p1.30  as gpio
	    		LPC_PINCON->PINSEL3 &= ~(3 << 6);      //set p1.19  as gpio
	    		//   LPC_PINCON->PINSEL0 &= ~(3 << 0);           // this is for sd car, dont real need it
	    		LPC_GPIO1->FIODIR |= (1 << 22);   //set pin P1.22 as output -- CS signal
	    		LPC_GPIO1->FIODIR |= (1 << 28); //set pin P1.28 as output -- RESET signal
	    		LPC_GPIO1->FIODIR |= (1 << 30);  //set pin P1.30 as output -- DCS signal
	    		LPC_GPIO1->FIODIR &= ~(1 << 19); //set pin P1.19 as input -- DREQ signal
	    		LPC_GPIO1->FIOSET = (1 << 28);    //set pin P1.28 high initially
	    		LPC_GPIO1->FIOSET = (1 << 30);    //set pin P1.30 high initially
	    		LPC_GPIO1->FIOSET = (1 << 22);    //set pin P1.22 high initially
	    		// ssp0_set_max_clock(1);
	    	}
	    	uint8_t sendDataToRegister(uint8_t data) {
	    		LPC_SSP0->DR = data; //Send the data Out
	    		while (LPC_SSP0->SR & (1 << 4))
	    			; // Wait until SSP is busy
	    		return LPC_SSP0->DR;
	    	}
	    	void setClock(uint16_t clockData) {
	    		writeRegister(SCI_CLOCKF, clockData);
	    	}
	    	bool dreqStatus() {
	    		if (LPC_GPIO1->FIOPIN & (1 << 19)) {
	    			return true;
	    		} else {
	    			return false;
	    		}
	    	}
	    	uint16_t readRegister(uint8_t readReg) {
	    		while (!(LPC_GPIO1->FIOPIN & (1 << 19)))
	    			; //0 = not ready, wait till ready
	    		setCS();      //Send Low on CS to enable VS1053, active low
	    		sendDataToRegister(0x3);         //Send Read Opcode
	    		sendDataToRegister(readReg);      //Send Address to read from
	    		char firstByte = sendDataToRegister(0x99);
	    		while (!(LPC_GPIO1->FIOPIN & (1 << 19)))
	    			;
	    		char secondByte = sendDataToRegister(0x98);
	    		while (!(LPC_GPIO1->FIOPIN & (1 << 19)))	    			;
	    		resetCS();     //Send High on CS to disable VS1053
	    		uint16_t data = firstByte << 8;
	    		data = data | secondByte;
	    		return data;
	    	}
	    	// this function tells the music data can be transfer to the decoder
	    	void setDataCS() {
	    		ssp0_set_max_clock(1);
	    		LPC_GPIO1->FIOCLR |= (1 << 30);
	    	}
	    	//clean the data cs
	    	void resetDataCS() {
	    		LPC_GPIO1->FIOSET |= (1 << 30);
	    	}
	    	// this function tells that the chip select is select the decoder and sending the data
	    	void setCS() {
	    		// ssp0_set_max_clock(12);
	    		LPC_GPIO1->FIOCLR |= (1 << 22);
	    	}
	    	// this tells that the chip select is not been selected
	    	void resetCS() {
	    		LPC_GPIO1->FIOSET |= (1 << 22);
	    	}
	    	// when writing to the register, need to enable the CS for decoder
	    	void writeRegister(uint8_t address, uint16_t data) {
	    		uint8_t lowByteData = data & 0x00FF;
	    		uint8_t highByteData = data >> 8;
	    		// lowByteData = (uint8_t)data;        // to get the low byte of data
	    		// highByteData = (uint8_t)(data>>8);  //to get the high byte of data
	    		resetDataCS();          // set the pin to high
	    		setCS();             //enable the chip select, the cs is active low
	    		while (!(LPC_GPIO1->FIOPIN & (1 << 19)))
	    			;
	    		sendDataToRegister(0x02); //send the opcode fist, this is operation of write data
	    		sendDataToRegister(address);             // send the address
	    		sendDataToRegister(highByteData);
	    		sendDataToRegister(lowByteData);
	    		//setDataCS();
	    		while (!(LPC_GPIO1->FIOPIN & (1 << 19)))
	    			; // wait unitl the dreq is low
	    		resetCS();
	    	}
	    	void setVolum(uint8_t volumnData) {
	    		uint16_t volumn16Bit = (volumnData << 8) | volumnData;
	    		writeRegister(SCI_VOL, volumn16Bit);
	    	}
	    	void mp3_SDI_Write_32(char *data) {
	    		resetCS();       //disable CS
	    		setDataCS();     //enable DCS
	    		while (dreqStatus() == 0)
	    			;  //wait until DREQ pin goes high
	    		//    ssp0_exchange_byte(0x00);
	    		for (int i = 0; i < 32; i++) {
	    			ssp0_exchange_byte(data[i]);
	    		}
	    		resetDataCS();    //disable DCS
	    	}
	    	bool updataSong() {}

	        bool run(void *p){

	                /* We first get the queue handle the other task added using addSharedObject() */
	                musicBlock_t* data;
	                QueueHandle_t qid = getSharedObject(song_queue);

	                uint16_t volumnChange;
	               	QueueHandle_t volumControl = getSharedObject(volumnQueue);
	                //wait for queue to be not empty

	                while(xQueueReceive(qid, &data, portMAX_DELAY)){

	                    //get pointer to data chunk
	                    uint8_t* dataPtr = data->data;
	                    int z = 0;
	                    while(z < BLOCK_SIZE)
	                    {
	                    	while (dreqStatus() == 0)
	                    	{
	                    		                if (xQueueReceive(volumControl, &volumnChange,15))
	                    		                {
	                    		                	setVolum(volumnChange);
	                    	      		            							}
	                    		                		            	}
	                        //check dreq every 32 bytes
	                    	resetCS();
	                 		setDataCS();
	                        for(int i = 0; i < 32; i++)
	                        {
	                            //send byte to decoder
	                        	sendDataToRegister(*dataPtr++);
	                            z++;
	                        }
	                        resetDataCS();
//
	                    }

	                }
	                return true;
	        }

	};

	////////////////end of Stanley portion code
int main(void)
{
//    scheduler_add_task(new terminalTask(PRIORITY_HIGH));
	/* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
	//  scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));
	scheduler_add_task(new mp3Project(2));
//    scheduler_add_task(new diplayScreen(2));
	scheduler_add_task(new controlPanel(4));
	scheduler_add_task(new sendMusic(3));

	GPIOInt_instance.init();
		GPIOInt_instance.attachInterruptHandler(0,0,callback_from_interrupt,raising_edge);//port 0.0
		isr_register(EINT3_IRQn,eint3_IRQHandle);
		GPIOInt_instance.attachInterruptHandler(2,1,callback_from_interrupt,both_edge);//port2.1
		isr_register(EINT3_IRQn,eint3_IRQHandle);

	scheduler_start(); ///< This shouldn't return
	return -1;
}

