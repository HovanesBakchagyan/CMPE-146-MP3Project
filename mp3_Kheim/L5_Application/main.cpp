//#include "tasks.hpp"
//#include "utilities.h"
//#include "io.hpp"
//#include "printf_lib.h"
//#include <iostream>
//#include "uart0_min.h"
//
//#include <string.h>
//#include "stdio.h"
//#include "lpc_sys.h"
//#include "soft_timer.hpp"
//#include "queue.h"
//#include "task.h"
//#include "ff.h"
//#include<stdlib.h>
//using namespace std;
//
////-lcd-display--------------------------------------------------------------------------
//#define LCD_RS  0 //P0.0 Reg Sel
//#define LCD_EN  1 //P0.1 Enable
//#define LCD_D7  9 //P2.6
//#define LCD_D6  8 //P2.7
//#define LCD_D5  7 //P2.8
//#define LCD_D4  6 //P2.9
//
//
//// commands
//#define LCD_CLEARDISPLAY    0x01
//#define LCD_RETURNHOME      0x02
//#define LCD_ENTRYMODESET    0x04
//#define LCD_DISPLAYCONTROL  0x08
//#define LCD_CURSORSHIFT     0x10
//#define LCD_FUNCTIONSET     0x20
//#define LCD_SETCGRAMADDR    0x40
//#define LCD_SETDDRAMADDR    0x80
//
//// flags for display entry mode
//#define LCD_ENTRYRIGHT          0x00
//#define LCD_ENTRYLEFT           0x02
//#define LCD_ENTRYSHIFTINCREMENT 0x01
//#define LCD_ENTRYSHIFTDECREMENT 0x00
//
//// flags for display on/off control
//#define LCD_DISPLAYON   0x04
//#define LCD_DISPLAYOFF  0x00
//#define LCD_CURSORON    0x02
//#define LCD_CURSOROFF   0x00
//#define LCD_BLINKON     0x01
//#define LCD_BLINKOFF    0x00
//
//// flags for display/cursor shift
//#define LCD_DISPLAYMOVE 0x08
//#define LCD_CURSORMOVE  0x00
//#define LCD_MOVERIGHT   0x04
//#define LCD_MOVELEFT    0x00
//
//// flags for function set
//#define LCD_8BITMODE    0x10
//#define LCD_4BITMODE    0x00
//#define LCD_2LINE       0x08
//#define LCD_1LINE       0x00
//#define LCD_5x10DOTS    0x04
//#define LCD_5x8DOTS     0x00
//
//// flags for backlight control
//#define LCD_BACKLIGHT   0x08
//#define LCD_NOBACKLIGHT 0x00
//
//
////-vs0153-------------------------------------------------------------------------------
////chip select vs1053, active low
//#define ON      1
//#define OFF     0
//
////FIODIR input/output
//#define INPUT   0
//#define OUTPUT  1
//
////vs1053 OP codes
//#define WRITE   2
//#define READ    3
//
////vs1053 register addresses
//#define SCI_MODE        0x0 // mode control
//#define SCI_STATUS      0x1 // Status of VS1053b
//#define SCI_BASS        0x2 // Built-in bass/tremble control
//#define SCI_CLOCKF      0x3 // clock freq + multiplier
//#define SCI_DECODE_TIME 0x4 // Decode time in seconds
//#define SCI_AUDATA      0x5 // Misc. audio data
//#define SCI_WRAM        0x6 // RAM write/read
//#define SCI_WRAMADDR    0x7 // Base address for RAM write/read
//#define SCI_HDAT0       0x8 // Stream header data 0
//#define SCI_HDAT1       0x9 // Stream header data 1
//#define SCI_AIADDR      0xA // Start address of application
//#define SCI_VOL         0xB // Volume control
//#define SCI_AICTRL0     0xC // Application control regeister 0
//#define SCI_AICTRL1     0xD // Application control register 1
//#define SCI_AICTRL2     0xE // Application control rgister 2
//#define SCI_AICTRL3     0xF // Application control register 3
//
////pins, must use SSP0, SSP1 is used by sd card reader
//#define CS      23 //P1.23 CS(SSEL), PINSEL1, use random pin to chip select vs1053, P0.16=CS for noric wireless
//#define MISO    17 //P0.17
//#define MOSI    18 //P0.18
//#define SCK     15 //P0.15
//#define XCS     19 //P1.19
//#define XRESET  20 //P1.20
//#define XDCS    28 //P1.28
//#define DREQ    30 //P0.30 , P1.22 is broken
//
////gpio buttons
//#define GPIO_SM_YELLOW  5 //P2.5, mode
//#define GPIO_SM_BLUE    4 //P2.4, prev song, prev, vol down
//#define GPIO_SM_GREY    3 //P2.3, next song, next, vol up
//#define GPIO_SM_RED     2 //P2.2, back, stop
//#define GPIO_BIG_GREEN  1 //P2.1, play,pause,mute?,select
//
//#define NORIC_PIN 16
//
//#define BLOCK_SIZE          512
//#define MUSIC_QUEUE_SIZE    4
//#define MAX_SONGS           100
//#define MAX_SONG_LENGTH     13
//
////-Default-User-Settings-------------------------------------------------------------------
//
//#define DEFAULT_VOLUME 100
//
//
//
////----------------------------------------------------------------------------------------
//
////lcd update flags
//typedef struct{
//    uint8_t updateSong;
//    uint8_t updateState;
//    uint8_t updateVolume;
//    uint8_t isFourBit;
//    uint8_t updateMode;
//    uint8_t updateSelect;
//} lcdFlags_t;
//
////chunks of music, struct is used to pass data through queues
//typedef struct {
//    FILE* file;
//    uint8_t data[BLOCK_SIZE];
//    uint32_t fileSize;
//    uint32_t position;
//} musicBlock_t;
//
////music states
//typedef enum {
//    PLAY,
//    PAUSE,
//    STOP
//} state_t;
//
////mp3 mode
//typedef enum {
//    HOME,
//    VOLUME,
//    SELECT
//} playerMode_t;
//
///// IDs used for getSharedObject() and addSharedObject()
//typedef enum {
//   shared_MusicQueueId,
//} sharedHandleId_t;
//
//QueueHandle_t musicQueue;
//SemaphoreHandle_t musicSemaphore, displaySemaphore;
//
//lcdFlags_t lcdFlags = { 0, 0, 0, 0, 0, 0};
////-info-for-display--------------------------------------------------------------------------
//char songList[MAX_SONGS][MAX_SONG_LENGTH];
//uint8_t totalSongs = 0;
//state_t state = STOP;
//mode_t playerMode = HOME;
//uint8_t currentSong = 0;
//uint8_t currentVolume = DEFAULT_VOLUME;
//uint8_t selectedSong = 0;
//
////-----------------------------------------------------------------------------------------
//void pinSet(uint32_t pin, uint8_t mode){
//    (mode == 1)? LPC_GPIO1->FIOSET = (1 << pin) : LPC_GPIO1->FIOCLR = (1 << pin);
//}
//
////XDCS and CS are active low
//void controlMode(uint8_t mode){
//    if(mode == 1){
//        pinSet(XDCS, ON);
//        pinSet(XCS, OFF);
//    }
//    else{
//        pinSet(XCS, ON);
//    }
//}
//
////XDCS and CS are active low
//void dataMode(uint8_t mode){
//    if(mode == 1){
//        pinSet(XDCS, OFF);
//        pinSet(XCS, ON);
//    }
//    else{
//        pinSet(XDCS, ON);
//    }
//}
//
//uint8_t spiExchangeByte(uint8_t out)
//{
//    LPC_SSP0->DR = out;
//    while(LPC_SSP0->SR & (1 << 4)); // Wait until SSP is not busy
//    return LPC_SSP0->DR;
//}
//
////return true if 1053's 2048-byte FIFO can take at least 32 bytes of SDI data or one SCI cmd
//bool isReady(void){
//    return (LPC_GPIO0->FIOPIN & (1 << DREQ))? true : false;
//}
//
////write SCI registers
//void sciWrite(uint8_t address, uint16_t value){
//    controlMode(ON);
//    spiExchangeByte(WRITE);
//    spiExchangeByte(address);
//    spiExchangeByte(value >> 8);
//    spiExchangeByte(value & 0x00FF);
//    while(!isReady()); //wait until DREQ is high(ready)
//    controlMode(OFF);
//}
////-------------------------------------------------------------------------------------------
//
////MP3 decoder interface
//class MP3 : public scheduler_task
//{
//    bool mute = false;
//    uint8_t savedVolume = DEFAULT_VOLUME;
//    public:
//        MP3(uint8_t priority): scheduler_task("MP3Task", 2000, priority){}
//        bool init(void){
//
//            //SPI config
//            LPC_SC->PCONP |= (1 << 21);     // SPI0 Power Enable
//            LPC_SC->PCLKSEL1 &= ~(3 << 10); // Clear clock Bits
//            LPC_SC->PCLKSEL1 |=  (1 << 10); // CLK / 1
//
//            // Select MISO, MOSI, and SCK pin-select functionality
//            LPC_PINCON->PINSEL0 &= ~(3 << 30);
//            LPC_PINCON->PINSEL0 |=  (2 << 30);
//            LPC_PINCON->PINSEL1 &= ~( (3 << 2) | (3 << 4) );
//            LPC_PINCON->PINSEL1 |=  ( (2 << 2) | (2 << 4) );
//
//            LPC_SSP0->CR0 = 7;          // 8-bit mode
//            LPC_SSP0->CR1 |= (1 << 1);   // Enable SSP as Master
//            LPC_SSP0->CPSR = 1;         // SCK speed = CPU / 1
//
//            //pin config
//            LPC_PINCON->PINSEL3 &= ~(3 << 14);
//            LPC_PINCON->PINSEL3 |=  (0 << 14);// CS P1.23
//            LPC_PINCON->PINSEL3 &= ~(3 << 6);
//            LPC_PINCON->PINSEL3 |=  (0 << 6);// XCS P1.19
//            LPC_PINCON->PINSEL3 &= ~(3 << 8);
//            LPC_PINCON->PINSEL3 |=  (0 << 8);// XRESET P1.20
//            LPC_PINCON->PINSEL3 &= ~(3 << 24);
//            LPC_PINCON->PINSEL3 |=  (0 << 24);// XDCS P1.28
////             LPC_PINCON->PINSEL3 &= ~(3 << 12);
////             LPC_PINCON->PINSEL3 |=  (0 << 12);// DREQ P1.22, pin broken
//            LPC_PINCON->PINSEL1 &= ~(3 << 28);
//            LPC_PINCON->PINSEL1 |=  (0 << 28);// DREQ P0.30
//
//            //make sure P0.16(noric wireless) is high
//            LPC_PINCON->PINSEL1 &= ~(3 << 0);
//            LPC_GPIO0->FIODIR |= (1 << NORIC_PIN);
//            LPC_GPIO0->FIOSET = (1 << NORIC_PIN);
//
//            //MP3 pin configurations
//            pinSetMode(CS, OUTPUT);
//            pinSetMode(XCS, OUTPUT);
//            pinSetMode(XRESET, OUTPUT);
//            pinSetMode(XDCS, OUTPUT);
//            //pinSetMode(DREQ, INPUT); //pin broken
//            LPC_GPIO0->FIODIR &= ~(1 << 30); // Set P0.30 as input dreq
//
//            //initialize for pin on MP3 decoder
//            pinSet(CS, ON); //Disable SD read on vs1053 board, active low
//            pinSet(XCS, ON); //XCS active low
//            pinSet(XRESET, ON); //XRESET active low
//            pinSet(XDCS, ON); //XDCS active low
//
//            sciWrite(SCI_MODE, 0x4800);
//            sciWrite(SCI_CLOCKF, 0x6000);
//            sciWrite(SCI_AUDATA, 0xAC45);
//            setVolume(DEFAULT_VOLUME);
//            //sciWrite(SCI_BASS, 0x0000); //bass off
//
//            //done configuring vs1053
//
//            //configuring GPIO buttons
//            LPC_PINCON->PINSEL4 &= ~( (3 << 2) | (3 << 4) | (3 << 6) | (3 << 8) | (3 << 10));
//            LPC_PINCON->PINSEL4 |=  ( (0 << 2) | (0 << 4) | (0 << 6) | (0 << 8) | (0 << 10));
//
//            //set GPIO inputs as input
//            LPC_GPIO2->FIODIR &= ~( (1 << GPIO_SM_BLUE) | (1 << GPIO_SM_YELLOW) | (1 << GPIO_SM_GREY) |
//                                      (1 << GPIO_SM_RED)  | (1 << GPIO_BIG_GREEN));
//
//            //setup queues and semaphores
//            musicQueue = xQueueCreate(MUSIC_QUEUE_SIZE , sizeof(musicBlock_t*));
//            addSharedObject(shared_MusicQueueId, musicQueue);
//            musicSemaphore = xSemaphoreCreateBinary();
//            displaySemaphore = xSemaphoreCreateBinary();
//
//            updateSongList();
//
//            return true;
//        }
//
//        bool run(void *p){
//
//            if(button(GPIO_SM_YELLOW)){
////        	if (SW.getSwitch(0)){
//                (playerMode != SELECT)? playerMode++ : playerMode = HOME;
//                lcdFlags.updateMode = 1;
//                xSemaphoreGive(displaySemaphore);
//            }
//
//            if(playerMode == HOME){
//                if(button(GPIO_SM_BLUE))
//                    playPrevNext(0);
//                if(button(GPIO_SM_GREY))
//                    playPrevNext(1);
//                if(button(GPIO_BIG_GREEN))
//                    play();
//                if(button(GPIO_SM_RED))
//                    stop();
//            }
//            else if(playerMode == VOLUME){
//                if(button(GPIO_SM_BLUE)){
//                    volumeDown();
//                    lcdFlags.updateVolume = 1;
//                    xSemaphoreGive(displaySemaphore);
//                }
//                if(button(GPIO_SM_GREY)){
//                    volumeUp();
//                    lcdFlags.updateVolume = 1;
//                    xSemaphoreGive(displaySemaphore);
//                }
//                if(button(GPIO_BIG_GREEN)){
//                    volumeMute();
//                    lcdFlags.updateVolume = 1;
//                    xSemaphoreGive(displaySemaphore);
//                }
//                if(button(GPIO_SM_RED)){
//                    playerMode = HOME;
//                    lcdFlags.updateMode = 1;
//                    xSemaphoreGive(displaySemaphore);
//                }
//            }
//            else if(playerMode == SELECT){
//                if(button(GPIO_SM_BLUE)){
////                	if (SW.getSwitch(1)){
//                    selectedSong = (selectedSong-1 < 0)? totalSongs-1: --selectedSong;
//                    lcdFlags.updateSelect = 1;
//                    xSemaphoreGive(displaySemaphore);
//                }
//                if(button(GPIO_SM_GREY)){
//                    selectedSong = (selectedSong+1 > totalSongs-1)? 0: ++selectedSong;
//                    lcdFlags.updateSelect = 1;
//                    xSemaphoreGive(displaySemaphore);
//                }
//                if(button(GPIO_BIG_GREEN)){
//                    stop();
//                    currentSong = selectedSong;
//                    vTaskDelay(50);
//                    playerMode = HOME;
//                    lcdFlags.updateMode = 1;
//                    setState(PLAY);
//                }
//                if(button(GPIO_SM_RED)){
//                    playerMode = HOME;
//                    lcdFlags.updateMode = 1;
//                    xSemaphoreGive(displaySemaphore);
//                }
//
//            }
//
//            vTaskDelay(150);
//            return true;
//        }
//        void reset(void){
//            sciWrite(SCI_MODE, (1<<2));
//        }
//
//        void pinSetMode(uint32_t pin, uint8_t direction){
//            (direction == 1)? LPC_GPIO1->FIODIR |= (1 << pin) : LPC_GPIO1->FIODIR &= ~(1 << pin);
//        }
//
//        //read SCI registers
//        uint16_t sciRead(uint8_t address){
//            controlMode(ON);
//            spiExchangeByte(READ);
//            spiExchangeByte(address);
//            uint16_t result = (spiExchangeByte(0xFF) << 8) | spiExchangeByte(0xFF);
//            controlMode(OFF);
//            while(!isReady()); //wait until DREQ is high(ready)
//            return result;
//        }
//
//        //set vs1053 volume register
//        void setVolume(uint8_t volume){
//            currentVolume = volume;
//            uint16_t vol = (volume << 8) | volume;
//            sciWrite(SCI_VOL, vol);
//            lcdFlags.updateVolume = 1;
//        }
//
//        void volumeMute(void){
//            if(mute){
//                setVolume(savedVolume);
//                mute = false;
//            }
//            else{
//                savedVolume = currentVolume;
//                setVolume(255);
//                mute = true;
//            }
//            lcdFlags.updateVolume++;
//            xSemaphoreGive(displaySemaphore);
//        }
//
//        void volumeDown(void){
//            if(savedVolume+10 <= 254){
//                setVolume(savedVolume+=10);
//            }
//            else{
//                savedVolume= 254;
//                setVolume(savedVolume);
//            }
//            xSemaphoreGive(displaySemaphore);
//        }
//
//        void volumeUp(void){
//            if(savedVolume-10 >= 0) {
//                setVolume(savedVolume-=10);
//            }
//            else{
//                savedVolume= 0;
//                setVolume(savedVolume);
//            }
//            xSemaphoreGive(displaySemaphore);
//        }
//
//        void play(void){
//            (state == STOP)?    setState(PLAY):
//            (state == PAUSE)?   setState(PLAY):
//                                setState(PAUSE);
//        }
//
//        //clear queue
//        void stop(void){
//            setState(STOP);
//        }
//
//        void playPrevNext(uint8_t value){
//            stop();
//            if(value == 0)
//                currentSong = (currentSong <= 0)? totalSongs-1 : currentSong-1 ;
//            else
//                currentSong = (currentSong >= totalSongs-1)? 0: currentSong+1 ;
//            vTaskDelay(50);
//            setState(PLAY);
//        }
//
//        void setState(state_t nextState){
//            state_t prevState = state;
//            state = nextState;
//            scheduler_task *task = scheduler_task::getTaskPtrByName("sendMusicTask");
//            switch(state){
//                case PLAY:
//                    if(prevState == STOP)
//                        xSemaphoreGive(musicSemaphore);
//                    else
//                        vTaskResume(task->getTaskHandle());
//                    break;
//                case PAUSE:
//                    vTaskSuspend(task->getTaskHandle());
//                    break;
//                case STOP:
//                    xQueueReset(getSharedObject(shared_MusicQueueId));
//                    vTaskResume(task->getTaskHandle());
//                    break;
//                default:
//                    break;
//            }
//            lcdFlags.updateState = 1;
//            xSemaphoreGive(displaySemaphore);
//        }
//
//        void updateSongList(void){
//            FRESULT res;
//            DIR dir;
//            static FILINFO fno;
//
//            res = f_opendir(&dir, "1:");                       /* Open the directory */
//            if (res == FR_OK) {
//                for (;;) {
//                    res = f_readdir(&dir, &fno);                   /* Read a directory item */
//                    if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
//                    if (!(fno.fattrib & AM_DIR))                   /* It is a file. */
//                        strcpy(songList[totalSongs++], fno.fname);
//                }
//                f_closedir(&dir);
//            }
//        }
//
//        //return status of gpio buttons, active low
//        bool button(uint8_t pin){
//            return (LPC_GPIO2->FIOPIN & (1 << pin))? false : true;
//        }
//};
//
//
////MP3 decoder interface
//class sendMusic : public scheduler_task
//{
//    public:
//        sendMusic(uint8_t priority): scheduler_task("sendMusicTask", 2000, priority){}
//        bool init(void){
//            return true;
//        }
//
//        bool run(void *p){
//
//            //wait for semaphore
//            if(state == STOP)
//                xSemaphoreTake(musicSemaphore, portMAX_DELAY);
//
//            lcdFlags.updateSong = 1;
//            xSemaphoreGive(displaySemaphore);
//
//            char prefix[] = "1:";
//            char* songName = songList[currentSong];
//            u0_dbg_printf("currently playing: %s\n", songName);
//            char* fileName = (char*)malloc(strlen(prefix)+strlen(songName)+1);
//            strcpy(fileName, prefix);
//            strcat(fileName, songName);
//
//            FILE* f = fopen( fileName, "r");
//            if(f == NULL)
//                return true;
//
//            //get filesize
//            fseek(f, 0, SEEK_END);
//            int fileSize = ftell(f);
//            fseek(f, 0, SEEK_SET);
//
//            //read in chunks of 512 bytes and send to queue
//            musicBlock_t m;
//            musicBlock_t* mPtr;
//            for(int i = 0; i < fileSize/BLOCK_SIZE+1; i++){
//                if(state == STOP)
//                    break;
//                m.file = f;
//                m.fileSize = fileSize;
//                m.position = 0;
//                fread(m.data, 1, BLOCK_SIZE, m.file);
//                mPtr = &m;
//                xQueueSend(getSharedObject(shared_MusicQueueId), &mPtr, portMAX_DELAY);
//            }
//            fclose(f);
//
//            if(state != STOP)
//                currentSong = (currentSong >= totalSongs-1)? 0: currentSong+1 ;
//
//            return true;
//        }
//};
//
//class processMusic : public scheduler_task
//{
//    public:
//        processMusic(uint8_t priority): scheduler_task("processMusicTask", 2000, priority)
//    {}
//        bool init(void){
//            return true;
//        }
//
//        bool run(void *p){
//
//                /* We first get the queue handle the other task added using addSharedObject() */
//                musicBlock_t* data;
//                QueueHandle_t qid = getSharedObject(shared_MusicQueueId);
//
//                //wait for queue to be not empty
//                while(xQueueReceive(qid, &data, portMAX_DELAY)){
//                    while(!isReady());
//                    //get pointer to data chunk
//                    uint8_t* dataPtr = data->data;
//                    int z = 0;
//                    dataMode(ON);
//                    while(z < BLOCK_SIZE){
//                        //check dreq every 32 bytes
//                        for(int i = 0; i < 32; i++){
//                            //send byte to decoder
//                            spiExchangeByte(*dataPtr++);
//                            z++;
//                        }
//                        while(!isReady());
//                    }
//                    dataMode(OFF);
//                }
//                return true;
//        }
//
//};
//
////-------------------------------------------------------------------------
////The lcd library was ported from the Arduino LiquidCrystal Library which
////uses the Hitachi HD44780 Datasheet. Most of the functions are the
////same as the Arduino library for readability. Initialization and low level
////code has been modified to make the Arduino library usable on the SJ One
////Board (LPC1768).
////-------------------------------------------------------------------------
//
////class lcd : public scheduler_task
////{
////    uint8_t playSymbol[8] = {
////        0b01000,
////        0b01100,
////        0b01110,
////        0b01111,
////        0b01110,
////        0b01100,
////        0b01000,
////        0b00000
////    };
////
////    uint8_t pauseSymbol[8] = {
////        0b00000,
////        0b11011,
////        0b11011,
////        0b11011,
////        0b11011,
////        0b11011,
////        0b11011,
////        0b00000
////    };
////
////    uint8_t stopSymbol[8] = {
////        0b00000,
////        0b11111,
////        0b11111,
////        0b11111,
////        0b11111,
////        0b11111,
////        0b00000,
////        0b00000
////    };
////
////    uint8_t musicSymbol[8] = {
////        0b00100,
////        0b00110,
////        0b00101,
////        0b00101,
////        0b11100,
////        0b11100,
////        0b11100,
////        0b00000
////    };
////
////    uint8_t displayfunction;
////    uint8_t displaycontrol;
////    uint8_t displaymode;
////    uint8_t numlines, currline;
////    public:
////        lcd(uint8_t priority): scheduler_task("lcdTask", 2000, priority){}
////        bool init(void){
////
////            //pinconfig GPIO for D7, D6, D5, D4, RS, EN
////            LPC_PINCON->PINSEL4 &= ~((3<<12)|(3<<14)|(3<<16)|(3<<18));
////            LPC_PINCON->PINSEL0 &= ~((3<<0)|(3<<2));
////
////            //set pins to output
////            LPC_GPIO2->FIODIR |= ((1<<LCD_D7)|(1<<LCD_D6)|(1<<LCD_D5)|(1<<LCD_D4));
////            LPC_GPIO0->FIODIR |= ((1<<LCD_RS)|(1<<LCD_EN));
////
////            //make sure RS and EN are low
////            LPC_GPIO0->FIOCLR = ((1 << LCD_RS) | (1 << LCD_EN));
////
////            return true;
////        }
////
////        bool run(void *p){
////
////            if(lcdFlags.isFourBit++!=1){
////                fourBitMode();
////            }
////            while(1){
////                //wait for semaphore
////                xSemaphoreTake(displaySemaphore, portMAX_DELAY);
////
////                if(lcdFlags.updateMode-- == 1){
////                    (playerMode == HOME)?   updateSong()    :
////                    (playerMode == VOLUME)? updateVolume()  :
////                                            updateSelect()  ;
////                }
////
////                //update display and reset flags
////                else if(lcdFlags.updateSong-- == 1)
////                    updateSong();
////                else if(lcdFlags.updateVolume-- == 1)
////                    updateVolume();
////                else if(lcdFlags.updateSelect-- == 1)
////                    updateSelect();
////                if(lcdFlags.updateState-- == 1)
////                    updateState();
////                vTaskDelay(50);
////            }
////            return true;
////        }
////
////        //initializer, only runs once
////        void fourBitMode(void){
////            displayfunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
////            numlines = 2;
////            setData(0x03);
////            setData(0x03);
////            setData(0x03);
////            setData(0x02);
////
////            // finally, set # lines, font size, etc.
////            sendInstruction(LCD_FUNCTIONSET | displayfunction);
////
////            // turn the display on with no cursor or blinking default
////            displaycontrol = LCD_DISPLAYOFF | LCD_CURSOROFF | LCD_BLINKOFF;
////            display();
////
////            // clear it off
////            clear();
////
////            // Initialize to default text direction (for romance languages)
////            displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
////            // set the entry mode
////            sendInstruction(LCD_ENTRYMODESET | displaymode);
////
////            //create custom char
////            createChar(0, playSymbol);
////            createChar(1, pauseSymbol);
////            createChar(2, stopSymbol);
////            createChar(3, musicSymbol);
////
////            //update screen
////            updateSong();
////            updateState();
////            lcdFlags.updateVolume = 0;
////        }
////
////        //pulse enable
////        void pulseEnable(void){
////            LPC_GPIO0->FIOSET = (1 << LCD_EN);  // En high
////            vTaskDelay(1);		                // enable pulse must be >450ns
////            LPC_GPIO0->FIOCLR = (1 << LCD_EN); 	// En low
////            vTaskDelay(1);		                // commands need > 37us to settle
////        }
////
////        //set data bits and write lcd
////        void setData(uint8_t data){
////            (data & (1 << 3))? LPC_GPIO2->FIOSET = (1 << LCD_D7) : LPC_GPIO2->FIOCLR = (1 << LCD_D7);
////            (data & (1 << 2))? LPC_GPIO2->FIOSET = (1 << LCD_D6) : LPC_GPIO2->FIOCLR = (1 << LCD_D6);
////            (data & (1 << 1))? LPC_GPIO2->FIOSET = (1 << LCD_D5) : LPC_GPIO2->FIOCLR = (1 << LCD_D5);
////            (data & (1 << 0))? LPC_GPIO2->FIOSET = (1 << LCD_D4) : LPC_GPIO2->FIOCLR = (1 << LCD_D4);
////            pulseEnable();
////        }
////
////        //instruction mode
////        void sendInstruction(uint8_t value){
////            send(value, 0);
////        }
////
////        //data mode
////        void write(uint8_t value){
////            send(value, 1);
////        }
////
////        //send in 4 bit mode
////        void send(uint8_t value, uint8_t mode){
////            (mode == 1)? LPC_GPIO0->FIOSET = (1 << LCD_RS) : LPC_GPIO0->FIOCLR = (1 << LCD_RS);
////            setData(value >> 4);
////            setData(value);
////        }
////
////       /********** high level commands, for the user! */
////        void clear()
////        {
////        sendInstruction(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
////        vTaskDelay(2);  // this command takes a long time!
////        }
////
////        void home()
////        {
////        sendInstruction(LCD_RETURNHOME);  // set cursor position to zero
////        vTaskDelay(2);  // this command takes a long time!
////        }
////
////        //set cursor position
////        void setCursor(uint8_t col, uint8_t row)
////        {
////            int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
////            if ( row > numlines ) {
////                row = numlines-1;    // we count rows starting w/0
////        }
////
////        sendInstruction(LCD_SETDDRAMADDR | (col + row_offsets[row]));
////        }
////
////        // Turn the display on/off (quickly)
////        void noDisplay() {
////        displaycontrol &= ~LCD_DISPLAYON;
////        sendInstruction(LCD_DISPLAYCONTROL |displaycontrol);
////        }
////        void display() {
////        displaycontrol |= LCD_DISPLAYON;
////        sendInstruction(LCD_DISPLAYCONTROL |displaycontrol);
////        }
////
////        // Allows us to fill the first 8 CGRAM locations
////        // with custom characters
////        void createChar(uint8_t location, uint8_t charmap[]) {
////            location &= 0x7; // we only have 8 locations 0-7
////            sendInstruction(LCD_SETCGRAMADDR | (location << 3));
////            for (int i=0; i<8; i++) {
////                write(charmap[i]);
////            }
////        }
////
////        //helper functions for FreeRTOS task
////
////        //update song on lcd
////        void updateSong(void){
////            setCursor(0,0);
////            write(3);
////            write(':');
////            char* s = songList[currentSong];
//////             s[strlen(s)-4] = '\0';
////            for(int i = 0; i < MAX_SONG_LENGTH; i++){
////                write((*s == '\0')? ' ' : *s++);
////            }
////            setCursor(0,1);
////            for(int i = 0; i < 16; i++){
////                write(' ');
////            }
////        }
////
////        //update state on lcd
////        void updateState(void){
////            setCursor(15,0);
////            write((state==PLAY)? 0 :
////                  (state==STOP)? 2 :
////                                 1);
////        }
////
////        //update volume on lcd
////        void updateVolume(void){
////            setCursor(0,0);
////            char* ss = (char*)"VOLUME:";
////            while(*ss != '\0'){
////                write(*ss++);
////            }
////            char vol[3];
////            char* s = vol;
////            u0_dbg_printf("volume:%d\n", currentVolume);
////        //    itoa((254-currentVolume)/2.54, vol, 10);
////            for(int i = 0; i < 8; i++){
////                write((*s == '\0')? ' ' : *s++);
////            }
////            setCursor(0,1);
////            for(int i = 0; i < 16; i++){
////                write(' ');
////            }
////        }
////
////        void updateSelect(void){
////            setCursor(0,0);
////            char* ss = (char*)"PICK ";
////            while(*ss != '\0'){
////                write(*ss++);
////            }
////            write(3);
////            write(':');
////            write(' ');
////            char selectedSongNumber[3];
////            char* s = selectedSongNumber;
////          //  itoa(selectedSong+1, selectedSongNumber, 10);
////            for(int i = 0; i < 3; i++){
////                write((*s == '\0')? ' ' : *s++);
////            }
////            write('/');
////            char totalSongNumber[3];
////            s = totalSongNumber;
////          //  itoa(totalSongs, totalSongNumber, 10);
////            for(int i = 0; i < 3; i++){
////                write((*s == '\0')? ' ' : *s++);
////            }
////            setCursor(0,1);
////            write('<');
////            s = songList[selectedSong];
////            for(int i = 0; i < 14; i++){
////                write((*s == '\0')? ' ' : *s++);
////            }
////            write('>');
////        }
////};
//
//
//int main(void)
//{
//    scheduler_add_task(new MP3(PRIORITY_HIGH));
//    scheduler_add_task(new sendMusic(PRIORITY_LOW));
//    scheduler_add_task(new processMusic(PRIORITY_MEDIUM));
////    scheduler_add_task(new lcd(PRIORITY_HIGH));
//    scheduler_add_task(new terminalTask(PRIORITY_HIGH));
//    scheduler_start(); //should not return
//    vSemaphoreDelete(musicSemaphore);
//    vSemaphoreDelete(displaySemaphore);
//    return 0;
//}
//
//

#include "tasks.hpp"
#include "stdio.h"
#include "stdlib.h"
#include "i2c_base.hpp"
#include "i2c2.hpp"
#include "i2c2_device.hpp"
#include <stdint.h>
#include "command_handler.hpp"
#include "uart0_min.h"// For the use of uart0_puts() to replace for printf()
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lpc_sys.h"
#include"scheduler_task.hpp"
#include"handlers.hpp"
#include "periodic_scheduler/periodic_callback.h"
#include "LPC17xx.h"
#include "printf_lib.h"
#include "rtc.h"
#include "string.h"
#include "storage.hpp"
#include "printf_lib.h"
#include "eint.h"
#include "utilities.h"
#include "ssp0.h"
#include "ssp1.h"
#include "io.hpp"

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

SemaphoreHandle_t scrollDown, Select;
QueueHandle_t selectedFilename;
char SongList[200][100] = { NULL };

/*Display Macros*/
#define clearLCD()     U2_Tx('C');/*Send CL to display to clear the display*/\
                       U2_Tx('L');

#define setBacklightON()    U2_Tx('B');/*Send BL to Display*/  \
                            U2_Tx('L');                        \
                            U2_Tx(0x01);

#define sendData()         U2_Tx('M');/* Send MDT for single byte data send */ \
                           U2_Tx('D'); \
                           U2_Tx('T');

#define setCursorON()    U2_Tx('C');/*Send BL to Display*/  \
                         U2_Tx('S');                        \
                         U2_Tx(0x01);

#define setCursorOFF()    U2_Tx('C');/*Send BL to Display*/  \
                          U2_Tx('S');                        \
                          U2_Tx(0x00);

#define secondLine()      U2_Tx('T');\
                          U2_Tx('R');\
                          U2_Tx('T');
//U2_Tx(0x01);

#define stopText()    U2_Tx(0);

#define sendText()     U2_Tx('T');\
                          U2_Tx('T');

#define setPosition() U2_Tx('T');\
                       U2_Tx('P');

bool getSongNames()
{
    DIR Dir;
    FILINFO Finfo;
    FATFS *fs;
    FRESULT returnCode = FR_OK;
    char Lfname[_MAX_LFN];
    int order = 0;
    unsigned int fileBytesTotal = 0, numFiles = 0, numDirs = 0;

    const char *dirPath = "1:Music:";
    if (FR_OK != (returnCode = f_opendir(&Dir, dirPath))) {
        printf("Invalid directory: |%s| (Error %i)\n", dirPath, returnCode);
        return true;
    }

    Finfo.lfname = Lfname;
    Finfo.lfsize = sizeof(Lfname);

//    u0_dbg_printf("Directory listing of: %s\n\n", dirPath);
    for (;;) {

        returnCode = f_readdir(&Dir, &Finfo);
        if ((FR_OK != returnCode) || !Finfo.fname[0]) {

            break;
        }

        if (Finfo.fattrib & AM_DIR) {

            numDirs++;
        }
        else {

            numFiles++;
            fileBytesTotal += Finfo.fsize;
        }

        strcpy(SongList[order], &(Finfo.lfname[0]));
        //u0_dbg_printf("%s\n",SongList[order]);
        // LFN names tend to increase memory requirements for output str, enable with caution
        order++;
        printf("\n");
    }

    return true;

}

class uart_display: public scheduler_task {
public:
    uart_display(uint8_t priority) :
            scheduler_task("Display", 2000, priority)
    {
    }

    bool U2_init(int baud)
    {
        //Turn on PCONP for UART2
        LPC_SC->PCONP |= (1 << 24);
        //   lpc_pconp(pconp_uart2, true);
        // BIT(LPC_PINCON).b10=1;

        //Reset the PCLKSEL1 value
        LPC_SC->PCLKSEL1 &= ~(3 << 16);

        //Turn off the CLK to save power
        BIT(LPC_SC->PCLKSEL1).b16 = 1;
        //LPC_SC->PCLKSEL1 |= (1<<16);
        //   lpc_pclk(pclk_uart2,clkdiv_1);
        //Enable access to divisor latch bit DLAB by set bit 7 to 1
        LPC_UART2->LCR |= (1 << 7);
        //Formula to calculate the baud clock rate
        uint16_t baud_rate = sys_get_cpu_clock() / (16 * baud);
        //Determine the UnDLM register, it helps to calculate the baud rate of the UART pin
        LPC_UART2->DLM = baud_rate >> 8;
        //Determine the UnDLL register, it helps to calculate the baud rate of the UART pin
        LPC_UART2->DLL = baud_rate;

        //over write DLAB 8 bit character length
        LPC_UART2->LCR = 3;

        //Choose PINSEL0 port P 2.8 for UART2-Tx and P2.9 for UART2-Rx
        // BIT(LPC_PINCON->PINSEL0).b21_b20 = 1;
        LPC_PINCON->PINSEL4 &= ~(3 << 16); //Clear all the bit
        LPC_PINCON->PINSEL4 |= (2 << 16); //Set bit 17_16 as 10
        //Choose port P0.11 for UART2=Rx
        LPC_PINCON->PINSEL4 &= ~(3 << 18); //Clear all the bit
        LPC_PINCON->PINSEL4 |= (2 << 18); //Set bit 19_18 as 10

        //Extra credit
        //Disable DLAB bit =0 to enable UART Interupt
        //  LPC_UART2->LCR |= (1<<7);
//		  //Enable Register
//		  LPC_UART2->IER |=(1<<2);
//		 LPC_UART2->IER |=(1<<0);
        return true;
    }

    void U2_Tx(char data)
    {

        //Wait until the line is not busy and send the data
        //Check to see if the Line status register LSR is empty or not(bit 5) if empty, send data.
        while (!(LPC_UART2->LSR & (1 << 5)))
            ;

        //Send the data to the transmit holding register
        LPC_UART2->THR = data;

    }

    char U2_Rx(void)
    {

        //Wait for the data to come by checking the LSR
        //Check to see if there is a valid character in Line status register LSR
        while (!(LPC_UART2->LSR & (1 << 0)))
            ;
        //When the LSR buffer contains a valid character, read it
        return (LPC_UART2->RBR);
    }

    void clearFirstLine()
    {
        setPosition()
        ;
        U2_Tx(0x00);
        U2_Tx(0x00);
        sendText()
        ;
        for (int i = 0; i < 16; i++) {
            U2_Tx(' ');
        }
        stopText()
        ;
    }

    void clearSecondLine()
    {
        setPosition()
        ;
        U2_Tx(0x00);
        U2_Tx(0x01);
        sendText()
        ;
        for (int i = 0; i < 16; i++) {
            U2_Tx(' ');
        }
        stopText()
        ;
    }

    void setCursorPosition(int x, int y)
    {
        setPosition()
        ;
        U2_Tx(x);
        U2_Tx(y);
    }

    void sendChar(char c)
    {
        sendText()
        ;
        U2_Tx(c);
        U2_Tx(0);
    }

    void sendLine(char* song)
    {
        sendText()
        ;
        if (song[0] != NULL && song[1] != NULL) {
            for (int i = 0; i < 16; i++) {

                if (song[i] != NULL) {
                    //sendData()
                    //                ;
                    //vTaskDelay(4);
                    U2_Tx(song[i]); //send values in array one at a time
                    //                u0_dbg_printf("%c\n", songPointer0[i]);
                    //sendChar(song[i]);
                }
                else {
                    break;
                }
            }
        }
        else {
            sendChar(' ');
        }

        U2_Tx(0);

    }

    void slowStepLine(char* song)
    {
        sendText()
        ;
        for (int i = 0; i < 16; i++) {
            if (song[i] != NULL) {
                //sendData()
                //                ;
                vTaskDelay(100);
                U2_Tx(song[i]); //send values in array one at a time
                //                u0_dbg_printf("%c\n", songPointer0[i]);
                //sendChar(song[i]);
            }
            else {
                break;
            }
        }
        U2_Tx(0);

    }

    bool run(void *p)
    {
        //Initial the baud rate to 9600 bps
        U2_init(9600); //is the baud rate for the display 9600?
        vTaskDelay(500);
        stopText()
        ;
        char* songPointer0;
        char* songPointer1;
        int songSelector = 0;
        int listNum = 0;

//        char c[12] = "Hello world"; //for testing display

//        printf("Start to Transmit : \n");

//        u0_dbg_printf("Initializing LCD Display...\n");
        vTaskDelay(5);

        clearLCD()
        ;

        setCursorOFF()
        ;

        vTaskDelay(10);

        setBacklightON()
        ;

        vTaskDelay(10);

//        u0_dbg_printf("Attempting to Read from SD card...");

        songPointer0 = &SongList[listNum][0];
        sendLine(songPointer0);
        listNum++;
//        u0_dbg_printf("%s\n", songPointer0);
        songPointer1 = &SongList[listNum][0];
        sendLine(songPointer1);
        //listNum++;
//        u0_dbg_printf("%s\n", songPointer1);
        while (1) {
            while (songSelector < 2) {
                if (xSemaphoreTake(scrollDown, 0)) {
                    songSelector++;
//                    u0_dbg_printf("songSelector is at %i\n",songSelector);
                }
                switch (songSelector) {
                    case 0:
                        clearFirstLine();
                        setCursorPosition(0, 0);
                        slowStepLine(songPointer0);
                        u0_dbg_printf("we are stepping the first song\n");
//                        if (xSemaphoreTake(Select, 0)) {
                            //  xQueueSend(/*Handle Name*/,songPointer0,0);
//                        }
                        break;
                    case 1:
                        clearSecondLine();
                        setCursorPosition(0, 1);
                        slowStepLine(songPointer1);
                        u0_dbg_printf("we are stepping the second song\n");
//                        if (/*xSemaphoreTake(Select, 0)*/) {
                            //  xQueueSend(/*Handle Name*/,songPointer1,0);
//                        }

                        break;
                    default:

                        break;
                }
            }
            songSelector = 0;
            listNum++;
            songPointer0 = &SongList[listNum][0];
            sendLine(songPointer0);
            u0_dbg_printf("%s\n", songPointer0);
            listNum++;
            songPointer1 = &SongList[listNum][0];
            sendLine(songPointer1);
            u0_dbg_printf("%s\n", songPointer1);
        }
        return -1;

    }
};

//void buttonScrollDown()
//{
//    u0_dbg_printf("Interrupt Detected. Port 0 pin 0");
//    xSemaphoreGiveFromISR(scrollDown, NULL);
//}
//
//void buttonScrollUp()
//{
//    u0_dbg_printf("Interrupt Detected. Port 0 pin 0");
//    xSemaphoreGiveFromISR(scrollDown, NULL);
//}
//
//void selectSong()
//{
//    u0_dbg_printf("Interrupt Detected. Port 0 pin X");
//    xSemaphoreGiveFromISR(Select, NULL);
//}

//if(xSemaphoreTake(nextButton,portMAX_DELAY)){
//    currentSong++;
//}

typedef enum {
    volumnQueue,
} sharedHandleId_t;

SemaphoreHandle_t resumeButtonSemaphore = NULL;
SemaphoreHandle_t displayScreenSemaphore = NULL;

uint8_t currentSong = 0;
uint8_t totalSong = 15;
uint8_t currentVolumn = 100;       //0 is max, fffe is muted
uint8_t volumnDisplay = 9;
bool pausedFlag = true;         // the pausedFlage is used to display the lcd status
bool reStartFlag = false;

class controlPanel: public scheduler_task {
public:
    controlPanel(uint8_t priority) :
            scheduler_task("controlPanel", 4096, priority)
    {
        QueueHandle_t myVolumnQueue = xQueueCreate(3, sizeof(int));
        addSharedObject(volumnQueue, myVolumnQueue);

    }

    void PauseMusic()
    {
        pausedFlag = true;
        reStartFlag = false;
        long pauseSem = 0;
        scheduler_task *musicTask = scheduler_task::getTaskPtrByName("musicPlayer");
        vTaskSuspend(musicTask->getTaskHandle());

        scheduler_task *LcdTask = scheduler_task::getTaskPtrByName("LcdDisplay");
        vTaskResume(LcdTask->getTaskHandle());
        xSemaphoreGiveFromISR(displayScreenSemaphore, &pauseSem);
        if (pauseSem) {
            portYIELD_FROM_ISR(pauseSem);
        }
    }

    void ResumePlayMusic()
    {
        pausedFlag = false;
        reStartFlag = false;
        scheduler_task *musicTask = scheduler_task::getTaskPtrByName("musicPlayer");
        vTaskResume(musicTask->getTaskHandle());     // resume the task

        scheduler_task *LcdTask = scheduler_task::getTaskPtrByName("LcdDisplay");
        vTaskResume(LcdTask->getTaskHandle());
        long startPlay = 0;
        xSemaphoreGiveFromISR(displayScreenSemaphore, &startPlay);
        if (startPlay) {
            portYIELD_FROM_ISR(startPlay);
        }
    }
    void startMusic()
    {
        //  reStartFlag = false;
        //scheduler_task *task = scheduler_task::getTaskPtrByName("musicPlayer");
        //setVolum(100);
        pausedFlag = false;
        long startPlay = 0;
        scheduler_task *Musictask = scheduler_task::getTaskPtrByName("musicPlayer");
        vTaskResume(Musictask->getTaskHandle());
        xSemaphoreGiveFromISR(resumeButtonSemaphore, &startPlay);
        if (startPlay) {
            portYIELD_FROM_ISR(startPlay);
        }
    }
    void selectNextSong()
    {
        //  PauseMusic();
        PauseMusic();
        pausedFlag = true;
        reStartFlag = true;
        currentSong++;

        //        startMusic();

    }

    void selectPrevSong()
    {

        reStartFlag = true;
        pausedFlag = true;
        PauseMusic();
        currentSong--;

    }
    void volumnUp()
    {
        printf("Current volumn is: %d\n", currentVolumn);
        currentVolumn = currentVolumn - 5;      //- volumn is to max, 0 is max volumn
        volumnDisplay++;
        long volumnUpSem = 0;
        xQueueSend(getSharedObject(volumnQueue), &currentVolumn, 100);
        scheduler_task *task = scheduler_task::getTaskPtrByName("LcdDisplay");
        vTaskResume(task->getTaskHandle());

        xSemaphoreGiveFromISR(displayScreenSemaphore, &volumnUpSem);
        if (volumnUpSem) {
            portYIELD_FROM_ISR(volumnUpSem);
        }
    }
    void volumnDown()
    {
        printf("Current volumn is: %d\n", currentVolumn);
        currentVolumn = currentVolumn + 5;      // + volumn is to silence
        volumnDisplay--;
        long volumnDownSem = 0;
        xQueueSend(getSharedObject(volumnQueue), &currentVolumn, 100);
        scheduler_task *task = scheduler_task::getTaskPtrByName("LcdDisplay");
        vTaskResume(task->getTaskHandle());
        xSemaphoreGiveFromISR(displayScreenSemaphore, &volumnDownSem);
        if (volumnDownSem) {
            portYIELD_FROM_ISR(volumnDownSem);
        }
    }

    bool init(void)
    {
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
        NVIC_EnableIRQ(EINT3_IRQn);
        // isr_register(EINT3_IRQn, EINT3_IRQHandler);
        resumeButtonSemaphore = xSemaphoreCreateBinary(); // to create the semaphore
        displayScreenSemaphore = xSemaphoreCreateBinary();

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

    bool run(void *p)
    {

        if (SW.getSwitch(1)) {        // Button 1 for play and resume the music
            volumnDown();
        }
        else if (SW.getSwitch(2)) {  //Button 2 for pause the music
            PauseMusic();
        }
        else if(SW.getSwitch(3))
        {
            volumnUp();
        }
        else if(SW.getSwitch(4)) {
            startMusic();
        }
        else if(!(LPC_GPIO0->FIOPIN & (1 << 0))) {              // p0.0 -->select the next song
            selectNextSong();

        }
        else if(!(LPC_GPIO0->FIOPIN & (1 << 1))) {              // p0.1--> select to previous song
            selectPrevSong();
        }
        else if(!(LPC_GPIO2->FIOPIN & (1 << 0))) {  // p2.0, to set volumn up
            volumnUp();

        }
        else if(!(LPC_GPIO2->FIOPIN & (1 << 1))) {      // p2.1, to set volumn down
            volumnDown();

        }

        vTaskDelay(100);
        return true;
    }
};

class mp3Project: public scheduler_task                                          // this task in to produce the sensor value and then send it to queue
{
public:
    mp3Project(uint8_t priority) :
            scheduler_task("musicPlayer", 5000, priority)
    {
    }

    bool init(void)
    {
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
        LPC_PINCON->PINSEL0 &= ~((3 << 30));                      // clear the sck0
        LPC_PINCON->PINSEL1 &= ~((3 << 2) | (3 << 4));          //  clear the miso0 and mosi0
        LPC_PINCON->PINSEL0 |= (2 << 30);                     // set set the function for sck0
        LPC_PINCON->PINSEL1 |= ((2 << 2) | (2 << 4));          //and then set the function respectively
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
        LPC_GPIO1->FIODIR |= (1 << 28);   //set pin P1.28 as output -- RESET signal
        LPC_GPIO1->FIODIR |= (1 << 30);   //set pin P1.30 as output -- DCS signal
        LPC_GPIO1->FIODIR &= ~(1 << 19);  //set pin P1.19 as input -- DREQ signal
        LPC_GPIO1->FIOSET = (1 << 28);    //set pin P1.28 high initially
        LPC_GPIO1->FIOSET = (1 << 30);    //set pin P1.30 high initially
        LPC_GPIO1->FIOSET = (1 << 22);    //set pin P1.22 high initially
        // ssp0_set_max_clock(1);
    }

    uint8_t sendDataToRegister(uint8_t data)
    {
        LPC_SSP0->DR = data; //Send the data Out
        while (LPC_SSP0->SR & (1 << 4))
            ; // Wait until SSP is busy
        return LPC_SSP0->DR;
    }

    void setClock(uint16_t clockData)
    {
        writeRegister(SCI_CLOCKF, clockData);
    }

    bool dreqStatus()
    {
        if (LPC_GPIO1->FIOPIN & (1 << 19)) {
            return true;
        }
        else {
            return false;
        }
    }

    uint16_t readRegister(uint8_t readReg)
    {
        while (!(LPC_GPIO1->FIOPIN & (1 << 19)))
            ; //0 = not ready, wait till ready
        setCS();      //Send Low on CS to enable VS1053, active low
        sendDataToRegister(0x3);         //Send Read Opcode
        sendDataToRegister(readReg);      //Send Address to read from
        char firstByte = sendDataToRegister(0x99);
        while (!(LPC_GPIO1->FIOPIN & (1 << 19)))
            ;
        char secondByte = sendDataToRegister(0x98);
        while (!(LPC_GPIO1->FIOPIN & (1 << 19)))
            ;
        resetCS();     //Send High on CS to disable VS1053
        uint16_t data = firstByte << 8;
        data = data | secondByte;
        return data;
    }
    // this function tells the music data can be transfer to the decoder
    void setDataCS()
    {
        ssp0_set_max_clock(1);
        LPC_GPIO1->FIOCLR |= (1 << 30);
    }
    //clean the data cs
    void resetDataCS()
    {
        LPC_GPIO1->FIOSET |= (1 << 30);
    }
    // this function tells that the chip select is select the decoder and sending the data
    void setCS()
    {
        // ssp0_set_max_clock(12);
        LPC_GPIO1->FIOCLR |= (1 << 22);
    }
    // this tells that the chip select is not been selected
    void resetCS()
    {
        LPC_GPIO1->FIOSET |= (1 << 22);
    }
    // when writing to the register, need to enable the CS for decoder
    void writeRegister(uint8_t address, uint16_t data)
    {
        uint8_t lowByteData = data & 0x00FF;
        uint8_t highByteData = data >> 8;
        // lowByteData = (uint8_t)data;        // to get the low byte of data
        // highByteData = (uint8_t)(data>>8);  //to get the high byte of data
        resetDataCS();          // set the pin to high
        setCS();             //enable the chip select, the cs is active low
        while (!(LPC_GPIO1->FIOPIN & (1 << 19)))
            ;
        sendDataToRegister(0x02);     //send the opcode fist, this is operation of write data
        sendDataToRegister(address);             // send the address
        sendDataToRegister(highByteData);
        sendDataToRegister(lowByteData);
        //setDataCS();
        while (!(LPC_GPIO1->FIOPIN & (1 << 19)))
            ; // wait unitl the dreq is low
        resetCS();
    }
    void setVolum(uint8_t volumnData)
    {
        uint16_t volumn16Bit = (volumnData << 8) | volumnData;
        writeRegister(SCI_VOL, volumn16Bit);
    }
    void mp3_SDI_Write_32(char *data)
    {
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
    bool updataSong()
    {      // this is function use to updata song when user select next and prev song from the list

        return true;
    }

    bool run(void *p)
    {
        if (xSemaphoreTake(resumeButtonSemaphore, portMAX_DELAY)) {

            long int lSize;
            char mp3_buffer[512];       //can use the array
            FILE * mpFile;
            int i, y = 0;
            uint16_t volumnChange;
            QueueHandle_t volumControl = getSharedObject(volumnQueue);

            char prefix[] = "1:Music:";
            char* songName = &SongList[currentSong][0];
            char* fileLoc = (char*) malloc(strlen(prefix) + strlen(songName) + 1);
            strcpy(fileLoc, prefix);
            strcat(fileLoc, songName);
            u0_dbg_printf("%s\n",fileLoc);
            mpFile = fopen(fileLoc, "r");

            if (!mpFile)        // to make sure the file is open success
            {
                fputs("File error", stderr);
                exit(1);
            }

            fseek(mpFile, 0, SEEK_END);    // obtain file size:
            lSize = ftell(mpFile);     // get the pointer of the file
            rewind(mpFile);             // go the begining of the file

            if (mp3_buffer == NULL) {
                printf("mp3 data memory allocate fail");
                exit(1);
            }
            while (mpFile) {
//                if( reStartFlag == true){
//                    u0_dbg_printf(" the data send to mp3 is true \n");
//                    vTaskDelay(1000);
//                    reStartFlag = false;
//                    break;
//                }
                for (i = 0; i < lSize; i++) {
                    fseek(mpFile, 512 * i, SEEK_SET);     //re direction the reading file
                    int readResult = fread(mp3_buffer, 1, 512, mpFile);    // since cpu only can store at most 512k
                    // vTaskDelay(100);
                    while (y < readResult) {
                        // u0_dbg_printf(" the data send to mp3 is %x \n", mp3_buffer[x]);
                        while (dreqStatus() == 0) {
                            if (xQueueReceive(volumControl, &volumnChange, 15)) {
                                setVolum(volumnChange);
                            }
                        }
                        if (dreqStatus()) {
                            resetCS();
                            setDataCS();
                            for (int j = 0; j < 32; j++) {
                                ssp0_exchange_byte(mp3_buffer[y]);
                                //  u0_dbg_printf(" the data send to mp3 is %x \n", mp3_buffer[y]);
                                // vTaskDelay(100);
                                y++;
                            }
                            // u0_dbg_printf(" the data send to mp3 is %x \n", *mp3_buffer);
                            //  while(!(LPC_GPIO1->FIOPIN & (1 << 19)));
                            resetDataCS();
                        }
                        vTaskDelay(0);
                    }
                    y = 0;
                }
                // u0_dbg_printf(" the buffer size is %i ", sizeof(buffer));

            }

            // terminate
            fclose(mpFile);
            free(mp3_buffer);
        }

        return true;
    }
};

int main(void)
{
    //eint3_enable_port0(Place pin number for enter button here, eint_rising_edge,selectSong);
    //eint3_enable_port0(0, eint_rising_edge, buttonScrollDown);
    //eint3_enable_port0(1, eint_rising_edge, buttonScrollUp);
    //xQueueCreate();

//    resumeButtonSemphr = xSemaphoreCreateBinary();
//    nextSongButton = xSemaphoreCreateBinary();

    if (!getSongNames()) {
        u0_dbg_printf("Dir Read Failed");
    }
    resumeButtonSemaphore =xSemaphoreCreateMutex();
    scrollDown = xSemaphoreCreateBinary();
    Select = xSemaphoreCreateBinary();
//    scheduler_add_task(new terminalTask(PRIORITY_MEDIUM));
//    scheduler_add_task(new uart_display(PRIORITY_MEDIUM));
    scheduler_add_task(new controlPanel(PRIORITY_HIGH));

    scheduler_add_task(new mp3Project(PRIORITY_MEDIUM));
//    scheduler_add_task(new mp3);
    //xTaskCreate((TaskFunction_t )buttonScroll, "Button Interrupt", 100, NULL, 1, NULL);

    scheduler_start(); //< This shouldn't return
    //vTaskStartScheduler();

    return -1;
}
