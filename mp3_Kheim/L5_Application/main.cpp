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

#define BLOCK_SIZE          512
#define MUSIC_QUEUE_SIZE    4
#define MAX_SONGS           100
#define MAX_SONG_LENGTH     13

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
#define stopText()    U2_Tx(0);
#define sendText()     U2_Tx('T');\
                          U2_Tx('T');
#define setPosition() U2_Tx('T');\
                       U2_Tx('P');

SemaphoreHandle_t scrollDown, Select;
QueueHandle_t selectedFilename;
char musicList[200][100] = { NULL };
//vector<char*> musicList;

int numberOfSongs = 0;
uint8_t currentSong = 0;
int checkSong =0;
uint8_t currentVolumn = 50;       //0 is max, fffe is muted
uint8_t volumnDisplay = 9;
bool pausedFlag = true;         // the pausedFlage is used to display the lcd status
bool reStartFlag = false;

typedef enum {
    invalid = 1, left = 2, right = 3
} orientation_t;
orientation_t orien;

typedef enum {
    volumnQueue, song_queue, tilt_queue //Add by Khiem
} sharedHandleId_t;

QueueHandle_t mysongQueue = xQueueCreate(1, sizeof(int));

bool getSongNames()
{
    DIR Dir;
    FILINFO Finfo;
    FATFS *fs;
    FRESULT returnCode = FR_OK;
    char Lfname[_MAX_LFN];
    int order = 0;
    char* name;
    const char *dirPath = "1:Music";
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
        //name = &(Finfo.lfname[0]);
        //u0_dbg_printf("%s\n", name);
        strcpy(musicList[order], &(Finfo.lfname[0]));
        //musicList.push_back(name);
        numberOfSongs++;
        //u0_dbg_printf("%s\n", musicList[order]);
        // LFN names tend to increase memory requirements for output str, enable with caution
        order++;
        u0_dbg_printf("%i\n", numberOfSongs);
    }
    f_closedir(&Dir);
    return true;

}

class uart_display: public scheduler_task {
public:

    uart_display(uint8_t priority) :
            scheduler_task("Display", 2000, priority)
    {
    }
    typedef enum {
            flashFirst, flashSecond, goingUp, goingDown,
        } displayState;
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
        stopText()
        ;
    }

    void sendLine(char* song)
    {
        char listend[13] = { "End Of List " };

        if (song[0] != NULL && song[1] != NULL) {
            sendText()
            ;
            for (int i = 0; i < 16; i++) {

                if (song[i] != NULL) {
                    U2_Tx(song[i]); //send values in array one at a time
                }
                else {
                    break;
                }
            }
            stopText()
            ;
        }
        else {
            sendLine(&listend[0]);

        }

    }

    void slowStepLine(char* song)
    {
        sendText()
        ;
        for (int i = 0; i < 16; i++) {
            if (song[i] != NULL) {
                vTaskDelay(100);
                U2_Tx(song[i]); //send values in array one at a time
            }
            else {
                break;
            }
        }
        stopText()
        ;
    }

    bool run(void *p)
    {
        // char* endofList[12] = { "End Of List" };
        //Initial the baud rate to 9600 bps
        U2_init(9600); //is the baud rate for the display 9600?
        vTaskDelay(500);
        stopText()
        ;
        char* songPointer0;
        char* songPointer1;
//        int songSelector = 0;
        int listNum = currentSong;
//        int count = 0;
        displayState state = flashFirst;

        vTaskDelay(5);

        clearLCD()
        ;

        setCursorOFF()
        ;

        vTaskDelay(10);

        setBacklightON()
        ;

        vTaskDelay(10);

        songPointer0 = &musicList[listNum][0];
//        sendLine(songPointer0);
        listNum++;

        songPointer1 = &musicList[listNum][0];
//        sendLine(songPointer1);

//        bool step = true;
        while(1) {
            switch (state) {
                       case flashFirst:
           //                int temp = currentSong;
                           clearFirstLine();
                           setCursorPosition(0, 0);
                           slowStepLine(musicList[currentSong]);
//                           u0_dbg_printf("stepping first song\n");
//                           vTaskDelay(1000);
                           if ((currentSong % 2 == 1) && (currentSong > checkSong)) {
                               state = flashSecond;
                               checkSong ++;
                           }
                           else if ((currentSong % 2 == 1) && (currentSong < checkSong)) {
                               state = goingUp;
                               checkSong --;
                           }
                           break;
                       case flashSecond:
           //                int temp = currentSong;
                           clearSecondLine();
                           setCursorPosition(0, 1);
                           slowStepLine(musicList[currentSong]);
//                           u0_dbg_printf("stepping second song\n");
                           if ((currentSong % 2 == 0) && (currentSong > checkSong)) {
                               state = goingDown;
                               checkSong = currentSong;
                           }
                           else if ((currentSong % 2 == 0) && (currentSong < checkSong)) {
                               state = flashFirst;
                               checkSong--;
                           }
                           break;
                       case goingDown:
                           if (currentSong < numberOfSongs) {
                               listNum = currentSong;
                               songPointer0 = &musicList[listNum][0];
                               sendLine(songPointer0);
                               u0_dbg_printf("%s\n", songPointer0);
                           }
                           listNum++;
                           songPointer1 = &musicList[listNum][0];
                           sendLine(songPointer1);
                           u0_dbg_printf("%s\n", songPointer1);
                           state = flashFirst;
                           break;

                       case goingUp:
                           if (currentSong > 0) {
                               listNum = currentSong - 1;
                               songPointer0 = &musicList[listNum][0];
                               sendLine(songPointer0);
                               u0_dbg_printf("%s\n", songPointer0);
                           }
                           listNum++;
                           songPointer1 = &musicList[listNum][0];
                           sendLine(songPointer1);
                           u0_dbg_printf("%s\n", songPointer1);
                           state = flashSecond;

                           break;

                   }
//            while (step) {
//
//                songSelector = currentSong % 2;
//                switch (songSelector) {
//                    case 0:
//                        clearFirstLine();
//                        setCursorPosition(0, 0);
//
//                        slowStepLine(musicList[currentSong]);
//                        u0_dbg_printf("stepping first song\n");
//                        vTaskDelay(1000);
//                        break;
//                    case 1:
//                        clearSecondLine();
//                        setCursorPosition(0, 1);
//                        slowStepLine(musicList[currentSong]);
//                        u0_dbg_printf("stepping second song\n");
//                        if ((currentSong % 3) == 2) {
//                            step = false;
//                        }
//                        vTaskDelay(1000);
//                        break;
//                    default:
//                        break;
//                }
//            }
//            if (currentSong < numberOfSongs) {
//                listNum = currentSong;
//                songPointer0 = &musicList[listNum][0];
//                sendLine(songPointer0);
//                u0_dbg_printf("%s\n", songPointer0);
//
//            }
//            listNum++;
//            songPointer1 = &musicList[listNum][0];
//            sendLine(songPointer1);
//            u0_dbg_printf("%s\n", songPointer1);
//            step = true;

        }
        return -1;

    }
}
;

SemaphoreHandle_t resumeButtonSemaphore = NULL;
SemaphoreHandle_t displayScreenSemaphore = NULL;

class controlPanel: public scheduler_task {
public:
    controlPanel(uint8_t priority) :
            scheduler_task("controlPanel", 4096, priority)
    {
        QueueHandle_t myVolumnQueue = xQueueCreate(3, sizeof(int));
        addSharedObject(volumnQueue, myVolumnQueue);

        //Add by Khiem
        addSharedObject(song_queue, mysongQueue);

        //add extra feature by Khiem
        QueueHandle_t q = xQueueCreate(1, sizeof(orientation_t));
        addSharedObject(tilt_queue, q);
    }

    void PauseMusic()
    {
        pausedFlag = true;
        reStartFlag = false;
        //long pauseSem = 0;
        scheduler_task *musicTask = scheduler_task::getTaskPtrByName("musicPlayer");
        vTaskSuspend(musicTask->getTaskHandle());

//        scheduler_task *LcdTask = scheduler_task::getTaskPtrByName("LcdDisplay");
//        vTaskResume(LcdTask->getTaskHandle());
//         xSemaphoreGiveFromISR(displayScreenSemaphore, &pauseSem);
//        if(pauseSem){
//            portYIELD_FROM_ISR(pauseSem);
//        }
    }

    void ResumePlayMusic()
    {
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
    void startMusic()
    {
        //  reStartFlag = false;

        // pausedFlag = false;

        scheduler_task *Musictask = scheduler_task::getTaskPtrByName("musicPlayer");
        vTaskResume(Musictask->getTaskHandle());

//        xQueueSend(getSharedObject(song_queue), &currentSong,0);//add by Khiem
//      long startPlay = 0;
//      xSemaphoreGiveFromISR(resumeButtonSemaphore, &startPlay);
//
//      if (startPlay) {
//          portYIELD_FROM_ISR(startPlay);
//      }
    }
    void selectNextSong()
    {     //not work yet

//        PauseMusic();
//        pausedFlag = true;
        // reStartFlag = false;

//      scheduler_task *sendmusicTask = scheduler_task::getTaskPtrByName("sendMusicTask");
//      vTaskSuspend(sendmusicTask->getTaskHandle());
//    xQueueReset(getSharedObject(song_queue));
        if (currentSong < numberOfSongs) {
            currentSong++;
            u0_dbg_printf("current song inc\n");
        }
        else {
            u0_dbg_printf("end of list\n");
        }
        vTaskDelay(200);
//      vTaskResume(sendmusicTask->getTaskHandle());

//u0_dbg_printf("current song number is %d\t in selectSong\n",currentSong);//Add by Khiem
//      long startPlay = 0;
//scheduler_task *Musictask = scheduler_task::getTaskPtrByName("musicPlayer");
//      vTaskResume(Musictask->getTaskHandle());

//      xSemaphoreGiveFromISR(resumeButtonSemaphore, &startPlay);
//      if (startPlay) {
//          portYIELD_FROM_ISR(startPlay);
//          u0_dbg_printf("Song has been incremented");
//      }

    }

    void selectPrevSong()
    {       // not work yet

//      reStartFlag = true;
//      pausedFlag = true;
//      PauseMusic();
        if (currentSong <= 0) printf("Invalid Song\n");
        else
            currentSong--;

    }
    void volumnUp()
    {
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
    void volumnDown()
    {
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

    bool init(void)
    {
        //configure the pin p0.0. p0.1 as gpio
        LPC_PINCON->PINSEL0 &= ~(3 << 0);   // set the pin p0.0 as gpio
        LPC_PINCON->PINSEL0 &= ~(3 << 2);   // set the pin p0.1 as gpio

        LPC_GPIO0->FIODIR &= ~(1 << 0); // set the port p0.0 as input
        LPC_GPIO0->FIODIR &= ~(1 << 1); // set the port p0.1 as input

        LPC_PINCON->PINSEL4 &= ~(3 << 0);   // set the pin p2.0 as gpio
        LPC_PINCON->PINSEL4 &= ~(3 << 2);   // set the pin p2.1 as gpio
        LPC_PINCON->PINSEL4 &= ~(3 << 4);   // set the pin p2.2 as gpio
        LPC_PINCON->PINSEL4 &= ~(3 << 6);   // set the pin p2.3 as gpio

        LPC_GPIO2->FIODIR &= ~(1 << 0); // set the port p2.0 as input
        LPC_GPIO2->FIODIR &= ~(1 << 1); // set the port p2.1 as input
        LPC_GPIO2->FIODIR &= ~(1 << 2); // set the port p2.2 as input
        LPC_GPIO2->FIODIR &= ~(1 << 3); // set the port p2.3 as input
        //use to clear all the interrupts
        LPC_GPIOINT->IO0IntClr = 0xFFFFFFFF;
//      NVIC_EnableIRQ(EINT3_IRQn);
        // isr_register(EINT3_IRQn, EINT3_IRQHandler);
        resumeButtonSemaphore = xSemaphoreCreateBinary(); // to create the semaphore
//        displayScreenSemaphore = xSemaphoreCreateBinary();

        //upDataSongNameList();
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

        if (SW.getSwitch(4)) {        // Button 1 for play and resume the music
//            volumnDown();
//          selectNextSong();
//          long startPlay=0;
//          xSemaphoreGiveFromISR(resumeButtonSemaphore, &startPlay);
//          currentSong++;

            //xSemaphoreGive(resumeButtonSemaphore);
            selectNextSong();

//                  if(startPlay)
//                  {
//                  portYIELD_FROM_ISR(startPlay);
//                  }

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
//                              {
//                              portYIELD_FROM_ISR(startPlay);
//                              }

        }
//        else if((LPC_GPIO0->FIOPIN & (1 << 0))) { // p0.0 -->select the next song
//            selectNextSong();
//
//        }
//        else if((LPC_GPIO0->FIOPIN & (1 << 1))||(xQueueReceive(getSharedObject(tilt_queue), &orien, portMAX_DELAY)!=invalid)) { // p0.1--> select to previous song
////          selectPrevSong();
//            volumnDown();
//            u0_dbg_printf("volume decreasing\n");
//
//        }
//        else if((LPC_GPIO2->FIOPIN & (1 << 0))||(xQueueReceive(getSharedObject(tilt_queue), &orien, portMAX_DELAY)==invalid)) {  // p2.0, to set volumn up
////          ||((x>-30)&&(x<-15)&&(y<1045)&&(y>75)&&(z<1025)&&(z>7))
//            volumnUp();
//            u0_dbg_printf("volume increasing\n");
////          u0_dbg_printf("x=%d\n",x);
////          u0_dbg_printf("y=%d\n",y);
////          u0_dbg_printf("z=%d\n",z);
//        }
//        else if((LPC_GPIO2->FIOPIN & (1 << 2))) {   // p2.2, to start music
////          volumnDown();
//            startMusic();
//            u0_dbg_printf("start music\n");
//            xSemaphoreGive(resumeButtonSemaphore);
//
//        }
//        else if((LPC_GPIO2->FIOPIN & (1 << 3))) { // p2.3 -->select the next song
//            PauseMusic();
//
//        }
        ////Add by Khiem
//       if (xSemaphoreTake(resumeButtonSemaphore, portMAX_DELAY))

//          else if (LPC_GPIO0->FIOPIN & 1<<0)
//          {
//              xSemaphoreTake(resumeButtonSemaphore, portMAX_DELAY);
//              {   u0_dbg_printf("Interrupt from port 0.0");
//              xQueueReset(getSharedObject(song_queue));
//              currentSong++;
//               xQueueSend(getSharedObject(song_queue), &currentSong,portMAX_DELAY);
////                    startMusic();
//              }
//          }
        else if((LPC_GPIO2->FIOPIN & (1 << 1)))
        {
//            u0_dbg_printf("Interrupt from port 2.1\n");
//                  xSemaphoreGive(resumeButtonSemaphore);

//            selectPrevSong();

        }
        vTaskDelay(100);
        return true;
    }
};

//Add interrupr by Khiem

LabGPIOInterrupt GPIOInt_instance;

void eint3_IRQHandle(void)            //Handle Interrupt
{
    GPIOInt_instance.handle_interrupt();
}

void callback_from_interrupt(void) //Call back function from interrupt
{
    long yield = 0;
    {
        xSemaphoreGiveFromISR(resumeButtonSemaphore, &yield);
        if (yield) {
            u0_dbg_printf("YIELD from ISR\n");
            portYIELD_FROM_ISR(yield);
        }
    }

}

///Add on Stanley code

//MP3 decoder interface
typedef struct {
    FIL* file;
    uint8_t data[BLOCK_SIZE];
    uint32_t fileSize;
    UINT position;
} musicBlock_t;

class sendMusic: public scheduler_task {
public:
    sendMusic(uint8_t priority) :
            scheduler_task("sendMusicTask", 2000, priority)
    {
    }
    bool init(void)
    {
        return true;
    }

    bool run(void *p)
    {

        FRESULT returnCode = FR_OK;

        xSemaphoreTake(resumeButtonSemaphore, portMAX_DELAY);

        char prefix[] = "1:Music/";
        char* songName = musicList[currentSong];

        u0_dbg_printf("currently playing:  %i: %s\n", currentSong, songName);
        char* fileName = (char*) malloc(strlen(prefix) + strlen(songName) + 1);
        strcpy(fileName, prefix);
        strcat(fileName, songName);

        u0_dbg_printf("File Location: %s\n", fileName);
        FIL file;
        if (FR_OK != (returnCode = f_open(&file, fileName, FA_READ))) {
            u0_dbg_printf("File Error %i", returnCode);
        }

        f_lseek(&file, 0);
        int fileSize = f_size(&file);
        f_lseek(&file, 0);
        u0_dbg_printf("size: %i\n", fileSize);

        //read in chunks of 512 bytes and send to queue
        musicBlock_t m;
        musicBlock_t* mPtr;
        for (int i = 0; i < fileSize / BLOCK_SIZE + 1; i++) {
            if (LPC_GPIO0->FIOPIN & (1 << 0) || (LPC_GPIO2->FIOPIN & (1 << 1))) break;
//            UINT bytesRead = 0;
            m.file = &file;
            m.fileSize = fileSize;
            // u0_dbg_printf("size: %i\n",m.fileSize);
            m.position = 0;
            f_read(m.file, m.data, BLOCK_SIZE, &m.position);

            //fread();
            mPtr = &m;
            xQueueSend(getSharedObject(song_queue), &mPtr, portMAX_DELAY);
            if (musicList[currentSong] != songName) {
                break;
            }
        }
        f_close(&file);

        return true;
    }

};

class mp3Project: public scheduler_task {
public:
    mp3Project(uint8_t priority) :
            scheduler_task("musicPlayer", 2000, priority)
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
        LPC_SC->PCLKSEL1 &= ~(3 << 10);     // Clear clock Bits
        LPC_SC->PCLKSEL1 |= (1 << 10);     // 01: CLK / 1. ||  00: /4    || 10: /2
        // Select MISO, MOSI, and SCK pin-select functionality
        LPC_PINCON->PINSEL0 &= ~((3 << 30));            // clear the sck0
        LPC_PINCON->PINSEL1 &= ~((3 << 2) | (3 << 4));            //  clear the miso0 and mosi0
        LPC_PINCON->PINSEL0 |= (2 << 30);            // set set the function for sck0
        LPC_PINCON->PINSEL1 |= ((2 << 2) | (2 << 4));            //and then set the function respectively
        LPC_SSP0->CR0 = 7;            // 8-bit mode
        LPC_SSP0->CR1 = (1 << 1);            // Enable SSP as Master
        LPC_SSP0->CPSR = 1;            // SCK speed = CPU / 1
        LPC_SSP0->CR1 &= ~(1 << 2);            // to set the spi1 as master
        // gpio initlization, configure the pin as gpio first
        LPC_PINCON->PINSEL3 &= ~(3 << 12);            // set p1.22 as gpio
        LPC_PINCON->PINSEL3 &= ~(3 << 24);            // set p1.28 as gpio
        LPC_PINCON->PINSEL3 &= ~(3 << 28);            //set p1.30  as gpio
        LPC_PINCON->PINSEL3 &= ~(3 << 6);            //set p1.19  as gpio
        //   LPC_PINCON->PINSEL0 &= ~(3 << 0);           // this is for sd car, dont real need it
        LPC_GPIO1->FIODIR |= (1 << 22);            //set pin P1.22 as output -- CS signal
        LPC_GPIO1->FIODIR |= (1 << 28);            //set pin P1.28 as output -- RESET signal
        LPC_GPIO1->FIODIR |= (1 << 30);            //set pin P1.30 as output -- DCS signal
        LPC_GPIO1->FIODIR &= ~(1 << 19);            //set pin P1.19 as input -- DREQ signal
        LPC_GPIO1->FIOSET = (1 << 28);            //set pin P1.28 high initially
        LPC_GPIO1->FIOSET = (1 << 30);            //set pin P1.30 high initially
        LPC_GPIO1->FIOSET = (1 << 22);            //set pin P1.22 high initially
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
        setCS(); //Send Low on CS to enable VS1053, active low
        sendDataToRegister(0x3); //Send Read Opcode
        sendDataToRegister(readReg); //Send Address to read from
        char firstByte = sendDataToRegister(0x99);
        while (!(LPC_GPIO1->FIOPIN & (1 << 19)))
            ;
        char secondByte = sendDataToRegister(0x98);
        while (!(LPC_GPIO1->FIOPIN & (1 << 19)))
            ;
        resetCS(); //Send High on CS to disable VS1053
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
        resetDataCS();            // set the pin to high
        setCS();            //enable the chip select, the cs is active low
        while (!(LPC_GPIO1->FIOPIN & (1 << 19)))
            ;
        sendDataToRegister(0x02);            //send the opcode fist, this is operation of write data
        sendDataToRegister(address);            // send the address
        sendDataToRegister(highByteData);
        sendDataToRegister(lowByteData);
        //setDataCS();
        while (!(LPC_GPIO1->FIOPIN & (1 << 19)))
            ;            // wait unitl the dreq is low
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
        setDataCS();       //enable DCS
        while (dreqStatus() == 0)
            ;       //wait until DREQ pin goes high
                    //    ssp0_exchange_byte(0x00);
        for (int i = 0; i < 32; i++) {
            ssp0_exchange_byte(data[i]);
        }
        resetDataCS();    //disable DCS
    }
    // bool updataSong() {}

    bool run(void *p)
    {

        /* We first get the queue handle the other task added using addSharedObject() */
        musicBlock_t* data;
        QueueHandle_t qid = getSharedObject(song_queue);

        uint16_t volumnChange;
        QueueHandle_t volumControl = getSharedObject(volumnQueue);
        //wait for queue to be not empty

        while (xQueueReceive(qid, &data, portMAX_DELAY)) {

            //get pointer to data chunk
            uint8_t* dataPtr = data->data;
            int z = 0;
            while (z < BLOCK_SIZE) {
                while (dreqStatus() == 0) {
                    if (xQueueReceive(volumControl, &volumnChange, 15)) {
                        setVolum(volumnChange);
                    }
                }
                //check dreq every 32 bytes
                resetCS();
                setDataCS();
                for (int i = 0; i < 32; i++) {
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

//Add extra feature
class tiltControl: public scheduler_task {
public:
    tiltControl(uint8_t priority) :
            scheduler_task("tiltControlTask", 4096, priority)
    {
    }

    void producer() /* LOW priority */
    {

        int x = AS.getX();    //Get the vertical orientation
        int y = AS.getY();//Get the horizontal orientation
        int z = AS.getZ();//Get the 3rd orientation
        u0_dbg_printf("\nX: %d", x);
        u0_dbg_printf("\nY: %d", y);
        u0_dbg_printf("\nZ: %d\n", z);

        if((y<1045)&&(y>75)&&(z<1025)&&(z>7))
        {
            orien= right;
        }
        else if(!((x>-30)&&(x<-15)&&(y<1045)&&(y>75)&&(z<1025)&&(z>7)))
        {
            orien= left;
        }
        else
        {
            orien=invalid;
        }

        u0_dbg_printf("Sending an orientation to the queue\n");
        xQueueSend(getSharedObject(tilt_queue), &orien, 500);

        vTaskDelay(1000);

    }
    bool run (void *p)
    {
        producer();
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

    for (int i = 0; i < numberOfSongs; i++) {
        u0_dbg_printf("%s\n", musicList[i]);
    }

    resumeButtonSemaphore = xSemaphoreCreateMutex();
    scrollDown = xSemaphoreCreateBinary();
    Select = xSemaphoreCreateBinary();
    scheduler_add_task(new terminalTask(PRIORITY_HIGH));
    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
    //  scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));
    scheduler_add_task(new mp3Project(2));
    //    scheduler_add_task(new diplayScreen(2));
    scheduler_add_task(new controlPanel(4));
    scheduler_add_task(new sendMusic(3));
//      scheduler_add_task(new tiltControl(1));
//    GPIOInt_instance.init();
//    GPIOInt_instance.attachInterruptHandler(0, 0, callback_from_interrupt, raising_edge);    //port 0.0
//    isr_register(EINT3_IRQn, eint3_IRQHandle);
//    GPIOInt_instance.attachInterruptHandler(2, 1, callback_from_interrupt, both_edge);    //port2.1
//    isr_register(EINT3_IRQn, eint3_IRQHandle);

    scheduler_add_task(new uart_display(4));

    //xTaskCreate((TaskFunction_t )buttonScroll, "Button Interrupt", 100, NULL, 1, NULL);

    scheduler_start();        //< This shouldn't return
    //vTaskStartScheduler();

    return -1;
}
