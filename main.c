// Lab 4, uP2 Fall 2023
// Created: 2023-07-31
// Updated: 2023-08-01
// Lab 4 is intended to introduce you to more advanced RTOS concepts. In this, you will
// - implement blocking, yielding, sleeping
// - Thread priorities, aperiodic & periodic threads
// - IPC using FIFOs
// - Dynamic thread creation & deletion

/************************************Includes***************************************/

#include "G8RTOS/G8RTOS.h"
#include "./MultimodDrivers/multimod.h"

#include "./threads.h"

/************************************Includes***************************************/

/*************************************Defines***************************************/
/*************************************Defines***************************************/

/********************************Public Variables***********************************/
/********************************Public Variables***********************************/

/********************************Public Functions***********************************/



/********************************Public Functions***********************************/

/************************************MAIN*******************************************/

int main(void)
{
    //
    //frog.y_pos = 0;
    frog.length = 20;
    frog.width = 20;
    frog.pos_count = 0;
    uint16_t x_pos = 120;
    uint16_t y_pos = 0;
    Character_AddPosition(x_pos, y_pos);

    obs.length = 20;
    obs.width = 40;
    obs.pos_count = 0;
    obs.partial = obs.width;
    x_pos = 0;
    y_pos = 20;
    Obstacle_AddPosition(x_pos, y_pos);

    // Sets clock speed to 80 MHz. You'll need it!

    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    G8RTOS_Init();
    multimod_init();

    G8RTOS_InitSemaphore(&sem_UART, 1);
    G8RTOS_InitSemaphore(&sem_I2CA, 1);
    G8RTOS_InitSemaphore(&sem_SPIA, 1);
    G8RTOS_InitSemaphore(&sem_PCA9555_Debounce, 0);
    G8RTOS_InitSemaphore(&sem_Joystick_Debounce, 0);
    G8RTOS_InitSemaphore(&sem_KillCube, 1);

    G8RTOS_InitFIFO(SPAWNCOOR_FIFO);
    G8RTOS_InitFIFO(JOYSTICK_FIFO);
    G8RTOS_InitFIFO(CHAR_POS_FIFO);


    //G8RTOS_AddThread(task1, 200, "idle\0");
    G8RTOS_AddThread(Idle_Thread, 255, "idle\0");
    //G8RTOS_AddThread(CamMove_Thread, 253, "camera\0");
    G8RTOS_AddThread(Read_Buttons, 252, "buttons\0");
    G8RTOS_AddThread(Read_JoystickPress, 252, "joystick_s\0");
    G8RTOS_AddThread(Game_Thread, 4, "game_thread\0");
    //G8RTOS_AddThread(LED_Thread, 254, "threads\0");
    

    G8RTOS_Add_APeriodicEvent(GPIOE_Handler, 5, 20);
    G8RTOS_Add_APeriodicEvent(GPIOD_Handler, 5, 19);

    //G8RTOS_Add_PeriodicEvent(Print_WorldCoords, 100, 2);
    G8RTOS_Add_PeriodicEvent(Get_Joystick, 50, 2);
    G8RTOS_Add_PeriodicEvent(Draw_Display, 100, 4);

    InitWorld();
    G8RTOS_Launch();
    while (1);

}

/************************************MAIN*******************************************/
