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
    obs.pos_write_index, obs.pos_read_index = 0;
    x_pos = 0;
    y_pos = 20;
    Obstacle_AddPosition(x_pos, y_pos, &obs);

    obs2.length = 20;
    obs2.width = 40;
    obs2.pos_count = 0;
    obs2.partial = obs2.width;
    obs2.pos_write_index, obs2.pos_read_index = 0;
    x_pos = 0;
    y_pos = 60;
    Obstacle_AddPosition(x_pos, y_pos, &obs2); // modify obstacle_addposition to include obs param

    obs3.length = 20;
    obs3.width = 40;
    obs3.pos_count = 0;
    obs3.partial = obs3.width;
    obs3.pos_write_index, obs3.pos_read_index = 0;
    x_pos = 0;
    y_pos = 100;
    Obstacle_AddPosition(x_pos, y_pos, &obs3); // modify obstacle_addposition to include obs param

    obs4.length = 20;
    obs4.width = 40;
    obs4.pos_count = 0;
    obs4.partial = obs4.width;
    obs4.pos_write_index, obs4.pos_read_index = 0;
    x_pos = 200;
    y_pos = 40;
    Obstacle_AddPosition(x_pos, y_pos, &obs4);

    obs5.length = 20;
    obs5.width = 40;
    obs5.pos_count = 0;
    obs5.partial = obs5.width;
    obs5.pos_write_index, obs5.pos_read_index = 0;
    x_pos = 200;
    y_pos = 80;
    Obstacle_AddPosition(x_pos, y_pos, &obs5);

    wood.length = 20;
    wood.width = 60;
    wood.pos_count = 0;
    wood.partial = wood.width;
    wood.pos_write_index, wood.pos_read_index = 0;
    x_pos = 0;
    y_pos = 140;
    Obstacle_AddPosition(x_pos, y_pos, &wood);

    wood2.length = 20;
    wood2.width = 80;
    wood2.pos_count = 0;
    wood2.partial = wood2.width;
    wood2.pos_write_index, wood2.pos_read_index = 0;
    x_pos = 0;
    y_pos = 200;
    Obstacle_AddPosition(x_pos, y_pos, &wood2);

    wood3.length = 20;
    wood3.width = 80;
    wood3.pos_count = 0;
    wood3.partial = wood3.width;
    wood3.pos_write_index, wood3.pos_read_index = 0;
    x_pos = 160;
    y_pos = 160;
    Obstacle_AddPosition(x_pos, y_pos, &wood3);

    wood4.length = 20;
    wood4.width = 60;
    wood4.pos_count = 0;
    wood4.partial = wood4.width;
    wood4.pos_write_index, wood4.pos_read_index = 0;
    x_pos = 140;
    y_pos = 180;
    Obstacle_AddPosition(x_pos, y_pos, &wood4);

    wood5.length = 20;
    wood5.width = 80;
    wood5.pos_count = 0;
    wood5.partial = wood5.width;
    wood5.pos_write_index, wood5.pos_read_index = 0;
    x_pos = 160;
    y_pos = 220;
    Obstacle_AddPosition(x_pos, y_pos, &wood5);

    // Sets clock speed to 80 MHz. You'll need it!

    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    G8RTOS_Init();
    multimod_init();

    G8RTOS_InitSemaphore(&sem_UART, 1);
    G8RTOS_InitSemaphore(&sem_I2CA, 1);
    G8RTOS_InitSemaphore(&sem_SPIA, 1);
    G8RTOS_InitSemaphore(&sem_PCA9555_Debounce, 0);
    G8RTOS_InitSemaphore(&sem_Joystick_Debounce, 0);
    //G8RTOS_InitSemaphore(&sem_KillCube, 1);
    G8RTOS_InitSemaphore(&sem_attached, 1);

    G8RTOS_InitFIFO(SPAWNCOOR_FIFO);
    G8RTOS_InitFIFO(JOYSTICK_FIFO);
    //G8RTOS_InitFIFO(CHAR_POS_FIFO);


    G8RTOS_AddThread(Idle_Thread, 255, "idle\0");
    G8RTOS_AddThread(check_win_lose, 253, "check_win_lose\0");
    //G8RTOS_AddThread(Read_Buttons, 252, "buttons\0");
    //G8RTOS_AddThread(Read_JoystickPress, 252, "joystick_s\0");
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
