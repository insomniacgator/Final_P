// threads.h
// Date Created: 2023-07-26
// Date Updated: 2023-07-26
// Threads

#ifndef THREADS_H_
#define THREADS_H_

/************************************Includes***************************************/

#include "./G8RTOS/G8RTOS.h"

/************************************Includes***************************************/

/*************************************Defines***************************************/

#define SPAWNCOOR_FIFO      0
#define JOYSTICK_FIFO       1

#define MAX_NUM_POS         100

/*************************************Defines***************************************/

/***********************************Semaphores**************************************/

semaphore_t sem_I2CA;
semaphore_t sem_SPIA;
semaphore_t sem_PCA9555_Debounce;
semaphore_t sem_Joystick_Debounce;
semaphore_t sem_KillCube;
semaphore_t sem_UART;

/***********************************Semaphores**************************************/

/***********************************Structures**************************************/

// Character
typedef struct character_t
{
    struct position_t positions[MAX_NUM_POS];
    struct positon_t *head_pos;
    struct position_t *tail_pos;
    uint8_t width;
    uint8_t length;

} character_t; // should I make a linked list of the positions, or add a var
               // for the old position, so that I have the old position to erase?

// Obstacle
typedef struct obstacle_t
{
    struct position_t positions[MAX_NUM_POS];
    struct positon_t *head_pos;
    struct position_t *tail_pos;
    uint8_t width;
    uint8_t length;

} obstacle_t;

typedef struct position_t
{
    uint16_t x_pos;
    uint16_t y_pos;
    struct position_t *next_pos;
    struct position_t *previous_pos;
};


character_t frog;
obstacle_t obs;

/***********************************Structures**************************************/


/*******************************Background Threads**********************************/

void Idle_Thread(void);
void Cube_Thread(void);
void CamMove_Thread(void);
void Read_Buttons(void);
void Read_JoystickPress(void);

void LED_Thread(void);

void CharacterMove_Thread(void);
/*******************************Background Threads**********************************/

/********************************Periodic Threads***********************************/

void Print_WorldCoords(void);
void Get_Joystick(void);
void Draw_Display(void);

/********************************Periodic Threads***********************************/

/*******************************Aperiodic Threads***********************************/

void GPIOE_Handler(void);
void GPIOD_Handler(void);

/*******************************Aperiodic Threads***********************************/
void InitWorld(void);

#endif /* THREADS_H_ */

