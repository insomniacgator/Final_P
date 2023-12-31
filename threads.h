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
#define CHAR_POS_FIFO       2

#define MAX_NUM_POS         150

/*************************************Defines***************************************/

/***********************************Semaphores**************************************/

semaphore_t sem_I2CA;
semaphore_t sem_SPIA;
semaphore_t sem_PCA9555_Debounce;
semaphore_t sem_Joystick_Debounce;
semaphore_t sem_KillCube;
semaphore_t sem_UART;
semaphore_t sem_attached;

/***********************************Semaphores**************************************/

/***********************************Structures**************************************/
typedef struct position_t
{
    uint16_t x_pos;
    uint16_t y_pos;
    bool     draw_done;
    bool     not_empty;

    struct position_t *next_pos;
    struct position_t *previous_pos;
} position_t;


// Character
typedef struct character_t
{
    //uint16_t x_pos;
    //uint16_t y_pos;
    struct position_t positions[MAX_NUM_POS];
    struct position_t *head_pos;
    struct position_t *tail_pos;
    uint16_t pos_count;
    uint8_t width;
    uint8_t length;
    uint8_t attached;

} character_t; // should I make a linked list of the positions, or add a var
               // for the old position, so that I have the old position to erase?

// Obstacle
typedef struct obstacle_t
{
    struct position_t positions[10];
    struct position_t *head_pos;
    struct position_t *tail_pos;
    uint16_t pos_count;
    uint8_t width;
    uint8_t length;
    uint8_t partial;
    uint8_t pos_read_index;
    uint8_t pos_write_index;

} obstacle_t;

// Board
/*
typedef struct board_t
{
    uint8_t grid[13][12];
};*/

uint8_t board[13][12]; // we could initialize this in main?
character_t frog;
obstacle_t obs;
obstacle_t obs2;
obstacle_t obs3;
obstacle_t obs4;
obstacle_t obs5;
obstacle_t wood;
obstacle_t wood2;
obstacle_t wood3;
obstacle_t wood4;
obstacle_t wood5;

/***********************************Structures**************************************/


/*******************************Background Threads**********************************/

void Idle_Thread(void);
void Cube_Thread(void);
void CamMove_Thread(void);
void Read_Buttons(void);
void Read_JoystickPress(void);

void LED_Thread(void);

void Game_Thread(void);
void check_win_lose(void);
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

/****** Functions ******/
void InitWorld(void);
void Character_AddPosition(uint16_t x_pos, uint16_t y_pos);
void Obstacle_AddPosition(uint16_t x_pos, uint16_t y_pos, struct obstacle_t *obs_add);
#endif /* THREADS_H_ */

