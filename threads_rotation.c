// G8RTOS_Threads.c
// Date Created: 2023-07-25
// Date Updated: 2023-07-27
// Defines for thread functions.

/************************************Includes***************************************/

#include "./threads.h"

#include "./MultimodDrivers/multimod.h"
#include "./MiscFunctions/Shapes/inc/cube.h"
#include "./MiscFunctions/LinAlg/inc/linalg.h"
#include "./MiscFunctions/LinAlg/inc/quaternions.h"
#include "./MiscFunctions/LinAlg/inc/vect3d.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/************************************Includes***************************************/

/*************************************Defines***************************************/

// Change this to change the number of points that make up each line of a cube.
// Note that if you set this too high you will have a stack overflow!
// sizeof(float) * num_lines * (Num_Interpolated_Points + 2) = ?
#define Num_Interpolated_Points     3


/*************************************Defines***************************************/

/*********************************Global Variables**********************************/

Quat_t world_camera_pos = {0, 0, 0, 50};
Quat_t world_camera_frame_offset = {0, 0, 0, 50};
Quat_t world_camera_frame_rot_offset;
Quat_t world_view_rot = {1, 0, 0, 0};
Quat_t world_view_rot_inverse = {1, 0, 0, 0};

// How many cubes?
uint8_t num_cubes = 0;

// y-axis controls z or y
uint8_t joystick_y = 1;

uint8_t move_or_rot = 1;

// Kill a cube?
uint8_t kill_cube = 0;

/*********************************Global Variables**********************************/

/*************************************Threads***************************************/

void Idle_Thread(void) {
    time_t t;
    srand((unsigned) time(&t));
    while(1);
}



void Draw_Display(void)
{


        // position linked list is going to have a marker to indicate already drawn


    if (frog.pos_count) // only if we have 1 position in character position we start drawing everything?
    {
        uint16_t x_pos = frog.tail_pos->x_pos;
        uint16_t y_pos = frog.tail_pos->y_pos;
        uint16_t x_oldpos = frog.tail_pos->previous_pos->x_pos;
        uint16_t y_oldpos = frog.tail_pos->previous_pos->y_pos;

        uint16_t bg_color = 0;
        if (y_oldpos == 0 || y_oldpos == 120)
        {
            bg_color = 0b1100100000011000;
        }
        else if (y_oldpos >= 140 && y_oldpos < 240 )
        {
            bg_color = 0b1011000100100000;
        }
        else
        {
            bg_color = 0x0000;
        }


        // Character draw
        if ((x_pos || y_pos) && !frog.tail_pos->draw_done)
        {
            G8RTOS_WaitSemaphore(&sem_SPIA);
            ST7789_DrawRectangle(x_pos, y_pos, frog.width, frog.length, ST7789_WHITE);
            if (frog.pos_count > 1)
                ST7789_DrawRectangle(x_oldpos, y_oldpos, frog.width, frog.length, bg_color);
            //ST7789_DrawRectangle(frog.x_pos, frog.y_pos, frog.width, frog.length, ST7789_WHITE);
            G8RTOS_SignalSemaphore(&sem_SPIA);
            //frog.x_pos = x_pos;
            //frog.y_pos = y_pos;
            frog.tail_pos->draw_done = 1;
        }

        // Vehicle 1 Draw
        x_pos = obs.tail_pos->x_pos;
        y_pos = obs.tail_pos->y_pos;
        x_oldpos = obs.tail_pos->previous_pos->x_pos;
        y_oldpos = obs.tail_pos->previous_pos->y_pos;
        //obs.pos_read_index++;

        if ((x_pos || y_pos) && !obs.tail_pos->draw_done)
        {


            // if we reached wall we have to start disappearing
            if (x_pos >= 200)
            {
                if (obs.partial >= 2)
                {
                    //obs.partial = obs.partial - 2;
                    //UARTprintf("partial %d, x_pos: %d, y_pos: %d\n", obs.partial, x_pos, y_pos);
                    G8RTOS_WaitSemaphore(&sem_SPIA);
                    ST7789_DrawRectangle(x_pos, y_pos, obs.partial, obs.length, ST7789_YELLOW);
                    ST7789_DrawRectangle(x_pos-2, y_pos, 4, obs.length, ST7789_BLACK);
                    G8RTOS_SignalSemaphore(&sem_SPIA);
                }
                else
                {
                    //obs.partial = 0;
                    //Obstacle_AddPosition(0, 20);
                }
            }
            else
            {
                G8RTOS_WaitSemaphore(&sem_SPIA);
                ST7789_DrawRectangle(x_pos, y_pos, obs.width, obs.length, ST7789_YELLOW);
                ST7789_DrawRectangle(x_pos-2, y_pos, 4, obs.length, ST7789_BLACK);
                G8RTOS_SignalSemaphore(&sem_SPIA);
            }

            obs.tail_pos->draw_done = 1;
        }

        // Vehicle 2 Draw
        x_pos = obs2.tail_pos->x_pos;
        y_pos = obs2.tail_pos->y_pos;
        x_oldpos = obs2.tail_pos->previous_pos->x_pos;
        y_oldpos = obs2.tail_pos->previous_pos->y_pos;
        //obs.pos_read_index++;

        if ((x_pos || y_pos) && !obs2.tail_pos->draw_done)
        {


            // if we reached wall we have to start disappearing
            if (x_pos >= 200)
            {
                if (1)//(obs2.partial >= 4)
                {
                    //obs.partial = obs.partial - 2;
                    //UARTprintf("partial %d, x_pos: %d, y_pos: %d\n", obs2.partial, x_pos, y_pos);
                    G8RTOS_WaitSemaphore(&sem_SPIA);
                    ST7789_DrawRectangle(x_pos, y_pos, obs2.partial, obs2.length, ST7789_YELLOW);
                    ST7789_DrawRectangle(x_pos-4, y_pos, 8, obs2.length, ST7789_BLACK);
                    G8RTOS_SignalSemaphore(&sem_SPIA);
                }
                else
                {
                    //obs.partial = 0;
                    //Obstacle_AddPosition(0, 20);
                }
            }
            else
            {
                G8RTOS_WaitSemaphore(&sem_SPIA);
                ST7789_DrawRectangle(x_pos, y_pos, obs2.width, obs2.length, ST7789_YELLOW);
                ST7789_DrawRectangle(x_pos-4, y_pos, 8, obs2.length, ST7789_BLACK);
                G8RTOS_SignalSemaphore(&sem_SPIA);
            }

            obs2.tail_pos->draw_done = 1;
        }

        // Vehicle 3 Draw
        x_pos = obs3.tail_pos->x_pos;
        y_pos = obs3.tail_pos->y_pos;
        x_oldpos = obs3.tail_pos->previous_pos->x_pos;
        y_oldpos = obs3.tail_pos->previous_pos->y_pos;
        //obs.pos_read_index++;

        if ((x_pos || y_pos) && !obs3.tail_pos->draw_done)
        {


            // if we reached wall we have to start disappearing
            if (x_pos >= 200)
            {
                if (1)
                {
                    //obs.partial = obs.partial - 2;
                    //UARTprintf("partial %d, x_pos: %d, y_pos: %d\n", obs3.partial, x_pos, y_pos);
                    G8RTOS_WaitSemaphore(&sem_SPIA);
                    ST7789_DrawRectangle(x_pos, y_pos, obs3.partial, obs3.length, ST7789_YELLOW);
                    ST7789_DrawRectangle(x_pos-2, y_pos, 4, obs3.length, ST7789_BLACK);
                    G8RTOS_SignalSemaphore(&sem_SPIA);
                }
                else
                {
                    //obs.partial = 0;
                    //Obstacle_AddPosition(0, 20);
                }
            }
            else
            {
                G8RTOS_WaitSemaphore(&sem_SPIA);
                ST7789_DrawRectangle(x_pos, y_pos, obs3.width, obs3.length, ST7789_YELLOW);
                ST7789_DrawRectangle(x_pos-2, y_pos, 4, obs3.length, ST7789_BLACK);
                G8RTOS_SignalSemaphore(&sem_SPIA);
            }

            obs3.tail_pos->draw_done = 1;
        }

        // Vehicle 4 Draw
        x_pos = obs4.tail_pos->x_pos;
        y_pos = obs4.tail_pos->y_pos;
        x_oldpos = obs4.tail_pos->previous_pos->x_pos;
        y_oldpos = obs4.tail_pos->previous_pos->y_pos;
        //obs.pos_read_index++;

        if ((x_pos || y_pos) && !obs4.tail_pos->draw_done)
        {


            // if we reached wall we have to start disappearing
            if (x_pos <= 2)
            {
                if (1)
                {
                    //obs.partial = obs.partial - 2;
                    //UARTprintf("partial %d, x_pos: %d, y_pos: %d\n", obs4.partial, x_pos, y_pos);
                    G8RTOS_WaitSemaphore(&sem_SPIA);
                    ST7789_DrawRectangle(x_pos, y_pos, obs4.partial, obs4.length, ST7789_ORANGE);
                    ST7789_DrawRectangle(x_pos+obs4.partial, y_pos, 10, obs4.length, ST7789_BLACK);
                    G8RTOS_SignalSemaphore(&sem_SPIA);
                }
                else
                {
                    //obs.partial = 0;
                    //Obstacle_AddPosition(0, 20);
                }
            }
            else
            {
                G8RTOS_WaitSemaphore(&sem_SPIA);
                ST7789_DrawRectangle(x_pos, y_pos, obs4.width, obs4.length, ST7789_ORANGE);
                ST7789_DrawRectangle(x_pos+obs4.width, y_pos, 4, obs4.length, ST7789_BLACK);
                G8RTOS_SignalSemaphore(&sem_SPIA);
            }

            obs4.tail_pos->draw_done = 1;
        }

        // Vehicle 5 Draw
        x_pos = obs5.tail_pos->x_pos;
        y_pos = obs5.tail_pos->y_pos;
        x_oldpos = obs5.tail_pos->previous_pos->x_pos;
        y_oldpos = obs5.tail_pos->previous_pos->y_pos;
        //obs.pos_read_index++;

        if ((x_pos || y_pos) && !obs5.tail_pos->draw_done)
        {


            // if we reached wall we have to start disappearing
            if (x_pos <= 2)
            {
                if (1)
                {
                    //obs.partial = obs.partial - 2;
                    //UARTprintf("partial %d, x_pos: %d, y_pos: %d\n", obs5.partial, x_pos, y_pos);
                    G8RTOS_WaitSemaphore(&sem_SPIA);
                    ST7789_DrawRectangle(x_pos, y_pos, obs5.partial, obs5.length, ST7789_ORANGE);
                    ST7789_DrawRectangle(x_pos+obs5.partial, y_pos, 12, obs5.length, ST7789_BLACK);
                    G8RTOS_SignalSemaphore(&sem_SPIA);
                }
                else
                {
                    //obs.partial = 0;
                    //Obstacle_AddPosition(0, 20);
                }
            }
            else
            {
                G8RTOS_WaitSemaphore(&sem_SPIA);
                ST7789_DrawRectangle(x_pos, y_pos, obs5.width, obs5.length, ST7789_ORANGE);
                ST7789_DrawRectangle(x_pos+obs5.width-2, y_pos, 12, obs5.length, ST7789_BLACK);
                G8RTOS_SignalSemaphore(&sem_SPIA);
            }

            obs5.tail_pos->draw_done = 1;
        }

        // wood 1 Draw
        x_pos = wood.tail_pos->x_pos;
        y_pos = wood.tail_pos->y_pos;
        x_oldpos = wood.tail_pos->previous_pos->x_pos;
        y_oldpos = wood.tail_pos->previous_pos->y_pos;
        //obs.pos_read_index++;

        if ((x_pos || y_pos) && !wood.tail_pos->draw_done)
        {


            // if we reached wall we have to start disappearing
            if (x_pos >= 200)
            {
                if (1)//(obs2.partial >= 4)
                {
                    //obs.partial = obs.partial - 2;
                    //UARTprintf("partial %d, x_pos: %d, y_pos: %d\n", obs2.partial, x_pos, y_pos);
                    G8RTOS_WaitSemaphore(&sem_SPIA);
                    ST7789_DrawRectangle(x_pos, y_pos, wood.partial, wood.length, ST7789_BROWN);
                    ST7789_DrawRectangle(x_pos-4, y_pos, 8, wood.length, ST7789_BLUE);
                    G8RTOS_SignalSemaphore(&sem_SPIA);
                }
                else
                {
                    //obs.partial = 0;
                    //Obstacle_AddPosition(0, 20);
                }
            }
            else
            {
                G8RTOS_WaitSemaphore(&sem_SPIA);
                ST7789_DrawRectangle(x_pos, y_pos, wood.width, wood.length, ST7789_BROWN);
                ST7789_DrawRectangle(x_pos-4, y_pos, 8, wood.length, ST7789_BLUE);
                G8RTOS_SignalSemaphore(&sem_SPIA);
            }

            wood.tail_pos->draw_done = 1;
        }

        // wood 2 Draw
        x_pos = wood2.tail_pos->x_pos;
        y_pos = wood2.tail_pos->y_pos;
        x_oldpos = wood2.tail_pos->previous_pos->x_pos;
        y_oldpos = wood2.tail_pos->previous_pos->y_pos;
        //obs.pos_read_index++;

        if ((x_pos || y_pos) && !wood2.tail_pos->draw_done)
        {


            // if we reached wall we have to start disappearing
            if (x_pos >= 200)
            {
                if (1)//(obs2.partial >= 4)
                {
                    //obs.partial = obs.partial - 2;
                    //UARTprintf("partial %d, x_pos: %d, y_pos: %d\n", obs2.partial, x_pos, y_pos);
                    G8RTOS_WaitSemaphore(&sem_SPIA);
                    ST7789_DrawRectangle(x_pos, y_pos, wood2.partial, wood2.length, ST7789_BROWN);
                    ST7789_DrawRectangle(x_pos-2, y_pos, 4, wood2.length, ST7789_BLUE);
                    G8RTOS_SignalSemaphore(&sem_SPIA);
                }
                else
                {
                    //obs.partial = 0;
                    //Obstacle_AddPosition(0, 20);
                }
            }
            else
            {
                G8RTOS_WaitSemaphore(&sem_SPIA);
                ST7789_DrawRectangle(x_pos, y_pos, wood2.width, wood2.length, ST7789_BROWN);
                ST7789_DrawRectangle(x_pos-2, y_pos, 4, wood2.length, ST7789_BLUE);
                G8RTOS_SignalSemaphore(&sem_SPIA);
            }

            wood2.tail_pos->draw_done = 1;
        }


        // Wood 3 Draw
        x_pos = wood3.tail_pos->x_pos;
        y_pos = wood3.tail_pos->y_pos;
        x_oldpos = wood3.tail_pos->previous_pos->x_pos;
        y_oldpos = wood3.tail_pos->previous_pos->y_pos;
        //obs.pos_read_index++;

        if ((x_pos || y_pos) && !wood3.tail_pos->draw_done)
        {


            // if we reached wall we have to start disappearing
            if (x_pos <= 2)
            {
                if (1)
                {
                    //obs.partial = obs.partial - 2;
                    UARTprintf("partial %d, x_pos: %d, y_pos: %d\n", wood3.partial, x_pos, y_pos);
                    G8RTOS_WaitSemaphore(&sem_SPIA);
                    ST7789_DrawRectangle(x_pos, y_pos, wood3.partial, wood3.length, ST7789_BROWN);
                    ST7789_DrawRectangle(x_pos+wood3.partial, y_pos, 12, wood3.length, ST7789_BLUE);
                    G8RTOS_SignalSemaphore(&sem_SPIA);
                }
                else
                {
                    //obs.partial = 0;
                    //Obstacle_AddPosition(0, 20);
                }
            }
            else
            {
                G8RTOS_WaitSemaphore(&sem_SPIA);
                ST7789_DrawRectangle(x_pos, y_pos, wood3.width, wood3.length, ST7789_BROWN);
                ST7789_DrawRectangle(x_pos+wood3.width-2, y_pos, 12, wood3.length, ST7789_BLUE);
                G8RTOS_SignalSemaphore(&sem_SPIA);
            }

            wood3.tail_pos->draw_done = 1;
        }

        // Wood 4 Draw
        x_pos = wood4.tail_pos->x_pos;
        y_pos = wood4.tail_pos->y_pos;
        x_oldpos = wood4.tail_pos->previous_pos->x_pos;
        y_oldpos = wood4.tail_pos->previous_pos->y_pos;
        //obs.pos_read_index++;

        if ((x_pos || y_pos) && !wood4.tail_pos->draw_done)
        {


            // if we reached wall we have to start disappearing
            if (x_pos <= 2)
            {
                if (1)
                {
                    //obs.partial = obs.partial - 2;
                    UARTprintf("partial %d, x_pos: %d, y_pos: %d\n", wood4.partial, x_pos, y_pos);
                    G8RTOS_WaitSemaphore(&sem_SPIA);
                    ST7789_DrawRectangle(x_pos, y_pos, wood4.partial, wood4.length, ST7789_BROWN);
                    ST7789_DrawRectangle(x_pos+wood4.partial, y_pos, 12, wood4.length, ST7789_BLUE);
                    G8RTOS_SignalSemaphore(&sem_SPIA);
                }
                else
                {
                    //obs.partial = 0;
                    //Obstacle_AddPosition(0, 20);
                }
            }
            else
            {
                G8RTOS_WaitSemaphore(&sem_SPIA);
                ST7789_DrawRectangle(x_pos, y_pos, wood4.width, wood4.length, ST7789_BROWN);
                ST7789_DrawRectangle(x_pos+wood4.width-2, y_pos, 12, wood4.length, ST7789_BLUE);
                G8RTOS_SignalSemaphore(&sem_SPIA);
            }

            wood4.tail_pos->draw_done = 1;
        }


        // Wood 5 Draw
        x_pos = wood5.tail_pos->x_pos;
        y_pos = wood5.tail_pos->y_pos;
        x_oldpos = wood5.tail_pos->previous_pos->x_pos;
        y_oldpos = wood5.tail_pos->previous_pos->y_pos;
        //obs.pos_read_index++;

        if ((x_pos || y_pos) && !wood5.tail_pos->draw_done)
        {


            // if we reached wall we have to start disappearing
            if (x_pos <= 2)
            {
                if (1)
                {
                    //obs.partial = obs.partial - 2;
                    UARTprintf("partial %d, x_pos: %d, y_pos: %d\n", wood5.partial, x_pos, y_pos);
                    G8RTOS_WaitSemaphore(&sem_SPIA);
                    ST7789_DrawRectangle(x_pos, y_pos, wood5.partial, wood5.length, ST7789_BROWN);
                    ST7789_DrawRectangle(x_pos+wood5.partial, y_pos, 12, wood5.length, ST7789_BLUE);
                    G8RTOS_SignalSemaphore(&sem_SPIA);
                }
                else
                {
                    //obs.partial = 0;
                    //Obstacle_AddPosition(0, 20);
                }
            }
            else
            {
                G8RTOS_WaitSemaphore(&sem_SPIA);
                ST7789_DrawRectangle(x_pos, y_pos, wood5.width, wood5.length, ST7789_BROWN);
                ST7789_DrawRectangle(x_pos+wood5.width-2, y_pos, 12, wood5.length, ST7789_BLUE);
                G8RTOS_SignalSemaphore(&sem_SPIA);
            }

            wood5.tail_pos->draw_done = 1;
        }

    }


}

void Obstacle_AddPosition(uint16_t x_pos, uint16_t y_pos, struct obstacle_t *obs_ref)
{
    uint8_t j;
    //struct obstacle_t obs_ref = *obs_add;
    if (obs_ref->pos_write_index == 0)
        j = 9; // max_num_pos_obstacle(10) - 1
    else
        j = obs_ref->pos_write_index - 1;

    // this needs to be number of positions and not pos_count, hence commented below
    if (obs_ref->pos_count >= 1000000) // if counter exits max number of positions
    {
        //return -1; // exit with error
    }
    else // else if not exceeding max we insert item
    {
        if (obs_ref->pos_count == 0) // if first item we insert
        {
            obs_ref->positions[0].next_pos = &(obs_ref->positions[0]);
            obs_ref->positions[0].previous_pos = &obs_ref->positions[0];
            obs_ref->head_pos = &obs_ref->positions[0];
            obs_ref->tail_pos = &obs_ref->positions[0];


        }
        else // else if not first item we insert
        {
            obs_ref->positions[obs_ref->pos_write_index].next_pos = obs_ref->positions[j].next_pos;
            obs_ref->positions[j].next_pos = &obs_ref->positions[obs_ref->pos_write_index];
            obs_ref->positions[j].next_pos->previous_pos = &obs_ref->positions[obs_ref->pos_write_index];
            obs_ref->positions[obs_ref->pos_write_index].previous_pos = &obs_ref->positions[j];

            obs_ref->tail_pos = &obs_ref->positions[obs_ref->pos_write_index];
        }

        // then write data and increment position counter
        obs_ref->positions[obs_ref->pos_write_index].x_pos = x_pos;
        obs_ref->positions[obs_ref->pos_write_index].y_pos = y_pos;
        obs_ref->positions[obs_ref->pos_write_index].draw_done = 0;
        obs_ref->pos_count++;
        obs_ref->pos_write_index++;
        if (obs_ref->pos_write_index >= 10)
            obs_ref->pos_write_index = 0;
    }

}

void Character_AddPosition(uint16_t x_pos, uint16_t y_pos)
{

    if (frog.pos_count >= MAX_NUM_POS) // if counter exits max number of positions
    {
        return -1; // exit with error
    }
    else // else if not exceeding max we insert item
    {
        if (frog.pos_count == 0) // if first item we insert
        {
            frog.positions[0].next_pos = &frog.positions[0];
            frog.positions[0].previous_pos = &frog.positions[0];
            frog.head_pos = &frog.positions[0];
            frog.tail_pos = &frog.positions[0];
        }
        else // else if not first item we insert
        {
            frog.positions[frog.pos_count].next_pos = frog.positions[frog.pos_count-1].next_pos;
            frog.positions[frog.pos_count-1].next_pos = &frog.positions[frog.pos_count];
            frog.positions[frog.pos_count-1].next_pos->previous_pos = &frog.positions[frog.pos_count];
            frog.positions[frog.pos_count].previous_pos = &frog.positions[frog.pos_count-1];

            frog.tail_pos = &frog.positions[frog.pos_count];
        }

        // then write data and increment position counter
        frog.positions[frog.pos_count].x_pos = x_pos;
        frog.positions[frog.pos_count].y_pos = y_pos;
        frog.positions[frog.pos_count].draw_done = 0;
        frog.pos_count++;
    }

}

void Game_Thread(void) {



    // Initialize / declare any variables here
    int32_t joy_x, joy_y = 0;
    float joy_x_n, joy_y_n = 0;
    bool down, up, left, right, center_x, center_y = 0;
    // center is a flag for when the joystick returns to center position
    // used to avoid multiple joystick inputs when moving.
    uint32_t result = 0;

    // This clears a couple of weird inputs the first time the program starts
    result = G8RTOS_ReadFIFO(JOYSTICK_FIFO);
    result = G8RTOS_ReadFIFO(JOYSTICK_FIFO);
    result = 0;

    while(1) {
        // Get result from joystick

        result = G8RTOS_ReadFIFO(JOYSTICK_FIFO);

        joy_x = ((result >> 0) & 0xFFFF);
        joy_y = ((result >> 16) & 0xFFFF);

        // JOYSTICK NORMALIZATION

        // If joystick axis within deadzone, set to 0. Otherwise normalize it.
        if (joy_x < 1500 || joy_x > 2600)
        {
            joy_x_n = (2.0 * (joy_x - 0) / (4095 - 0)) - 1.0;
            if (joy_x_n < 0 && center_x && center_y)
            {
                right = 1;
                left, up, down, center_x = 0;
                UARTprintf("right\n");
            }
            else if (joy_x_n > 0 && center_x && center_y)
            {
                left = 1;
                right, up, down, center_x = 0;
                UARTprintf("left\n");
            }
        }
        else
        {
            joy_x_n = 0;
            left, right = 0;
            center_x = 1;
        }

        if (joy_y < 1500 || joy_y > 2600)
        {
            joy_y_n = (2.0 * (joy_y - 0) / (4095 - 0)) - 1.0;

            if (joy_y_n > 0 && center_y && center_x)
            {
                up = 1;
                down, left, right, center_y = 0;
                UARTprintf("up\n");
            }
            else if (joy_y_n < 0 && center_y && center_x)
            {
                down = 1;
                up, left, right, center_y = 0;
                UARTprintf("down\n");
            }


        }
        else
        {
            joy_y_n = 0;
            up, down = 0;
            center_y = 1;
        }


        // Character move
        if (up)
        {
            // we should add a semaphore for a new position buffer thingie
            uint16_t x_pos = frog.tail_pos->x_pos;
            uint16_t y_pos = frog.tail_pos->y_pos + 20;
            Character_AddPosition(x_pos, y_pos);
            up = 0;
        }
        if (right)
        {
            uint16_t x_pos = frog.tail_pos->x_pos + 20;
            uint16_t y_pos = frog.tail_pos->y_pos;
            Character_AddPosition(x_pos, y_pos);
            right = 0;
        }
        if (left)
        {
            uint16_t x_pos = frog.tail_pos->x_pos - 20;
            uint16_t y_pos = frog.tail_pos->y_pos;
            Character_AddPosition(x_pos, y_pos);
            left = 0;
        }
        if (down)
        {
            uint16_t x_pos = frog.tail_pos->x_pos;
            uint16_t y_pos = frog.tail_pos->y_pos - 20;
            Character_AddPosition(x_pos, y_pos);
            down = 0;
        }


        // should we spawn and kill vehicle threads here?
        // or should we just do the whole vehicle thing here? lets try option 2 first

        // Obstacle move
        if (obs.tail_pos->x_pos < 238)
        {
            uint16_t x_pos = obs.tail_pos->x_pos+2;
            uint16_t y_pos = obs.tail_pos->y_pos;
            if (obs.tail_pos->x_pos > 200)
            {
                obs.partial = obs.partial - 2;
            }
            Obstacle_AddPosition(x_pos, y_pos, &obs);
        }
        else
        {
            Obstacle_AddPosition(0, 20, &obs);
            obs.partial = 40;
        }

        // Obstacle2 move
        if (obs2.tail_pos->x_pos < 238)
        {
            uint16_t x_pos = obs2.tail_pos->x_pos+4;
            uint16_t y_pos = obs2.tail_pos->y_pos;
            if (obs2.tail_pos->x_pos > 200)
            {
                obs2.partial = obs.partial - 4;
            }
            Obstacle_AddPosition(x_pos, y_pos, &obs2);
        }
        else
        {
            Obstacle_AddPosition(0, 60, &obs2);
            obs2.partial = 40;
        }

        // Obstacle3 move
        if (obs3.tail_pos->x_pos < 239)
        {
            uint16_t x_pos = obs3.tail_pos->x_pos+1;
            uint16_t y_pos = obs3.tail_pos->y_pos;
            if (obs3.tail_pos->x_pos > 200)
            {
                obs3.partial = obs3.partial - 2;
            }
            Obstacle_AddPosition(x_pos, y_pos, &obs3);
        }
        else
        {
            Obstacle_AddPosition(0, 100, &obs3);
            obs3.partial = 40;
        }

        // Obstacle4 move
        if (obs4.tail_pos->x_pos > 2)
        {
            uint16_t x_pos = obs4.tail_pos->x_pos-2;
            uint16_t y_pos = obs4.tail_pos->y_pos;

            Obstacle_AddPosition(x_pos, y_pos, &obs4);
        }
        else //if (obs4.tail_pos->x_pos < 4)
        {

            if (obs4.partial >= 2)
            {
                obs4.partial = obs4.partial - 2;
                Obstacle_AddPosition(0, 40, &obs4);
            }
            else
            {
                Obstacle_AddPosition(200, 40, &obs4);
                obs4.partial = 40;
            }
        }

        // Obstacle5 move
        if (obs5.tail_pos->x_pos > 4)
        {
            uint16_t x_pos = obs5.tail_pos->x_pos-4;
            uint16_t y_pos = obs5.tail_pos->y_pos;

            Obstacle_AddPosition(x_pos, y_pos, &obs5);
        }
        else //if (obs4.tail_pos->x_pos < 4)
        {

            if (obs5.partial >= 4)
            {
                obs5.partial = obs5.partial - 4;
                Obstacle_AddPosition(0, 80, &obs5);
            }
            else
            {
                Obstacle_AddPosition(200, 80, &obs5);
                obs5.partial = 40;
            }
        }

        // wood1 move
        if (wood.tail_pos->x_pos < 238)
        {
            uint16_t x_pos = wood.tail_pos->x_pos+4;
            uint16_t y_pos = wood.tail_pos->y_pos;
            if (frog.attached == 1)
            {
                uint16_t x_pos_frog = frog.tail_pos->x_pos+4;
                uint16_t y_pos_frog = frog.tail_pos->y_pos;
                Character_AddPosition(x_pos_frog, y_pos_frog);
            }
            if (wood.tail_pos->x_pos > 200)
            {
                wood.partial = wood.partial - 4;
            }
            Obstacle_AddPosition(x_pos, y_pos, &wood);
        }
        else
        {
            Obstacle_AddPosition(0, 140, &wood);
            wood.partial = 60;
        }

        // check if frog attached
        int16_t x_diff = frog.tail_pos->x_pos - wood.tail_pos->x_pos;
        int16_t y_diff = frog.tail_pos->y_pos - wood.tail_pos->y_pos;
        /*
        if (x_diff >= -20 && x_diff < 40 && y_diff >= 0 && y_diff < 20)
        {

            UARTprintf("Attached to wood 1\n");
            frog.attached = 1;

        }
        else
        {
            frog.attached = 0;
        }*/

        // wood2 move
        if (wood2.tail_pos->x_pos < 238)
        {
            uint16_t x_pos = wood2.tail_pos->x_pos+2;
            uint16_t y_pos = wood2.tail_pos->y_pos;
            if (frog.attached == 2)
            {
                uint16_t x_pos_frog = frog.tail_pos->x_pos+2;
                uint16_t y_pos_frog = frog.tail_pos->y_pos;
                Character_AddPosition(x_pos_frog, y_pos_frog);
            }
            if (wood2.tail_pos->x_pos > 200)
            {
                wood2.partial = wood2.partial - 2;
            }
            Obstacle_AddPosition(x_pos, y_pos, &wood2);
        }
        else
        {
            Obstacle_AddPosition(0, 200, &wood2);
            wood2.partial = 80;
        }

        // check if frog attached
        int16_t x_diff2 = frog.tail_pos->x_pos - wood2.tail_pos->x_pos;
        int16_t y_diff2 = frog.tail_pos->y_pos - wood2.tail_pos->y_pos;


        // wood3 move
        if (wood3.tail_pos->x_pos > 2)
        {
            uint16_t x_pos = wood3.tail_pos->x_pos-2;
            uint16_t y_pos = wood3.tail_pos->y_pos;

            Obstacle_AddPosition(x_pos, y_pos, &wood3);
        }
        else //if (obs4.tail_pos->x_pos < 4)
        {

            if (wood3.partial >= 2)
            {
                wood3.partial = wood3.partial - 2;
                Obstacle_AddPosition(0, 160, &wood3);
            }
            else
            {
                Obstacle_AddPosition(160, 160, &wood3);
                wood3.partial = 80;
            }
        }

        // check if frog attached
        int16_t x_diff3 = frog.tail_pos->x_pos - wood3.tail_pos->x_pos;
        int16_t y_diff3 = frog.tail_pos->y_pos - wood3.tail_pos->y_pos;

        // wood4 move
                if (wood4.tail_pos->x_pos > 2)
                {
                    uint16_t x_pos = wood4.tail_pos->x_pos-2;
                    uint16_t y_pos = wood4.tail_pos->y_pos;

                    Obstacle_AddPosition(x_pos, y_pos, &wood4);
                }
                else //if (obs4.tail_pos->x_pos < 4)
                {

                    if (wood4.partial >= 2)
                    {
                        wood4.partial = wood4.partial - 2;
                        Obstacle_AddPosition(0, 180, &wood4);
                    }
                    else
                    {
                        Obstacle_AddPosition(180, 180, &wood4);
                        wood4.partial = 80;
                    }
                }

                // check if frog attached
                int16_t x_diff4 = frog.tail_pos->x_pos - wood4.tail_pos->x_pos;
                int16_t y_diff4 = frog.tail_pos->y_pos - wood4.tail_pos->y_pos;

        // wood5 move
        if (wood5.tail_pos->x_pos > 3)
        {
            uint16_t x_pos = wood5.tail_pos->x_pos-3;
            uint16_t y_pos = wood5.tail_pos->y_pos;

            Obstacle_AddPosition(x_pos, y_pos, &wood5);
        }
        else //if (obs4.tail_pos->x_pos < 4)
        {

            if (wood5.partial >= 3)
            {
                wood5.partial = wood3.partial - 3;
                Obstacle_AddPosition(0, 220, &wood5);
            }
            else
            {
                Obstacle_AddPosition(160, 220, &wood5);
                wood5.partial = 80;
            }
        }

        // check if frog attached
        int16_t x_diff5 = frog.tail_pos->x_pos - wood5.tail_pos->x_pos;
        int16_t y_diff5 = frog.tail_pos->y_pos - wood5.tail_pos->y_pos;


        if (x_diff >= -20 && x_diff < 60 && y_diff >= 0 && y_diff < 20)
        {

            UARTprintf("Attached to wood 1\n");
            G8RTOS_WaitSemaphore(&sem_attached);
            frog.attached = 1;
            G8RTOS_SignalSemaphore(&sem_attached);

        }
        else if (x_diff2 >= -20 && x_diff2 < 80 && y_diff2 >= 0 && y_diff2 < 20)
        {

            UARTprintf("Attached to wood 2\n");
            G8RTOS_WaitSemaphore(&sem_attached);
            frog.attached = 2;
            G8RTOS_SignalSemaphore(&sem_attached);

        }
        else if (x_diff3 >= -20 && x_diff3 < 100 && y_diff3 >= 0 && y_diff3 < 20)
        {
            G8RTOS_WaitSemaphore(&sem_attached);
            frog.attached = 3;
            G8RTOS_SignalSemaphore(&sem_attached);
            UARTprintf("Attached to wood 3\n");
        }
        else if (x_diff4 >= -20 && x_diff4 < 100 && y_diff4 >= 0 && y_diff4 < 20)
        {
            G8RTOS_WaitSemaphore(&sem_attached);
            frog.attached = 4;
            G8RTOS_SignalSemaphore(&sem_attached);
            UARTprintf("Attached to wood 4\n");
        }
        else if (x_diff5 >= -20 && x_diff5 < 100 && y_diff5 >= 0 && y_diff5 < 20)
        {
            G8RTOS_WaitSemaphore(&sem_attached);
            frog.attached = 5;
            G8RTOS_SignalSemaphore(&sem_attached);
            UARTprintf("Attached to wood 5\n");
        }
        else
        {
            G8RTOS_WaitSemaphore(&sem_attached);
            frog.attached = 0;
            G8RTOS_SignalSemaphore(&sem_attached);
        }

        // sleep
        sleep(50);
    }
}


void check_win_lose(void)
{
    while(1)
    {
        // check win
        int16_t x_diff = frog.tail_pos->x_pos - 20;
        int16_t y_diff = frog.tail_pos->y_pos - 240;
        if (x_diff >= 0 && x_diff < 40 && y_diff >= 0 && y_diff <= 20)
        {
            G8RTOS_KillThread(2);
            G8RTOS_KillThread(3);
            G8RTOS_KillThread(4);
            UARTprintf("YOU WON!!\n");
            G8RTOS_WaitSemaphore(&sem_SPIA);
            ST7789_Fill(ST7789_GREEN);
            G8RTOS_SignalSemaphore(&sem_SPIA);
        }
        x_diff = frog.tail_pos->x_pos - 80;
        y_diff = frog.tail_pos->y_pos - 240;
        if (x_diff >= 0 && x_diff < 40 && y_diff >= 0 && y_diff <= 20)
        {
            G8RTOS_KillThread(2);
                        G8RTOS_KillThread(3);
                        G8RTOS_KillThread(4);
            UARTprintf("YOU WON!!\n");
            G8RTOS_WaitSemaphore(&sem_SPIA);
            ST7789_Fill(ST7789_GREEN);
            G8RTOS_SignalSemaphore(&sem_SPIA);
        }
        x_diff = frog.tail_pos->x_pos - 140;
        y_diff = frog.tail_pos->y_pos - 240;
        if (x_diff >= 0 && x_diff < 40 && y_diff >= 0 && y_diff <= 20)
        {
            G8RTOS_KillThread(2);
                        G8RTOS_KillThread(3);
                        G8RTOS_KillThread(4);
            UARTprintf("YOU WON!!\n");
            G8RTOS_WaitSemaphore(&sem_SPIA);
            ST7789_Fill(ST7789_GREEN);
            G8RTOS_SignalSemaphore(&sem_SPIA);
        }
        x_diff = frog.tail_pos->x_pos - 200;
        y_diff = frog.tail_pos->y_pos - 240;
        if (x_diff >= 0 && x_diff < 40 && y_diff >= 0 && y_diff <= 20)
        {
            G8RTOS_KillThread(2);
                        G8RTOS_KillThread(3);
                        G8RTOS_KillThread(4);
            UARTprintf("YOU WON!!\n");
            G8RTOS_WaitSemaphore(&sem_SPIA);
            ST7789_Fill(ST7789_GREEN);
            G8RTOS_SignalSemaphore(&sem_SPIA);
        }

        // check collision
        x_diff = frog.tail_pos->x_pos - obs.tail_pos->x_pos;
        y_diff = frog.tail_pos->y_pos - obs.tail_pos->y_pos;
        if (x_diff >= -20 && x_diff < 40 && y_diff >= 0 && y_diff < 20)
        {
            G8RTOS_KillThread(2);
                        G8RTOS_KillThread(3);
                        G8RTOS_KillThread(4);
            UARTprintf("YOU LOST!!\n");
            G8RTOS_WaitSemaphore(&sem_SPIA);
            ST7789_Fill(0x001F);
            G8RTOS_SignalSemaphore(&sem_SPIA);
        }

        // check collision 2
        x_diff = frog.tail_pos->x_pos - obs2.tail_pos->x_pos;
        y_diff = frog.tail_pos->y_pos - obs2.tail_pos->y_pos;
        if (x_diff >= -20 && x_diff < 40 && y_diff >= 0 && y_diff < 20)
        {
            G8RTOS_KillThread(2);
                        G8RTOS_KillThread(3);
                        G8RTOS_KillThread(4);
            UARTprintf("YOU LOST!!\n");
            G8RTOS_WaitSemaphore(&sem_SPIA);
            ST7789_Fill(0x001F);
            G8RTOS_SignalSemaphore(&sem_SPIA);
        }

        // check collision 3
        x_diff = frog.tail_pos->x_pos - obs3.tail_pos->x_pos;
        y_diff = frog.tail_pos->y_pos - obs3.tail_pos->y_pos;
        if (x_diff >= -20 && x_diff < 40 && y_diff >= 0 && y_diff < 20)
        {
            G8RTOS_KillThread(2);
                        G8RTOS_KillThread(3);
                        G8RTOS_KillThread(4);
            UARTprintf("YOU LOST!!\n");
            G8RTOS_WaitSemaphore(&sem_SPIA);
            ST7789_Fill(0x001F);
            G8RTOS_SignalSemaphore(&sem_SPIA);
        }

        // check collision 4
        x_diff = frog.tail_pos->x_pos - obs4.tail_pos->x_pos;
        y_diff = frog.tail_pos->y_pos - obs4.tail_pos->y_pos;
        if (x_diff >= -20 && x_diff < 40 && y_diff >= 0 && y_diff < 20)
        {
            G8RTOS_KillThread(2);
                        G8RTOS_KillThread(3);
                        G8RTOS_KillThread(4);
            UARTprintf("YOU LOST!!\n");
            G8RTOS_WaitSemaphore(&sem_SPIA);
            ST7789_Fill(0x001F);
            G8RTOS_SignalSemaphore(&sem_SPIA);
        }

        // check collision 5
        x_diff = frog.tail_pos->x_pos - obs5.tail_pos->x_pos;
        y_diff = frog.tail_pos->y_pos - obs5.tail_pos->y_pos;
        if (x_diff >= -20 && x_diff < 40 && y_diff >= 0 && y_diff < 20)
        {
            G8RTOS_KillThread(2);
                        G8RTOS_KillThread(3);
                        G8RTOS_KillThread(4);
            UARTprintf("YOU LOST!!\n");
            G8RTOS_WaitSemaphore(&sem_SPIA);
            ST7789_Fill(0x001F);
            G8RTOS_SignalSemaphore(&sem_SPIA);
        }

        G8RTOS_WaitSemaphore(&sem_attached);
        uint8_t attached = frog.attached;
        G8RTOS_SignalSemaphore(&sem_attached);

        // check if on water
        if ((frog.tail_pos->y_pos >= 140 && frog.tail_pos->x_pos <= 240) && (attached == 0))
        {

            G8RTOS_KillThread(2);
            G8RTOS_KillThread(3);
            G8RTOS_KillThread(4);
            UARTprintf("YOU LOST!!\n");
            G8RTOS_WaitSemaphore(&sem_SPIA);
            ST7789_Fill(0x001F);
            G8RTOS_SignalSemaphore(&sem_SPIA);
        }
    }
    sleep(50);
}


void Read_Buttons() {
    uint8_t buttons;

    while(1) {
        G8RTOS_WaitSemaphore(&sem_PCA9555_Debounce);

        // For switch debouncing
        sleep(15);

        // Get buttons
        G8RTOS_WaitSemaphore(&sem_I2CA);
        buttons = ~(MultimodButtons_Get());
        G8RTOS_SignalSemaphore(&sem_I2CA);

        /*
        if (buttons & SW1) {
            // Spawn cube

            int8_t result = G8RTOS_AddThread(Cube_Thread, 254, "cube\0");

            if (result == NO_ERROR) {
                num_cubes++;

                // Get random positions
                uint32_t x_pos, y_pos, z_pos;

                x_pos = (rand() % 200);
                y_pos = (rand() % 200);
                z_pos = (rand() % 100);

                // Write to FIFO
                uint32_t packet = ((x_pos & 0xFFF) << 20) | ((y_pos & 0xFFF) << 8) | ((z_pos & 0xFF) << 0);
                G8RTOS_WriteFIFO(SPAWNCOOR_FIFO, packet);
            }
        }

        if (buttons & SW2) {
            if (num_cubes > 0) {
                kill_cube = 1;
            }
        }

        if (buttons & SW3) {
            if (move_or_rot) {
                move_or_rot = 0;
            } else {
                move_or_rot = 1;
            }
        }*/

        GPIOIntClear(BUTTONS_INT_GPIO_BASE, BUTTONS_INT_PIN);
        GPIOIntEnable(BUTTONS_INT_GPIO_BASE, BUTTONS_INT_PIN);

    }
}

void Read_JoystickPress() {
    uint8_t joystick_s;

    while(1) {
        G8RTOS_WaitSemaphore(&sem_Joystick_Debounce);

        // For switch debouncing
        sleep(5);

        joystick_s = JOYSTICK_GetPress();

        if (joystick_s) {
            if (joystick_y) {
                joystick_y = 0;
            } else {
                joystick_y = 1;
            }
        }

        GPIOIntClear(JOYSTICK_INT_GPIO_BASE, JOYSTICK_INT_PIN);
        GPIOIntEnable(JOYSTICK_INT_GPIO_BASE, JOYSTICK_INT_PIN);

    }
}

void LED_Thread(void) {
    uint8_t i = 2;
    while(1) {
        PCA9956b_SetAllOff();
        PCA9556b_SetLED(i, 128, 128);
        i++;
        if (i == 22) {
            i = 2;
        }

        sleep(100);
    }
}


/********************************Periodic Threads***********************************/

void Print_WorldCoords(void) {
    UARTprintf("Cam Pos, X: %d, Y: %d, Z: %d\n", (int32_t) world_camera_pos.x, (int32_t)world_camera_pos.y, (int32_t)world_camera_pos.z);
}

void Get_Joystick(void) {
    uint32_t packet;
    packet = JOYSTICK_GetXY();
    G8RTOS_WriteFIFO(JOYSTICK_FIFO, packet);
}

/********************************Periodic Threads***********************************/


/*******************************Aperiodic Threads***********************************/

void GPIOE_Handler() {
    GPIOIntDisable(BUTTONS_INT_GPIO_BASE, BUTTONS_INT_PIN);
    G8RTOS_SignalSemaphore(&sem_PCA9555_Debounce);
}

void GPIOD_Handler() {
    GPIOIntDisable(JOYSTICK_INT_GPIO_BASE, JOYSTICK_INT_PIN);
    G8RTOS_SignalSemaphore(&sem_Joystick_Debounce);
}

/*******************************Aperiodic Threads***********************************/

void InitWorld(void)
{
    G8RTOS_WaitSemaphore(&sem_SPIA);
    ST7789_DrawRectangle(0, 240, 240, 40, ST7789_GREEN);
    ST7789_DrawRectangle(0, 140, 240, 100, ST7789_BLUE);
    ST7789_DrawRectangle(0, 120, 240, 20, ST7789_PURPLE);
    ST7789_DrawRectangle(0, 0, 240, 20, ST7789_PURPLE);
    ST7789_DrawRectangle(20, 240, 40, 20, ST7789_BLACK);
    ST7789_DrawRectangle(80, 240, 40, 20, ST7789_BLACK);
    ST7789_DrawRectangle(140, 240, 40, 20, ST7789_BLACK);
    ST7789_DrawRectangle(200, 240, 40, 20, ST7789_BLACK);
    G8RTOS_SignalSemaphore(&sem_SPIA);

}

