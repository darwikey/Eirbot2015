/**
 ********************************************************************
 * @file    ax12.c
 * @author  Xavier Maupeu
 * @version V1.0
 * @date    20-Feb-2014
 * @brief   This file provides functions for AX12 servomotors
 ********************************************************************
 * @attention
 *
 * This file is part of LIBAUSBEE.
 * 
 * LIBAUSBEE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * LIBAUSBEE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with LIBAUSBEE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * <h2><centor>&copy;  Copyright 2013-2014 (C) EIRBOT </center></h2>
 ********************************************************************
 */

/* Includes */
#include "ax12_driver.h"
#include <stm32f4xx_usart.h>

/**
 * @addtogroup Libausbee
 * @{
 */

/**
 * @defgroup AX12
 * @brief AX12 driver modules
 * @{
 */

//#define AX12_WRITE_DEBUG 0
//#define AX12_READ_DEBUG 0
//#define AX12_TRIGGER_DEBUG 0
//#define AX12_DEBUG 0

#define AX12_REG_ID 0x3
#define AX12_REG_BAUD 0x4
#define AX12_REG_CW_LIMIT 0x06
#define AX12_REG_CCW_LIMIT 0x08
#define AX12_REG_LED 0x19
#define AX12_REG_GOAL_POSITION 0x1E
#define AX12_REG_MOVING_SPEED 0x20
#define AX12_REG_VOLTS 0x2A
#define AX12_REG_TEMP 0x2B
#define AX12_REG_MOVING 0x2E
#define AX12_REG_POSITION 0x24

#define AX12_MODE_POSITION  0
#define AX12_MODE_ROTATION  1

#define AX12_CW 1
#define AX12_CCW 0

#define AX12_UART_PORT          xUART1_BASE
#define AX12_UART_TX_PIN        PA9
#define AX12_UART_RX_PIN        PA10
#define AX12_UART_TX            UART1TX
#define AX12_UART_RX            UART1RX

#define ABS(x) (((x) < 0)? -(x): (x))


int ausbee_ax12_read(ausbee_ax12_chip* chip, int start, int bytes, char* data);
int ausbee_ax12_write(ausbee_ax12_chip* chip, int start, int bytes, char* data, int flag);
void ausbee_ax12_delay(unsigned int iteration);


// Set the mode of the servo
//  0 = Positional (0-300 degrees)
//  1 = Rotational -1 to 1 speed
int ausbee_ax12_set_mode(ausbee_ax12_chip* chip, int mode) {

    if (mode == 1) { // set CR
        ausbee_ax12_set_CW_limit(chip, 0);
        ausbee_ax12_set_CCW_limit(chip, 0);
        ausbee_ax12_set_CR_speed(chip, 0.0);
    } else {
        ausbee_ax12_set_CW_limit(chip, 0);
        ausbee_ax12_set_CCW_limit(chip, 300);
        ausbee_ax12_set_CR_speed(chip, 0.0);
    }
    return(0);
}


// if flag[0] is set, were blocking
// if flag[1] is set, we're registering
// they are mutually exclusive operations
int ausbee_ax12_set_goal(ausbee_ax12_chip* chip, int degrees, int flags) {

    char reg_flag = 0;
    char data[2];

    // set the flag is only the register bit is set in the flag
    if (flags == 0x2) {
        reg_flag = 1;
    }

    // 1023 / 300 * degrees
    short goal = (1023 * degrees) / 300;
#ifdef AX12_DEBUG
    printf("SetGoal to 0x%x\n",goal);
#endif

    data[0] = goal & 0xff; // bottom 8 bits
    data[1] = goal >> 8;   // top 8 bits

    // AX12_write the packet, return the error code
    int rVal = ausbee_ax12_write(chip, AX12_REG_GOAL_POSITION, 2, data, reg_flag);

    if (flags == 1) {
        // block until it comes to a halt
        while (ausbee_ax12_is_moving(chip)) {}
    }
    return(rVal);
}


// Set continuous rotation speed from -1 to 1
int ausbee_ax12_set_CR_speed(ausbee_ax12_chip* chip, float speed)
{

    // bit 10     = direction, 0 = CCW, 1=CW
    // bits 9-0   = Speed
    char data[2];

    int goal = (0x3ff * ABS(speed));

    // Set direction CW if we have a negative speed
    if (speed < 0)
    {
        goal |= (0x1 << 10);
    }

    data[0] = goal & 0xff; // bottom 8 bits
    data[1] = goal >> 8;   // top 8 bits

    // AX12_write the packet, return the error code
    int rVal = ausbee_ax12_write(chip, 0x20, 2, data,1);

    return(rVal);
}


int ausbee_ax12_set_CW_limit (ausbee_ax12_chip* chip, int degrees) {

    char data[2];

    // 1023 / 300 * degrees
    short limit = (1023 * degrees) / 300;

#ifdef AX12_DEBUG
    printf("ausbee_ax12_set_CW_limit to 0x%x\n",limit);
#endif

    data[0] = limit & 0xff; // bottom 8 bits
    data[1] = limit >> 8;   // top 8 bits

    // AX12_write the packet, return the error code
    return (ausbee_ax12_write(chip, AX12_REG_CW_LIMIT, 2, data,1));

}

int ausbee_ax12_set_CCW_limit(ausbee_ax12_chip* chip, int degrees) {

    char data[2];

    // 1023 / 300 * degrees
    short limit = (1023 * degrees) / 300;

#ifdef AX12_DEBUG
    printf("AX12_SetCCWLimit to 0x%x\n",limit);
#endif

    data[0] = limit & 0xff; // bottom 8 bits
    data[1] = limit >> 8;   // top 8 bits

    // AX12_write the packet, return the error code
    return (ausbee_ax12_write(chip, AX12_REG_CCW_LIMIT, 2, data,1));
}


int ausbee_ax12_set_ID(ausbee_ax12_chip* chip, int NewID) {

    char data[1];
    data[0] = NewID;

#ifdef AX12_DEBUG
    printf("Setting ID from 0x%x to 0x%x\n", chip->id, NewID);
#endif

    return (ausbee_ax12_write(chip, AX12_REG_ID, 1, data,1));

}


int ausbee_ax12_set_baud(USART_TypeDef* usart, int baud) {

    char data[1];
    data[0] = baud;

#ifdef AX12_DEBUG
    printf("Setting Baud rate to %d\n",baud);
#endif

    ausbee_ax12_chip chips;
    chips.usart = usart;
    chips.id = 0xFE; // broadcast to all chips
    return (ausbee_ax12_write(&chips, AX12_REG_BAUD, 1, data,1));

}



// return 1 is the servo is still in flight
int ausbee_ax12_is_moving(ausbee_ax12_chip* chip) {

    char data[1];
    ausbee_ax12_read(chip, AX12_REG_MOVING,1,data);
    return(data[0]);
}


void ausbee_ax12_trigger(ausbee_ax12_chip* chip) {

    char TxBuf[16];
    char sum = 0;
    int i;

#ifdef AX12_TRIGGER_DEBUG
    // Build the TxPacket first in RAM, then we'll send in one go
    printf("\nTriggered\n");
    printf("\nTrigger Packet\n  Header : 0xFF, 0xFF\n");
#endif

    TxBuf[0] = 0xFF;
    TxBuf[1] = 0xFF;

    // ID - Broadcast
    TxBuf[2] = 0xFE;
    sum += TxBuf[2];

#ifdef AX12_TRIGGER_DEBUG
    printf("  ID : %d\n",TxBuf[2]);
#endif

    // Length
    TxBuf[3] = 0x02;
    sum += TxBuf[3];

#ifdef AX12_TRIGGER_DEBUG
    printf("  Length %d\n",TxBuf[3]);
#endif

    // Instruction - ACTION
    TxBuf[4] = 0x04;
    sum += TxBuf[4];

#ifdef AX12_TRIGGER_DEBUG
    printf("  Instruction 0x%X\n",TxBuf[5]);
#endif

    // Checksum
    TxBuf[5] = 0xFF - sum;
#ifdef AX12_TRIGGER_DEBUG
    printf("  Checksum 0x%X\n",TxBuf[5]);
#endif

    // Transmit the packet in one burst with no pausing
    for ( i = 0; i < 6 ; i++) {
        USART_SendData(chip->usart, TxBuf[i]);
    }

    // This is a broadcast packet, so there will be no reply
    return;
}


float ausbee_ax12_get_position(ausbee_ax12_chip* chip) {

#ifdef AX12_DEBUG
    printf("\nGetPosition(%d)", chip->id);
#endif

    char data[2];
    short position;
    float angle;

    ausbee_ax12_read(chip, AX12_REG_POSITION, 2, data);
    position = data[0] + (data[1] << 8);
    angle = (position * 300)/1024;
    return (angle);
}


float ausbee_ax12_get_temp(ausbee_ax12_chip* chip) {

#ifdef AX12_DEBUG
    printf("\nGetTemp(%d)", chip->id);
#endif

    char data[1];
    float temp;
    ausbee_ax12_read(chip, AX12_REG_TEMP, 1, data);
    temp = data[0];
    return(temp);
}


float ausbee_ax12_get_volts(ausbee_ax12_chip* chip) {

#ifdef AX12_DEBUG
    printf("\nGetVolts(%d)",chip->id);
#endif

    char data[1];
    float volts;
    ausbee_ax12_read(chip, AX12_REG_VOLTS, 1, data);
    volts = data[0]/10.0;
    return(volts);
}


void ausbee_ax12_set_led(ausbee_ax12_chip* chip, int fState) {
    char data[1];
    data[0] = fState;

	ausbee_ax12_write(chip, AX12_REG_LED, 1, data, 1);
}


int ausbee_ax12_read(ausbee_ax12_chip* chip, int start, int bytes, char* data) {

    char PacketLength = 0x4;
    char TxBuf[16];
    char sum = 0;
    char Status[16];
    int i;

    Status[4] = 0xFE; // return code

#ifdef AX12_READ_DEBUG
    printf("\nread(%d,0x%x,%d,data)\n",chip->id,start,bytes);
#endif

    // Build the TxPacket first in RAM, then we'll send in one go
#ifdef AX12_READ_DEBUG
    printf("\nInstruction Packet\n  Header : 0xFF, 0xFF\n");
#endif

    TxBuf[0] = 0xff;
    TxBuf[1] = 0xff;

    // ID
    TxBuf[2] = chip->id;
    sum += TxBuf[2];

#ifdef AX12_READ_DEBUG
    printf("  ID : %d\n",TxBuf[2]);
#endif

    // Packet Length
    TxBuf[3] = PacketLength;    // Length = 4 ; 2 + 1 (start) = 1 (bytes)
    sum += TxBuf[3];            // Accululate the packet sum

#ifdef AX12_READ_DEBUG
    printf("  Length : 0x%x\n",TxBuf[3]);
#endif

    // Instruction - ausbee_ax12_read
    TxBuf[4] = 0x2;
    sum += TxBuf[4];

#ifdef AX12_READ_DEBUG
    printf("  Instruction : 0x%x\n",TxBuf[4]);
#endif

    // Start Address
    TxBuf[5] = start;
    sum += TxBuf[5];

#ifdef AX12_READ_DEBUG
    printf("  Start Address : 0x%x\n",TxBuf[5]);
#endif

    // Bytes to ausbee_ax12_read
    TxBuf[6] = bytes;
    sum += TxBuf[6];

#ifdef AX12_READ_DEBUG
    printf("  No bytes : 0x%x\n",TxBuf[6]);
#endif

    // Checksum
    TxBuf[7] = 0xFF - sum;
#ifdef AX12_READ_DEBUG
    printf("  Checksum : 0x%x\n",TxBuf[7]);
#endif

    // Transmit the packet in one burst with no pausing
    for ( i = 0; i<8 ; i++) {
    	USART_SendData(chip->usart, TxBuf[i]);
    }

    // ausbee_ax12_delay for the bytes to be transmitted
    ausbee_ax12_delay (20);

    // Skip if the ausbee_ax12_read was to the broadcast address
    if (chip->id != 0xFE) {



        // response packet is always 6 + bytes
        // 0xFF, 0xFF, ID, Length Error, Param(s) Checksum
        // timeout is a little more than the time to transmit
        // the packet back, i.e. (6+bytes)*10 bit periods

        int timeout = 0;
        int plen = 0;
        while ((timeout < ((6+bytes)*10)) && (plen<(6+bytes))) {

                Status[plen] = USART_ReceiveData(chip->usart);
                plen++;

            // ausbee_ax12_delay for the bit period
            ausbee_ax12_delay (100);
            timeout++;
        }

        if (timeout == ((6+bytes)*10) ) {
            return(-1);
        }

        // Copy the data from Status into data for return
        for (i=0; i < Status[3]-2 ; i++) {
            data[i] = Status[5+i];
        }

#ifdef AX12_READ_DEBUG
        printf("\nStatus Packet\n");
        printf("  Header : 0x%x\n",Status[0]);
        printf("  Header : 0x%x\n",Status[1]);
        printf("  ID : 0x%x\n",Status[2]);
        printf("  Length : 0x%x\n",Status[3]);
        printf("  Error Code : 0x%x\n",Status[4]);

        for (int i=0; i < Status[3]-2 ; i++) {
            printf("  Data : 0x%x\n",Status[5+i]);
        }

        printf("  Checksum : 0x%x\n",Status[5+(Status[3]-2)]);
#endif

    } // if (ID!=0xFE)

    return(Status[4]);
}


int ausbee_ax12_write(ausbee_ax12_chip* chip, int start, int bytes, char* data, int flag) {
// 0xff, 0xff, ID, Length, Intruction(AX12_write), Address, Param(s), Checksum

    char TxBuf[16];
    char sum = 0;
    char Status[6];
    int i;

#ifdef AX12_WRITE_DEBUG
    printf("\nwrite(%d,0x%x,%d,data,%d)\n",chip->id,start,bytes,flag);
#endif

    // Build the TxPacket first in RAM, then we'll send in one go
#ifdef AX12_WRITE_DEBUG
    printf("\nInstruction Packet\n  Header : 0xFF, 0xFF\n");
#endif

    TxBuf[0] = 0xff;
    TxBuf[1] = 0xff;

    // ID
    TxBuf[2] = chip->id;
    sum += TxBuf[2];

#ifdef AX12_WRITE_DEBUG
    printf("  ID : %d\n",TxBuf[2]);
#endif

    // packet Length
    TxBuf[3] = 3+bytes;
    sum += TxBuf[3];

#ifdef AX12_WRITE_DEBUG
    printf("  Length : %d\n",TxBuf[3]);
#endif

    // Instruction
    if (flag == 1) {
        TxBuf[4]=0x04;
        sum += TxBuf[4];
    } else {
        TxBuf[4]=0x03;
        sum += TxBuf[4];
    }

#ifdef AX12_WRITE_DEBUG
    printf("  Instruction : 0x%x\n",TxBuf[4]);
#endif

    // Start Address
    TxBuf[5] = start;
    sum += TxBuf[5];

#ifdef AX12_WRITE_DEBUG
    printf("  Start : 0x%x\n",TxBuf[5]);
#endif

    // data
    for ( i=0; i<bytes ; i++) {
        TxBuf[6+i] = data[i];
        sum += TxBuf[6+i];

#ifdef AX12_WRITE_DEBUG
        printf("  Data : 0x%x\n",TxBuf[6+i]);
#endif

    }

    // checksum
    TxBuf[6+bytes] = 0xFF - sum;

#ifdef AX12_WRITE_DEBUG
    printf("  Checksum : 0x%x\n",TxBuf[6+bytes]);
#endif

    // Transmit the packet in one burst with no pausing
    for ( i = 0; i < (7 + bytes) ; i++) {
    	USART_SendData(chip->usart ,TxBuf[i]);
    	// Loop until the end of transmission
		while (USART_GetFlagStatus(chip->usart, USART_FLAG_TC) == RESET);
    }

    // ausbee_ax12_delay for data to transmit
    ausbee_ax12_delay (20);

    // make sure we have a valid return
    Status[4]=0x00;

    // we'll only get a reply if it was not broadcast
    if (chip->id != 0xFE) {


        // response packet is always 6 bytes
        // 0xFF, 0xFF, ID, Length Error, Param(s) Checksum
        // timeout is a little more than the time to transmit
        // the packet back, i.e. 60 bit periods, round up to 100
        int timeout = 0;
        int plen = 0;
        while ((timeout < 100) && (plen<6)) {

            Status[plen] = USART_ReceiveData(chip->usart);
            plen++;

            // ausbee_ax12_delay for the bit period
            ausbee_ax12_delay (100);
            timeout++;
        }


        // Build the TxPacket first in RAM, then we'll send in one go
#ifdef AX12_WRITE_DEBUG
        printf("\nStatus Packet\n  Header : 0x%X, 0x%X\n",Status[0],Status[1]);
        printf("  ID : %d\n",Status[2]);
        printf("  Length : %d\n",Status[3]);
        printf("  Error : 0x%x\n",Status[4]);
        printf("  Checksum : 0x%x\n",Status[5]);
#endif


    }

    return(Status[4]); // return error code
}


void ausbee_ax12_delay(unsigned int iteration){
	volatile unsigned int i = iteration;
	while(i>0)
		i--;
}


/**
 * @}
 */
/**
 * @}
 */

/************** (C) COPYRIGHT 2013-2014 Eirbot **** END OF FILE ****/
