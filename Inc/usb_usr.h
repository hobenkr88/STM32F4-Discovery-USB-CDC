#ifndef USB_USR_H
#define USB_USR_H

#include <inttypes.h>

#include "usbd_cdc_if.h"

//extern USBD_HandleTypeDef;

// --------------------------

typedef struct character_node {
  char value;
  struct character_node * next_character;
} character_node_t;

typedef struct string_node {
  character_node_t *first_character;
  struct string_node *next_string;
} string_node_t;

typedef struct Queue
{
  uint8_t size;
  string_node_t * head_string;
} Queue;

uint8_t Queue_Create();

uint8_t Queue_Size();

uint8_t Queue_PushBack(char *a_string);

uint8_t Queue_PopFront();

string_node_t* Queue_Front();

// ---------------------------

uint8_t USB_Init(void);
uint8_t USB_Transmit();
uint8_t USB_Send(char *msg);

//int VCP_Read(void *, int);
int VCP_Write(const void *, int);

uint8_t USB_DataReceivedCallback1(USBD_HandleTypeDef *);
uint8_t USB_DataReceivedCallback2(char *Buff, uint16_t Len);
uint8_t USB_DataReceivedCallback3(uint8_t *Buff, uint32_t *Len);

uint8_t USB_AssembleTransmission(uint8_t *Buff, uint32_t *Len);

uint8_t USB_Write1(char* msg);
uint8_t USB_Write2(char* msg, uint8_t len);

uint8_t USB_WriteChar(uint8_t* character);

uint8_t USB_WriteString1(char* msg);
uint8_t USB_WriteString2(char* msg, uint8_t len);

#endif
