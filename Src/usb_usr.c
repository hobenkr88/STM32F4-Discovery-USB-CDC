#include <inttypes.h>
#include <stdlib.h>

#include "usb_usr.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t ProcessUSBCommand(char* command, uint32_t len);

char command_buffer[256];
uint8_t command_buffer_index = 0;

char *transmission;
uint32_t transmission_length;
uint8_t new_transmission;


// -------------------------------------------------------
char mystring[] = "abc";


//-------------------------------------------------

Queue *queue = NULL;

uint8_t Queue_Create() {
  queue = (Queue *)malloc(sizeof(Queue));
  if(queue == NULL) {
    //HAL_GPIO_TogglePin(GPIOD , GPIO_PIN_15);
    return 1;
  }
  /* Initialise its properties */
  queue->size = 0;
  queue->head_string = NULL;
  return 0;
}

uint8_t Queue_Size() {
  return queue->size;
}

uint8_t Queue_PushBack(char *a_string) {
  character_node_t *first_character = NULL;
  character_node_t *previous_character = NULL;
  uint32_t current_character_index = -1;
  do {
    current_character_index++;

    character_node_t *new_character = NULL;
    new_character = malloc(sizeof(character_node_t));
    if(new_character == NULL) {
      return 1;
    }

    if(previous_character == NULL) {
      first_character = new_character;
    }
    else {
      previous_character->next_character = new_character;
    }

    new_character->value = *(a_string+current_character_index);
    new_character->next_character = NULL;

    previous_character = new_character;
  } while(*(a_string+current_character_index) != 0x00);

  string_node_t *new_string = NULL;
  new_string = malloc(sizeof(string_node_t));
  if(new_string == NULL) {
    return 1;
  }
  new_string->first_character = first_character;
  new_string->next_string = NULL;

  if(queue->size == 0) {
    queue->head_string = new_string;
  }
  else {
    string_node_t *current_string = queue->head_string;
    while(current_string->next_string != NULL) {
      current_string = current_string->next_string;
    }
    current_string->next_string = new_string;
  }
  queue->size++;

  return 0;
}

/*uint8_t Queue_PushBack(char *a_string) {
  // Add characters
  character_node_t * first_character = NULL;
  character_node_t * previous_character = NULL;
  char current_character_index = -1;
  do {
    current_character_index++;

    character_node_t * new_character = NULL;
    new_character = malloc(sizeof(character_node_t));
    if(new_character == NULL) {
      return 1;
    }

    if(current_character_index == 0) {
      first_character = new_character;
    }
    else {
      previous_character->next_character = new_character;
    }

    new_character->value = *(a_string+current_character_index);
    new_character->next_character = NULL;

    previous_character = new_character;

  } while (*(a_string+current_character_index) != 0x00);

  // Create a string node for the linked list of characters
  string_node_t * the_string = NULL;
  the_string = malloc(sizeof(string_node_t));
  if (the_string == NULL) {
    return 1;
  }
  the_string->first_character = first_character;
  the_string->next_string = NULL;

  if(queue->size == 0) {
    // Add the string to the queue
    queue->head_string = the_string;

    queue->size++;
  }
  else {
    string_node_t * current_string = queue->head_string;
    while(current_string->next_string != NULL) {
      current_string = current_string->next_string;
    }
    current_string->next_string = the_string;
  }

  return 0;
}*/

uint8_t Queue_PopFront() {
  // Free characters
  character_node_t *current_character = queue->head_string->first_character;
  while(current_character != NULL) {
    character_node_t *next_character = current_character->next_character;

    free(current_character);

    current_character = next_character;
  }

  // Place the next string on the front of the queue and free the popped one
  string_node_t *next_string = queue->head_string->next_string;
  free(queue->head_string);
  queue->head_string = next_string;

  if(queue->size != 0) {
    queue->size--;
  }

  return 0;
}

string_node_t* Queue_Front() {
  if((queue != NULL) & (queue->head_string != NULL) & (queue->size != 0)) {
    string_node_t *character = queue->head_string;
    return character;
  }
  else {
    return NULL;
  }
}

//--------------------------------------------------------

uint8_t USB_Init(void) {
  transmission_length = 0;
  new_transmission = 1;

  Queue_Create();
  Queue_PushBack(mystring);

  return 0;
}

uint8_t USB_Transmit() {
  string_node_t *string = Queue_Front();
  //USB_WriteString2(character,2);
  if((string != NULL) & (string->first_character->value != 0x00)) {
    char to_send[256];
    int to_send_index = 0;
    character_node_t *current_character = string->first_character;

    while(current_character != NULL) {
      to_send[to_send_index] = current_character->value;
      current_character = current_character->next_character;
      to_send_index++;
    }
    USB_WriteString2(to_send, to_send_index-1);
    Queue_PopFront();
  }

  //char *character2 = &character;
  //USB_WriteString2(character2, 1);
  //Queue_Pop();

  return 0;
}

uint8_t USB_Send(char *msg) {
  Queue_PushBack(msg);


  return 0;
}

/*
int VCP_Read(void *pBuffer, int size)
{

	//si read done =0
  if (!s_RxBuffer.ReadDone)
        return 0;

    int remaining = s_RxBuffer.Size - s_RxBuffer.Position;
    int todo = MIN(remaining, size);
    if (todo <= 0)
        return 0;

    memcpy(pBuffer, s_RxBuffer.Buffer + s_RxBuffer.Position, todo);
    s_RxBuffer.Position += todo;
    if (s_RxBuffer.Position >= s_RxBuffer.Size)
    {
        s_RxBuffer.ReadDone = 0;
        USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    }

        //s_RxBuffer.ReadDone = 0;

    return todo;


	//return 1;
}

int VCP_Write(const void *pBuffer, int size)
{
    HAL_GPIO_TogglePin(GPIOD , GPIO_PIN_13);
	if (size > CDC_DATA_HS_OUT_PACKET_SIZE)
    {

        int offset;
        for (offset = 0; offset < size; offset++)
        {
            int todo = MIN(CDC_DATA_HS_OUT_PACKET_SIZE,
                           size - offset);
            int done = VCP_Write(((char *)pBuffer) + offset, todo);
            if (done != todo)
                return offset + done;
        }

        return size;
    }


    USBD_CDC_HandleTypeDef *pCDC = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    //while(pCDC->TxState) { } //Wait for previous transfer -- Not sure why I had to remove this
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t *)pBuffer, size);
    if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) != USBD_OK)
        return 0;


    //while(pCDC->TxState) { } //Wait until transfer is done
    return size;


	//return 1;
}
 */

uint8_t USB_DataReceivedCallback1(USBD_HandleTypeDef *hUsbDeviceFS) {
	HAL_GPIO_TogglePin(GPIOD , GPIO_PIN_13);

	//CDC_Transmit_FS(*hUsbDeviceFS->pData, 1);

	/*char buffer [50];
	int n;
	n = sprintf(buffer, *hUsbDeviceFS->pData);

	USB_Write1(buffer);*/

	return 0;
}

uint8_t USB_DataReceivedCallback2(char *Buff, uint16_t Len) {
	HAL_GPIO_TogglePin(GPIOD , GPIO_PIN_13);

	//CDC_Transmit_FS(*hUsbDeviceFS->pData, 1);

	//char buffer [50];
	//int n;
	//n = sprintf(buffer, *hUsbDeviceFS->pData);

	//int n;
	//n = strlen(Buff);

	//USB_Write2(Buff, n);

	CDC_Transmit_FS(Buff, Len);

	return 0;
}

uint8_t USB_DataReceivedCallback3(uint8_t *Buff, uint32_t *Len) {

  /*command_buffer[command_buffer_index] = (char) *Buff;

  if(*Buff != 0x0a) {
    command_buffer_index += 1;
  }
  else {
    command_buffer_index = 0;
    //ProcessUSBCommand(Buff);
    ProcessUSBCommand(Buff, *Len);
  }*/

  USB_AssembleTransmission(Buff, Len);

	return 0;
}

// FIXME: Change size to something reasonable one working
//        Keep in mind that the transmission length may not
//        be the same as the 'size' in general even though it is now
uint8_t USB_AssembleTransmission(uint8_t *Buff, uint32_t *Len) {
  if(new_transmission) {
    new_transmission = 0;
    //uint32_t size = transmission_length + 1;
    transmission = realloc(transmission, sizeof(char)*(transmission_length + 1));  // keep an eye on this line
    if(transmission == NULL) {
      return 1;
    }
  }

  int i;
  for(i=0;i<*Len;i++) {
    if(*(Buff+i) != 0x0a) {
      transmission[transmission_length++] = *(Buff+i);
    }
    else {
      transmission[transmission_length++] = 0x00;
    }
    //if(transmission_length == size) {
    if(1) {
      transmission = realloc(transmission, sizeof(char)*(transmission_length + 1));
      if(!transmission) {
        return 1;
      }
    }

    if(*(Buff+i) == 0x0a) {
      ProcessUSBCommand(transmission, transmission_length);
      transmission_length = 0;
      new_transmission = 1;
      transmission = realloc(transmission, sizeof(char));
    }
  }

  return 0;
}

uint8_t USB_Write1(char* msg) {

	char buffer [50];
	int n;
	n=sprintf (buffer, msg);

	CDC_Transmit_FS(buffer, n);

	return 0;
}

uint8_t USB_Write2(char* msg, uint8_t len) {

	char buffer [50];
	//int n;
	//n=sprintf (buffer, msg);

	CDC_Transmit_FS(buffer, 2);

	return 0;
}

uint8_t USB_WriteChar(uint8_t* character) {
  CDC_Transmit_FS(character, 1);

  return 0;
}

uint8_t USB_WriteString1(char* msg) {
	uint8_t msg_len = strlen(msg);

	for(uint8_t i = 0;i<msg_len;i+=8) {
        char msg_part[8];
        strncpy(msg_part, msg+i, 8);
        //printf(msg_part);

		int n = strlen(msg_part);
		CDC_Transmit_FS(msg_part, n);
		//int busy;
		/*while(busy) {
			USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
			if (hcdc->TxState == 0){
				busy = 0;
			}
		}*/
		//USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
		//busy = hcdc->TxState;


		//CDC_Transmit_FS("Loop", 4);
		//HAL_Delay(1);
    }

    return 0;
}

uint8_t USB_WriteString2(char* msg, uint8_t len) {

  CDC_Transmit_FS(msg, len);

  return 0;
}
