#include "main.h"
#include "serialio.h"
#include <stdlib.h>
#include <stdio.h>

/* UART2 -> comunicação serial*/
extern UART_HandleTypeDef huart2;

/*tamanho máximo das strings recebidas e transmitidas*/
#define LEN 200

void serial_putc(int ch){
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
}

int serial_puts(char *ptr){
	int DataIdx;

	for (DataIdx = 0; DataIdx < LEN; DataIdx++){
		if(*ptr == '\0') break;
		serial_putc(*ptr++);
	}
	return DataIdx;
}

int serial_putd(int number){
	int DataIdx;
	char str[20];

	sprintf(str, "%d", number);

	DataIdx = serial_puts(str);

	return DataIdx;
}

int serial_putf(float number){
	int DataIdx;
	char str[20];

	sprintf(str, "%.2f", number);

	DataIdx = serial_puts(str);

	return DataIdx;
}

void serial_clearscr(){
	serial_puts("\x1b[1;1H");
	serial_puts("\x1b[3J");
}

void serial_nl(int i){
	for(int j=0; j<i; j++){
		serial_putc('\r');
		serial_putc('\n');
	}
}

void serial_pause(){
	uint8_t ch = 0;
	serial_puts("Presssione qualquer tecla para continuar...");
	while(!ch) HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY);
	serial_nl(1);
}

int serial_getc(){
	uint8_t ch = 0;

	/*limpa a flag de overrun antes de receber o caractere*/
	__HAL_UART_CLEAR_OREFLAG(&huart2);

	/*espera um caractere e printa ele no console*/
	HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY);
	if(ch == '\b'){	//backspace
		HAL_UART_Transmit(&huart2, (uint8_t *)"\b", 1, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, (uint8_t *)" " , 1, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, (uint8_t *)"\b", 1, HAL_MAX_DELAY);
	}else
		HAL_UART_Transmit(&huart2, &ch, 1, HAL_MAX_DELAY);

	return ch;
}

int serial_gets(char *ptr){
	int DataIdx;
	char *startPtr = ptr;

	for (DataIdx = 0; DataIdx < LEN; DataIdx++)
	{
		*ptr = serial_getc();

		if(*ptr == '\r'){	//carriage return
			*ptr = '\0';
			serial_putc('\n');
			break;
		}else if(*ptr == '\b'){	//backspace
			if(ptr!=startPtr){
				ptr--;
				DataIdx-=2;
			}else{
				DataIdx--;
			}
		}else{
			ptr++;
		}

	}

	return DataIdx;
}

int serial_getd(int *number){
	char str[20];
	int DataIdx;

	DataIdx = serial_gets(str);

	*number = atoi(str);

	return DataIdx;
}

int serial_getf(float *number){
	char str[20];
	int DataIdx;

	DataIdx = serial_gets(str);

	*number = atof(str);

	return DataIdx;
}
