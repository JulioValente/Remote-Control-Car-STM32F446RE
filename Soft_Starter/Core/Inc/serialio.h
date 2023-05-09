#ifndef INC_SERIALIO_H_
#define INC_SERIALIO_H_

/*manda 1 caractere na porta serial*/
void serial_putc(int ch);

/*manda 1 string na porta serial*/
int serial_puts(char *ptr);

/*manda 1 número na porta serial*/
int serial_putd(int number);

/*manda 1 float na porta serial*/
int serial_putf(float number);

/*limpa a tela*/
void serial_clearscr();

/*pula uma linha i vezes*/
void serial_nl(int i);

/*pausa o programa e espera o usuário digitar algum caractere*/
void serial_pause();

/*recebe 1 caractere da porta serial*/
int serial_getc();

/*recebe 1 string da porta serial*/
int serial_gets(char *ptr);

/*recebe 1 inteiro da porta serial*/
int serial_getd(int *number);

/*recebe 1 float da porta serial*/
int serial_getf(float *number);

#endif /* INC_SERIALIO_H_ */
