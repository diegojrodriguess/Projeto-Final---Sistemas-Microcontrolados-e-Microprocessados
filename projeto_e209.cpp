#define PINO 4
#define AMOSTRAS 50
#define MOTOR (1 << PD6)
#define SENSOR_GOTAS (1 << PD2)
#define ALARME (1 << PD7)
/*
//Declarações para UART
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FOSC 16000000U
#define BAUD 9600
#define MYUBRR FOSC / 16 / (BAUD - 1)
#define TAMANHO 3

char msg_tx[20];
char msg_rx[32];
int pos_msg_rx = 0;
int tamanho_msg_rx = TAMANHO;

//void UART_config(unsigned int ubrr);
//void UART_Transmit(char *dados);
//void UART_Receive(char *dados, int tamanho);
//
*/
int contador = 0;  //variavel pro timer
float DC = 0;  //variavel auxiliar para o duty cycle
float vazao;   //variavel auxiliar para o calculo da vazao entregue
unsigned int Leitura_AD;          //variavel auxiliar para a leitura ad
float tensao;                     //tensao lida na leitura ad
unsigned int volume = 0;          //volume dado em ml que sera impresso pelo usuario
unsigned int tempo = 0;           //tempo em minutos que sera impresso pelo usuario
float tempo_horas;                //tempo em horas
float erro;                       //variavel para a formula do erro
float fluxo_def;                  //variavel para o calculo do fluxo definido para a infusao [ml/min]
float fluxo_real;                 //variavel para o fluxo real de infusao
bool flag = false;                // Se for TRUE o erro deve ser mostrado na tela
#define FLUXO_M 450               //fluxo maximo do motor, para a formula da potencia
float potencia;                   //potencia que ira pro motor
int reloop;                       //variavel auxiliar
unsigned int tempo_segundos = 0;  //tempo em segundos
unsigned int n_gotas = 0;         //numero de gotas que o sensor detectar

void configuratePWM() {
  TCCR0A = (1 << COM0A1) + (0 << COM0A0) + (1 << WGM01) + (1 << WGM00);
  TCCR0B = (1 << CS02) + (1 << CS00);
  OCR0A = 0;
}

void configurateInterruption() {
  EICRA |= (1 << ISC01) | (1 << ISC00);  // Habilita a interrupção externa INT1 na borda de subida
  EIMSK |= (1 << INT0);
}

void configurateAD() {
  ADMUX = (0 << REFS1) + (1 << REFS0);                                //Utiliza 5V como referência (1023)
  ADCSRA = (1 << ADEN) + (1 << ADPS2) + (1 << ADPS1) + (1 << ADPS0);  //Habilita ADC e PS 128 (10 bits)
  ADCSRB = 0;                                                         //Conversão Única
  DIDR0 = (1 << PINO);                                                //Desabilita o PC4 como pino digital
}

void configurateTimer2() {
  TCCR2A = (1 << WGM21);   //Configuração do modo de funcionamento para Comparador
  TCCR2B = (1 << CS21);    //Pre-scaler de 8 (Frequência de 2MHz - Período de 500 ns em cada contagem)
  OCR2A = 199;             //200 contagens de 500 ns, o que gera uma interrupção a cada 100 us
  TIMSK2 = (1 << OCIE2A);  //Gerar uma interrupção no estouro do comparador A
}

int main(void) {
  Serial.begin(9600);
  Serial.println("Bem vindo");
  //UART_config(MYUBRR);       //funcao que seria equivalente ao serial.begin
  PORTD |= SENSOR_GOTAS;     //declarando o circuito de pull-up no botao pushbotton que representa o sensor de gotas.
  DDRD |= (MOTOR + ALARME);  //declarando o motor e o alarme como saidas

  PORTD &= ~(MOTOR + ALARME);  //comecando o motor e o alarme desligados

  //configuracoes necessarias para o PWM
  configuratePWM();

  //funcao com as configuracoes das interrupcoes geradas na pb1
  configurateInterruption();

  //funcao com as configuracoes necessarias para o conversor AD
  configurateAD();

  //funcao com as configuracoes do timer
  configurateTimer2();

  sei();  //habilita interrupcoes
  for (;;) {
    /*
    UART_Transmit("Entre com o volume: \n");
    char valor[3];
    UART_Receive(valor, 3);
    volume = atoi(valor);
    */


    Serial.print("Entre com o volume: ");
    while (Serial.available() == 0)
      ;
    volume = Serial.parseInt();
    Serial.println(volume);

    /*
      UART_Transmit("Entre com o tempo de infusao em minutos: \n");
      char valor1[3];
      UART_Receive(valor1, 3);
      tempo = atoi(valor);
    */
    Serial.print("Entre com o tempo de infusao em minutos: ");
    while (Serial.available() == 0)
      ;
    tempo = Serial.parseInt();
    Serial.println(tempo);
    tempo_horas = tempo / 60;

    //calculando o fluxo definido
    fluxo_def = volume / tempo_horas;
    Serial.println(fluxo_def);
    potencia = (fluxo_def / FLUXO_M) * 100;
    Serial.println(potencia);

    DC = (255 * potencia) / 100;
    OCR0A = DC;

    flag = true;
    Serial.println("Pressione a tecla 7 se quiser alterar os parametros: ");
    while (Serial.available() == 0)
      ;
    reloop = Serial.parseInt();
    flag = false;  //o erro ira parar de mostrar.
  }
}
ISR(INT0_vect) {  //interrupcoes gerada pelo SENSOR DE GOTAS
  n_gotas++;
}



ISR(TIMER2_COMPA_vect) {
  contador++;
  if (contador >= 10000) {  // 1 segundo
    tempo_segundos++;
    contador = 0;
    //Determinar o pino de leitura
    ADMUX = (ADMUX & 0xF8) | PINO;
    //Leitura do ADC (Com média)
    unsigned int SomaLeitura = 0, MediaLeitura;
    for (int i = 0; i < AMOSTRAS; i++) {

      ADCSRA |= (1 << ADSC);  //Inicia a conversão

      while ((ADCSRA & (1 << ADSC)) == (1 << ADSC))
        ;  //Esperar a conversão

      Leitura_AD = ADC;

      SomaLeitura += Leitura_AD;
    }

    MediaLeitura = SomaLeitura / AMOSTRAS;

    tensao = (MediaLeitura * 5) / 1023.0;  //Cálculo da Tensão
    if (tensao > 3.75) {
      PORTD |= ALARME;
      OCR0A = 0;  //desligando o motor
    } else PORTD &= ~ALARME;
  }

  //a cada cinco segundos, o erro e calculado
  if (tempo_segundos % 5 == 0 && contador == 0) {
    fluxo_real = ((n_gotas / 5) * 0.05) * 3600;  //convertendo minutos para horas e gotas para ml
    erro = ((fluxo_real - fluxo_def) / fluxo_def) * 100;
    n_gotas = 0;

    if (flag == true) {
      //UART_Transmit("Erro: \n");
      //char msg [3];
      //itoa(erro,msg,3);
      //UART_Transmit(msg);
      Serial.print("Erro: ");
      Serial.println(erro);
    }
  }
}


/*
//Interrupção de Recebimento da Serial
ISR(USART_RX_vect) {

  // Escreve o valor recebido pela UART na posição pos_msg_rx do buffer msg_rx
  msg_rx[pos_msg_rx++] = UDR0;

  if (pos_msg_rx == tamanho_msg_rx)
    pos_msg_rx = 0;
}

//Transmissão de Dados Serial
void UART_Transmit(char *dados) {

  // Envia todos os caracteres do buffer dados ate chegar um final de linha
  while (*dados != 0) {
    while ((UCSR0A & (1 << UDRE0)) == 0)
      ;  // Aguarda a transmissão acabar

    // Escreve o caractere no registro de tranmissão
    UDR0 = *dados;
    // Passa para o próximo caractere do buffer dados
    dados++;
  }
}

//Configuração da Serial
void UART_config(unsigned int ubrr) {

  // Configura a  baud rate
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  // Habilita a recepcao, tranmissao e interrupcao na recepcao 
  UCSR0B = ((1 << RXCIE0) + (1 << RXEN0) + (1 << TXEN0));
  // Configura o formato da mensagem: 8 bits de dados e 1 bits de stop 
  UCSR0C = ((1 << UCSZ01) + (1 << UCSZ00));
}
void UART_Receive(char *dados, int tamanho) {
  int i;
  for (i = 0; i < tamanho - 1; i++) {
    while (!(UCSR0A & (1 << RXC0)))
      ;
    char caractere = UDR0;

    if (caractere == '\n') {
      i--;  // Descarta o caractere '\n' e lê o próximo caractere
      continue;
    }

    dados[i] = caractere;
    UART_Transmit(&dados[i]);  // Ecoa o caractere recebido de volta
  }

  dados[i] = '\0';
  UART_Transmit("\n");
}
*/