#include <avr/io.h>
#include <stdio.h>
#include <math.h>

  /**
    Configurando os pinos C
    PC0 = Associado ao sensor de luminosidade -> ADC0
    PC1 = Associado ao sensor de gás -> ADC1
    PC2 = Associado ao sensor de movimento
    PC3 = Associado ao sensor de temperatura -> ADC3
    PC4 = Associado à chave Liga/Desliga
    */

/** 
    Configurando o ADC
    Para ADCSRA: 
    	ADEN = 1 (Habilitando o ADC);
        ADPS2 = 1, ADPS1 = 1, ADPS0 = 1 (Fator de divisão do clock = 128)
    Para o ADMUX: 
    	REFS1 = 0, REFS0 = 1 (Tensão de referência: AVCC = 5V)
        MUX0 = 0, MUX1 = 0, MUX2 = 0 (Valor do registrador 0)
*/

/**
    Configurando os pinos B
    PB1 = Controle de PWM do servo motor - OC1A
    PB2 = Controle de direção do motor da direita (cima) - Positivo
    PB3 = Controle de direção do motor da direita (cima) - Negativo
    PB4 = Controle de direção do motor da esquerda (baixo) - Positivo
    PB5 = Controle de direção do motor da esquerda (baixo) - Negativo
*/

/**
    Configurando os pinos D
    PD2 = Associado ao LED verde - L1
    PD3 = Associado ao LED vermelho - L2
    PD5 = PWM do motor da direita (cima) - OC0B
    PD6 = PWM do motor da esquerda (baixo) - OC0A
    */
/**
    Configurando o PWM dos dois motores (Contador 0)
    Para TCCR0A:
    	COM0A1 = 1, COM0A0 = 0 (Sinal começa alto e termina baixo)
        COM0B1 = 1, COM0B0 = 0 (Sinal começa alto e termina baixo)
        WGM01 = 1, WGM00 = 1 (Fast PWM de 8 bits)
    Para o TCCR0B:
    	WGM02 = 0 (Fast PWM de 8 bits)
        CS02 = 0, CS01 = 0, CS00 = 1 (Prescaling de 64, frequência de contagem = 250KHz)
    	Período = 1,024 ms e frequência = 976Hz (PWM)
    */

  /**
  	Configurando o servo motor
 	Para o TCCR1A:
    	COM1A1 = 1, COM1A0 = 0 (Sinal começa alto e termina baixo)
        WGM11 = 1, WGM10 = 1 (Fast PWM de 10 bits)
    Para o TCCR1B:
    	WGM12 = 1 (Fast PWM de 10 bits)
        CS12 = 0, CS11 = 1, CS10 = 1 (Prescaling de 64, frequência de contagem = 250KHz)
    	Período = 4,096 ms e frequência = 244Hz (PWM)
    */
/*
raio da roda = 3,5cm
velocidade angular maxima = 17,80 rad/s
velocidade linear maxima = 62,3 cm/s
V(0-1) = 46.725cm/s
V(1-2) = 31.15cm/s
V(2-3) = 15.57cm/s
V(3-4) = 31.15cm/s
V(4-5) = 62.3cm/s
Tempo/trajeto = 4s
PONTOS = (0,0); (0, 186.9); (124.6,186.9); (124.6, 249.66); (250.12,249.66); (250.12, 0)
L = 10cm
*/


float cX = 0, cY = 0;

int positionCurrent = 1; 

bool keepWalking = false;

/*Velocity linear max cm/s*/
float vmaxCM = 62.30;

float periodoClock = 16.384 / 1000;

//velocity rotate 
int vRotate = 64;
int cont = 0;
bool realizeMeasurement = false;
int vCurrent = 0;

//Counter 2 overflow interruption
int powerEngines = 255;


int v01 = 191; 
int v12 = 127; 
int v23 = 64;  
int v34 = 127; 
int v45 = 255; 

void walk()
{

  float increase = ((float)vCurrent / powerEngines) * vmaxCM * periodoClock;

  switch (positionCurrent)
  {
  case 1:
    cY = cY + increase;
    break;
  case 2:
    cX = cX + increase;
    break;
  case 3:
    cY = cY - increase;
    break;
  case 4:
    cX = cX - increase;
    break;
  }
}

// setting data to use USART
void configUSART()
{
  // setting to 38.4 kbps
  UBRR0L = 25;
  UBRR0H = 0;

  // enable transmission and reception data (USART)
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0);

  // frame  8 bits
  UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
}

ISR(TIMER2_OVF_vect)
{
  if (cont != 0)
  {
    if (cont % 61 == 0)
    {
      USART_write_txt("Coordenada X: ");
      txValueFloat(cX);
      USART_write_txt("\n");
      USART_write_txt("Coordenada Y: ");
      txValueFloat(cY);
      USART_write_txt("\n");
    }
    else if (cont % 246 == 0)
    {
      USART_write_txt("Coordenada X: ");
      txValueFloat(cX);
      USART_write_txt("\n");
      USART_write_txt("Coordenada Y: ");
      txValueFloat(cY);
      USART_write_txt("\n");
      stop();
      cont = 0;
    }
  }
  cont++;

  if (keepWalking)
  {
    walk();
  }
}

// data transmission (USART)
void txByte(uint8_t data)
{
  while (!(UCSR0A & (1 << UDRE0)))
    ;
  UDR0 = data;
}

// data read (USART)
uint8_t rxByte()
{
  while (!(UCSR0A & (1 << RXC0)))
    ;
  return UDR0;
}

// sending data char to USART
void USART_write_txt(char *txt)
{
  while (*txt != 0x00)
  {
    txByte(*txt);
    txt++;
  }
}



int readPortsADC(int opcao)
{
  ADMUX &= 0b11111000;

  switch (opcao)
  {
  case 0:
    //light sensor
    ADMUX |= 0b00000000; 
  case 1:
    //sensor de gás
    ADMUX |= 0b00000001; 
    break;
  case 2:
    //sensor de temperatura
    ADMUX |= 0b00000011; 
    break;
  }

  ADCSRA |= 0b01000000; // initializing ADC
  while (!(ADCSRA & 0b00010000)); 

  return ADC;
}

int readLight()
{
  return readPortsADC(0);
}

int readGas()
{
  return readPortsADC(1);
}

int readTemp()
{
  return readPortsADC(2);
}

int readMov()
{
  if ((PINC & 0b00000100) == 0b00000100)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void txValueFloat(float number)
{
  int partInt = (int)number;

  float resto = number - partInt;

  resto = resto * 1000;

  int parteInteiraResto = (int)resto;

  txValueInt(partInt);
  USART_write_txt(".");

  if (resto < 100)
  {
    txValueInt(0);
  }
  if (resto < 10)
  {
    txValueInt(0);
  }
  txValueInt(parteInteiraResto);
}

void txValueInt(int number)
{
  if (number != 0)
  {
    int n = log10(number) + 1;
    int i = n - 1;
    int x = 0;

    while (i >= 0)
    {
      int pot10 = 1;
      int aux = i;

      while (aux > 0)
      {
        pot10 *= 10;
        aux--;
      }

      x = number / pot10;
      txByte(x + '0');

      number = number % pot10;
      i--;
    }
  }
  else
  {
    txByte('0');
  }
}

//1 = gas, 2 = light, 3 = temperature
void sendingData(char *dataType, int info, int sensor)
{
  USART_write_txt(dataType);

  //GAS
  if(sensor == 1 && (info >=300 && info < 400)){
      USART_write_txt("baixo");
      USART_write_txt("\n");
  }  
  if(sensor == 1 && (info >=400 && info < 500)){
      USART_write_txt("medio");
      USART_write_txt("\n");
  } 
  if(sensor == 1 && (info >=500 && info < 600)){
      USART_write_txt("alto");
      USART_write_txt("\n");
  } 
  if(sensor == 1 && (info >=600)){
      USART_write_txt("muito alto");
      USART_write_txt("\n");
  } 
/*-----------------------------------------------------------*/
    //luz
  if(sensor == 2 && (info >=0 && info < 150)){
      USART_write_txt("baixo");
      USART_write_txt("\n");
  }  
  
  if(sensor == 2 && (info >=150 && info < 300)){
      USART_write_txt("medio");
      USART_write_txt("\n");
  } 
  if(sensor == 2 && (info >=300 && info < 450)){
      USART_write_txt("alto");
      USART_write_txt("\n");
  } 
  if(sensor == 2 && (info >450)){
      USART_write_txt("muito alto");
      USART_write_txt("\n");
  } 
  /*-------------------------------------------------*/
  if(sensor == 3  && (info >=20 && info < 80)){
      USART_write_txt("baixo");
      USART_write_txt("\n");
  }  
  if(sensor == 3  && (info >=80 && info < 160)){
      USART_write_txt("medio");
      USART_write_txt("\n");
  }  
  if(sensor == 3  && (info >=160 && info < 240)){
      USART_write_txt("alto");
      USART_write_txt("\n");
  }  
  if(sensor == 3  && (info >240)){
      USART_write_txt("muito alto");
      USART_write_txt("\n");
  }    
  /*-------------------------------------------------*/
  if(sensor == 4 && (info == 0)){
      USART_write_txt("Desligado: ");
      txValueInt(info);  
      USART_write_txt("\n");
  }  
  if(sensor == 4 && (info == 1)){
      USART_write_txt("Ligado: ");
      txValueInt(info);
      USART_write_txt("\n");
  }  
  
}

void verify()
{
  PORTD |= 0b00001000; //on LED L2
  int gas, luz, mov, temp;
  char *valorGas;
  USART_write_txt("--------------------------------------------------> ");
  USART_write_txt("Girando servo motor em -90 graus\n");

  //make measurement (-90º)
  OCR1A = 0;
  gas = readGas();
  sendingData("Valor do sensor de gas: ", gas, 1);
  mov = readMov();
  sendingData("Estado do sensor de movimento: ", mov, 4);

  USART_write_txt("Realizando medicao em 0 graus\n");
  //make measurement (0º)
  OCR1A = 375;
  gas = readGas();
  sendingData("Valor do sensor de gas: ", gas, 1);
  luz = readLight();
  sendingData("Valor do sensor de luminosidade: ", luz, 2);
  mov = readMov();
  sendingData("Estado do sensor de movimento: ", mov, 4);
  temp = readTemp();
  sendingData("Valor do sensor de temperatura: ", temp, 3);

  USART_write_txt("Girando servo motor em 90 graus\n");
  //make measurement (90º)
  OCR1A = 700;
  gas = readGas();
  sendingData("Valor do sensor de gas: ", gas, 1);
  mov = readMov();
  sendingData("Estado do sensor de movimento: ", mov, 4);
  USART_write_txt("---------------------------------------------------> ");

  //back initial state (0º)
  OCR1A = 375;
  realizeMeasurement = false;
  PORTD &= 0b11110111; //of LED L2
}

void rotate(bool deveGirarDireita)
{

  PORTB &= 0b11000011; //stop robô

  OCR0A = vRotate; //velocity of spin
  OCR0B = vRotate; //velocity of spin

  if (deveGirarDireita)
  {
    USART_write_txt("Rotacionando 90 graus para a direita.\n");
    PORTB |= 0b00011000; //spin 90º right
  }
  else
  {
    USART_write_txt("Rotacionando 90 graus para a esquerda\n");
    PORTB |= 0b00100100; //spin 90º left
  }
  _delay_ms(500);

  PORTB &= 0b11000011; //stop robô
  OCR0A = 0;
  OCR0B = 0;

  if (deveGirarDireita)
  {
    updatePostion(1);
  }
  else
  {
    updatePostion(-1);
  }
}

void finish()
{
  USART_write_txt("parando o robo...\n");
  PORTB &= 0b11000011; //Parar robô

  OCR0A = vRotate; //velocity of spin
  OCR0B = vRotate; //velocity of spin

  PORTB |= 0b00011000; //rotate to right 

  for (int i = 0; i < 8; i++)
  {
    PORTD |= 0b00001100; //on LED L1 e L2
    _delay_ms(200);
    PORTD &= 0b11110011; // off LED L1 e L2
    _delay_ms(200);
  }
  //stop robô
  PORTB &= 0b11000011; 
  OCR0A = 0;
  OCR0B = 0;
}

void updatePostion(int giro)
{
  positionCurrent = positionCurrent + giro;

  if (positionCurrent == 0)
  {
    positionCurrent = 4;
  }
  else if (positionCurrent == 5)
  {
    positionCurrent = 1;
  }
}

void InitWalk(int vel)
{
  //stop robô
  PORTB &= 0b11000011; 
  //on engines
  PORTB |= 0b00010100;  

  OCR0A = vel;
  OCR0B = vel;

  vCurrent = vel;
  keepWalking = true;

  TCNT2 = 0b00000000;

  TIMSK2 = 0b00000001;
  USART_write_txt("Iniciando trajeto do robo...\n");
}

void stop()
{
  //stop robô
  PORTB &= 0b11000011; 

  OCR0A = 0;
  OCR0B = 0;

  keepWalking = false;
  TIMSK2 = 0b00000000;
}

void setup()
{
  
  /**
    Configurando os pinos B
    PB1 = Controle de PWM do servo motor - OC1A
    PB2 = Controle de direção do motor da direita (cima) - Positivo
    PB3 = Controle de direção do motor da direita (cima) - Negativo
    PB4 = Controle de direção do motor da esquerda (baixo) - Positivo
    PB5 = Controle de direção do motor da esquerda (baixo) - Negativo
*/
  DDRB |= 0b00111110;
  PORTB &= 0b11000001;

  /**
    Configurando os pinos D
    PD2 = Associado ao LED verde - L1
    PD3 = Associado ao LED vermelho - L2
    PD5 = PWM do motor da direita (cima) - OC0B
    PD6 = PWM do motor da esquerda (baixo) - OC0A
    */
  DDRD |= 0b01101100;
  PORTD &= 0b10010011;

  /**
    Configurando os pinos C
    PC0 = Associado ao sensor de luminosidade -> ADC0
    PC1 = Associado ao sensor de gás -> ADC1
    PC2 = Associado ao sensor de movimento
    PC3 = Associado ao sensor de temperatura -> ADC3
    PC4 = Associado à chave Liga/Desliga
    */
  DDRC &= 0b11100000;

  /** 
    Configurando o ADC
    Para ADCSRA: 
    	ADEN = 1 (Habilitando o ADC);
        ADPS2 = 1, ADPS1 = 1, ADPS0 = 1 (Fator de divisão do clock = 128)
    Para o ADMUX: 
    	REFS1 = 0, REFS0 = 1 (Tensão de referência: AVCC = 5V)
        MUX0 = 0, MUX1 = 0, MUX2 = 0 (Valor do registrador 0)
    */
  ADCSRA = 0b10000111;
  ADMUX = 0b01000000;

  /**
    Configurando o PWM dos dois motores (Contador 0)
    Para TCCR0A:
    	COM0A1 = 1, COM0A0 = 0 (Sinal começa alto e termina baixo)
        COM0B1 = 1, COM0B0 = 0 (Sinal começa alto e termina baixo)
        WGM01 = 1, WGM00 = 1 (Fast PWM de 8 bits)
    Para o TCCR0B:
    	WGM02 = 0 (Fast PWM de 8 bits)
        CS02 = 0, CS01 = 0, CS00 = 1 (Prescaling de 64, frequência de contagem = 250KHz)
    	Período = 1,024 ms e frequência = 976Hz (PWM)
    */
  TCCR0A = 0b10100011;
  TCCR0B = 0b00000011;

  /**
  	Configurando o servo motor
 	Para o TCCR1A:
    	COM1A1 = 1, COM1A0 = 0 (Sinal começa alto e termina baixo)
        WGM11 = 1, WGM10 = 1 (Fast PWM de 10 bits)
    Para o TCCR1B:
    	WGM12 = 1 (Fast PWM de 10 bits)
        CS12 = 0, CS11 = 1, CS10 = 1 (Prescaling de 64, frequência de contagem = 250KHz)
    	Período = 4,096 ms e frequência = 244Hz (PWM)
    */
  TCCR1A = 0b10000011;
  TCCR1B = 0b00001011;

  /* 
  Configurando o contador 2 de 8 bits no modo mais básico 
  Divisão do clock em prescale de 1024:
  CS22 = 1, CS21 = 1, CS20 = 1 
  Com isso o período do PWM é 16,384 ms (61 Hz)
  */
  TCCR2A = 0b00000000;
  TCCR2B = 0b00000111; // DIV o clock por 8

  // no enable interruption overflow
  TIMSK2 = 0b00000000;

  // enable flag global interruption
  sei();

  OCR1A = 375;

  configUSART();
}

void loop()
{
  char in;

  while (in != '1')
  {
    USART_write_txt("Informe 1 para ativar o robo:\n");
    in = rxByte();
    if (in != '1')
    {
      USART_write_txt("Erro\n");
    }
  }

  PORTD |= 0b00000100; //on LED L1

  _delay_ms(200);

  USART_write_txt("Saindo do ponto 0 ate o ponto 1\n");
  InitWalk(v01); //walk 0 a 1
  while (keepWalking);
  verify(); //realize measurement em P1
  rotate(true);

  USART_write_txt("Saindo do ponto 1 ate o ponto 2\n");
  InitWalk(v12); //walk 1 a 2
  while (keepWalking);
  verify(); //realize measurement em P2
  rotate(false);

  USART_write_txt("Saindo do ponto 2 ate o ponto 3\n");
  InitWalk(v23); //walk 2 a 3
  while (keepWalking);
  verify(); 
  rotate(true);

  USART_write_txt("Saindo do ponto 3 ate o ponto 4\n");
  InitWalk(v34); //walk 3 a 4
  while (keepWalking);
  verify(); 
  rotate(true);

  USART_write_txt("Saindo do ponto 4 ate o ponto 5\n");
  InitWalk(v45); //walk 4 a 5
  while (keepWalking);

  finish();
  USART_write_txt("Sistema desativado\n");
  PORTD &= 0b11111011; //off LED L1
  in = 'a';
}
