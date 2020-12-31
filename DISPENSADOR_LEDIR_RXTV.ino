/*_______________________________________________________________________________________
//                        MSP430G2452 (FLASH 8KB)
//                          +-\/-+
//                   VCC   1|    |20  GND
//             (A0)  P1.0  2|    |19  XIN   P2.6
//       (TX)  (A1)  P1.1  3|    |18  XOUT  P2.7
// (PWM) (RX)  (A2)  P1.2  4|    |17  TEST
//             (A3)  P1.3  5|    |16  RST#
//             (A4)  P1.4  6|    |15  P1.7  (A7) (SDA) (MISO) 
//      (SCLK) (A5)  P1.5  7|    |14  P1.6  (A6) (SCL) (MOSI) (PWM)
//      (CS)         P2.0  8|    |13  P2.5
//                   P2.1  9|    |12  P2.4
//                   P2.2 10|    |11  P2.3
//                          +----+
// UART por Software (SW UART) ubicar los Jumpers EN REV.1.5
_______________________________________________________________________________________
//                        MSP430G2553 (FLASH 16KB)
//                          +-\/-+
//                   VCC   1|    |20  GND
//             (A0)  P1.0  2|    |19  XIN   P2.6               (PWMx)
//       (RX)  (A1)  P1.1  3|    |18  XOUT  P2.7
//(PWMx) (TX)  (A2)  P1.2  4|    |17  TEST
//             (A3)  P1.3  5|    |16  RST#
//             (A4)  P1.4  6|    |15  P1.7  (A7) (SDA0) (MISO) 
//      (SCLK) (A5)  P1.5  7|    |14  P1.6  (A6) (SCL0) (MOSI) (PWMx)
//      (CS)         P2.0  8|    |13  P2.5                     (PWMz)
//(PWMy)(SCL1)       P2.1  9|    |12  P2.4                     (PWMz)
//(PWMy)(SDA1)       P2.2 10|    |11  P2.3
//                          +----+
// (A10) --> Temperature
// Escoges: PWM4 o PWM14 o PWM19
// Escoges: PWM9 o PWM10
// Escoges: PWM12 o PWM13
// I2C por defecto el 1
// Software I2C maestro unicamente
// Uart por Hardware (HW UART) ubicar los Jumpers EN REV.1.5

Con el MSP430G2452 y el MSP430G2553 podemos usar cualquiera de dos o tres opciones para cada pin:
P1_0 --> 2  --> RED_LED
P1_1 --> 3
P1_2 --> 4
P1_3 --> 5  --> PUSH2
P1_4 --> 6
P1_5 --> 7
_____________________________
P2_0 --> 8
P2_1 --> 9
P2_2 --> 10
P2_3 --> 11
P2_4 --> 12
P2_5 --> 13
_____________________________
P1_6 --> 14 --> GREEN_LED
P1_7 --> 15
P2_7 --> 18  marcado con XOUT
P2_6 --> 19  marcado con XIN
*/
#include <msp430.h>

const byte interruptPin = 5;//P1_3
volatile byte state = LOW;

void setup()
{
  pinMode(interruptPin, INPUT);
  pinMode(P2_6,OUTPUT); //Led Infrarrojo 
  pinMode(P1_0,OUTPUT); //Led Rojo
  pinMode(P1_4,OUTPUT); //Bomba
  pinMode(P1_6,OUTPUT); //Led Verde
  
  digitalWrite(P1_0,HIGH);//Led rojo Encendido
  digitalWrite(P1_4,HIGH);//Bomba apagada
  digitalWrite(P1_6,LOW); //Led Verde apagado
  digitalWrite(P2_6,LOW); //Led Infrarrojo apagado
  setupTimer(13);         //Arranca el timer en 13 micro segundos  
  /*
  Frecuencia
  F=38KHz
  Periodo
  T = 1/38KHz = 26.31uS
  _|¯¯¯¯¯¯|______|¯
     13uS   13uS
   |<----26uS--->|      
  */
  delay(1000);            //Para asegurar que todo inicia bien luego de energizar
}
 
void loop()
{
   if(digitalRead(interruptPin)==LOW)//5 p1_3
   {
    digitalWrite(P1_0,LOW);//led rojo apagado
    digitalWrite(P1_4,LOW);//enciende bomba (activa la base del tip127)
    digitalWrite(P1_6,HIGH);//led verde encendido
    delay(2500);
    digitalWrite(P1_0,HIGH);//led rojo encendido
    digitalWrite(P1_4,HIGH);//apaga bomba (desactiva la base del tip127)
    digitalWrite(P1_6,LOW); //led verde apagado
    while(digitalRead(interruptPin)==LOW);
   }
}
//________________________________________________________________________________
void OnTimer()
{
  static int msCount=0;// variable 
  static int state=0;  // variable
 
  msCount++;
  if (msCount >= 1)//ESTE VALOR SE MULTIPLICA POR EL QUE ESTA EN EL SETUPTIMER (setupTimer(13))
  {
    msCount = 0;
    digitalWrite(P2_6,state); // Led Infrarrojo
    state=~state;             // invierte el valor de state
  }
 }
//________________________________________________________________________________ 

void setupTimer(unsigned Period)
{
  // Configuration word
  // Bits 15-10: Unused
  // Bits 9-8: Clock source select: set to SMCLK (16MHz)
  // Bits 7-6: Input divider: set to 8
  // Bits 5-4: Mode control: Count up to TACCRO and reset
  // Bit 3: Unused
  // Bits 2: TACLR : set to initially clear timer system
  // Bit 1: Enable interrupts from TA0
  // Bit 0: Interrupt (pending) flag : set to zero (initially)
  TA0CTL=0b0000001011010010; 
  TACCR0=Period*2; // Set TACCR0 = Period*2 (2MHz clock)
  TACCTL0=BIT4; // Enable interrupts when TAR = TACCR0  
}
// The address function that follows this vector statement is placed in the specified location Interrupt Vector table 
#pragma vector=TIMER0_A0_VECTOR
__interrupt  void timerA0ISR(void)
{
// Timer A0 Interrupt service routine
  OnTimer();
  TA0CTL &= ~BIT0;     // Acknowledge the interrupt
}
