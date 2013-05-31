#include <SPI.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

// Definicion de pines de activacion
#define ACT_DIST   5
#define ACT_IRR    6
#define ACT_ACC    7
#define IN_DIST    3
#define IN_IRR     4
#define IN_PLUV    2

#define SCA_SELECT 10
#define MISO       12
#define MOSI       11
#define SCK        13
 
#define LEER_INT_STATUS 0x58
#define LEER_CTRL       0x04
#define LEER_X_MSB      0x15
#define LEER_X_LSB      0x10
#define LEER_Y_MSB      0x1C
#define LEER_Y_LSB      0x19
#define LEER_Z_MSB      0x25
#define LEER_Z_LSB      0x20
#define ESCRIBIR_CTRL   0x07

float Dx;
float Dy;
int pluv_tips;
volatile int f_wdt = 1;

// Configuraciones iniciales
void setup() {            
  pinMode(SCA_SELECT, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK, OUTPUT);
  pinMode(ACT_DIST, OUTPUT);
  pinMode(ACT_IRR, OUTPUT);
  pinMode(ACT_ACC, OUTPUT);
  pinMode(IN_DIST, INPUT);
  pinMode(IN_IRR, INPUT);
  pinMode(IN_PLUV, INPUT);
  
  // Configuracion SPI
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  digitalWrite(SCA_SELECT, HIGH);
  SPI.begin();
  
  //Configuracion WDT
  MCUSR &= ~(1 << WDRF);
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR |= (1 << WDP2) | (1 << WDP1) | (1 << WDP0);
  WDTCSR |= _BV(WDIE);
  
  // Configuracion Serial 
  Serial.begin(115200);
  
  // Configuracion de interrupciones
  attachInterrupt(0, incrementaPluviometro, RISING); //Pluviometro
  pluv_tips = 0;
}

void incrementaPluviometro(){
  static unsigned long t_ultima_int = 0;
  unsigned long t_interrupcion = millis();
  if (t_interrupcion - t_ultima_int > 200)
  {
    pluv_tips++;
    
    Serial.println(pluv_tips);
  }
  t_ultima_int = t_interrupcion;

}

ISR(WDT_vect){
  if (f_wdt == 0){
    f_wdt = 1;
  }
  else{
    Serial.println("WDT!");
  }
}

void dormirTarjeta(void){
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  
  sleep_mode();
  
  // Regresa 
  
  sleep_disable();
  power_all_enable();
}


void encenderAcelerometro(){
  digitalWrite(ACT_ACC, HIGH);
  delay(35);
}

void apagarAcelerometro(){
  digitalWrite(ACT_ACC, LOW);
}

void encenderMaxSonar(){
  digitalWrite(ACT_DIST, HIGH);
  delay(180); // Esperar 180 milisegundos para que encienda
}

void apagarMaxSonar(){
  digitalWrite(ACT_DIST, LOW);
}

void encenderIrrometer(){
  digitalWrite(ACT_IRR, HIGH);
  delay(15);
}

void apagarIrrometer(){
  digitalWrite(ACT_IRR, LOW);
}

// Leer un registro del acelerometro. 
// Necesita la direccion del registro como parametro

byte leerRegAcelerometro(byte dirRegistro)
{
  byte res;
  digitalWrite(SCA_SELECT, LOW);
  SPI.transfer(dirRegistro);
  res = SPI.transfer(0x00);
  digitalWrite(SCA_SELECT, HIGH);
  return res;
}

byte escribirRegAcelerometro(byte dirRegistro, byte datos){
  digitalWrite(SCA_SELECT, LOW);
  SPI.transfer(dirRegistro);
  SPI.transfer(datos);
  digitalWrite(SCA_SELECT, HIGH);
}


byte iniciarAcelerometro(){
  byte result;
  encenderAcelerometro();
  // Leyendo registro INT_STATUS
  result = leerRegAcelerometro(LEER_INT_STATUS);
  // Escribiendo en registro CTRL
  // Esperar 10ms 
  escribirRegAcelerometro(ESCRIBIR_CTRL, 0x08);
  delay(10);
  // Leyendo CTRL de nuevo
  result = leerRegAcelerometro(LEER_CTRL);
  if (bitRead(result, 6) != 0)
  {
    return 1; // Error en bit PORST
  }

  return 0;
}

float leerEjeX(){
  int X;
  X = leerRegAcelerometro(LEER_X_MSB) << 8;
  X |= leerRegAcelerometro(LEER_X_LSB) & 0x00FF;
  return aMG(X);
}

float leerEjeY(){
  int Y;
  Y = leerRegAcelerometro(LEER_Y_MSB) << 8;
  Y |= leerRegAcelerometro(LEER_Y_LSB) & 0x00FF;
  return aMG(Y);
}


float leerXPromedio(){
  float X;
  for (int i=0; i<10; i++){
    X += leerEjeX();  
  }
  return X / 10;
}

float leerYPromedio(){
  float Y;
  for (int i=0; i<10; i++){
    Y += leerEjeY();
  }
  return Y / 10;
}

/**
* Convierte la salida del acelerometro a mg
* Se utiliza la formula A[mg] de la hoja de datos 
* del acelerometro
*/
float aMG(int lectura)
{
  float tmp_lectura; 
  tmp_lectura = (float) ( ( (lectura >> 1) & 0x3FFF) - ( (lectura >> 1) & 0x4000 ) );
  return (10 * (tmp_lectura) / 18);
}


// Rutina principal
void loop() {
//  while(!(Serial.available()));
//  Serial.read();

if (f_wdt == 1)
{
  Serial.println("Despierto");
  f_wdt = 0;
  dormirTarjeta();
}


/*
  delay(2000);
  digitalWrite(ACT_DIST, HIGH);   
  iniciarAcelerometro();
    while(!(Serial.available())){    
    
    Dx = leerEjeX();
    Dy = leerEjeY();
//    Dy = leerYPromedio(); 
    Serial.print(Dx);
    Serial.print(":");
    Serial.println(Dy);
    delay(500);
  }
  
  Serial.read();
    
    
  //Serial.println(leerXFiltro());
//  while(1);
*/
}
