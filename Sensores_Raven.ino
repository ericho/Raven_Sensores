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
 
 
// Comandos internos del acelerometro
#define LEER_INT_STATUS 0x58
#define LEER_CTRL       0x04
#define LEER_X_MSB      0x15
#define LEER_X_LSB      0x10
#define LEER_Y_MSB      0x1C
#define LEER_Y_LSB      0x19
#define LEER_Z_MSB      0x25
#define LEER_Z_LSB      0x20
#define ESCRIBIR_CTRL   0x07

#define MAX_MILLIS_TO_WAIT 1000

// Sensores habilitados 
// Puede que no todos los sensores esten presentes en la tarjeta 
// y por lo tanto se omite su lectura
#define SENSOR_IRROMETER    1
#define SENSOR_DISTANCIA    1
#define SENSOR_PLUVIOMETRO  0
#define SENSOR_ACELEROMETRO 1


float Dx;
float Dy;
int pluv_tips;
byte tipo_nodo;
volatile int f_wdt = 1;
unsigned long tiempo_inicial;

union float_bytes{
  byte b[4];
  float val;
}u_fb;

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
 
  // Configuracion Serial 
  Serial.begin(9600);
  
  // Definiendo el tipo de nodo
  tipo_nodo = 0;
  tipo_nodo |= (SENSOR_ACELEROMETRO << 3) | (SENSOR_DISTANCIA << 2) | (SENSOR_IRROMETER << 1) | (SENSOR_PLUVIOMETRO);
  // Configuracion de interrupciones
  attachInterrupt(0, incrementaPluviometro, RISING); //Pluviometro
  pluv_tips = 0;
}


/**
* Interrupcion 0 que cuenta los golpes del pluviometro
* y almacena la informacion en una variable estatica
*/

void incrementaPluviometro(){
  static unsigned long t_ultima_int = 0;
  unsigned long t_interrupcion = millis();
  if (t_interrupcion - t_ultima_int > 200)
  {
    pluv_tips++;
  }
  t_ultima_int = t_interrupcion;

}


/**
* 
*
*/
ISR(WDT_vect){
  f_wdt++;
}

/**
* Funcion que duerme la tarjeta apagando modulos selectivamente 
*
*/

void dormirTarjeta(void){
  
  // Configura modo de ahorro de energia. 
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  // Se desactivan selectivamente los modulos que no son necesarios
  // dejando solamente al uart. 
  
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();
  
  // A dormir
  sleep_mode();
  
  // Despertando...
  
  sleep_disable();
  // Se activan todos los modulos 
  power_all_enable();
}


/**
* Conjunto de funciones que activan o desactivan a los demas sensores 
* en la tarjeta. 
*/

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

/**
* Lee un registro del acelerometro 
* Recibe la direccion del registro y devuelve en un byte el contenido del registro
*/

byte leerRegAcelerometro(byte dirRegistro)
{
  byte res;
  digitalWrite(SCA_SELECT, LOW);
  SPI.transfer(dirRegistro);
  res = SPI.transfer(0x00);
  digitalWrite(SCA_SELECT, HIGH);
  return res;
}

/**
* Escribe datos en un registro del acelerometo. 
* Recibe la direccion y los datos del registro como parametro
* Devuelve la respueta al comando enviado desde el acelerometro. 
*/

byte escribirRegAcelerometro(byte dirRegistro, byte datos){
  digitalWrite(SCA_SELECT, LOW);
  SPI.transfer(dirRegistro);
  SPI.transfer(datos);
  digitalWrite(SCA_SELECT, HIGH);
}

/**
* Realiza el proceso de inicializacion del acelerometro. 
* Verificando el estado de unos registro 
*/
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


/**
* Lee el contenido del registro X del acelerometro 
* Devuelve el valor de X en mg
*/

float leerEjeX(){
  int X;
  X = leerRegAcelerometro(LEER_X_MSB) << 8;
  X |= leerRegAcelerometro(LEER_X_LSB) & 0x00FF;
  return aMG(X);
}

/**
* Lee el contenido del registro Y del acelerometro 
* Devuelve el valor de Y en mg
*/

float leerEjeY(){
  int Y;
  Y = leerRegAcelerometro(LEER_Y_MSB) << 8;
  Y |= leerRegAcelerometro(LEER_Y_LSB) & 0x00FF;
  return aMG(Y);
}

/**
* Devuelve el promedio de 10 lecturas sobre el eje X 
* del acelerometro
*/

float leerXPromedio(){
  float X;
  for (int i=0; i<10; i++){
    X += leerEjeX();  
  }
  return X / 10;
}

/**
* Devuelve el promedio de 10 lecturas sobre el eje Y 
* del acelerometro
*/

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

/**
* Lee entrada PWM desde el sensor ultrasonico
* devuelve la distancia en centimetros
*/

float leerDist(){
  float cm;
  cm = ((float) pulseIn(IN_DIST, HIGH)) / 58; // Resolucion 58uS por cm. 
  return cm;
}

/**
* Funcion que devuelve el promedio de 10 lecturas del pulso del 
* sensor ultrasonico.
*/

float promedioDist(){
  float X;
  for (int i=0; i<10; i++){
    X += leerDist();
  }
  return X / 10;
}


/**
* Devuelve periodo desde el generador de frecuencia
* del tensiometro. 
*/
long leerIrrometer(){
  long R;
  R = pulseIn(IN_IRR, HIGH);
  return R;
}

/**
* Devuelve el promedio de periodos desde el generador
* de frecuencia del tensiometro
*/
float promedioIrrometer(){
  float R;
  for (int i=0; i<10; i++){
    R += (float) leerIrrometer();
  }
  return R / 10;
}

/*
* Funcion que se encarga de procesar los comandos recibidos por la uart
* tambien ejecuta el ciclo de recoleccion de datos de los sensores 
* y por ultimo envia la informacion por la intefaz serial. 
*/
void procesadorComandos(){
  // Esperamos por lo menos 6 bytes, si pasa un segundo sin datos disponibles, entonces salimos 
  // a volver a dormir a la tarjeta. 
  byte trama[6];
  tiempo_inicial = millis();  
  while ( (Serial.available() < 6) && ((millis() - tiempo_inicial) < MAX_MILLIS_TO_WAIT));
  
  if (Serial.available() >= 6)
  {
    for (int i=0; i<6; i++)
      trama[i] = Serial.read(); // Leyendo bytes recibidos
    if (trama[0] == 0x55 && trama[5] == 0xaa){ // Verificando las cabeceras de la trama
      // Procesando comando 
      switch(trama[4]){
        case 1:
          enviarDatosSensores();
      }
    }

  } 
}


/**
* Se encarga de recolectar los datos de los sensores y enviar la informacion
* empaquetada en la estructura del protocolo. 
* Se recolectan los datos de todos los sensores. 
*/

void enviarDatosSensores(){
  int l_datos = SENSOR_ACELEROMETRO*8 + SENSOR_DISTANCIA*4 + SENSOR_IRROMETER*4 + SENSOR_PLUVIOMETRO;
  
  byte tramaDatos[l_datos + 7];
  float tmp_x;
  float tmp_y;
  float tmp_dist;
  float tmp_irr;
  
  // Agregando el tipo de nodo
  tramaDatos[5] = tipo_nodo;
  
  // Obteniendo datos de los sensores 
  if (SENSOR_ACELEROMETRO){
    iniciarAcelerometro();
    tmp_x = leerXPromedio();
    tmp_y = leerYPromedio();
    apagarAcelerometro();
    
    // Almacenando variables en la trama 
    u_fb.val = tmp_x;
    for (int i=0; i<4; i++)
      tramaDatos[6 + i] = u_fb.b[i];
    u_fb.val = tmp_y;
    for (int i=0; i<4; i++)
      tramaDatos[10 + i] = u_fb.b[i];
  }
  if (SENSOR_DISTANCIA){
    encenderMaxSonar();
    tmp_dist = promedioDist();
    apagarMaxSonar();
    
    u_fb.val = tmp_dist;
    for (int i=0; i<4; i++)
      tramaDatos[14 + i] = u_fb.b[i];
  }
  if (SENSOR_IRROMETER){
    encenderIrrometer();
    tmp_irr = promedioIrrometer();
    apagarIrrometer();
    
    u_fb.val = tmp_irr;
    for (int i=0; i<4; i++)
      tramaDatos[14 + SENSOR_DISTANCIA*4 + i] = u_fb.b[i];
  }
  
  // Armando cabeceras 
  tramaDatos[0] = 0x55;
  tramaDatos[l_datos + 6] = 0xAA;
  // Logitud de trama 
  tramaDatos[1] = 0;
  tramaDatos[2] = (byte) (l_datos + 7) & 0x00FF; // 22 bytes
  
  // Identificador y comando 
  tramaDatos[3] = 0;
  tramaDatos[4] = 1; // Comando de respuesta
 
  // Enviando la trama por el puerto serial 
 
  for (int i=0; i<(l_datos + 7); i++)
    Serial.write(tramaDatos[i]);
}

/**
*
*
*/




// Rutina principal
void loop() {
  long antes, despues;
  float d;
  Serial.println("Durmiendo tarjeta..");
  delay(100);
  dormirTarjeta();
  if (Serial.available()){
    procesadorComandos();
  }
  Serial.println("Desperto");
//  while(!(Serial.available()));
//  Serial.read();
//  Serial.println("Durmiendo");
//  delay(100);
//  dormirTarjeta();
//  digitalWrite(ACT_ACC, HIGH);
//  delay(1000);

  delay(1000);



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
    
  */  
  //Serial.println(leerXFiltro());
//  while(1);

}
