//ROBOT SUMO GODZILLA V 1.0 BETA Crado por Grupo de Trabajo 2
//------------------------------------------------------------------------------------------------------------------
//Diagrama Componenetes
//     {LFL}        {LFR}
//  (__)-|M1|--S0--|M2|-(__)
//             |
//           [PH2]
//      [PH1]  |  [PH3]   
//          ___|____
//     S2--|___B1___}--S3    
//           |_B2_}
//      {C}    | 
//            [F]
//           [_AM_]P}
//            [A]
//             |
//  (__)-|M3|--S1--|M4|-(__)
//     {LBL}        {LBR}
//-------------------------------------------------------------------------------------------------------------------
//Librerias
#include <NewPing.h>
#include <Wire.h>
//-------------------------------------------------------------------------------------------------------------------
//VARIABLES
//-------------------------------------------------------------------------------------------------------------------
//Variables Sensores Ultrasonicos - 22-23 sensor frontal, 24-25 sensor trasero, 26-27 sensor izquierdo, 28-29 sensor derecho
#define SONAR_NUM     4 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 30 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
NewPing sonar[SONAR_NUM] = {NewPing(22, 23, MAX_DISTANCE), NewPing(24, 25, MAX_DISTANCE), NewPing(26, 27, MAX_DISTANCE), NewPing(28, 29, MAX_DISTANCE)};//trigger y echo
int sensorUS[4] = {0,0,0,0};
//int antsensorUS[4] = {0,0,0,0};
const int DistAtac = 20;//50 
bool atacFront = true;
bool atacBack = true;
//senorUF = senorUS[0]
//senorUB = senorUS[1]
//senorUL = senorUS[2]
//senorUR = senorUS[3]
//-------------------------------------------------------------------------------------------------------------------
//Variables Boton Arranque - pin 31
const int pinStartButtom = 31;//interruptor de inicio
int startButtom;
//-------------------------------------------------------------------------------------------------------------------
//Variables Sesnosres de Linea - sensor frontal izquierdo pin 32, sensor frontal derecho pin 33, sensor trasero izquierdo pin 34, sensor trasero derecho pin 35
const int pinLineFrontL = 32;
const int pinLineFrontR = 33;
const int pinLineBackL = 34;
const int pinLineBackR = 35;
int lineFrontL;
int lineFrontR;
int lineBackL;
int lineBackR;
//---------------------------------------------------------------------------------------------------------------------
//Variables Motores - 40-41 motor frontal izquierdo 3 enableA PH1, 42-43 motor frontal derecho 4 enableB PH2, 44-45 motor trasero izquierdo 5 enableA PH2, 46-47 motor trasero derecho 6 enableA PH3
// MOTOR 1      
const int M1PH1_IN1 = 40;                         
const int M1PH1_IN2 = 41;                         
const int M1PH1_ENA = 3;                          
int velMaxM1 = 255;                               
//int velM1;                                      
// MOTOR 2
const int M2PH2_IN3 = 42; 
const int M2PH2_IN4 = 43;    
const int M2PH2_ENB = 4;
int velMaxM2 = 255;
//int velM2;
// MOTOR 3
const int M3PH2_IN1 = 44; 
const int M3PH2_IN2 = 45;    
const int M3PH2_ENA = 5;
int velMaxM3 = 255;
//int velM3;
// MOTOR 4
const int M4PH3_IN1 = 46; 
const int M4PH3_IN2 = 47;    
const int M4PH3_ENA = 6;
int velMaxM4 = 255;
//int velM4;
const int TimeMov = 3000;
const int TimeMov2 = TimeMov+TimeMov;
//------------------------------------------------------------------------------------------------------------------------
//Variables Acelerometro
#define MPU 0x68 //Direccion I2C de la IMU
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ; //MPU-6050 da los valores en enteros de 16 bits
//------------------------------------------------------------------------------------------------------------------------
//Variables Sensor de Corriente
const int pinCorriente = A3;
double Corriente;
double promI;
int contI;
double ConsumoWatt;
double ConsumoJoule;
double totalConsumJoule;
double CapCarga = 5.5;//Amperios
double CapVoltaje = 12.0;//Voltios
double EnergiaBateria = CapCarga*3600.0*CapVoltaje;//Energia que posee la bateria en joule
bool LowBattery1 = true;
bool LowBattery2 = true;
bool LowBattery3 = true;
//------------------------------------------------------------------------------------------------------------------------
//Variables Auxiliares
bool EstadoAtacar = false;
bool EstadoBuscar = false;
const int pinLEDBuscarAtacar = 52;
const int pinLEDDetecLine = 51;
bool Start = false;
bool WaitTime = false;
unsigned long delayant1;
unsigned long delaynow1;
unsigned long delayant2;
unsigned long delaynow2;
unsigned long delayant3;
unsigned long delaynow3;
//unsigned long delayGiroant;
//unsigned long delayGironow;
//------------------------------------------------------------------------------------------------------------------------

void setup() 
{
//------------------------------------------------
//Sensores Ultrasonicos
  pingTimer[0] = millis() + 75;           
  for (uint8_t i = 1; i < SONAR_NUM; i++)
  {
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  }
//-------------------------------------------------
//Boton Arranque
  pinMode(pinStartButtom,INPUT);
//-------------------------------------------------
//Sesores de Linea
  pinMode(pinLineFrontL,INPUT);
  pinMode(pinLineFrontR,INPUT);
  pinMode(pinLineBackL,INPUT);
  pinMode(pinLineBackR,INPUT);
//-------------------------------------------------
//Motores
//Motor 1
  pinMode(M1PH1_IN1, OUTPUT);
  pinMode(M1PH1_IN2, OUTPUT);
  pinMode(M1PH1_ENA, OUTPUT);
  digitalWrite(M1PH1_IN1, LOW);
  digitalWrite(M1PH1_IN2, LOW);
//Motor 2
  pinMode(M2PH2_IN3, OUTPUT);
  pinMode(M2PH2_IN4, OUTPUT);
  pinMode(M2PH2_ENB, OUTPUT);
  digitalWrite(M2PH2_IN3, LOW);
  digitalWrite(M2PH2_IN4, LOW);
//Motor 3
  pinMode(M3PH2_IN1, OUTPUT);
  pinMode(M3PH2_IN2, OUTPUT);
  pinMode(M3PH2_ENA, OUTPUT);
  digitalWrite(M3PH2_IN1, LOW);
  digitalWrite(M3PH2_IN2, LOW);
//Motor 4
  pinMode(M4PH3_IN1, OUTPUT);
  pinMode(M4PH3_IN2, OUTPUT);
  pinMode(M4PH3_ENA, OUTPUT);
  digitalWrite(M4PH3_IN1, LOW);
  digitalWrite(M4PH3_IN2, LOW);
//-------------------------------------------------  
//Acelerometro
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
//-------------------------------------------------
//Auxiliares
  Start = false;
  WaitTime = false;
  pinMode(pinLEDBuscarAtacar, OUTPUT);
  pinMode(pinLEDDetecLine, OUTPUT);
  digitalWrite(pinLEDBuscarAtacar, LOW);
  digitalWrite(pinLEDDetecLine, LOW);
}
//-------------------------------------------------

void loop() 
//Programa de Inicio
{
  readSensorUS();
  startButtom = digitalRead(pinStartButtom);
  
  if(startButtom == HIGH)
  {
    WaitTime = true;
    Start = false;
    digitalWrite(pinLEDBuscarAtacar, HIGH);
    digitalWrite(pinLEDDetecLine, HIGH);
  }
  else if(startButtom == LOW)
  {
    WaitTime = false;
    Start = false;
    EstadoAtacar = false;
    EstadoBuscar = false;
    digitalWrite(pinLEDBuscarAtacar, LOW);
    digitalWrite(pinLEDDetecLine, LOW);
    AvanzarGirar(0);
  }
  
  delayant1 = millis();
  while(WaitTime && startButtom)
  { 
    readSensorUS();
    delaynow1 = millis() - delayant1;
    if(delaynow1 >= 5000)
    {
      Start = true;
      EstadoAtacar = false;
      EstadoBuscar = true;
      WaitTime = false;
    }
    startButtom = digitalRead(pinStartButtom);
  }
//-----------------------------------------------------  
//Programa Principla  
  
  while(Start && startButtom)
  {
    readSensorUS();
    Buscar();
    startButtom = digitalRead(pinStartButtom);//ultimo
  }
}


//-------------------------------------------------------------------------------
//Funciones

int Buscar()
{
  delayant3 = millis();
  while(EstadoBuscar && startButtom)
  {
    readSensorUS();
    lineFrontL = digitalRead(pinLineFrontL);
    lineFrontR = digitalRead(pinLineFrontR);
    lineBackL = digitalRead(pinLineBackL);
    lineBackR = digitalRead(pinLineBackR);
    digitalWrite(pinLEDBuscarAtacar, LOW);
    digitalWrite(pinLEDDetecLine, LOW);
    if(lineFrontL == LOW || lineFrontR == LOW || lineBackL  == LOW || lineBackR  == LOW)
    {
      DetecLine();
    }
    else
    {//Programa de Busqueda
      if((sensorUS[0] > DistAtac || sensorUS[0] == 0) and (sensorUS[1] > DistAtac || sensorUS[1] == 0) and (sensorUS[2] > DistAtac || sensorUS[2] == 0) and (sensorUS[3] > DistAtac || sensorUS[3] == 0) and (startButtom))
      {
        AvanzarGirar(4);
      }
      else
      {
        EstadoAtacar = true;
        EstadoBuscar = false;
        atacFront = true;
        atacBack = true;
        AvanzarGirar(0); 
        Atacar();
      }
      delaynow3 = millis() - delayant3;//Verificacion de consumo de energia
      if(delaynow3 >= 1000)
      {
        readPower();
        delayant3 = millis();
      }
      startButtom = digitalRead(pinStartButtom);
    }
  }
}


int Atacar()
{ 
  delayant3 = millis();
  while(EstadoAtacar && startButtom)
  {
    readSensorUS();
    lineFrontL = digitalRead(pinLineFrontL);
    lineFrontR = digitalRead(pinLineFrontR);
    lineBackL = digitalRead(pinLineBackL);
    lineBackR = digitalRead(pinLineBackR);
    digitalWrite(pinLEDBuscarAtacar, HIGH);
    digitalWrite(pinLEDDetecLine, LOW);
    if(lineFrontL == LOW || lineFrontR == LOW || lineBackL  == LOW || lineBackR  == LOW)
    {
      DetecLine();
    }
    else
    {//Programa de Ataque
      if((sensorUS[0] <= DistAtac && sensorUS[0] != 0) and (atacFront) and (startButtom))
      {
        AvanzarGirar(1);
        atacBack = false;
      }
      else if((sensorUS[1] <= DistAtac && sensorUS[1] != 0) and (atacBack) and (startButtom))
      {
        AvanzarGirar(2); 
        atacFront = false;
      }
      else if((sensorUS[2] <= DistAtac && sensorUS[2] != 0) and (startButtom))
      {
        delayant3 = millis();
        while(startButtom)
        {
          readSensorUS();
          lineFrontL = digitalRead(pinLineFrontL);
          lineFrontR = digitalRead(pinLineFrontR);
          lineBackL = digitalRead(pinLineBackL);
          lineBackR = digitalRead(pinLineBackR);
          digitalWrite(pinLEDBuscarAtacar, HIGH);
          digitalWrite(pinLEDDetecLine, LOW);
          if(lineFrontL == LOW || lineFrontR == LOW || lineBackL  == LOW || lineBackR  == LOW)
          {
            DetecLine();
          }
          else
          {
            AvanzarGirar(3);
            if(sensorUS[0] <= DistAtac && sensorUS[0] != 0)
            {
              atacFront = true;
              atacBack = false;
              AvanzarGirar(0);
              break;
            }
            else if(sensorUS[1] <= DistAtac && sensorUS[1] != 0)
            {
              atacBack = true;
              atacFront = false;
              AvanzarGirar(0);
              break;
            }
            if(sensorUS[3] <= DistAtac && sensorUS[3] != 0)
            {
              AvanzarGirar(0);
              break;
            }
           }
          delaynow3 = millis() - delayant3;//Verificacion de consumo de energia
          if(delaynow3 >= 1000)
          {
            readPower();
            delayant3 = millis();
          }
          startButtom = digitalRead(pinStartButtom);
        }
      }
      else if((sensorUS[3] <= DistAtac && sensorUS[3] != 0) and (startButtom))
      {
        delayant3 = millis();
        while(startButtom)
        {
          readSensorUS();
          lineFrontL = digitalRead(pinLineFrontL);
          lineFrontR = digitalRead(pinLineFrontR);
          lineBackL = digitalRead(pinLineBackL);
          lineBackR = digitalRead(pinLineBackR);
          digitalWrite(pinLEDBuscarAtacar, HIGH);
          digitalWrite(pinLEDDetecLine, LOW);
          if(lineFrontL == LOW || lineFrontR == LOW || lineBackL  == LOW || lineBackR  == LOW)
          {
            DetecLine();
          }
          else
          {
            AvanzarGirar(4);
            if(sensorUS[0] <= DistAtac && sensorUS[0] != 0)
            {
              atacFront = true;
              atacBack = false;
              AvanzarGirar(0);
              break;
            }
            else if(sensorUS[1] <= DistAtac && sensorUS[1] != 0)
            {
              atacBack = true;
              atacFront = false;
              AvanzarGirar(0);
              break;
            }
            if(sensorUS[2] <= DistAtac && sensorUS[2] != 0)
            {
              AvanzarGirar(0);
              break;
            }
          }
          delaynow3 = millis() - delayant3;//Verificacion de consumo de energia
          if(delaynow3 >= 1000)
          {
            readPower();
            delayant3 = millis();
          }
          startButtom = digitalRead(pinStartButtom);
        }
      }
      else
      {
        if(sensorUS[0] != 0 && sensorUS[1] != 0)
        {
          atacFront = true;
          atacBack = true;
        }
      }
    }
    delaynow3 = millis() - delayant3;//Verificacion de consumo de energia
    if(delaynow3 >= 1000)
    {
      readPower();
      delayant3 = millis();
    }
    startButtom = digitalRead(pinStartButtom);
    if((sensorUS[0] > DistAtac) and (sensorUS[1] > DistAtac) and (sensorUS[2] > DistAtac) and (sensorUS[3] > DistAtac) and (startButtom))
      {
        AvanzarGirar(0);
        EstadoAtacar = false;
        EstadoBuscar = true;
      }
    if(mpuAcelX() < -6500 && sensorUS[0] <= DistAtac && sensorUS[0] != 0)
    {
      Huir();
    }
  }
}


int Huir()
{ 
  digitalWrite(pinLEDBuscarAtacar, HIGH);
  digitalWrite(pinLEDDetecLine, HIGH);
  if(lineFrontL == LOW || lineFrontR == LOW || lineBackL  == LOW || lineBackR  == LOW)
  {
    DetecLine();
  }
  else
  {
    delayant2 = millis();
    while(startButtom)
    {
      readSensorUS();
      lineFrontL = digitalRead(pinLineFrontL);
      lineFrontR = digitalRead(pinLineFrontR);
      lineBackL = digitalRead(pinLineBackL);
      lineBackR = digitalRead(pinLineBackR);
      if(lineFrontL == LOW || lineFrontR == LOW || lineBackL  == LOW || lineBackR  == LOW)
      {
        DetecLine();
        break;
      }
      else
      {
        delaynow2 = millis() - delayant2;
        if(delaynow2 >= 0 && delaynow2 < TimeMov)
        {
          AvanzarGirar(4);
        }
        else if(delaynow2 >= TimeMov && delaynow2 < TimeMov2)
        {
          AvanzarGirar(1);
        }
        else if(delaynow2 > TimeMov2)
        {
          break;
        }
        startButtom = digitalRead(pinStartButtom);
      }
    }
  }
  digitalWrite(pinLEDBuscarAtacar, LOW);
  digitalWrite(pinLEDDetecLine, LOW);
}


int DetecLine()
{
  digitalWrite(pinLEDDetecLine, HIGH);
  digitalWrite(pinLEDBuscarAtacar, LOW);
  if(lineFrontL == LOW && lineFrontR == LOW)
  {
    delayant2 = millis();
    while(startButtom)
    {
      readSensorUS();
      lineFrontL = digitalRead(pinLineFrontL);
      lineFrontR = digitalRead(pinLineFrontR);
      lineBackL = digitalRead(pinLineBackL);
      lineBackR = digitalRead(pinLineBackR);
      if(lineBackL  == LOW || lineBackR  == LOW)
      {
        break;
      }
      delaynow2 = millis() - delayant2;
      if(delaynow2 >= 0 && delaynow2 < TimeMov)
      {
        AvanzarGirar(2);
      }
      if(delaynow2 > TimeMov)
      {
        break;
      }
      if((EstadoAtacar == true) and (lineFrontL == LOW || lineFrontR == LOW))
      {
        EstadoAtacar = false;
        EstadoBuscar = true;
      }
      startButtom = digitalRead(pinStartButtom);
    }
  }

  else if(lineBackL  == LOW && lineBackR  == LOW)
  {
    delayant2 = millis();
    while(startButtom)
    {
      readSensorUS();
      lineFrontL = digitalRead(pinLineFrontL);
      lineFrontR = digitalRead(pinLineFrontR);
      lineBackL = digitalRead(pinLineBackL);
      lineBackR = digitalRead(pinLineBackR);
      if(lineFrontR == LOW || lineBackL  == LOW || lineBackR  == LOW)
      {
        break;
      }
      delaynow2 = millis() - delayant2;
      if(delaynow2 >= 0 && delaynow2 < TimeMov)
      {
        AvanzarGirar(1);
      }
      else if(delaynow2 > TimeMov)
      {
        break;
      }
      if((EstadoAtacar == true) and (lineBackL == LOW || lineBackR == LOW))
      {
        EstadoAtacar = false;
        EstadoBuscar = true;
      }
      startButtom = digitalRead(pinStartButtom);
    }
  }
  
  
  else if(lineFrontL == LOW)
  {
    delayant2 = millis();
    while(startButtom)
    {
      readSensorUS();
      lineFrontL = digitalRead(pinLineFrontL);
      lineFrontR = digitalRead(pinLineFrontR);
      lineBackL = digitalRead(pinLineBackL);
      lineBackR = digitalRead(pinLineBackR);
      if(lineFrontR == LOW || lineBackL  == LOW || lineBackR  == LOW)
      {
        break;
      }
      delaynow2 = millis() - delayant2;
      if(delaynow2 >= 0 && delaynow2 < TimeMov)
      {
        AvanzarGirar(2);
      }
      else if(delaynow2 >= TimeMov && delaynow2 < TimeMov2 )
      {
        AvanzarGirar(3);
      }
      else if(delaynow2 > TimeMov2)
      {
        break;
      }
      if((EstadoAtacar == true) and (lineFrontL == LOW || lineFrontR == LOW))
      {
        EstadoAtacar = false;
        EstadoBuscar = true;
      }
      startButtom = digitalRead(pinStartButtom);
    }
  }
  else if(lineFrontR == LOW)
  {
    delayant2 = millis();
    while(startButtom)
    {
      readSensorUS();
      lineFrontL = digitalRead(pinLineFrontL);
      lineFrontR = digitalRead(pinLineFrontR);
      lineBackL = digitalRead(pinLineBackL);
      lineBackR = digitalRead(pinLineBackR);
      if(lineFrontL == LOW || lineBackL  == LOW || lineBackR  == LOW)
      {
        break;
      }
      delaynow2 = millis() - delayant2;
      if(delaynow2 >= 0 && delaynow2 < TimeMov)
      {
        AvanzarGirar(2);
      }
      else if(delaynow2 >= TimeMov && delaynow2 < TimeMov2 )
      {
        AvanzarGirar(4);
      }
      else if(delaynow2 > TimeMov2)
      {
        break;
      }
      if((EstadoAtacar == true) and (lineFrontL == LOW || lineFrontR == LOW))
      {
        EstadoAtacar = false;
        EstadoBuscar = true;
      }
      startButtom = digitalRead(pinStartButtom);
    }
  }

  else if(lineBackL  == LOW)
  {
    delayant2 = millis();
    while(startButtom)
    {
      readSensorUS();
      lineFrontL = digitalRead(pinLineFrontL);
      lineFrontR = digitalRead(pinLineFrontR);
      lineBackL = digitalRead(pinLineBackL);
      lineBackR = digitalRead(pinLineBackR);
      if(lineFrontR == LOW || lineFrontL  == LOW || lineBackR  == LOW)
      {
        break;
      }
      delaynow2 = millis() - delayant2;
      if(delaynow2 >= 0 && delaynow2 < TimeMov)
      {
        AvanzarGirar(1);
      }
      else if(delaynow2 >= TimeMov && delaynow2 < TimeMov2 )
      {
        AvanzarGirar(4);
      }
      else if(delaynow2 > TimeMov2)
      {
        break;
      }
      if((EstadoAtacar == true) and (lineBackL == LOW || lineBackR == LOW))
      {
        EstadoAtacar = false;
        EstadoBuscar = true;
      }
      startButtom = digitalRead(pinStartButtom);
    }
  }
  else if(lineBackR  == LOW)
  {
    delayant2 = millis();
    while(startButtom)
    {
      readSensorUS();
      lineFrontL = digitalRead(pinLineFrontL);
      lineFrontR = digitalRead(pinLineFrontR);
      lineBackL = digitalRead(pinLineBackL);
      lineBackR = digitalRead(pinLineBackR);
      if(lineFrontR == LOW || lineFrontL  == LOW || lineBackL  == LOW)
      {
        break;
      }
      delaynow2 = millis() - delayant2;
      if(delaynow2 >= 0 && delaynow2 < TimeMov)
      {
        AvanzarGirar(1);
      }
      else if(delaynow2 >= TimeMov && delaynow2 < TimeMov2 )
      {
        AvanzarGirar(3);
      }
      else if(delaynow2 > TimeMov2)
      {
        break;
      }
      if((EstadoAtacar == true) and (lineBackL == LOW || lineBackR == LOW))
      {
        EstadoAtacar = false;
        EstadoBuscar = true;
      }
      startButtom = digitalRead(pinStartButtom);
    }
  }

  else if(lineFrontL == LOW && lineBackL  == LOW)
  {
    delayant2 = millis();
    while(startButtom)
    {
      readSensorUS();
      lineFrontL = digitalRead(pinLineFrontL);
      lineFrontR = digitalRead(pinLineFrontR);
      lineBackL = digitalRead(pinLineBackL);
      lineBackR = digitalRead(pinLineBackR);
      if(lineFrontR == LOW || lineBackR  == LOW)
      {
        break;
      }
      delaynow2 = millis() - delayant2;
      if(delaynow2 >= 0 && delaynow2 < TimeMov)
      {
        AvanzarGirar(4);
      }
      else if(delaynow2 >= TimeMov && delaynow2 < TimeMov2 )
      {
        AvanzarGirar(1);
      }
      else if(delaynow2 > TimeMov2)
      {
        break;
      }
      
      startButtom = digitalRead(pinStartButtom);
    }
  }
  else if(lineFrontR == LOW && lineBackR  == LOW)
  {
    delayant2 = millis();
    while(startButtom)
    {
      readSensorUS();
      lineFrontL = digitalRead(pinLineFrontL);
      lineFrontR = digitalRead(pinLineFrontR);
      lineBackL = digitalRead(pinLineBackL);
      lineBackR = digitalRead(pinLineBackR);
      if(lineFrontL == LOW || lineBackL  == LOW)
      {
        break;
      }
      delaynow2 = millis() - delayant2;
      if(delaynow2 >= 0 && delaynow2 < TimeMov)
      {
        AvanzarGirar(3);
      }
      else if(delaynow2 >= TimeMov && delaynow2 < TimeMov2 )
      {
        AvanzarGirar(1);
      }
      else if(delaynow2 > TimeMov2)
      {
        break;
      }
      
      startButtom = digitalRead(pinStartButtom);
    }
  } 
  digitalWrite(pinLEDDetecLine, LOW);
}


int AvanzarGirar(int x)
{
  switch(x)
  {
    case 0://quieto
      digitalWrite (M1PH1_IN1, LOW);
      digitalWrite (M1PH1_IN2, LOW);
      
      digitalWrite (M2PH2_IN3, LOW);
      digitalWrite (M2PH2_IN4, LOW);
      
      digitalWrite (M3PH2_IN1, LOW);
      digitalWrite (M3PH2_IN2, LOW);
      
      digitalWrite (M4PH3_IN1, LOW);
      digitalWrite (M4PH3_IN2, LOW);
      
      analogWrite(M1PH1_ENA,0);
      analogWrite(M2PH2_ENB,0);
      analogWrite(M3PH2_ENA,0);
      analogWrite(M4PH3_ENA,0);
      break;
      
    case 1://adelante
      digitalWrite (M1PH1_IN1, HIGH);
      digitalWrite (M1PH1_IN2, LOW);
      
      digitalWrite (M2PH2_IN3, HIGH);
      digitalWrite (M2PH2_IN4, LOW);
      
      digitalWrite (M3PH2_IN1, HIGH);
      digitalWrite (M3PH2_IN2, LOW);
      
      digitalWrite (M4PH3_IN1, HIGH);
      digitalWrite (M4PH3_IN2, LOW);
      
      analogWrite(M1PH1_ENA,velMaxM1);
      analogWrite(M2PH2_ENB,velMaxM2);
      analogWrite(M3PH2_ENA,velMaxM3);
      analogWrite(M4PH3_ENA,velMaxM4);
      break;
      
    case 2://atras
      digitalWrite (M1PH1_IN1, LOW);
      digitalWrite (M1PH1_IN2, HIGH);
      
      digitalWrite (M2PH2_IN3, LOW);
      digitalWrite (M2PH2_IN4, HIGH);
      
      digitalWrite (M3PH2_IN1, LOW);
      digitalWrite (M3PH2_IN2, HIGH);
      
      digitalWrite (M4PH3_IN1, LOW);
      digitalWrite (M4PH3_IN2, HIGH);
      
      analogWrite(M1PH1_ENA,velMaxM1);
      analogWrite(M2PH2_ENB,velMaxM2);
      analogWrite(M3PH2_ENA,velMaxM3);
      analogWrite(M4PH3_ENA,velMaxM4);
      break;
      
    case 3://izquierda
      digitalWrite (M1PH1_IN1, LOW);
      digitalWrite (M1PH1_IN2, HIGH);
      
      digitalWrite (M2PH2_IN3, HIGH);
      digitalWrite (M2PH2_IN4, LOW);
      
      digitalWrite (M3PH2_IN1, LOW);
      digitalWrite (M3PH2_IN2, HIGH);
      
      digitalWrite (M4PH3_IN1, HIGH);
      digitalWrite (M4PH3_IN2, LOW);
      
      analogWrite(M1PH1_ENA,velMaxM1);
      analogWrite(M2PH2_ENB,velMaxM2);
      analogWrite(M3PH2_ENA,velMaxM3);
      analogWrite(M4PH3_ENA,velMaxM4);
      break;
    
    case 4://derecha
      digitalWrite (M1PH1_IN1, HIGH);
      digitalWrite (M1PH1_IN2, LOW);
      
      digitalWrite (M2PH2_IN3, LOW);
      digitalWrite (M2PH2_IN4, HIGH);
      
      digitalWrite (M3PH2_IN1, HIGH);
      digitalWrite (M3PH2_IN2, LOW);
      
      digitalWrite (M4PH3_IN1, LOW);
      digitalWrite (M4PH3_IN2, HIGH);
      
      analogWrite(M1PH1_ENA,velMaxM1);
      analogWrite(M2PH2_ENB,velMaxM2);
      analogWrite(M3PH2_ENA,velMaxM3);
      analogWrite(M4PH3_ENA,velMaxM4);
      break;
  }
}



int mpuAcelX()
{
  //Leer los valores del Acelerometro de la IMU
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true); //A partir del 0x3B, se piden 6 registros
  AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  return AcX;
}


double readPower()
{
  promI+=(((5.00*analogRead(pinCorriente))/1023.00)-2.49022483)/0.066;
  contI++;
  Serial.println(contI);
  if(contI >= 60)
  {
    Corriente=abs(promI/60);
    ConsumoWatt = Corriente*CapVoltaje;
    ConsumoJoule = ConsumoWatt*60;//consumo por minuto
    totalConsumJoule+=ConsumoJoule;
    if(totalConsumJoule > EnergiaBateria*0.60 && totalConsumJoule < EnergiaBateria*0.70 && LowBattery1)//EnergiaBateria = 237600 Joules aprox
    {
      velMaxM1 = velMaxM1-40;
      velMaxM2 = velMaxM2-40;
      velMaxM3 = velMaxM3-40;
      velMaxM4 = velMaxM4-40;
      LowBattery1 = false;
    }
    else if(totalConsumJoule > EnergiaBateria*0.70 && totalConsumJoule < EnergiaBateria*0.79 && LowBattery2)
    {
      velMaxM1 = velMaxM1-40;
      velMaxM2 = velMaxM2-40;
      velMaxM3 = velMaxM3-40;
      velMaxM4 = velMaxM4-40;
      LowBattery2 = false;
    }
    else if(totalConsumJoule > EnergiaBateria*0.80 && LowBattery3)
    {
      velMaxM1 = 0;
      velMaxM2 = 0;
      velMaxM3 = 0;
      velMaxM4 = 0;
      LowBattery3 = false;
    }
    promI = 0;
    contI = 0;
  }
}


void readSensorUS()
{
  for (uint8_t i = 0; i < SONAR_NUM; i++) 
  { 
    if (millis() >= pingTimer[i]) 
    {        
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  
      if (i == 0 && currentSensor == SONAR_NUM - 1)
      {
        oneSensorCycle(); 
      }
      sonar[currentSensor].timer_stop();          
      currentSensor = i;                          
      cm[currentSensor] = 0;                      
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }
}


void echoCheck() 
{ 
  if (sonar[currentSensor].check_timer())
  {
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
  }
}

void oneSensorCycle() 
{
  for (uint8_t i = 0; i < SONAR_NUM; i++) 
  {
      sensorUS[i] = cm[i];   
  }
}
//------------------------------------------FIN----------------------------------------------------
