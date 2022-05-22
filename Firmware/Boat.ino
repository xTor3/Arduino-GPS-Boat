#include <TinyGPS++.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <EEPROM.h>


//Variabili GPS:
TinyGPSPlus gps;
#define GPSBaud 9600
#define LatTo             //Destination LAT
#define LonTo             //Destination LON

double ActualCourse;
double course_a, course_b, course_c, base_course_mirrored, actual_course_mirrored;
double heading_error;
#define satsnecessary 4


//PID Variabili:
#define Kp 1
#define Ki 0.00
#define Kd 0.00
float Pid_p, Pid_i, Pid_d = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int Pid_Value = 0;


//Variabili Radio:
RF24 radio(7,8);
const byte address[6] = "00001";
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

struct Data_Package{
  byte Speed;
  byte Direction;
  byte Button;
  byte Button2;
};
Data_Package data; 
bool LostConnection = 0;


//Brushless Motor:
#define motore 6
Servo Brushless_Motor;

#define SpeedMotorMin 1500
#define SpeedMotorMax 1850
#define starting_speed 1750
#define auto_pilot_min_speed  1750

//Servo:
#define servo1 5
Servo Timone;

#define MID 90 
#define MaxAngleDx 0
#define Tolerance_Dx 45
#define MaxAngleSx 180
#define Tolerance_Sx 45
int sterzata = 0;

//Led debug:
#define ledsegnalazione 2
unsigned long timerled = 0;
bool statoled = 0;


//Variabili Per Algoritmi:
#define DEG_TO_RAD 0.017453292519943295769236907684886
unsigned long unlock_delay = 0;
int Auto_Pilot_Mode = 0;
bool partito = 0;
bool AvvioAutoPilota = 0;
unsigned long Avvio_Debounce = 0;

//Altro:
#define pompa 4
//unsigned long delay_pompa = 0;
//bool stato_pompa = 0;



//Setup:
void setup(){
  //Setup Servo:
  Timone.attach(3);  
  pinMode(servo1, OUTPUT);    //Led stato Servo.

  //Setup Motore Brushless:
  Brushless_Motor.attach(9);
  pinMode(motore, OUTPUT);    //Led stato Motore.

  //Setup GPS:  
  Serial.begin(GPSBaud);

  //Setup Radio:
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening(); 

  //Setup Led debug: 
  pinMode(ledsegnalazione, OUTPUT);

  //Setup Other:
  pinMode(pompa, OUTPUT);
  digitalWrite(pompa, LOW);

  //Ritorno allo stato precedente allo spegnimento:
  switch(EEPROM.read(1024)){
    case 2:                     //Attesa Satelliti
      Auto_Pilot_Mode = 1;
      break;
    case 3:                     //Autopilota Avviato
      Auto_Pilot_Mode = 1;
      AvvioAutoPilota = 1;
      break;
    default:                    //Modalità Radio
      Auto_Pilot_Mode = 0;
      AvvioAutoPilota = 0;
      break;
  }

  resetData();    //Reset Variabili Radiocomando
    
  timerled = millis();
  Time = millis();
}


//Loop:
void loop(){
  radio_read();
  
  while(radio_available() && !Auto_Pilot_Mode){
    radio_read();
    digitalWrite(pompa, LOW);
    lampeggio(100);
  }

  if(unlock_function() && Auto_Pilot_Mode == 0){
    digitalWrite(pompa, LOW);
    Auto_Pilot_Mode = 1;  
    resetData();
  }

  if(!Auto_Pilot_Mode){
    Radio_Pilot();
    if(data.Speed > 10) digitalWrite(pompa, HIGH);   
    else digitalWrite(pompa, LOW);
    lampeggio(300);
  }

  while(Auto_Pilot_Mode){
  
    if(EEPROM.read(1024) != 3)  EEPROM.update(1024,2); 
    
    smartDelay(1000);
    
    radio_read();

    if(unlock_function()){
      EEPROM.write(1024,0);
      Auto_Pilot_Mode = 0;  
      AvvioAutoPilota = 0;
    }

    if(data.Button2 && (millis() - Avvio_Debounce) >= 2000){                                             //Avvio partenza con autopilota
      AvvioAutoPilota = !AvvioAutoPilota;
      digitalWrite(pompa, LOW); 
      if(AvvioAutoPilota) EEPROM.update(1024,3); 
      else{
        EEPROM.update(1024,2); 
        Brushless_Motor.writeMicroseconds(SpeedMotorMin);
        partito = 0;
      }
      Avvio_Debounce = millis();
    }

    if(gps.satellites.isValid() && (gps.satellites.value() >= satsnecessary)){
      if(AvvioAutoPilota){
        digitalWrite(pompa, HIGH); 
        digitalWrite(ledsegnalazione, LOW);
        lampeggio(0);

        if(!partito) Partenza();
        else  AutoPilota();
      }
      else{
        digitalWrite(ledsegnalazione, HIGH);
        lampeggio(0);
        Brushless_Motor.writeMicroseconds(SpeedMotorMin);
        analogWrite(motore, 0);                 
      } 
    }
    else{
      if(!AvvioAutoPilota && !(millis() > 5000 && gps.charsProcessed() < 10)) lampeggio(200);
      else if(!(millis() > 5000 && gps.charsProcessed() < 10)) lampeggio(3000);
      partito = 0;

      Brushless_Motor.writeMicroseconds(SpeedMotorMin);
      analogWrite(motore, 0);                   

      Timone.write(MID);
      analogWrite(servo1, MID);

      if(millis() > 5000 && gps.charsProcessed() < 10){
        digitalWrite(ledsegnalazione, LOW);
        lampeggio(0);
      }
    }    
  }
  
}


//Radio Function:
void radio_read(){
  if(radio.available()){
    radio.read(&data, sizeof(Data_Package));
    //if(data.Speed < 4) data.Speed = 0;
    //if(data.Direction < 4) data.Direction = 0;
    lastReceiveTime = millis(); // At this moment we have received the data
  }
}

bool radio_available(){
  currentTime = millis();
  if(currentTime - lastReceiveTime > 2000){
    resetData();
    return 1;
  }
  else  return 0;
}

bool unlock_function(){
  if(!data.Button) unlock_delay = millis();
  
  if(((millis() - unlock_delay) > 2000) && Auto_Pilot_Mode == 1){
    unlock_delay = millis();
    return 1;
  }
  else if(((millis() - unlock_delay) > 3000) && Auto_Pilot_Mode == 0){
    unlock_delay = millis();
    return 1;
  }
  else{
    return 0;
  }
}

void Radio_Pilot(){
  if(data.Direction == 127) Timone.write(MID);
  else if(data.Direction < 127) Timone.write(map(data.Direction, 0, 126, MaxAngleSx, (MID+1)));
  else Timone.write(map(data.Direction, 128, 255, (MID-1), MaxAngleDx));
  
  Brushless_Motor.writeMicroseconds(map(data.Speed, 0, 255, SpeedMotorMin, SpeedMotorMax));
  
  analogWrite(motore, data.Speed);                          
  analogWrite(servo1, data.Direction);                         
}

void resetData() {
  data.Speed = 0;
  data.Direction = 127;
  data.Button = 0;
  data.Button2 = 0;

  analogWrite(motore, data.Speed);                             
  analogWrite(servo1, data.Direction);                        

  Brushless_Motor.writeMicroseconds(SpeedMotorMin);
  Timone.write(MID);
}


//GPS Function:
void Partenza(){
  if(gps.location.isValid() && gps.course.isValid()){
    Timone.write(MID);
    delay(500);
    Brushless_Motor.writeMicroseconds(starting_speed);
    
    analogWrite(motore, 255);                      
    analogWrite(servo1, 127);
    
    delay(3500);
    partito = 1;
  }
  else{
    Brushless_Motor.writeMicroseconds(SpeedMotorMin);
    Timone.write(MID);
  }
}

void AutoPilota(){
  if((TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), LatTo, LonTo)) <= 6){
    Brushless_Motor.writeMicroseconds(SpeedMotorMin);
    Timone.write(MID);
    Auto_Pilot_Mode = 0;
    AvvioAutoPilota = 0;
    resetPid();
    
    EEPROM.write(1024,0); 
  }

  else if(gps.course.isValid()){
    ActualCourse = gps.course.deg();
    
    heading_error = course_deviation(ActualCourse,(TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), LatTo, LonTo)));
    
    sterzata = (MID + CalcoloPid());
 
    Timone.write(sterzata);
    analogWrite(servo1, map(sterzata, MaxAngleDx, MaxAngleSx, 0, 255));                                         

    Brushless_Motor.writeMicroseconds(auto_pilot_min_speed);
    analogWrite(motore, map(auto_pilot_min_speed, SpeedMotorMin, SpeedMotorMax, 0, 255));    
  
  /* 
    if(heading_error < 90 && heading_error > -90){
      float error_in_radianti = heading_error*DEG_TO_RAD;
      Brushless_Motor.writeMicroseconds(map((20+(int)(25 * cos(error_in_radianti))), 0, 255, SpeedMotorMin, SpeedMotorMax));

      analogWrite(motore, ((int)(255 * cos(error_in_radianti))));           
    }
    else{
      Brushless_Motor.writeMicroseconds(auto_pilot_min_speed);
      analogWrite(motore, 20);                                              
    }
  */
  
  }

}

float course_deviation(float course_b, float course_c) {
  course_a = course_b - course_c;
  if (course_a < -180 || course_a > 180) {
    if (course_c > 180)base_course_mirrored = course_c - 180;
    else base_course_mirrored = course_c + 180;
    if (course_b > 180)actual_course_mirrored = course_b - 180;
    else actual_course_mirrored = course_b + 180;
    course_a = actual_course_mirrored - base_course_mirrored;
  }
  return course_a;
}

static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do 
  {
    while (Serial.available()){
      gps.encode(Serial.read());
      radio_read();
    }
  } while (millis() - start < ms);
}



//PID Function:
int CalcoloPid(){
  Pid_p = Kp * heading_error;

  if(heading_error > -5 && heading_error < 5) Pid_i = Pid_i + (Ki * heading_error);
  if(Pid_i > 10) Pid_i = 10;
  else if(Pid_i < -10) Pid_i = -10;

  timePrev = Time;                           
  Time = millis();                           

  if(timePrev != 0){
    elapsedTime = (Time - timePrev) / 1000; 
    Pid_d = Kd*((heading_error - previous_error)/elapsedTime);
  }
  previous_error = heading_error;    

  Pid_Value = (int)(Pid_p + Pid_i + Pid_d);

  if(heading_error < 0){                                                //La Barca deve andare a destra.
    if(Pid_Value > 0) Pid_Value = 0; 
    if(Pid_Value < -(MID - (MaxAngleDx + Tolerance_Dx)))  Pid_Value = -(MID - (MaxAngleDx + Tolerance_Dx));    
  }
  else{                                                                 //La Barca deve andare a sinistra.
    if(Pid_Value < 0) Pid_Value = 0; 
    if(Pid_Value > ((MaxAngleSx - Tolerance_Sx) - MID))  Pid_Value = ((MaxAngleSx - Tolerance_Sx) - MID); 
  }
  return Pid_Value;
}

void resetPid(){
  Pid_p = 0;
  Pid_i = 0;
  Pid_d = 0;
  Pid_Value = 0;
}



//Other:
void lampeggio(long frequenzalampeggio){
  if((millis() - timerled) > frequenzalampeggio && frequenzalampeggio > 0){
    statoled = !statoled;
    digitalWrite(ledsegnalazione, statoled);
    timerled = millis();
  }
}
