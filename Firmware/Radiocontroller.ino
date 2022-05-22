/*  
  SW1 & SW2 NORMALMENTE ALTI
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>



#define CE 9
#define CSN 10

#define Acceleratore A0
#define Val_Analogico_Acceleratore_Min 575
#define Val_Analogico_Acceleratore_Max 680
#define Tolleranza_Acceleratore 15
int accelerazione = 0;

#define Timone A1
#define Val_Analogico_Timone_Min 390
#define Val_Analogico_Timone_Max 770
#define MID 573
#define Tolleranza_Timone 20
int sterzata = 0;

#define Pulsante_1 8
#define Pulsante_2 4

#define led_stato_acceleratore 3
#define led_stato_timone 6
#define led_stato_pulsante_1 5
#define led_stato_pulsante_2 7

RF24 radio(CE, CSN); 
const byte address[6] = "00001"; 

bool debug = 0;

struct Data_Package {
  byte Speed;
  byte Direction;
  byte Button;
  byte Button2;
};

Data_Package data;



void setup() {
  Serial.begin(115200);
  delay(250);

  Serial.println("Transmitter Started");
  
  pinMode(Acceleratore, INPUT);
  pinMode(Timone, INPUT);
  pinMode(Pulsante_1, INPUT_PULLUP);
  pinMode(Pulsante_2, INPUT_PULLUP);

  pinMode(led_stato_acceleratore, OUTPUT);
  pinMode(led_stato_timone, OUTPUT);
  pinMode(led_stato_pulsante_1, OUTPUT);
  pinMode(led_stato_pulsante_2, OUTPUT);
  digitalWrite(led_stato_acceleratore, LOW);
  digitalWrite(led_stato_timone, LOW);
  digitalWrite(led_stato_pulsante_1, LOW);
  digitalWrite(led_stato_pulsante_2, LOW);
  
  radio.begin();
  radio.openWritingPipe(address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  
  data.Speed = 0; 
  data.Direction = 127;
  data.Button = 0;
  data.Button2 = 0;
/*
  while(1){
    print_debug();
  }
*/
  if(!digitalRead(Pulsante_1) && !digitalRead(Pulsante_2)){
    Serial.println("Tieni premuto per entrare in modalità debug");
    debug = 1;
    while(debug){
      if(millis() < 5000){
        if(digitalRead(Pulsante_1) || digitalRead(Pulsante_2)) debug = 0;
      }
      else{
        print_debug();
        if(!digitalRead(Pulsante_1) && !digitalRead(Pulsante_2) && millis() > 20000) debug = 0;
      }
    }
  }
  
}

void loop() {
  sterzata = analogRead(Timone);
  
  //if(sterzata > (MID - Tolleranza_Timone) && sterzata < (MID + Tolleranza_Timone)) sterzata = (Val_Analogico_Timone_Max - ((Val_Analogico_Timone_Max - Val_Analogico_Timone_Min) / 2));
  if(sterzata < Val_Analogico_Timone_Min) sterzata = Val_Analogico_Timone_Min;
  else if(sterzata > Val_Analogico_Timone_Max) sterzata = Val_Analogico_Timone_Max;
  
  if(sterzata < (MID - Tolleranza_Timone)) data.Direction = map(sterzata, (MID - Tolleranza_Timone), Val_Analogico_Timone_Min , 126, 0);
  else if(sterzata > (MID + Tolleranza_Timone)) data.Direction = map(sterzata, (MID + Tolleranza_Timone), Val_Analogico_Timone_Max , 128, 255);
  else data.Direction = 127;


  accelerazione = analogRead(Acceleratore);
  
  if(accelerazione < (Val_Analogico_Acceleratore_Min + Tolleranza_Acceleratore)) accelerazione = Val_Analogico_Acceleratore_Min + Tolleranza_Acceleratore;
  else if(accelerazione > Val_Analogico_Acceleratore_Max) accelerazione = Val_Analogico_Acceleratore_Max;
  data.Speed = map(accelerazione,(Val_Analogico_Acceleratore_Min + Tolleranza_Acceleratore),(Val_Analogico_Acceleratore_Max) , 0, 255);

  //print_data_send();
 
  data.Button = !(digitalRead(Pulsante_1));
  digitalWrite(led_stato_pulsante_1, data.Button);
  
  data.Button2 = !(digitalRead(Pulsante_2));
  digitalWrite(led_stato_pulsante_2, data.Button2);

  radio.write(&data, sizeof(Data_Package));
}



void print_data_send(){
  Serial.print(data.Direction);
  Serial.print(" ");
  Serial.println(data.Speed);
}



void print_debug(){
  Serial.print("Acceleratore: ");
  Serial.print(analogRead(Acceleratore));
  Serial.print(" Sterzata: ");
  Serial.println(analogRead(Timone));

  Serial.print("Pulsante 1: ");
  Serial.print(!digitalRead(Pulsante_1));
  Serial.print(" Pulsante 2: ");
  Serial.println(!digitalRead(Pulsante_2));
  Serial.println("");
  delay(200);
}
