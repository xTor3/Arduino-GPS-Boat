#include <Servo.h>


Servo Timone;
#define pin_servo 3

Servo Brushless_Motor;
#define pin_motore 9

#define button 2
bool debug = 0;
unsigned long debounce_pulsante = 0;

#define pot A7
int val_pot = 0;

unsigned long delay_letture = 0;
#define periodo_letture 500

//Val Min/Max Brushless Motor: 1590
//Val Min/Max/Mid Servo:   60 dx, 90 mid, 100 sx


void setup() {
  Serial.begin(115200);
  delay(250);

  pinMode(pot, INPUT);
  
  Brushless_Motor.attach(9);
    
  pinMode(button, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  delay_letture = millis();
  debounce_pulsante = millis();

  Serial.println("Sketch Calibrazione Brushless/Servo Pronto");
  delay(2000);
}

void loop() {
  val_pot = analogRead(pot);
  if(debug) Timone.write(map(val_pot, 0, 1023, 0, 180));
  else  Brushless_Motor.writeMicroseconds(map(val_pot, 0, 1023, 1500, 2000));
  
  if(!digitalRead(button) && ((millis() - debounce_pulsante) > 300)){
    debug = !debug;
    if(debug){
      Brushless_Motor.detach();
      Timone.attach(3);
    }
    else{
      Brushless_Motor.attach(9);
      Timone.detach();
    }
    debounce_pulsante = millis();
  }

  if((millis() - delay_letture) > periodo_letture){
    if(debug){
      digitalWrite(LED_BUILTIN, LOW);
      Serial.print("Output Servo: ");
      Serial.println(map(val_pot, 0, 1023, 0, 180));
    }
    else{
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.print("Output Motore: ");
      Serial.println(map(val_pot, 0, 1023, 1500, 2000));
    }
    delay_letture = millis();
  }
}
