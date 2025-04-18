#include <math.h>
#include <string.h>

#define IN1_LEFT 6
#define IN2_LEFT 5
#define PWM_LEFT 7

int saturate(int value,int min,int max){
    if(value>max)value=max;
    else if(value<min)value=min;
    return value;
}

int serial_data = 0;
int control_action = 0;

void setup() {
  Serial.begin(115200);
  pinMode(IN1_LEFT, OUTPUT); 
  pinMode(IN2_LEFT, OUTPUT); 
  pinMode(PWM_LEFT, OUTPUT);

}

void loop() {

  if(Serial.available()){
    serial_data = Serial.readString().toInt();
  } else {
  }

  if(serial_data==1)control_action=100;
  else if(serial_data==2)control_action=170;
  else if(serial_data==3)control_action=255;
  else control_action=0;

  digitalWrite(IN1_LEFT,1);
  digitalWrite(IN2_LEFT,0);
  
  analogWrite(PWM_LEFT,control_action);
  
  Serial.print(serial_data);Serial.print("  ");Serial.println(control_action);
}
