#include <PID_v1.h>
double Setpoint, Input, Output;
double Kp = 2, Ki = 5, Kd = 1;
bool dir=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

#define motor_enable 3
#define motor_p1 6
#define motor_p2 5


#define A 8
#define B 9
int newA;
//int newB;
int oldA;
//int oldB;

double counter = 0;
double angle;


void encoder(){
  newA=digitalRead(A);
  if ((oldA == 0) && (newA == 1)) {
      oldA = newA;
      if (digitalRead(B) != newA) {
        Serial.println("C.W");
        counter++;
               }
      else {
        Serial.println("C.C.W");
        counter++;
      }
      
          
  }
  oldA = newA;
      angle = 18 * counter;
    
}
void setup() {
  
  angle = 18 * counter;
    Input = angle;
  Setpoint = 90;
 dir=1;
  myPID.SetMode(AUTOMATIC);

  pinMode(motor_enable, OUTPUT);
  pinMode(motor_p1, OUTPUT);
  pinMode(motor_p2, OUTPUT);
  pinMode(A, INPUT);
  pinMode(B, INPUT);
  Serial.begin (9600);
  oldA = digitalRead(A);
  
}

void loop() {
      angle = 18 * counter;
    
  Input = angle;
  myPID.Compute();
   
  

  if ( dir==0 && Setpoint != angle  ) {
     analogWrite(motor_enable, Output);
     digitalWrite(motor_p1, HIGH);
    digitalWrite(motor_p2, LOW);
    Serial.println("C.W");
    encoder();
    
  }
 else if ( dir==1 && Setpoint != angle  ){
    analogWrite(motor_enable, Output);
    digitalWrite(motor_p1, LOW);
    digitalWrite(motor_p2, HIGH);
    Serial.println("C.C.W");
    encoder();
    
  }

 else if ( Setpoint == angle  ){
  analogWrite(motor_enable, 0);
   Serial.println(angle);
   Serial.println(dir);
 }
     
}
