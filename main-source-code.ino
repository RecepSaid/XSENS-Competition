/* This code was written by Recep Said Dülger
	for Xsens competition - 2020  
	ElectroGtu Team							*/

/* Fuctions and Class Declarations */
class dcMotor{
  private:
    int in1,in2;
    int enable;
  public:
    void dcMotorData(int in1,int in2,int enable);
    void motorStop();
    void motorHalfSpeed(int pwm);
    void motorFullSpeed(int pwm);
};

void dcMotor::dcMotorData(int in1,int in2,int enable){
  this->in1=in1;
  this->in2=in2;
  this->enable=enable;
}

void dcMotor::motorStop(){
  pinMode(dcMotor::in1,OUTPUT);
  pinMode(dcMotor::in2,OUTPUT);
  digitalWrite(dcMotor::in1,HIGH);
  digitalWrite(dcMotor::in2,LOW);
  analogWrite(enable,0);
}

void dcMotor::motorHalfSpeed(int pwm){
  pinMode(dcMotor::in1,OUTPUT);
  pinMode(dcMotor::in2,OUTPUT);
  digitalWrite(dcMotor::in1,HIGH);
  digitalWrite(dcMotor::in2,LOW);
  analogWrite(enable,pwm);
}

void dcMotor::motorFullSpeed(int pwm){
  pinMode(dcMotor::in1,OUTPUT);
  pinMode(dcMotor::in2,OUTPUT);
  digitalWrite(dcMotor::in1,HIGH);
  digitalWrite(dcMotor::in2,LOW);
  analogWrite(enable,pwm);
}

class buzzer{
  private:
    int pin;
  public:
    void buzzerData(int pin);
    void buzzerBegin();
    void buzzerSystemReady();
    void buzzerMineDetect();
};

void buzzer::buzzerData(int pin){
  this->pin=pin;
}

void buzzer::buzzerBegin(){
    pinMode(buzzer::pin,OUTPUT);
    digitalWrite(buzzer::pin,HIGH);
    delay(200);
    digitalWrite(buzzer::pin,LOW);
}
  
void buzzer::buzzerSystemReady(){
    pinMode(buzzer::pin,OUTPUT);
    digitalWrite(buzzer::pin,HIGH);
    delay(60);
    digitalWrite(buzzer::pin,LOW);
    delay(60);
    digitalWrite(buzzer::pin,HIGH);
    delay(60);
    digitalWrite(buzzer::pin,LOW);
    delay(60);
    digitalWrite(buzzer::pin,HIGH);
    delay(60);
    digitalWrite(buzzer::pin,LOW);
}

void buzzer::buzzerMineDetect(){
  pinMode(buzzer::pin,OUTPUT);
  digitalWrite(buzzer::pin,HIGH);
  delay(100);
  digitalWrite(buzzer::pin,LOW);
  delay(100);
  }

class distance{
  private:
    int trigPin,echoPin;
  public:
    void distanceData(int trigPin,int echoPin);
    float calculateDistance(); 
};

void distance::distanceData(int trigPin,int echoPin){
  this->trigPin=trigPin;
  this->echoPin=echoPin; 
}

float distance::calculateDistance(){
  long times;
  long distance;
  
  pinMode(distance::trigPin,OUTPUT);
  pinMode(distance::echoPin,INPUT);

  digitalWrite(distance::trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(distance::trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(distance::trigPin, LOW);
  
  times = pulseIn(distance::echoPin, HIGH);
  distance = times /29.1/2;
  
  if(distance > 200){
  distance = 200;
  return distance;
  }
  else
  return distance;
}

bool hallEffectSensor(int pin){
  pinMode(pin,INPUT);
  if(digitalRead(pin) == LOW)
  return 1;
  else
  return 0;
}

/* Interrupts Setup Function*/
void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

/* Definitions */
#define ldr1pin 13
#define ldr2pin 12
#define ldrMidpin A0 
#define ldr3pin 6
#define ldr4pin 7
#define hallEffectPin A2

dcMotor motor1;
dcMotor motor2;

distance sensor1;
distance sensor2;

buzzer buzzer1;

/* Setup Part */
void setup() {
  //Serial.begin(9600);
  motor1.dcMotorData(2,3,10);
  motor2.dcMotorData(5,4,11);
  sensor1.distanceData(9,8);
  buzzer1.buzzerData(A1);
  buzzer1.buzzerBegin();
  pinMode(ldr2pin,INPUT);
  pinMode(ldr3pin,INPUT);
  
  pciSetup(ldr1pin);
  pciSetup(ldrMidpin);
  pciSetup(ldr4pin);

  if(sensor1.calculateDistance() >= 0 && hallEffectSensor(hallEffectPin) == 0){
    delay(500);
    buzzer1.buzzerSystemReady();
  }
}

//---LDR1 and LDR2 Interrupts---//
  ISR (PCINT0_vect){ //pin13
  while (digitalRead(ldr1pin) == 1 || digitalRead(ldr2pin) == 1){
    motor1.motorHalfSpeed(175);
    motor2.motorHalfSpeed(125);
    if(sensor1.calculateDistance() <= 15){
    motor1.motorStop();
    motor2.motorStop();
  }
    delay(1000);
    }
  }
//---LDRmid Interrupt---//
  ISR (PCINT1_vect){ //pinA0
  while (digitalRead(ldrMidpin) == 1){
    motor1.motorHalfSpeed(150);
    motor2.motorHalfSpeed(150);
    if(sensor1.calculateDistance() <= 15){
    motor1.motorStop();
    motor2.motorStop();
  }
    delay(1000);
    }
  }
//---LDR3 and LDR4 Interrupts---//
  ISR (PCINT2_vect){ //pin7
  while (digitalRead(ldr3pin) == 1 || digitalRead(ldr4pin) == 1){
    motor1.motorFullSpeed(125);
    motor2.motorFullSpeed(175);
    if(sensor1.calculateDistance() <= 15){
    motor1.motorStop();
    motor2.motorStop();
  }
    delay(1000);
    }
  }

/* Loop Part*/
void loop() {
  //Serial.print("LDR1: ");
  //Serial.println(digitalRead(A1));
  //Serial.print("\tLDR2: ");
  //Serial.print(ldr2Value(ldr2pin));
  //Serial.print("\tLDR3: ");
  //Serial.println(ldrMidValue(ldrMidpin));
  //Serial.print("\tLDR4: ");
  //Serial.print(ldr4Value(ldr4pin));
  //Serial.print("\tLDR MİD: ");
  //Serial.println(ldrMidValue(ldrMidpin));
  //Serial.print("Sensor1: ");
  //Serial.print(sensor1.calculateDistance());
  //Serial.print("\tSensor2: ");
  //Serial.println(sensor2.calculateDistance());
  //Serial.print("HallEffectSensor: ");
  //Serial.println(hallEffectSensor(hallEffectPin));

  if(digitalRead(ldr1pin) != 1 && digitalRead(ldr2pin) != 1 && digitalRead(ldrMidpin) != 1 && digitalRead(ldr3pin) != 1 &&  digitalRead(ldr4pin) != 1){
    motor1.motorStop();
    motor2.motorStop();
}

  while(hallEffectSensor(hallEffectPin) == 1){
    motor1.motorStop();
    motor2.motorStop();
    buzzer1.buzzerMineDetect();
  }
  
  if(sensor1.calculateDistance() <= 15){
    motor1.motorStop();
    motor2.motorStop();
  }
  
  delay(10);
}
