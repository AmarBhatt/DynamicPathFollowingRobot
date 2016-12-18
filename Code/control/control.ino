
const int M1 = 9; //right
const int M2 = 10; //left
const int D2n = 4;
const int M1_dir = 7;
const int M2_dir = 8;
const int SPEED_DELAY = 50;
const int SPEED = 30;
const int TURN_SPEED = 30;
int currentSpeed = SPEED;

char inst;

void accelerare(int _speed);
void deccelerate(int _speed);
void driveTime(int time, int _speed);
void setDirection (int dir);



void setup() {
  // put your setup code here, to run once:
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(D2n, OUTPUT);
  pinMode(M1_dir, OUTPUT);
  pinMode(M2_dir, OUTPUT);
  
  //Enable
  digitalWrite(D2n, HIGH);
  
  // Go Forward
  digitalWrite(M1_dir, HIGH);
  digitalWrite(M2_dir, LOW);
  
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly: 
//  for (int i = 0; i<3;i++){
//    setDirection (0);
//    driveTime(2000,128);
//    setDirection (1);
//    driveTime(2000,128);
//  }
//  
//  delay(5000);

while (Serial.available()){
  inst = Serial.read();
  switch (inst) {
    case 'F':
       setDirection(0);
       go();
       break;
    case  'L':
        setDirection(2);
        go();
        break;
    case 'R':
        setDirection(3);
        go();
        break;
    case 'S':
        brake();
        break;
    default:
        break;
  }
}
  
}

void accelerate(int _speed) {
  
  for(int i=0;i<=_speed;i+=_speed/10){
    analogWrite(M1,i);
    analogWrite(M2,i);
    delay(SPEED_DELAY);
    Serial.println(i);
  }
  
  analogWrite(M1,_speed);
  analogWrite(M2,_speed);

}

void deccelerate(int _speed) {
  
  for(int i=_speed;i>=0;i-=_speed/10){
    analogWrite(M1,i);
    analogWrite(M2,i);
    delay(SPEED_DELAY);
  }
  
  analogWrite(M1,0);
  analogWrite(M2,0);

}

void driveTime(int time, int _speed){
  
  accelerate(_speed);
  delay(time);
  deccelerate(_speed);  
}

void go() {
  analogWrite(M1,currentSpeed);
  analogWrite(M2,currentSpeed);
  //delay(100);
  //brake();
}

void brake() {
  analogWrite(M1,0);
  analogWrite(M2,0);
}

void setDirection (int dir) {

  switch(dir){
    case 0: //Forward
      digitalWrite(M1_dir, HIGH);
      digitalWrite(M2_dir, LOW);
      currentSpeed = SPEED;
      break;
    case 1: //Reverse
      digitalWrite(M1_dir, LOW);
      digitalWrite(M2_dir, HIGH);
      break;
    case 2: //Turn left
      digitalWrite(M1_dir, HIGH);
      digitalWrite(M2_dir, HIGH);
      currentSpeed = TURN_SPEED;
      break;
    default: //Turn right
      digitalWrite(M1_dir, LOW);
      digitalWrite(M2_dir, LOW);
      currentSpeed = TURN_SPEED;
      break;
  }       
    
}

