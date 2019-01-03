//#include<NewPing.h>
const int F_TrigPin=32;
const int F_EchoPin=33;
const int L_TrigPin=34;
const int L_EchoPin=35;
const int R_TrigPin=36;
const int R_EchoPin=37;
float dis_F;
float dis_L;
float dis_R;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(F_TrigPin,OUTPUT);
  pinMode(F_EchoPin,INPUT);
  pinMode(L_TrigPin,OUTPUT);
  pinMode(L_EchoPin,INPUT);
  pinMode(R_TrigPin,OUTPUT);
  pinMode(R_EchoPin,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(F_TrigPin,LOW);
  delayMicroseconds(2);
  digitalWrite(F_TrigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(F_TrigPin,LOW);
  dis_F=pulseIn(F_EchoPin,HIGH)/58.3;
  
  digitalWrite(L_TrigPin,LOW);
  delayMicroseconds(2);
  digitalWrite(L_TrigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(L_TrigPin,LOW);
  dis_L=pulseIn(L_EchoPin,HIGH)/58.3;
  
  digitalWrite(R_TrigPin,LOW);
  delayMicroseconds(2);
  digitalWrite(R_TrigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(R_TrigPin,LOW);
  dis_R=pulseIn(R_EchoPin,HIGH)/58.3;
  
  dis_F=(int(dis_F*100.0))/100.0;
  dis_L=(int(dis_L*100.0))/100.0;
  dis_R=(int(dis_R*100.0))/100.0;
  Serial.print(dis_F);
  Serial.print("cm");
  Serial.print(dis_L);
  Serial.print("cm");
  Serial.print(dis_R);
  Serial.print("cm\n");
  delay(100);
  
}
