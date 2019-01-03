#define PinA 2 //外部中断0
#define PinZ 3 //外部中断1
#define PinB 8 //编码器的OUT_B信号连接到数字端口8
 
//变量初始化
unsigned long time1 = 0; // 时间标记
float time_cw;
float time_ccw;
long count = 0;
const float d = 10.0; //轮子的直径
const float pi = 3.141592654;//圆周率
int num = 0;//圈数
double distance=0.0;

double t;//一圈的运动时间
float velocity;
double time3;//外部中断1产生时的时间，即捕捉到Z相的置零信号时，用于在loop循环内进行一圈时间长短的计算
 
void setup()
{
    pinMode(PinA, INPUT_PULLUP);//因为编码器信号为欧姆龙E6B2-CWZ6C，为开漏输出，因此需要上拉电阻，此处采用arduino的内部上拉输入模式，置高
    pinMode(PinB, INPUT_PULLUP);//同上
    pinMode(PinZ, INPUT_PULLUP);//同上
    attachInterrupt(0, Encode, FALLING);//脉冲中断函数：捕捉A相信号，并判断A、B相先后顺序
    attachInterrupt(1, Set_state , FALLING);//用于在捕捉到Z的零信号时，进行状态置零
 
 
    Serial.begin (9600);
 
}
 
void loop()
 
{
  distance+=count*d*pi/1000;
  count=0;
  Serial.print(distance); Serial.println("cm.");
  t = time_cw - time3;
  t = t / 1000;
  Serial.print("The time is ");Serial.print(t); Serial.println("s.");
  Serial.print("The NUM is ");Serial.print(num); Serial.println("r");
  delay(1000);
   /* 
    //正转
    if (count == 1000)
    {
      Serial.println("zzzok");//调试用
      num = num + 1;
      time_cw = millis();
      t = time_cw - time3;
      t = t / 1000;
      distance = num * d * pi; 
      velocity = d * pi / t;
      Serial.print("The wheel has run ");Serial.print(distance); Serial.println("cm.");
      Serial.print("The cw_speed is ");Serial.print(velocity); Serial.println("cm/s.");
      Serial.print("The time is ");Serial.print(t); Serial.println("s.");
    }
    //反转
    if (count == -1000)
    {
      Serial.println("ffffok");//调试用
      num = num + 1;
      time_ccw = millis();
      t = time_ccw - time3;
      t = t / 1000;
      distance = num * d * pi; 
      velocity = d * pi / t;
      Serial.print("The wheel has run ");Serial.print(distance); Serial.println("cm.");
      Serial.print("The ccw_speed is ");Serial.print(velocity); Serial.println("cm/s.");
      Serial.print("The time is ");Serial.print(t); Serial.println("s.");
    }
   // delay(2000);
    */
}
 
// 编码器计数中断子程序
 
void Encode()
 
{
 
    //为了不计入噪音干扰脉冲，
    
    //当2次中断之间的时间大于5ms时，计一次有效计数
    
    if ((millis() - time1) >8)
 
    {
        //当编码器码盘的OUTA脉冲信号下跳沿每中断一次，
        /*
        if ((digitalRead(PinA) == LOW) && (digitalRead(PinB) == HIGH))
        
        {
       // Serial.println("xxxxxx1");//调试用
            //count--;
        
        }
        
             else
        
        {
      //   Serial.println("uuuuxxxx1");//调试用
              count++;
         Serial.println(count);//调试用
        }
    */
      count++;
    }
    time1 == millis();
    
}
void Set_state(){
    Serial.println("zdzdz2222");//调试用
    num++;
    //count = 0;
    time3 = millis();//发生中断时的时间
}
