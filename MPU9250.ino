/*
This code is used for connecting arduino to serial  module, and test in arduino uno R3 and 2560 board.
connect map:

arduino2560               mpu6050/mpu9250 module
VCC                           5v/3.3v
RX1  -----------------  Tx

GND    GND

note: arduino 例子
版本号：v1.0
带最后一位校验位版本

1、烧写代码入arduino时，请暂时断开Tx,Rx 连接线，否则程序可能烧写失败。
2、如串口无数据请断开 RX1和TX重新连接一次。
3、先把arduino串口监视器打开后再连接  RX--TX
 */

unsigned char ucRxBuffer[52],counter=0;
unsigned char ubuf[52];

unsigned char sign=0;
float a[16],w[3],angle[3],T;

void CopeSerialData(unsigned char ucData);






void setup() {
//   initialize serial1:
 
  Serial.begin(115200);  
  Serial1.begin(115200);
  
}

void loop() 
{
 
    //           if (((ubuf[19] * 256.0*256.0 + ubuf[20] * 256.0 + ubuf[21] - 1000000) * 0.001>=980) && ((ubuf[19] * 256.0*256.0 + ubuf[20] * 256.0 + ubuf[21] - 1000000) * 0.001<=981) &&((ubuf[25] * 256.0 * 256.0 + ubuf[26] * 256.0 + ubuf[27] - 1000000) * 0.001== -128))  
                if ((ubuf[49] * 256.0 * 256.0 + ubuf[50] * 256.0 + ubuf[51] - 1000000) * 0.001==128)  
    { 
                a[0] = (ubuf[1] * 256.0 * 256.0 + ubuf[2] * 256.0 + ubuf[3] - 1000000) * 0.001;  //gx
                a[1] = (ubuf[4] * 256.0 * 256.0 + ubuf[5] * 256.0 + ubuf[6] - 1000000) * 0.001;
                a[2] = (ubuf[7] * 256.0 * 256.0 + ubuf[8] * 256.0 + ubuf[9] - 1000000) * 0.001;

                 a[3] = (ubuf[10] * 256.0 * 256.0 + ubuf[11] * 256.0 + ubuf[12] - 1000000) * 0.001;  //ax
                 a[4] = (ubuf[13] * 256.0 * 256.0 + ubuf[14] * 256.0 + ubuf[15] - 1000000) * 0.001;
                 a[5] = (ubuf[16] * 256.0 * 256.0 + ubuf[17] * 256.0 + ubuf[18] - 1000000) * 0.001;

                 a[6] = (ubuf[19] * 256.0 * 256.0 + ubuf[20] * 256.0 + ubuf[21] - 1000000) * 0.001; //mx
                 a[7] = (ubuf[22] * 256.0 * 256.0 + ubuf[23] * 256.0 + ubuf[24] - 1000000) * 0.001;
                 a[8] = (ubuf[25] * 256.0 * 256.0 + ubuf[26] * 256.0 + ubuf[27] - 1000000) * 0.001;

                 a[9]  = (ubuf[28] * 256.0 * 256.0 + ubuf[29] * 256.0 + ubuf[30] - 1000000) * 0.001; //roll
                 a[10] = (ubuf[31] * 256.0 * 256.0 + ubuf[32] * 256.0 + ubuf[33] - 1000000) * 0.001; //pitch
                 a[11] = (ubuf[34] * 256.0 * 256.0 + ubuf[35] * 256.0 + ubuf[36] - 1000000) * 0.001;  //raw

                 a[12] = (ubuf[37] * 256.0 * 256.0 + ubuf[38] * 256.0 + ubuf[39] - 1000000) * 0.001;   //q0
                 a[13] = (ubuf[40] * 256.0 * 256.0 + ubuf[41] * 256.0 + ubuf[42] - 1000000) * 0.001;  //q1
                 a[14] = (ubuf[43] * 256.0 * 256.0 + ubuf[44] * 256.0 + ubuf[45] - 1000000) * 0.001;  //q2
                 a[15] = (ubuf[46] * 256.0 * 256.0 + ubuf[47] * 256.0 + ubuf[48] - 1000000) * 0.001;  //q3 
            
                 a[16] = (ubuf[49] * 256.0 * 256.0 + ubuf[50] * 256.0 + ubuf[51] - 1000000) * 0.001;  //校验位 数值固定为128
              //  Serial.print("g:");
              //  Serial.print(a[0]);Serial.print(" ");
              //  Serial.print(a[1]);Serial.print(" ");
              //  Serial.print(a[2]);Serial.print(" ");
                
              
                
             //   Serial.print("a:");
              //  Serial.print(a[3]);Serial.print(" ");
              //  Serial.print(a[4]);Serial.print(" ");
             //   Serial.print(a[5]);Serial.print(" ");
                
             //   Serial.print("m:");
             //   Serial.print(a[6]);Serial.print(" ");
             //   Serial.print(a[7]);Serial.print(" ");
             //   Serial.print(a[8]);Serial.print(" ");
                
                Serial.print("eurl:");    
                Serial.print(a[9]);Serial.print(" ");  //roll  输出欧拉角
                Serial.print(a[10]);Serial.print(" "); //pitch
                Serial.print(a[11]);Serial.print(" ");  //yaw

                Serial.print(a[16]);Serial.print(" ");   //校验位
              //  Serial.print("q:");
             //   Serial.print(a[12]);Serial.print(" ");
             //   Serial.print(a[13]);Serial.print(" ");
             //   Serial.print(a[14]);Serial.print(" ");
              //  Serial.print(a[15]);Serial.print(" ");
                Serial.println(""); 

                 delay(500);
     
 }
   
 


while (Serial1.available())  //Serial1
     {    //
    
   // char inChar = (char)Serial.read(); Serial.print(inChar);   //Output Original Data, use this code 
  
     CopeSerialData(Serial1.read());  //Serial1
             
    } 
      
}


void CopeSerialData(unsigned char ucData)
{
  static unsigned char ucRxBuffer[250];
  static unsigned char ucRxCnt = 0; 
  
  ucRxBuffer[ucRxCnt++]=ucData;
  if (ucRxBuffer[0]!=0x50) 
  {
    ucRxCnt=0;
    return;
  }
  if (ucRxCnt<52) {return;}
  else
  {
   if (ucRxBuffer[0]==0x50)
    {
      
      
               
      memcpy(&ubuf,&ucRxBuffer[0],52);
          
      
     
    }
    ucRxCnt=0;
  }
}
