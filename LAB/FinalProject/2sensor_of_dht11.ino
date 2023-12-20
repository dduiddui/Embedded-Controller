#include <DHT11.h>  //아두이노 온습도센서 DHT11을 사용하기위해 위에서 설치해두었던 라이브러리를 불러옵니다.
#include <SoftwareSerial.h>                // Serial 통신을 하기 위해 선언
int blueTx = 7; //Tx (보내는 핀)
int blueRx = 8; //Rx (받는 핀)
char buf1[30], buf2[30];
SoftwareSerial BTSerial(blueTx, blueRx);             // HC-06모듈 7=TXD , 8=RXD 핀 선언 
DHT11 sensor1(3); /*불러온 라이브러리 안에 몇번 PIN에서 데이터값이 나오는*/
DHT11 sensor2(4);
void setup() {
  Serial.begin(9600); /*온습도값을 PC모니터로 확인하기위해 시리얼 통신을*/
  BTSerial.begin(9600);                     // HC-06 모듈 통신 선언 (보드레이트 9600)
  
  pinMode(10, OUTPUT); 
  pinMode(11, OUTPUT); 
}




void loop() {

  float temp, humi, temp1, temp2, humi1, humi2;
  int result = sensor1.read(humi, temp); /* DHT.h 함수안에 dht11이라는 메소드를 사용해서*/ // red line

  temp1 = temp;
  humi1 = humi;
  
//  String msg1 = "Sensor1 Temp: ";
//  msg1 += temp; 
//  msg1 += "[C]\t\n";
//
//  String msg2 = "Sensor1 Humi: ";
//  msg2 += humi;
//  msg2 += "[%]\n";
//
//  
//  msg1.toCharArray(buf1, 30);
//  msg2.toCharArray(buf2, 30);
//   
//  Serial.write(buf1);
//  Serial.write(buf2);
  //BTSerial.write(buf1);
  //BTSerial.write(buf2);

  result = sensor2.read(humi, temp); /* DHT.h 함수안에 dht11이라는 메소드를 사용해서*/

  temp2 = temp;
  humi2 = humi;
//  String msg3 = "Sensor2 Temp: ";
//  msg3 += temp;
//  msg3 += "[C]\t\n";
//
//  String msg4 = "Sensor2 Humi: ";
//  msg4 += humi;
//  msg4 += "[%]\n";
//
//  
//  msg3.toCharArray(buf1, 30);
//  msg4.toCharArray(buf2, 30);
//  
//  Serial.write(buf1);
//  Serial.write(buf2);
  //BTSerial.write(buf1);
  //BTSerial.write(buf2);

  temp = temp1;      
  humi = humi2;

  String msg3 = "Temp: ";
  msg3 += temp;
  msg3 += "[C]\t\n";

  String msg4 = "Humi: ";
  msg4 += humi;
  msg4 += "[%]\n";

  msg3.toCharArray(buf1, 30);
  msg4.toCharArray(buf2, 30);
  
  Serial.write(buf1);
  Serial.write(buf2);
  

  int temp_out, humi_out;
  if (temp > 25){
    temp_out = 1;
  }else{
    temp_out = 0;
  }

  if (humi > 50){
    humi_out = 1; 
  }else{
    humi_out = 0;
  }

  digitalWrite(10, temp_out);
  digitalWrite(11, humi_out);

 
  delay(1000); /* 일반적인 딜레이 값이 아니라 DHT11에서 권장하는*/
}
