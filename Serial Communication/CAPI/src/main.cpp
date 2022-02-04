#include <Arduino.h>
#include <Rover_SerialAPI.h>
#include <stdio.h>

//https://www.google.com/search?q=upload+code+to+arduino+with+platform+io&rlz=1C1SQJL_frCA922CA922&oq=upload+code+to+arduino+with+plat&aqs=chrome.1.69i57j33i22i29i30l2.7350j1j7&sourceid=chrome&ie=UTF-8#kpvalbx=_f9vpYdOvCf-ZptQP14KwgAI14

  /*
  To read/send data from the serial you can:
  
  Use CMD:

    Windows
    >powershell
    >[System.IO.Ports.SerialPort]::getportnames()
    >$port= new-Object System.IO.Ports.SerialPort COM#,Baudrate,None,8,one //This creates the port object

    Port object commands:
    >$port.open()
    >$port.WriteLine("some string")
    >$port.ReadLine()
    >$port.Close()

    >exit //this exits powershell


  Use the platform IO monitor

  Use PUTTY
  */



#define ob 13

#define ID 'a'

bool syn2master(HardwareSerial Serial);

static char buffer[SERIAL_RX_BUFFER_SIZE];
static float fbuffer[SERIAL_RX_BUFFER_SIZE];


void setup() {
  pinMode(ob, OUTPUT);
  SerialAPI::init(ID, 9600);
}

void loop() {

  memset(fbuffer,0,SERIAL_RX_BUFFER_SIZE);
  fbuffer[0]=(float) 5.23;
  SerialAPI::send_bytes('0',fbuffer,sizeof(fbuffer[0]));


  /*
   if(Serial.available() > 0) {
      memset(buffer, 0,SERIAL_RX_BUFFER_SIZE);
      //SerialAPI::update();
      //SerialAPI::read_data(buffer,sizeof(buffer));

      //const size_t bytes_read = Serial.readBytes(buffer, Serial.available());


      delay(3000);
      fbuffer[0]=(float) 5.23;
      char str[6];
      sprintf(str,"%.3f", fbuffer[0]);
      Serial.println(str);
      SerialAPI::send_bytes('1',fbuffer,sizeof(fbuffer[0]));
      

      delay(100);
   } 
   */
   delay(3000);
}



/*
void serialEvent()
{
   while(Serial.available()) 
   {
      char ch = Serial.read();
      //Serial.write(ch);
      Serial.print(ch);
    }  
   Serial.println();
}
*/



bool syn2master(HardwareSerial Serial){
   //Ask permission to write (SYN request)
  SerialAPI::send_bytes('S',"",0);

  //Wait for answer
  int tmp = Serial.available();
  while(Serial.available()==tmp) delay(1000);
  //while(!SerialAPI::update()) delay(1000); //Doesn't take into account the bytes that we sent but don't want to clear buffer

  //Read the answer
  char buffer[SERIAL_RX_BUFFER_SIZE];
  SerialAPI::read_data(buffer,sizeof(buffer));

  Serial.write(buffer);

  while(!(buffer[1] == 'Y')){
    char buffer[SERIAL_RX_BUFFER_SIZE];

    //Ask for a retransmit of wrong ID
    SerialAPI::send_retransmit();

    //Wait for answer
    int tmp = Serial.available();
    while(Serial.available()==tmp) delay(1000);

    SerialAPI::read_data(buffer,sizeof(buffer));
  }
  return true;

}