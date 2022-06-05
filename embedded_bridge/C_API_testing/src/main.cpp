#include <Arduino.h>
#include <Rover_SerialAPI.h>
#include <stdio.h>
#include <Servo.h>


#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 


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



#define SERIAL_RX_BUFFER_SIZE 10
#define ob LED_BUILTIN
#define ID '1'

#define PWM 9
Servo serv;

float val = 0;
int angle = 0;


//void AddFloatToBuffer(FloatBuffer fb, float val);
void print_byte_array(byte* byte_array, size_t size);
void char_to_float(char* str_byte, float* f);


uint8_t buffer[SERIAL_RX_BUFFER_SIZE];

void setup() {
  pinMode(ob, OUTPUT);
  Serial.begin(9600);
  Serial.println("YOOO");
  serv.attach(PWM);
  serv.write(angle);
  //SerialAPI::init(ID, 9600);
  Serial.println("heeyyy");
}

void loop() {

  //Send command request and wait for response
  while(!SerialAPI::update()){
    digitalWrite(ob, HIGH);
    SerialAPI::send_request();
    delay(10);
    //digitalWrite(ob, LOW);
    delay(10);
  }

  //Check message
  int cur_pack_id = SerialAPI::read_data(buffer,sizeof(buffer));  

  switch(cur_pack_id){
    case '0':
      //Check if master was actually talking to this system
      if(buffer[0]==ID){
        memcpy(&val, buffer+1, 4);
        angle = (int) val;
        serv.write(angle);
      }

    case '2':
      int i =0;

    case 'A':
      int i =0;

    case 'R':
      memset(buffer,0,SERIAL_RX_BUFFER_SIZE);
      val = (float) angle;
      memcpy(buffer+1,&val,4);
      SerialAPI::send_bytes('0', buffer, SERIAL_RX_BUFFER_SIZE);

    default:
      int i =0;
  }




  /*
  if(SerialAPI::update()){
        digitalWrite(ob, HIGH);

        memset(buffer, 0, SERIAL_RX_BUFFER_SIZE);
        int cur_pack_id = SerialAPI::read_data(buffer,sizeof(buffer));

        //const size_t payload_size = strlen(buffer); //DOESN'T WORK IF THERE ARE ZEROs BECAUSE IT'S CONSIDERED A NULL CHARACTER

        memcpy(&val, buffer+1, 4);

        angle = (int) val;

        delay(100);

        //SerialAPI::send_bytes('1', buffer, payload_size);
        if (SerialAPI::send_bytes('1', buffer, 5)) digitalWrite(ob, LOW);
  } 
  */
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



/*
//Will make a lib for that
void AddFloatToBuffer(FloatBuffer fb,float val)
{
    if(fb.count < SERIAL_RX_BUFFER_SIZE/sizeof(float))
    {
        fb.b[fb.count] = val;
        fb.count++;
    }
}



void decode_msg(char* buffer){




//Not used, just send or poll data.
char sync(void){
  //Ask permission to write (SYN request)
  SerialAPI::send_bytes('S',"",0);

  //Wait for answer
  int tmp = Serial.available();
  while(!SerialAPI::update()) delay(1000);

  //Read the answer
  char buffer[SERIAL_RX_BUFFER_SIZE];
  SerialAPI::read_data(buffer,sizeof(buffer));

  Serial.write(buffer);

  if(buffer[1] == 'Y'){
    return 'Y';
  }
  else if(buffer[1] == 'R'){
    return 'R';
  }
  else{
    return ' ';
  }


  while(!(buffer[1] == 'Y')){
    memset(buffer,0,SERIAL_RX_BUFFER_SIZE);

    //Ask for a retransmit of wrong ID
    //SerialAPI::send_retransmit(); Deprecated

    //Wait for answer
    while(!SerialAPI::update()) delay(10);

    //int tmp = Serial.available();
    //while(Serial.available()==tmp) delay(1000);

    SerialAPI::read_data(buffer,sizeof(buffer));
  }
  

  //External validation
  for(int i=0;i<5;i++){
    digitalWrite(ob,LOW);
    delay(200);
    digitalWrite(ob,HIGH);
    delay(200);
  }
}



void print_byte_array(byte* byte_array, size_t size){
  char buffer[1];
  memset(buffer,0,1);
  for (int i = 0; i<size;i++){
    sprintf(buffer, BYTE_TO_BINARY_PATTERN" ", BYTE_TO_BINARY(byte(byte_array[i])));
    Serial.print(buffer);
  }
}


void char_to_float(char* str_byte, float* f){
  char array[4];

  #ifdef BIGENDIAN
  array[0] = byte(str_byte[3]);
  array[1] = byte(str_byte[2]);
  array[2] = byte(str_byte[1]);
  array[3] = byte(str_byte[0]);
  #else 
  array[0] = byte(str_byte[0]);
  array[1] = byte(str_byte[1]);
  array[2] = byte(str_byte[2]);
  array[3] = byte(str_byte[3]);
  #endif

  memcpy(f,array,4);
}

*/