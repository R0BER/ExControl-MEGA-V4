#include <SimpleModbusMaster.h>

//#define EXC_ENABLE_WATCH_DOG
#ifdef EXC_ENABLE_WATCH_DOG
  #include <avr/wdt.h>
#endif
#define EXC_INTERNAL_RESISTOR //Enable internal pull-up
//////////////////// MODBUS SLAVES SETTING ///////////////////
//#define MBUS_SLAVE2
//#define MBUS_SLAVE3
//#define MBUS_SLAVE4

//////////////////// MODBUS PORT SETTING ///////////////////

#define baud 19200
#define timeout 1000
#define polling 50 // the scan rate
#define retry_count 10
#define TxEnablePin 2 
//////////////////// SYSTEM SETTING ///////////////////
enum InputType {Swicth,Button,Retroaviso};
struct Entrada {InputType Type;byte Value;unsigned int InState;unsigned long LastTimeInput;};
byte PinInput[]={3,4,5,6,7,8};
byte PinOutput[]={10,11,12,A0,A1,A2};

const byte Number_Input=sizeof(PinInput);
const byte Number_Output=sizeof(PinOutput);
Entrada Inputs[Number_Input];
boolean Outs[32];


//////////////////// SYSTEM PACKETS ///////////////////
enum
{
  #ifdef MBUS_SLAVE2
    PACKET1,
    PACKET6,
  #endif
  #ifdef MBUS_SLAVE3
    PACKET2,
    PACKET7,
  #endif
  #ifdef MBUS_SLAVE4
    PACKET3,
    PACKET8,
  #endif 
  PACKET4,
  PACKET5, 
  TOTAL_NO_OF_PACKETS // leave this last entry
};
// Create an array of Packets to be configured
Packet packets[TOTAL_NO_OF_PACKETS];
const byte DeviceNumber=TOTAL_NO_OF_PACKETS/2;
// Create a packetPointer to access each packet



#ifdef MBUS_SLAVE2
  packetPointer packet1 = &packets[PACKET1];
  packetPointer packet6 = &packets[PACKET6];
  unsigned int Input2[]={0,0,0,0,0,0,0,0};
#endif
#ifdef MBUS_SLAVE3
  packetPointer packet2 = &packets[PACKET2];
  packetPointer packet7 = &packets[PACKET7];
  unsigned int Input3[]={0,0,0,0,0,0,0,0};
#endif
#ifdef MBUS_SLAVE4
  packetPointer packet3 = &packets[PACKET3];
  packetPointer packet8 = &packets[PACKET8];
  unsigned int Input4[]={0,0,0,0,0,0,0,0};
#endif

packetPointer packet4 = &packets[PACKET4];
packetPointer packet5 = &packets[PACKET5];


unsigned int TotalInput[DeviceNumber*8];
// Data to be written to the arduino slave
unsigned int Output[2];

void setup(){
  for (int i=0; i<Number_Input;i++){Inputs[i].Type=Swicth;}
  //Set Input type
  
  
  for (int i=0; i<Number_Output;i++){pinMode(PinOutput[i], OUTPUT);SetRelay(PinOutput[i],false);}
  byte packetInNumber=8*DeviceNumber;

  #ifdef MBUS_SLAVE2
    // entradas Inputs
    modbus_construct(packet1, 3, READ_HOLDING_REGISTERS, 2, 8, Input2);
    // Salidas, output 
    modbus_construct(packet6, 3, PRESET_MULTIPLE_REGISTERS, 0, 2, Output);
  #endif
  #ifdef MBUS_SLAVE3
    // entradas Inputs
    modbus_construct(packet2, 4, READ_HOLDING_REGISTERS, 2, 8, Input3);
    // Salidas, output 
    modbus_construct(packet7, 4, PRESET_MULTIPLE_REGISTERS, 0, 2, Output);
  #endif
  #ifdef MBUS_SLAVE4
    // entradas Inputs
    modbus_construct(packet3, 5, READ_HOLDING_REGISTERS, 2, 8, Input4);
    // Salidas, output 
    modbus_construct(packet8, 5, PRESET_MULTIPLE_REGISTERS, 0, 2, Output);
  #endif
  // write input state 
  modbus_construct(packet4, 2, PRESET_MULTIPLE_REGISTERS, 2, packetInNumber, TotalInput);
  // read out starting at address 0  
  modbus_construct(packet5, 2, READ_HOLDING_REGISTERS, 0, 2, Output);
  
  
  
  // P.S. the register array entries above can be different arrays
  
  /* Initialize communication settings:
     parameters(HardwareSerial* SerialPort,
		long baud, 
		unsigned char byteFormat,
		unsigned int timeout, 
		unsigned int polling, 
		unsigned char retry_count, 
		unsigned char TxEnablePin,
		Packet* packets, 
		unsigned int total_no_of_packets);

     Valid modbus byte formats are:
     SERIAL_8N2: 1 start bit, 8 data bits, 2 stop bits
     SERIAL_8E1: 1 start bit, 8 data bits, 1 Even parity bit, 1 stop bit
     SERIAL_8O1: 1 start bit, 8 data bits, 1 Odd parity bit, 1 stop bit
     
     You can obviously use SERIAL_8N1 but this does not adhere to the
     Modbus specifications. That said, I have tested the SERIAL_8N1 option 
     on various commercial masters and slaves that were suppose to adhere
     to this specification and was always able to communicate... Go figure.
     
     These are already defined in the Arduino global name space. 
  */
  
  modbus_configure(&Serial, baud, SERIAL_8N2, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS);

  //

  
  

  unsigned int reading;
  for (int i=0; i<Number_Input;i++){
    pinMode(PinInput[i], INPUT);
    #ifdef EXC_INTERNAL_RESISTOR
     digitalWrite(PinInput[i], HIGH);       // turn on pullup resistors
    #endif
    Inputs[i].LastTimeInput=millis();
    if (Inputs[0].Type==Button){reading = LOW;Inputs[i].Value=100;Inputs[i].InState=0;}
    else{
      Inputs[i].InState=digitalRead(PinInput[i]);
      if (Inputs[i].InState==HIGH){Inputs[i].Value=0;}else{Inputs[i].Value=1;}    
    }
  }
  #ifdef EXC_ENABLE_WATCH_DOG
   wdt_enable(WDTO_8S );
 #endif 
}
void OutControl(){

}
void loop()
{
  #ifdef EXC_ENABLE_WATCH_DOG
    wdt_reset();
  #endif 
  InputState();
  modbus_update();
  
  int c;
  for  (c=0;c<6;c++){TotalInput[c]=Inputs[c].Value;}
  #ifdef MBUS_SLAVE2
    for  (c=8;c<16;c++){TotalInput[c]=Input2[c-8];}
  #endif
  #ifdef MBUS_SLAVE3
    for  (c=16;c<24;c++){TotalInput[c]=Input3[c-16];}
  #endif
  #ifdef MBUS_SLAVE4
    for  (c=24;c<32;c++){TotalInput[c]=Input4[c-24];}
  #endif
  
  int o=0;
  for (int o=0; o<16;o++){if (bitRead(Output[0], o)==1){Outs[o]= true;}else{Outs[o]= false;}}
  for (int o=0; o<16;o++){if (bitRead(Output[1], o)==1){Outs[o+16]= true;}else{Outs[o+16]= false;}}
  OutControl();

}
void SetRelay(byte Pin, boolean On){if (On){digitalWrite(Pin,HIGH);}else{digitalWrite(Pin,LOW);}}
void InputState(){
  int reading;
  for (int i=0; i<Number_Input;i++){
    unsigned long InputMillis = millis()-Inputs[i].LastTimeInput;
    reading = digitalRead(PinInput[i]);
    if (Inputs[i].Type==Button){

     
      if (Inputs[i].InState>=4){Inputs[i].InState=0;}
      if (Inputs[i].InState<0){Inputs[i].InState=0;}
      if (digitalRead(PinInput[i]) == LOW ) {
        if ((Inputs[i].InState==0)&&(InputMillis>=60)){Inputs[i].LastTimeInput=millis();Inputs[i].InState=1;}
        if ((Inputs[i].InState==1)&&(InputMillis>=440)){LongInput(i);Inputs[i].InState=2;Inputs[i].Value=110;}
        if (Inputs[i].InState==2){Inputs[i].LastTimeInput=millis();}
        if (Inputs[i].InState==3){Inputs[i].LastTimeInput=millis();Inputs[i].InState=1;}
      }
      else{
        
        if (Inputs[i].InState==0){Inputs[i].LastTimeInput=millis();}
        if (Inputs[i].InState==1){Inputs[i].LastTimeInput=millis();Inputs[i].InState=3;}
        if ((Inputs[i].InState==2) &&(InputMillis>=60)){Inputs[i].LastTimeInput=millis();Inputs[i].InState=0;LongInputEnd(i);Inputs[i].Value=100;}
        if ((Inputs[i].InState==3)&&(InputMillis>=60)){Inputs[i].LastTimeInput=millis();Inputs[i].InState=0;ShortInput(i);Inputs[i].Value++;if(Inputs[i].Value>108){Inputs[i].Value=101;}}
       }
    }
    else{
      
      if (reading==Inputs[i].InState){Inputs[i].LastTimeInput=InputMillis;}
      else{
        if(Inputs[i].LastTimeInput>InputMillis){Inputs[i].LastTimeInput=InputMillis;}
        if ((InputMillis-Inputs[i].LastTimeInput)>=60){Inputs[i].InState=reading;if (Inputs[i].InState==HIGH){Inputs[i].Value=0;}else{Inputs[i].Value=1;}SwicthStateChange(i,reading);}    
      }
    }   
  }
}

      
/******************************************************************************************************************************/
 /***********************************************************************************************************************/
/***********************************************************************************************************************/
//EVENTOS CONTROL ENTRADAS SALIDAS
//INPUT OUTPUT CONTROL EVENTS

//CUATRO EVENTOS PARA ENTRADAS DIGITALES
//CONMUTADOR CAMBIA VALOR
//PULSACION CORTA
//PULSACION LARGA, MAYOR DE 0.5 SEGUNDOS
//FINAL PULSACION LARGA

// FOUR EVENTS FOR DIGITAL INPUTS
//VALUE CHANGE SWITCH
// PRESS SHORT
// PRESS LONG, OVER 0.5 SECONDS
// LONG PRESS END.

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void SwicthStateChange(byte NumberInput, unsigned int value){


    /*****************************************************************/
  //Este evento se produce cuando un conmutador cambia posicion
  // This event occurs with swicth change state.
  //dos parametros
  //two parameters
  //Number input--numero de entrada
  //Value, if input is HIGH value = HIGH, If value=low then Value=Low
  //Value, si la entrada esta en nivel alto el valor es HIGH si es bajo el valor es LOW
  //Tambien puede acceder desde cualquier punto del codigo al valor de la entrada con SwicthState[]
  //You can know input state anywhere using SwictState[]

}
void ShortInput(byte NumberInput){
/*************************************************************/
//Este evento se produce con una pulsación corta..
//This event occurs with a short press.


}
void LongInputEnd(byte NumberInput){
/*************************************************************/
//Este evento se produce con una pulsación corta..
//This event occurs with a short press.

 
  /*****************************************************************/
  //FINAL DE PULSACION LARGA
  //LONG PRESS END, EVENT
  // This event occurs with end a long press.
  /*****************************************************************/
  
}
void LongInput(byte NumberInput){
/*************************************************************/
//Este evento se produce con una pulsación corta..
//This event occurs with a short press.
/*************************************************************/

}
