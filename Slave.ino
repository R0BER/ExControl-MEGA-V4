#include <SimpleModbusSlave.h>

//#define EXC_ENABLE_WATCH_DOG
#ifdef EXC_ENABLE_WATCH_DOG
  #include <avr/wdt.h>
#endif
#define EXC_INTERNAL_RESISTOR //Enable internal pull-up

#define BusSlaveNumber 3
#define baud 19200
#define TxEnablePin 2 

enum InputType {Swicth,Button,Retroaviso};
struct Entrada {InputType Type;byte Value;unsigned int InState;unsigned long LastTimeInput;};
byte PinInput[]={3,4,5,6,7,8};
byte PinOutput[]={10,11,12,A0,A1,A2};

const byte Number_Input=sizeof(PinInput);
const byte Number_Output=sizeof(PinOutput);
Entrada Inputs[Number_Input];
boolean Outs[32];

enum 
{
  OUT1,OUT2,     
  IN1,IN2,IN3,IN4,IN5,IN6,
  SENSOR1,SENSOR2,  
  HOLDING_REGS_SIZE
};

unsigned int holdingRegs[HOLDING_REGS_SIZE]; // function 3 and 16 register array
////////////////////////////////////////////////////////////

void setup()
{
  /* parameters(HardwareSerial* SerialPort,
                long baudrate, 
		unsigned char byteFormat,
                unsigned char ID, 
                unsigned char transmit enable pin, 
                unsigned int holding registers size,
                unsigned int* holding register array)
  */
  
  	
  modbus_configure(&Serial, baud, SERIAL_8N2, BusSlaveNumber, TxEnablePin, HOLDING_REGS_SIZE, holdingRegs);    
  for (int i=0; i<Number_Input;i++){Inputs[i].Type=Swicth;}
  //Set Input type
  
  for (int i=0; i<Number_Output;i++){pinMode(PinOutput[i], OUTPUT);SetRelay(PinOutput[i],false);}
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
  int o=0;
  for (int o=0; o<16;o++){if (bitRead(holdingRegs[OUT1], o)==1){Outs[o]= true;}else{Outs[o]= false;}}
  for (int o=0; o<16;o++){if (bitRead(holdingRegs[OUT2], o)==1){Outs[o+16]= true;}else{Outs[o+16]= false;}}
  OutControl();  
  
  holdingRegs[IN1]=Inputs[0].Value;holdingRegs[IN2]=Inputs[1].Value;holdingRegs[IN3]=Inputs[2].Value;
  holdingRegs[IN4]=Inputs[3].Value;holdingRegs[IN5]=Inputs[4].Value;holdingRegs[IN6]=Inputs[5].Value;
  //holdingRegs[IN1],holdingRegs[IN2],holdingRegs[IN3],holdingRegs[IN4],holdingRegs[IN5],holdingRegs[IN6]
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
