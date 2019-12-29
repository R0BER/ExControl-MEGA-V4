
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

#define EXC_ENABLE_WATCH_DOG
#ifdef EXC_ENABLE_WATCH_DOG
  #include <avr/wdt.h>
#endif
#define EXC_INTERNAL_RESISTOR //Enable internal pull-up
//#define EXC_NRF24SPEED 1

//Configuracion dispositvos

//#define EXC_LCD
//#define EXC_LED_IR 
//#define EXC_IR_RECIVE 
//#define EXC_RECEIVER_433
//#define EXC_TRANSMITER_433
//#define EXC_RGB_LED
//#define THERMOSTAT_DS18B20_NUMBER
//#define THERMOSTAT_DTH22
//#define EXC_DEBUG_MODE
#define EXC_InputOn  LOW


//carga de librerias  dispositivos
#if defined (EXC_TRANSMITER_433)  || defined (EXC_RECEIVER_433)
  //#include <RCSwitch.h>
#endif

#if defined (EXC_LED_IR)  || defined (EXC_IR_RECIVE)
  //#include <IRremote.h>
#endif
#ifdef THERMOSTAT_DS18B20_NUMBER 
  #include <OneWire.h>
  #include <DallasTemperature.h>
#endif 
#ifdef EXC_LCD 
  #include <Wire.h> 
 #include <LiquidCrystal_I2C.h>
 #include   "printLCD.h"
#endif 

#ifdef THERMOSTAT_DTH22 
  #include "DHT.h"
#endif 

/******************************USER CODE LIBRARY START******************************************************************/
//USER CODE LIBRARY START

/******************************USER  LIBRARY END********************************************************************/


//////////////////// SLAVES SETTING ///////////////////
//#define MBUS_SLAVE2
//#define MBUS_SLAVE3
//#define MBUS_SLAVE4

const uint16_t node_address_set[5] = { 00, 02, 03, 04, 05}; 

//const  node_address_set[5] = { 00, 02, 05, 015, 025}; 
// 0 = Master
// 1-2 (02,05)   = Children of Master(00)
// 3,5 (012,022) = Children of (02)
// 4,6 (015,025) = Children of (05)
// 7   (032)     = Child of (02)
// 8,9 (035,045) = Children of (05)

/*configuracion nueva*/
const uint8_t RF24channel = 90;
uint8_t NODE_ADDRESS = 0;  // Use numbers 0 through to select an address from the array
//RF24 (cepin, cspin) 
RF24 radio(9,10);                              // CE & CS pins to use (Using 7,8 on Uno,Nano)
RF24Network network(radio); 

uint16_t this_node;       
uint16_t NodoComunicando=0;
unsigned long TimSend=0;
byte TimRcv=0;
byte   LostPacket=0;
/*Fin configuracion Nueva*/

//////////////////// SYSTEM SETTING ///////////////////
unsigned short Output[2];
enum InputType {Swicth,Button,Retroaviso,ExcIDetector};
struct Entrada {InputType Type;byte Value;unsigned int InState;unsigned long LastTimeInput;};
byte PinInput[]={0,1,2,3,4,5};
byte PinOutput[]={6,7,8,A0,A1,A2};

const byte Number_Input=sizeof(PinInput);
const byte Number_Output=sizeof(PinOutput);
Entrada Inputs[Number_Input];
boolean Outs[32];

//unsigned short Input2[]={0,0,0,0,0,0,0,0};
//unsigned short Input3[]={0,0,0,0,0,0,0,0};
//unsigned short Input4[]={0,0,0,0,0,0,0,0};


#ifdef MBUS_SLAVE4
 const byte DeviceNumber=4;
#else
  #ifdef MBUS_SLAVE3
   const byte DeviceNumber=3;

  #else
    #ifdef MBUS_SLAVE2
     const byte DeviceNumber=2;
    #else
     const byte DeviceNumber=1;
    #endif
  #endif
#endif
//unsigned short TotalInput[DeviceNumber][8];
//unsigned short TotalInputOld[DeviceNumber][8];
unsigned short TotalInput[8];
unsigned short TotalInputOld[8];

boolean DvUpdated[DeviceNumber];

unsigned long TimSendRF24 = 0;
//const int TotalInputSize =sizeof(TotalInput);
const int OutputSize =sizeof(Output);
const int InputSize=sizeof(TotalInput);

// Data to be written to the arduino slave

//declarando dispositivos
unsigned long TimNow=0;
unsigned long ExTimer=0;
byte CountMsg=0;byte CountSg=0;byte CountRfSg=0;


boolean Escucha=false;
unsigned long TimAcDet=500;
#ifdef EXC_RGB_LED
  byte RGBredVal = 255;
  byte RGBblueVal = 0;
  byte RGBgreenVal =0;
  byte RGBSpeed=4;
  byte RGBrandomCount=0;
#endif
#ifdef THERMOSTAT_DTH22 
  #define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
  #define DHTPIN 2 //datos sensor dht pin 2
  DHT dht(DHTPIN, DHTTYPE);
  float HumedadDHT=0; //Variable para almacenar sonda de humedad.
  float TemperatureDHT=0; // Variable para almacenar sonda Temperatura
  boolean AveriaDHT=false;
  byte DhtErrorCount=0;

#endif 
#ifdef THERMOSTAT_DS18B20_NUMBER 

  #define TEMPERATURE_PRECISION 9
  #define ONE_WIRE_BUS 3           //Pin one wire donde estan conectadas las sondas  DeviceAddress Ds18B20Addres[THERMOSTAT_DS18B20_NUMBER] ={{0x28,0xAB,0x2A,0x46,0x04,0x00,0x00,0x88}};
  OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
  DallasTemperature sensorTemp(&oneWire);// Pass our oneWire reference to Dallas Temperature.   
  int Ds18b20Count;
  DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address
  float Temperature[]={20};
 
#endif 

#ifdef EXC_LED_IR  
  IRsend irsend;
#endif 

#ifdef EXC_IR_RECIVE
  IRrecv irrecv(19);//El 19 corresponde con el pin de arduino, cambiar para utilizar otro
  decode_results results;
#endif 

#if defined (EXC_TRANSMITER_433)  || defined (EXC_RECEIVER_433)
  RCSwitch mySwitch = RCSwitch();
#endif 

#ifdef EXC_LCD
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year,minutoMemory,TipoDia;
   LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 
#endif

/******************************GLOBAL VARIABLES ZONE******************************************************************/
//GLOBAL VARIABLES ZONE

/****************************** END VARIABLES ZONE ********************************************************************/


void setup(){
  
   for (int i=0; i<Number_Input;i++){Inputs[i].Type=Swicth;Inputs[i].Value=0;Inputs[i].InState=0;Inputs[i].LastTimeInput=0;}
  //Set Input type
  
  
  for (int i=0; i<Number_Output;i++){pinMode(PinOutput[i], OUTPUT);SetRelay(PinOutput[i],false);}
  for (int d=0;d<DeviceNumber;d++){DvUpdated[d]=true;}

  #ifdef MBUS_SLAVE2

  #endif
  #ifdef MBUS_SLAVE3

  #endif
  #ifdef MBUS_SLAVE4

  #endif

  
    
  this_node = node_address_set[NODE_ADDRESS];            // Which node are we?
  
  SPI.begin();     // Bring up the RF network 
  DevicesStart();
  IniciarNrf();
  

  for (int i=0; i<Number_Input;i++){
    pinMode(PinInput[i], INPUT);
    #ifdef EXC_INTERNAL_RESISTOR
     digitalWrite(PinInput[i], HIGH);       // turn on pullup resistors
    #endif
    Inputs[i].LastTimeInput=millis();
    if (Inputs[0].Type==Button){Inputs[i].Value=100;Inputs[i].InState=0;}
    
    else if ((Inputs[i].Type==Swicth)||(Inputs[i].Type==Retroaviso)){
      Inputs[i].LastTimeInput=millis();
      if (digitalRead(PinInput[i]) == EXC_InputOn){Inputs[i].InState=1;}else{Inputs[i].InState=0;}
      Inputs[i].Value=Inputs[i].InState;  
    }
  }
  #ifdef EXC_ENABLE_WATCH_DOG
   wdt_enable(WDTO_8S );
 #endif 
 UserSetup();
}
void UserSetup() { //EQUIVALENT ARDUINO SETUP  FUNCTION

}
void OutControl(){

}
void loop()
{
  #ifdef EXC_ENABLE_WATCH_DOG
    wdt_reset();
  #endif 
  InputState();
  GestRadioRf24();  
  
  OutByte();
  DevicesLoop();UserLoop();
  OutControl();

}
void OutByte(){
  int o=0;
  for (int o=0; o<16;o++){if (bitRead(Output[0], o)==1){Outs[o]= true;}else{Outs[o]= false;}}
  for (int o=0; o<16;o++){if (bitRead(Output[1], o)==1){Outs[o+16]= true;}else{Outs[o+16]= false;}}
}
void UserLoop(){ //EQUIVALENT ARDUINO LOOP FUNCTION

}
void SetRelay(byte Pin, boolean On){if (On){digitalWrite(Pin,HIGH);}else{digitalWrite(Pin,LOW);}}
void InputState(){
  unsigned long InputMillis;
  unsigned int v=0;
  boolean acT=false;
  
  InputMillis = millis();
  if (InputMillis<TimAcDet) {TimAcDet=InputMillis;acT=true;}
  else if ((InputMillis - TimAcDet)>=2)  {TimAcDet=InputMillis;acT=true;}
  
  for (int i=0; i<Number_Input;i++){
    switch (Inputs[i].Type) {
      case Button: {
        if (digitalRead(PinInput[i])==EXC_InputOn){v=1;}else{v=0;}
        InputMillis = millis();  
        if(Inputs[i].LastTimeInput>InputMillis){Inputs[i].LastTimeInput=InputMillis;}
        else{
          InputMillis = InputMillis -Inputs[i].LastTimeInput;       
          if ((Inputs[i].InState>=4)||(Inputs[i].InState<0)){Inputs[i].InState=0;}     
          if (v==1){ 
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
        break;}      

      case ExcIDetector: {
 

        if (acT==false){break;}
        v=analogRead(PinInput[i])/4;
        if (v>254){v=254;}
  
        byte Hv=highByte(Inputs[i].LastTimeInput); 
        byte Lv=lowByte(Inputs[i].LastTimeInput);
           
        if ((v<100)||(v>210)){if (Lv<110){Lv=Lv+4;}}
        else{
          if (v==Hv) {if (Lv>0){Lv--;}}
          else{ 
            byte d;
            if (v>Hv){d=v-Hv;}else{d=Hv-v;}
            if (d<=4){if (Lv>0){Lv--;}}else{if (Lv<110){Lv=Lv+4;}}           
          }
        }
  
        Inputs[i].LastTimeInput=word(v, Lv);
        
        if (Lv>100){v=1;} 
        else if (Lv==0){v=0;}
        else {break;}
        
        if (Inputs[i].Value!=v){
            Inputs[i].Value=v;
            if (v==1){SwicthStateChange(i,HIGH);}else{SwicthStateChange(i,LOW);}        
        }
        break;}      
      default:{
         if (digitalRead(PinInput[i])==EXC_InputOn){v=1;}else{v=0;}
          InputMillis = millis();
          if (v==Inputs[i].Value){Inputs[i].LastTimeInput=InputMillis;}
          else{
            if(Inputs[i].LastTimeInput>InputMillis){Inputs[i].LastTimeInput=InputMillis;}
            if ((InputMillis-Inputs[i].LastTimeInput)>=60){Inputs[i].LastTimeInput=InputMillis; Inputs[i].Value=v;if (v){SwicthStateChange(i,HIGH);}else{SwicthStateChange(i,LOW);}}    
          }
          break;}     
        
    }
    TotalInput[i]=Inputs[i].Value;    
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
void SwicthStateChange(int NumberInput,unsigned int value){


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
void LoopNew100MillisSg(){//This event occurs every 100Millis Sg.
  
  //Este evento se produce cada 100 milisegundos
  //This event occurs every 100Millis Sg.
}
void LoopNewSecond(){//This event occurs every second

}
void Loop30Sg(){//This event occurs every 30 second
  
  //Este evento se produce cada 30sg.
  //This event occurs every 30SG.
}



//Logica funcionamiento dispositivos
void Mhz433In(int value){//433Mhz reception

}
#ifdef EXC_IR_RECIVE
void IrRecive(int codeType,int Value){
  
  
}

void ComprobarInfrarro() {
  
  if (irrecv.decode(&results)){  
  #ifdef EXC_DEBUG_MODE        
      Serial.println("IR RECIVE");   
    #endif    
     int codeType = -1; // The type of code
     decode_results *result;
     result=&results;
     codeType = result->decode_type;
     
     IrRecive(codeType,result->value);     //Funcion de usuario de recepcion infrarojo
     irrecv.resume(); // Receive the next value.
     
   }
}
#endif 
 #ifdef EXC_RECEIVER_433  
  void Init433Mhz(){mySwitch.enableReceive(0);}// Receiver on inerrupt 0 => that is pin #2

  void Recepcion433Mhz() {
    if (mySwitch.available()) {
      
      int value = mySwitch.getReceivedValue();
      Mhz433In(value);
      #ifdef EXC_DEBUG_MODE  
          Serial.println("433 Mhz Received");
         if (value == 0) {
          Serial.println("Unknown encoding");
         } else {
          
          Serial.print("variable value= ");
          Serial.print(value);
          Serial.print(" / ");
          Serial.print( mySwitch.getReceivedBitlength() );
          Serial.print("bit ");
          Serial.print("Protocol: ");
          Serial.println( mySwitch.getReceivedProtocol() );
        }
      #endif
      
      
      mySwitch.resetAvailable();
   }
}
#endif
void GestRadioRf24(){
  network.update(); 
  //bool cmp=network.available();
  while ( network.available() )  {                      // Is there anything ready for us?
    Escucha=false;
    RF24NetworkHeader header;                            // If so, take a look at it
    network.peek(header);

    if (header.type == 'O'){
       #ifdef EXC_DEBUG_MODE
        Serial.println("Recepcion Arduino Mega");
      #endif 
      TimRcv=0;
      if (DeviceNumber>1){NodoComunicando=1;for (int d=0;d<DeviceNumber;d++){DvUpdated[d]=false;}}
      network.read(header,Output,OutputSize);OutByte();OutControl();
      
    }
    else{network.read(header,0,0); break;}
  }   
  SendTrama();
} 
void SendTrama(){

  if (Escucha){
    unsigned long t = millis();
    if (t<TimSend){TimSend=t;}
    if ((t - TimSend) >= 180 ){Escucha=false;}
    else{return;}
  }
    
    

 //Envio
    //boolean Fallo=false;   
  if (NodoComunicando==0){
        for (byte k=0;k<8; k++){if ( TotalInput[k]!=TotalInputOld[k]){MegaCom();return;}}  
        if (DeviceNumber>1){NodoComunicando=1;}
    }
    else{
      if (NodoComunicando>=DeviceNumber){NodoComunicando=0;}
      else{SlvCom(node_address_set[1+NodoComunicando]);}     
    }
   
}
void ActivarEscucha(){
  LostPacket++;
  if (LostPacket>50){delay(1000);IniciarNrf();}
  TimSend=millis();
  Escucha=true;
}
void   IniciarNrf(){
  #ifdef EXC_DEBUG_MODE        
      Serial.println("Iniciando NRF");   
   #endif
  TimRcv=0;
  LostPacket=0;
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  #ifdef EXC_NRF24SPEED 
    boolean res=false;
    switch (EXC_NRF24SPEED){
      case 0:{res= radio.setDataRate(RF24_250KBPS);}  
      case 1:{res= radio.setDataRate(RF24_1MBPS);} 
      case 2:{res= radio.setDataRate(RF24_2MBPS);}
      default:{res=true;}
    } 
    #ifdef EXC_DEBUG_MODE  
      if (res){Serial.println("Nrf new speed ok");}else{Serial.println("Nrf new speed ERROR");} 
    #endif     
  #endif
  radio.setChannel(RF24channel); 
  network.begin( this_node );
  //network.begin( RF24channel , /*node address*/ this_node );
}
boolean SlvCom(uint16_t Addr ){// Arduino mega, enviar entradas  
   boolean r=true;
  if (DvUpdated[NodoComunicando]==false){
    #ifdef EXC_DEBUG_MODE
      Serial.print("Enviando Slave Addr ");
      Serial.println(Addr);
    #endif 
    RF24NetworkHeader header(Addr, 'O');
    r =  network.write(header,Output,OutputSize);
    if (r){DvUpdated[NodoComunicando]=true;LostPacket=0;}else{ActivarEscucha();DvUpdated[NodoComunicando]=false;}
    #ifdef EXC_DEBUG_MODE
       if (r){ Serial.println("Enviado datos ok");}else{Serial.println("Error envio");}
    #endif 
  }
  
  NodoComunicando++;
  if (NodoComunicando>=DeviceNumber){NodoComunicando=0;}
  return r;
}
boolean MegaCom(){// Arduino mega, enviar entradas  
  CountRfSg=0;
  if (Escucha){return true;}
   #ifdef EXC_DEBUG_MODE
    Serial.println("Enviando datos arduino mega");
  #endif 
  RF24NetworkHeader header( node_address_set[1], 'I');
  boolean r = network.write(header,TotalInput,InputSize);
  if (r){LostPacket=0; for (byte k=0;k<8; k++){TotalInputOld[k]=TotalInput[k];}}
  else{ActivarEscucha();}

   #ifdef EXC_DEBUG_MODE
   if (r){ Serial.println("Enviado mega ok");}else{Serial.println("Error envio");}
  #endif 
  NodoComunicando++;
  if (NodoComunicando>=DeviceNumber){NodoComunicando=0;}
  return r;
}
void DevicesLoop(){
   #ifdef EXC_IR_RECIVE   
    ComprobarInfrarro();
  #endif 
  
  #ifdef EXC_RECEIVER_433  
    Recepcion433Mhz();
  #endif
  TimNow=millis();   
   if(TimNow < ExTimer ) {ExTimer=TimNow;}
   if((TimNow - ExTimer) >= 100) {ExTimer=TimNow;ExcTimer();}
  
 }
void DevicesStart(){
   #ifdef EXC_DEBUG_MODE   
    Serial.begin(9600);      
    Serial.println("Sytem Start");   
  #endif
 #ifdef EXC_LCD
    lcd.begin(20,4);loadCharsLCD(); 
    lcd.backlight();
  #endif
  #ifdef THERMOSTAT_DTH22 
    dht.begin();      
  #endif 
 #ifdef EXC_RECEIVER_433  
    Init433Mhz();// Receiver on inerrupt 0 => that is pin #2
 #endif
 #ifdef EXC_TRANSMITER_433 
  mySwitch.enableTransmit(EXC_433enableTransmit);
 #endif
   #ifdef EXC_IR_RECIVE
   irrecv.enableIRIn();
 #endif
  #ifdef THERMOSTAT_DS18B20_NUMBER
      sensorTemp.begin();
      Ds18b20Count = sensorTemp.getDeviceCount();
      for(int i=0;i<Ds18b20Count; i++){if(sensorTemp.getAddress(tempDeviceAddress, i)) {sensorTemp.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION); }}
      sensorTemp.setWaitForConversion(true);
      RefreshTemperature();
#endif 

}

#ifdef THERMOSTAT_DS18B20_NUMBER 

   void RefreshTemperature(){
     sensorTemp.requestTemperatures();
     float f= sensorTemp.getTempCByIndex(0);
     if (f>-100){Temperature[0] =f;}
  }
#endif 

//Variables de temporizacion

void ExcTimer(){
  LoopNew100MillisSg();
  #ifdef EXC_RGB_LED
    RgbRandomColor();
  #endif
  CountMsg++;
  if (CountMsg>=10){
      CountMsg=0;CountSg++;CountRfSg++;LoopNewSecond();
      #ifdef THERMOSTAT_DS18B20_NUMBER
        if (CountSg==10){RefreshTemperature();}
      #endif
      
      #ifdef THERMOSTAT_DTH22 
         if (CountSg==4){ReadDHT();}        
      #endif 
      
      TimRcv++;
      if (TimRcv>=70){delay(1000);IniciarNrf();}
      
      if (CountSg>=30){Exc30Seg();}
      if (CountRfSg>=30){UpdateData();}    
  }
}
void UpdateData(){
  for (int i=0; i<Number_Input;i++){
    if (Inputs[0].Type==Button){if (Inputs[i].Value < 110 )TotalInput[i]=100;Inputs[i].Value=100;}
  } 
  
   MegaCom();
}

void Exc30Seg(){
   CountSg=0;        
   Loop30Sg();   
}

#ifdef EXC_RGB_LED
  void RgbRandomColor(){    
    switch (RGBrandomCount) {
    case 2: 
      
      if ((RGBredVal + RGBSpeed)>254){RGBgreenVal = 0;RGBredVal = 255;RGBrandomCount=0;}
      else{
        RGBredVal += RGBSpeed;
        RGBblueVal -= RGBSpeed; 
        RGBgreenVal=0;
      }
      break;
    case 1:  
      if ((RGBblueVal + RGBSpeed)>254){RGBredVal = 0;RGBblueVal = 255;RGBrandomCount=2;}
      else{
        RGBblueVal += RGBSpeed;
        RGBgreenVal -= RGBSpeed; 
        RGBredVal=0;
      }
      break;
    case 0: 
      if ((RGBgreenVal + RGBSpeed)>254){RGBblueVal = 0;RGBgreenVal = 255;RGBrandomCount=1;}
      else{
        RGBgreenVal += RGBSpeed;
        RGBredVal -= RGBSpeed; 
        RGBblueVal=0;
      }
      break;
    } 
  }
  
  void SetRGBLed(byte RedPin, byte GreenPin, byte BluePin,  byte Value, boolean InvertirSalida){  
    byte R,G,B;
    
    switch (Value) {

    case 199:    //Random
      R=RGBredVal;G=RGBgreenVal;B=RGBblueVal;
      //analogWrite( RedPin, 255 - RGBredVal );
      //analogWrite( GreenPin, 255 - RGBgreenVal );
      //analogWrite( BluePin, 255 - RGBblueVal );
      break;
    case 1:    //// 8388736 Purple
      R=128;G=0;B=28;
      //analogWrite( RedPin, 128);
      //analogWrite( GreenPin, 0);
      //analogWrite( BluePin, 28);
      break;
    case 2://15631086 Viole
      R=238;G=130;B=238;
      //analogWrite( RedPin, 238);
      //analogWrite( GreenPin, 130);
      //analogWrite( BluePin, 238);
      break;
    case 3: //9055202 BlueViolet
      R=128;G=43;B=226;
      //analogWrite( RedPin, 138);
      //analogWrite( GreenPin, 43);
      //analogWrite( BluePin, 226);
      break;
    case 4://6970061 SlateBlue
      R=128;G=43;B=226;
      //analogWrite( RedPin, 106);
      //analogWrite( GreenPin, 90);
      //analogWrite( BluePin, 205);
      break;
    case 5:// 8087790 MediumSlateBlue
      R=123;G=104;B=238;
      //analogWrite( RedPin, 123);
      //analogWrite( GreenPin, 104);
      //analogWrite( BluePin, 238);
      break;
    case 6://255 blue
      R=0;G=0;B=255;
      //analogWrite( RedPin, 0);
      //analogWrite( GreenPin, 0);
      //analogWrite( BluePin, 255);
      break;
    case 7://65535 Aqua
      R=0;G=255;B=255;
      //analogWrite( RedPin, 0);
      //analogWrite( GreenPin, 255);
      //analogWrite( BluePin, 255);
      break;
    case 8://11591910 PowderBlue
      R=176;G=224;B=230;
      //analogWrite( RedPin, 176);
      //analogWrite( GreenPin, 224);
      //analogWrite( BluePin, 230);
      break;

    case 9://49151 DeepSkyBlue
      R=0;G=191;B=255;
      //analogWrite( RedPin, 0);
      //analogWrite( GreenPin, 191);
      //analogWrite( BluePin, 255);     
      break;
    case 10://2142890 LightSeaGreen
      R=32;G=178;B=170;
      //analogWrite( RedPin, 32);
      //analogWrite( GreenPin, 178);
      //analogWrite( BluePin, 170);
      break;
    
    case 11: //6737322 MediumAquamarine
      R=102;G=205;B=170;
      //analogWrite( RedPin, 102);
      //analogWrite( GreenPin, 205);
      //analogWrite( BluePin, 170);
      break;  
    case 12:  //10025880 PaleGreen
      R=152;G=251;B=152;
      //analogWrite( RedPin, 152);
      //analogWrite( GreenPin, 251);
      //analogWrite( BluePin, 152);
      break; 
    case 13: //64154 MediumSpringGreen
      R=0;G=250;B=154;
      //analogWrite( RedPin, 0);
      //analogWrite( GreenPin, 250);
      //analogWrite( BluePin, 154);
      break;  
    case 14:  //32768 Green
      R=0;G=128;B=0;
      //analogWrite( RedPin, 0);
      //analogWrite( GreenPin, 128);
      //analogWrite( BluePin, 0);
      break;  
    case 15: //65280  green1 
      R=0;G=255;B=0;
      //analogWrite( RedPin, 0);
      //analogWrite( GreenPin, 255);
      //analogWrite( BluePin, 0);
      break;   
      
      
      
    
    case 16: //14423100 Crimson
      R=220;G=20;B=60;
      //analogWrite( RedPin, 220);
      //analogWrite( GreenPin, 20);
      //analogWrite( BluePin, 60);
      break; 
    case 17: //13047173 MediumVioletRed
      R=199;G=21;B=133;
      //analogWrite( RedPin, 199);
      //analogWrite( GreenPin, 21);
      //analogWrite( BluePin, 133);
      break;  
    case 18: //16729344 OrangeRed
      R=255;G=69;B=0;
      //analogWrite( RedPin, 255);
      //analogWrite( GreenPin, 69);
      //analogWrite( BluePin, 0);
      break;   
    case 19: //16776960 Yelow
      R=255;G=255;B=0;
      //analogWrite( RedPin, 255);
      //analogWrite( GreenPin, 255);
      //analogWrite( BluePin, 0);
      break;  
    case 20: //10824234 Brown
      R=165;G=42;B=42;
      //analogWrite( RedPin, 165);
      //analogWrite( GreenPin, 42);
      //analogWrite( BluePin, 42);
      break; 
  case 21: //16711680 Red
      R=255;G=0;B=0;
      //analogWrite( RedPin, 255);
      //analogWrite( GreenPin, 0);
      //analogWrite( BluePin, 0);
      break; 
    case 22: //16716947 DeepPink
      R=255;G=20;B=147;
      //analogWrite( RedPin, 255);
      //analogWrite( GreenPin, 20);
      //analogWrite( BluePin, 147);
      break;   
    
    case 23: //16747520 DarkOrange
      R=255;G=140;B=0;
      //analogWrite( RedPin, 255);
      //analogWrite( GreenPin, 140);
      //analogWrite( BluePin, 0);
      break; 
    case 24: //12092939 DarkGoldenrod
      R=184;G=134;B=11;
      //analogWrite( RedPin, 184);
      //analogWrite( GreenPin, 134);
      //analogWrite( BluePin, 11);
      break;
    case 25: //16032864 SandyBrown
      R=244;G=164;B=96;
      //analogWrite( RedPin, 244);
      //analogWrite( GreenPin, 164);
      //analogWrite( BluePin, 96);
      break; 
      
      
      
    
    case 26: //16737094 tomato
      R=255;G=99;B=70;
      //analogWrite( RedPin, 255);
      //analogWrite( GreenPin, 99);
      //analogWrite( BluePin, 70);
      break; 
     
    
    case 27: //16761035 Pink
      R=255;G=192;B=203;
      //analogWrite( RedPin, 255);
      //analogWrite( GreenPin, 192);
      //analogWrite( BluePin, 203);
      break; 
    
    
    case 28: //16752762 LightSalmon
      R=255;G=160;B=122;
      //analogWrite( RedPin, 255);
      //analogWrite( GreenPin, 160);
      //analogWrite( BluePin, 122);
      break; 
     
     
    case 29: //15787660 Khaki
      R=240;G=230;B=140;
      //analogWrite( RedPin, 240);
      //analogWrite( GreenPin, 230);
      //analogWrite( BluePin, 140);
      break; 
    
    case 30: //16113331 wheat
      R=245;G=222;B=179;
      //analogWrite( RedPin, 245);
      //analogWrite( GreenPin, 222);
      //analogWrite( BluePin, 179);
      break; 

    default:   // apagado
      R=0;G=0;B=0;
      //analogWrite( RedPin, 0);
      //analogWrite( GreenPin, 0);
      //analogWrite( BluePin, 0);
      break;
    }
    if(InvertirSalida){
      analogWrite( RedPin,255 - R);
      analogWrite( GreenPin,255 -  G);
      analogWrite( BluePin,255 -  B);   
    }
    else{
      analogWrite( RedPin, R);
      analogWrite( GreenPin, G);
      analogWrite( BluePin, B);  
    
    }
  }
#endif
#ifdef THERMOSTAT_DTH22 
  void ReadDHT(){
    float t = dht.readTemperature();
    if ( isnan(t) ) {DhtReadError();}
    else{
       TemperatureDHT =  t;
       float h = dht.readHumidity();
       if ( isnan(h) ) {DhtReadError();return;}
       HumedadDHT=h;
       DhtErrorCount=0;
        #ifdef EXC_DEBUG_MODE   
          Serial.println("Lectura  DHT OK");  
          Serial.print(HumedadDHT, 1);
          Serial.print(",\t");
          Serial.print(TemperatureDHT, 1);
          Serial.print(",\t");  
          Serial.println(""); 
        #endif       
    } 
  }
  void DhtReadError(){
    if (DhtErrorCount<6){DhtErrorCount++;}
    else{ AveriaDHT=true;TemperatureDHT=0;HumedadDHT=0;}     
    #ifdef EXC_DEBUG_MODE   
      Serial.println("Error lectura sensor DHT");   
    #endif   
  }
#endif 
//USER FUNCTIONS. CREATE YOUR FUNCTION HERE
