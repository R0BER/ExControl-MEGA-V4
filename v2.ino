/*
#if 1
__asm volatile ("nop");
#endif
*/
#include   "Excontrol_def.h"

#define EXC_ARDUINO_MEGA
#define EXC_DEBUG_MODE

//Enable watch dogf
//habilita perro guardian
#define EXC_ENABLE_WATCH_DOG
//#define EXC_INTERNAL_RESISTOR
//#define EXC_WIFI_SHIELD
#define EXC_STATIC_IP
//#define EXC_I2C_BOARD 1
//#define ENABLE_SMODBUS


/********NRF24L01******************************************************************************/

//#define EXC_NRF24
//#define EXC_NRF24SPEED 1
//#define EXC_DEBUB_NRF24

/********NRF24L01******************************************************************************/


//#define EXC_SERVER

/***************************SETTING EXTERNAL CONECTION**************************************/

#define ExControlPass ""

//****************************************************
//CONFIGURACION EQUIPOS INTALADOS, TERMOSTATOS, ENCHUFES RADIOFRECUENCIA 433MHZ, INFARROJOS.
//EQUIPMENT CONFIGURATION , THERMOSTAT, RADIO 433MHZ, INFARROJOS.



//#define EXC_LCD
//#define EXC_LED_IR
//#define EXC_IR_RECIVE
//#define EXC_RECEIVER_433
//#define EXC_TRANSMITER_433
//#define EXC_RGB_LED
//#define THERMOSTAT_DS18B20_NUMBER
//#define THERMOSTAT_DTH22
//#define EXC_NumeroPersianas 1
#define EXC_Condicionados 10
#define EXC_Consignas 10
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/



#ifdef EXC_ARDUINO_MEGA
  //#define SD_CARD
#endif
#ifdef ENABLE_SMODBUS
//#define MODBUS_BAUD 19200
#endif

#ifdef EXC_ENABLE_WATCH_DOG
#include <avr/wdt.h>
#endif 
#include <SPI.h>  
#ifdef EXC_WIFI_SHIELD
//#include <WiFi.h>
//#include <WiFiUdp.h>
#else
#include <Ethernet.h>
#include <EthernetUdp.h>
#endif
#include <EEPROM.h>

#include <Wire.h>
#define UDP_TX_PACKET_MAX_SIZE 350 //increase UDP size

#if defined (EXC_TRANSMITER_433)  || defined (EXC_RECEIVER_433)
//#include <RCSwitch.h>
#endif

#if defined (EXC_LED_IR)  || defined (EXC_IR_RECIVE)
//#include <IRremote.h>
#endif



#ifdef ENABLE_SMODBUS
  #include <SimpleModbusSlave.h>
#endif
#ifdef THERMOSTAT_DS18B20_NUMBER 
  #include <OneWire.h>
  #include <DallasTemperature.h>
#endif 
#ifdef EXC_LCD 
 #include <LiquidCrystal_I2C.h>
 #include   "printLCD.h"
#endif 

#ifdef THERMOSTAT_DTH22 
  //#include "DHT.h"

#endif 

#ifdef EXC_NRF24 
  
  #include <RF24Network.h>
  #include <RF24.h>
#endif 
#define EXC_InputOn  LOW

/******************************USER CODE LIBRARY START******************************************************************/

/*************************************************************/
//USER SETTING


/******************************USER CODE LIBRARY END********************************************************************/
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
#define ONE_WIRE_BUS 
  DeviceAddress Ds18B20Addres[THERMOSTAT_DS18B20_NUMBER] ={{0x28,0xAB,0x2A,0x46,0x04,0x00,0x00,0x88}};
  OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
  DallasTemperature sensorTemp(&oneWire);// Pass our oneWire reference to Dallas Temperature.   

  float Temperature[]={20};
 
#endif 
  const byte Histeresis =5;
  
  
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
#ifdef EXC_TRANSMITER_433
  #define EXC_433enableTransmit 
  #define EXC_433setPulseLength 320
  #define EXC_setProtocol 1
  #define EXC_433setRepeatTransmit 15
#endif 
#ifdef ENABLE_SMODBUS
  const byte TxPin=;
  boolean ModbusOuts[32];
  //////////////// registers of your slave ///////////////////
  
  enum 
  {     
    // just add or remove registers and your good to go...
    // The first register starts at address 0
    OUT_REG1,OUT_REG2,      
    IN1_REG1,IN1_REG2,IN1_REG3,IN1_REG4,IN1_REG5,IN1_REG6,SENSOR1_REG1,SENSOR1_REG2,   
    
    //IN2_REG1,IN2_REG2,IN2_REG3,IN2_REG4,IN2_REG5,IN2_REG6,SENSOR2_REG1,SENSOR2_REG2, //slave 2
    //IN3_REG1,IN3_REG2,IN3_REG3,IN3_REG4,IN3_REG5,IN3_REG6,SENSOR3_REG1,SENSOR3_REG2, //slave 3
    //IN4_REG1,IN4_REG2,IN4_REG3,IN4_REG4,IN4_REG5,IN4_REG6,SENSOR4_REG1,SENSOR4_REG2, //slave 4
    
    HOLDING_REGS_SIZE 
  };
  
  unsigned int holdingRegs[HOLDING_REGS_SIZE]; // function 3 and 16 register array
  unsigned int OldholdingRegs[HOLDING_REGS_SIZE];
  ////////////////////////////////////////////////////////////

#endif
#ifdef EXC_LCD
   LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 
#endif
#ifdef EXC_NRF24

  const uint8_t RF24channel = 90;
  const uint16_t dest_address =00; 
  const uint16_t this_node =02; 
  //uint8_t NODE_ADDRESS = 0;  // Use numbers 0 through to select an address from the array
  //RF24 (cepin, cspin) 
  RF24 radio(CE_nRF24L01,SS_nRF24L01);                              // CE & CS pins to use (Using 7,8 on Uno,Nano)
  RF24Network network(radio); 


 boolean NrfOuts[32];
 
const byte NrfRegSize = 8;
const byte DevNumber = 1;
 
 unsigned short NrfInRegs[DevNumber][8]; 
 unsigned int OldNrfInRegs[DevNumber][8]; 
 
 const int NrfRS = sizeof(NrfInRegs[0]);


const byte UserRegNumber=2;
  unsigned short NRFRegs[UserRegNumber];
  unsigned short OldNRFRegs[UserRegNumber];
  const int NrfRegS = sizeof(NRFRegs);
  unsigned int NrfCiclos=0;
  boolean Escucha=false;
  unsigned long TimSend =0;
  byte TimRcv=0;
        
#endif 
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
//ZONA DE CONFIGURACIONES 
//SETTINGS ZONE
//Define numero de entradas salidas 
//Configuracion Red
//Activa o desactiva el perro guardian
/******************************************************************************************************************************/

//SETTINGS ZONE
//Defines number of inputs and outputs
//Network configuration
//Set or Restet Daylight saving time o DST
//Set or Reset watchdog
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
//Activa o desactiva cambio hora automatico invierno verano
//Set or Restet Daylight saving time o DST
//El modo dts esta configurado para europa
//Dts mode is set to europe
//Para otros paises configure funcion CargaHora()
//For other countries configure  function  CargaHora()
const boolean Enable_DaylightSavingTime  = false; 



//Define pines de Entradas y salidas
//Inputs pin and outs pin

  

const byte PinInput[]={23,25,27,29,31,33,35,37,39,41};
byte PinOutput[]={22,24,26,28,30,32,34,36,38,40};


/***********************************************************************************************************************/
/***********************************************************************************************************************/
//VARIABLES PARA CONTROL ESTADO CIRUCITOS
//EL RETORNO DE ESTADO SE RECOGE POR ENTRADAS DIGITALES.
/***********************************************************************************************************************/
/***********************************************************************************************************************/  


//CONFIGURACION DE RED
//Direccion IP ES MUY PROBABLE QUE TENGAS QUE AJUSTARLO A TU RED LOCAL
//Cambia la ip por una dentro de tu rango, tambien debes actualizarla en tu aplicacion android
//Tambien puedes cambiar el puerto, por defecto 5000

// NETWORK CONFIGURATION
// IP Address, ADJUST TO YOUR LOCAL NETWORK
//If you change the IP address will have to adjust in android application
//If you change the Local Port address will have to adjust in android application
//WIRELESS CONFIGURATION
#ifdef EXC_WIFI_SHIELD 
  bool ApOn=true; 
  enum Net_Security {OPEN,WEP,WPA,A_POINT};
  Net_Security Net_Type = WPA;
  char ssid[] = ""; //  your network SSID (name) 
  char pass[] = ""; //  // your network password (use for WPA, or use as key for WEP) 
  int keyIndex = 0;// your network key Index number (needed only for WEP)
  
  byte TimConexion=0;
  int status = WL_IDLE_STATUS;     // the Wifi radio's status
  char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
  WiFiUDP Udp;
  #ifdef ExControlMail 
    WiFiClient client;  
  #endif  
#else
  // buffers para recepcion y envio de datos
  char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; 
   // Instanacia Ethernet UDP Para enviar y recibir paqueteP
  EthernetUDP Udp;
  #ifdef ExControlMail 
    EthernetClient client;
  #endif
#endif

byte fcs[60];
byte mac[] = {0x90, 0xA2, 0xDA, 0x0F, 0x26, 0x20};
IPAddress ip(192,168,1,200);
unsigned int localPort = 5000;

const boolean SecureConnection=false;



#ifdef EXC_I2C_BOARD
unsigned long TimI2CFilter = 0;
struct I2cBoard {
  byte Inputs;
  byte Inputs2;
  byte Inputs3;
  byte Outputs;
  boolean Online;
};
I2cBoard IoBoards[EXC_I2C_BOARD];
byte ov[EXC_I2C_BOARD];
#endif


#ifdef EXC_SERVER
  IPAddress ServerIP(192,168,1,200);
  const int SeverPort= 21099;
struct RemotePacket {boolean Send; byte Data[16]; unsigned long LastSend;};
  const byte RemotePacketNumber= 0;
RemotePacket RemotePackets[RemotePacketNumber];
byte InternalComPacket = 0;
boolean Deslastre = false;
//Meteo
int8_t outside_temperature = 0;
byte hour_orto, minute_horto, hour_ocaso, minute_ocaso,  outside_humidity, wind_speed, wind_direction, rain_now, rain_today;
byte  IntCom = 0;
byte TimUdp = 20;
#else
boolean FalloUdp = false;
#endif


// Circuitos , entradas y salidas
//Circutis, inputs and outputs.

struct Sensor {byte Type;byte Device_Number;short Value;boolean Damaged;};
struct Entrada {byte Type;byte InState;unsigned long LastTimeInput;};
struct Circuit {byte Type;boolean Out1_Value;boolean Out2_Value;byte Device_Number;byte Value;byte CopyRef; byte OldValue;};




byte TypeSensors[] ={};
byte TypeCircuits[] ={Reserva,Reserva,Reserva};
byte TypeInputs[] ={Swicth,Swicth,Swicth,Swicth,Swicth,Swicth,Swicth,Swicth,Swicth,Swicth};
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
//FIN DE ZONAS DE CONFIGURACIONES
//END SETTINGS ZONE
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/




const byte Number_Input=sizeof(TypeInputs);
const byte Number_Output=sizeof(PinOutput);
const byte Number_Circuit=sizeof(TypeCircuits);
const byte Number_Sensor=sizeof(TypeSensors);

Sensor  Sensors[Number_Sensor];
Circuit circuits[Number_Circuit];
Entrada Inputs[Number_Input];




//Varibles Reloj
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year,minutoMemory,TipoDia;
boolean HoraRetrasa;
unsigned long TimNow=0;
unsigned long ExTimer=0;
byte CountMsg=0;
byte CountSg=0;
short CondiVer=0;
#ifdef EXC_RGB_LED
  #ifdef EXC_ARDUINO_MKR
    short RGBredVal = 65535;
    short RGBblueVal = 0;
    short RGBgreenVal =0;
    short RGBSpeed=1;
  #else
    byte RGBredVal = 255;
    byte RGBblueVal = 0;
    byte RGBgreenVal =0;
    byte RGBSpeed=1;
  #endif 
  
  byte RGBrandomCount=0;
#endif
//Variables Control Alarmas
byte  IntCom=0;
byte Alarms[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

#ifdef EXC_NumeroPersianas
  
  byte PosicionPersiana[EXC_NumeroPersianas];  //Controla la posicion de la persiana % Subida
  byte LocalizadorPersiana[EXC_NumeroPersianas];  //Controla posicion persiana en array de circuitos
  
  
  unsigned long TiempoMovPersiana[EXC_NumeroPersianas];  //Tiempo de mov desde el ultimo refresco
  
  //Memoria de tiempos y posicion respecto a tiempo
  unsigned long TiempoPosPersianaUp[EXC_NumeroPersianas];  //Posicion persiana en subida
  unsigned long TiempoPosPersianaDown[EXC_NumeroPersianas];  //Posicion persiana en Bajada
  unsigned long TimUpPersiana[EXC_NumeroPersianas];  //Tiempo en MicrosSg subida persiana
  unsigned long TimDowPersiana[EXC_NumeroPersianas];  //Tiempo en MicrosSg bajada persiana
  
  //Variables para salidas y entradas
  boolean OutUpPersiana[EXC_NumeroPersianas];  //Boleana para activar salida persiana
  boolean OutDowPersiana[EXC_NumeroPersianas];  //Boleana para activar salida persiana
  boolean InUpPersiana[EXC_NumeroPersianas];  //Boleana pulsador subida Persiana
  boolean InDowPersiana[EXC_NumeroPersianas];  //Boleana pulsador bajada Persiana
#endif
boolean Condicionados[10];              //Guarda el estado de los condicionados
word Consignas[10];                     //Guarda el valor de las consignas
boolean Connecting=false;
boolean ForcingSpecialDay1=false;
boolean ForcingSpecialDay2=false;
unsigned long TimAcDet=500;
byte SegUpdtHora=0;

/******************************GLOBAL VARIABLES ZONE******************************************************************/

/*************************************************************/
//USER SETTING


/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
//ZONA DE CONFIGURACIONES 
//SETTINGS ZONE

/******************************************************************************************************************************/
void UserSetup() { //EQUIVALENT ARDUINO SETUP  FUNCTION

//AUTO GENERATED CODE
/*************************************************************/

/*************************************************************/
//USER SETTING

  
//EQUIVALENT ARDUINO SETUP  FUNCTION
//EQUIVALENTE A FUNCION SETUP DEL PROGRAMA ARDUINO

}

void UserLoop(){ //EQUIVALENT ARDUINO LOOP FUNCTION

/*************************************************************/
//USER SETTING

/*************************************************************/
//AUTO GENERATED CODE
/*************************************************************/

/*************************************************************/
//END AUTO GENERATED CODE
/*************************************************************/

  
//EQUIVALENT ARDUINO LOOP  FUNCTION
//EQUIVALENTE A FUNCION LOOP DEL PROGRAMA ARDUINO
}
void LoopNew100MillisSg(){//This event occurs every 100Millis Sg.

/*************************************************************/
//USER SETTING

  
  //Este evento se produce cada 100 milisegundos
  //This event occurs every 100Millis Sg.
}
void LoopNewSecond(){//This event occurs every second

//AUTO GENERATED CODE
/*************************************************************/

/*************************************************************/
//USER SETTING



}
void Loop30Sg(){//This event occurs every 30 second

/*************************************************************/
//USER SETTING

  
  //Este evento se produce cada 30sg.
  //This event occurs every 30SG.
}
void NewMinute(){//This event occurs every minute.

/*************************************************************/
//USER SETTING

   
  //Este evento se produce cada cada minuto.
  //This event occurs every minute.
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
void SwicthStateChange(int NumberInput, int value){

/*************************************************************/
//USER SETTING

//AUTO GENERATED CODE
/*************************************************************/
/*************************************************************/
//END GENERATED CODE
/*************************************************************/
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
  

  #ifdef EXC_DEBUG_MODE   
   Serial.print("Swict State Change, input ");
   Serial.print(NumberInput);
    if (value==LOW){Serial.println(" Off");}else{Serial.println(" On");}
  #endif
}
void ShortInput(int NumberInput){
/*************************************************************/
//Este evento se produce con una pulsación corta..
//This event occurs with a short press.
//AUTO GENERATED CODE
/*************************************************************/

/*************************************************************/
//END GENERATED CODE
/*************************************************************/

/*************************************************************/
//USER SETTING

/*************************************************************/
//Este evento se produce con una pulsación corta..
//This event occurs with a short press.
//*************************************************************/

  #ifdef EXC_DEBUG_MODE   
   Serial.print("Short Input End ");
   Serial.println(NumberInput);
   if (NumberInput<Number_Input){Serial.print(" pin ");	Serial.println(PinInput[NumberInput]);	}  
  #endif
  

}
void LongInputEnd(int NumberInput){
/*************************************************************/
//Este evento se produce con una pulsación corta..
//This event occurs with a short press.
//AUTO GENERATED CODE
/*************************************************************/

/*************************************************************/
//END GENERATED CODE
/*************************************************************/

/*************************************************************/
//USER SETTING

//Este evento se produce cuando la pulsación larga acaba..
//This event occurs with  long press end.

  #ifdef EXC_DEBUG_MODE   
    Serial.print("Long Input End ");
    Serial.println(NumberInput);
    if (NumberInput<Number_Input){Serial.print(" pin ");Serial.println(PinInput[NumberInput]);} 
  #endif

  /*****************************************************************/
  //FINAL DE PULSACION LARGA
  //LONG PRESS END, EVENT
  // This event occurs with end a long press.
  /*****************************************************************/
  
}
void LongInput(int NumberInput){
/*************************************************************/
//Este evento se produce con una pulsación corta..
//This event occurs with a short press.
//AUTO GENERATED CODE
/*************************************************************/

/*************************************************************/
//END GENERATED CODE
/*************************************************************/

/*************************************************************/
//USER SETTING

/*************************************************************/
//Este evento se produce con una pulsación larga comienca..
//This event occurs with a long press start.
/*************************************************************/
  #ifdef EXC_DEBUG_MODE   
    Serial.print("Long Input  ");
    Serial.println(NumberInput);
    if (NumberInput<Number_Input){Serial.print(" pin ");Serial.println(PinInput[NumberInput]);} 
  #endif 
}

//Set or reset relay output
void SetRelay(byte Pin, boolean On){if (On){digitalWrite(Pin,HIGH);}else{digitalWrite(Pin,LOW);}}
  
void OutControl(){

/*************************************************************/
//USER SETTING

/*************************************************************/
//Activamos los reles de control. Activate control relays.
//AUTO GENERATED CODE
/*************************************************************/


/*************************************************************/
//End GENERATED CODE
/*************************************************************/

}

String RunCommand(byte CommandNumber){

/*************************************************************/
//USER SETTING


  return "COMPLETED";
}
void CommonOrders(byte CommandNumber){

/*************************************************************/
//USER SETTING

  //Este evento se produce cuando se ejecuta un comando desde el app
  //This event occurs when a command is executed from the app
  
  #ifdef EXC_DEBUG_MODE   
    Serial.println(CommandNumber);Serial.println(CommandNumber);             
  #endif
}
  //Free text tool
  //Herramienta de texto libre
  //parameter number is the number of scren
  //El parametro number indica el numero de lina en la app
String FreeText(byte Number){

/*************************************************************/
//USER SETTING



  return "RESERVA"; //No borrar, dont delete this line
}
void PersonalFunctions(byte Number){

/*************************************************************/
//USER SETTING

  
}
String GetAlarmsName(byte Number){
/*************************************************************/
//AUTO GENERATED CODE
/*************************************************************/

/*************************************************************/
//END AUTO GENERATED CODE
/*************************************************************/

  
  
    return "RESERVA";
}
void  Scene_Selected(byte Number){

/*************************************************************/
//USER SETTING

  
}
void InternalPacketIn(byte Data1,byte Data2,byte Data3,byte Data4,byte Data5,byte Data6,byte Data7,byte Data8,byte Data9,byte Data10){

/*************************************************************/
//USER SETTING

   #ifdef EXC_DEBUG_MODE   
       Serial.print("User packet in= ");Serial.print(Data1);Serial.print(Data2);Serial.print(Data3);Serial.print(Data4);Serial.print(Data5);Serial.print(Data6);Serial.print(Data7);Serial.print(Data8);Serial.print(Data9);Serial.println(Data10);          
   #endif
}

void Mhz433In(int value){

/*************************************************************/
//USER SETTING


}
#ifdef EXC_LCD
  void PrintMyLcd(){

}
#endif
#ifdef EXC_NRF24
void NRF24_CommOk(boolean Send){//Nrf24l01 complete communication

}
#endif
void AutomaticDST(){
//DST time change (default for Europe)
  if(month==3 && dayOfMonth >= 25 && dayOfWeek == 7 && hour==2){
    hour = 3;
    setDateCLOCK(second, minute, hour, dayOfWeek, dayOfMonth, month, year);
  }
  if(month==10 && dayOfMonth >= 25 && dayOfWeek == 7 && hour==3){
    if (HoraRetrasa==false){
      HoraRetrasa=true;
      hour = 2; 
      setDateCLOCK(second, minute, hour, dayOfWeek, dayOfMonth, month, year);
    }
  }
/*************************************************************/

}


void NewMeteo(){//Meteo data in

/*************************************************************/
//USER SETTING


}

/*************************************************************/
//USER FUNCTIONS
