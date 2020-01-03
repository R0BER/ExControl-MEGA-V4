void setup(){
   
  int c=0;
  #ifdef EXC_DEBUG_MODE   
    Serial.begin(9600);      
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.println("Sytem Start");   
  #endif

  //Iniciamos perro guardian
   #ifdef EXC_ENABLE_WATCH_DOG
     wdt_enable(WDTO_8S);
   #endif
  Wire.begin(); 
  
  #ifdef EXC_I2C_BOARD
    for (c=0;c<EXC_I2C_BOARD;c++){
      InitMCP23017(c);
      if (IoBoards[c].Online){IoBoards[c].Inputs=ReadMCP23017(c);IoBoards[c].Inputs2=IoBoards[c].Inputs;IoBoards[c].Inputs3=IoBoards[c].Inputs;}
      else{IoBoards[c].Inputs=0;IoBoards[c].Inputs2=0;IoBoards[c].Inputs3=0;}
      IoBoards[c].Outputs=0;
    }
  #endif
  
  #ifdef EXC_SERVER 
    for (byte pas = 0; pas <RemotePacketNumber; pas++){
     RemotePackets[pas].Data[0]=1;
     RemotePackets[pas].Data[1]=2;
     RemotePackets[pas].Data[2]=3;
     RemotePackets[pas].Data[3]=4;
     RemotePackets[pas].Data[4]=5;
     RemotePackets[pas].Data[5]=6;
     RemotePackets[pas].Data[6]=7;
     RemotePackets[pas].Data[7]=8;
     RemotePackets[pas].Data[8]=9;
     RemotePackets[pas].Data[9]=10;
     RemotePackets[pas].Data[10]=255;
     RemotePackets[pas].LastSend=0;
     RemotePackets[pas].Send=false; }
  #endif
  
 
  #ifdef EXC_DEBUG_MODE   
      Serial.println("SPI Completed");
  #endif 
  byte ControlPersiana=0;
  

  
  for (c=0; c<Number_Output;c++){pinMode(PinOutput[c], OUTPUT);SetRelay(PinOutput[c],false);}
   
   
  //enum CircuitsTypes {Persiana,ConsignaTemp,};
  for (c=0;c<Number_Sensor;c++){Sensors[c].Device_Number=240;Sensors[c].Damaged=false;Sensors[c].Value=0; Sensors[c].Type=TypeSensors[c];}
  
  for (c=0;c<Number_Circuit;c++){
    
    circuits[c].Device_Number=0;
    circuits[c].Out1_Value=false;
    circuits[c].Out2_Value=false;
    circuits[c].Value=0;
    circuits[c].Type=TypeCircuits[c];

    #ifdef EXC_NumeroPersianas
     if ((circuits[c].Type==Persiana)||(circuits[c].Type==Toldo)||(circuits[c].Type==Persiana2)){LocalizadorPersiana[ControlPersiana]=c;circuits[c].Device_Number=ControlPersiana;ControlPersiana++;}
    #endif 
  }

  for (c=0;c<Number_Input;c++){
    Inputs[c].InState = 0;
    Inputs[c].LastTimeInput = 0;
    Inputs[c].Type=TypeInputs[c];
    pinMode(PinInput[c], INPUT);

  }
  
   

  
  #ifdef EXC_NumeroPersianas
    //Fijamo valores y posicion inicio persianas
    //Fijamos el tiempo de subida bajada Persianas
    for (int per=0; per< EXC_NumeroPersianas; per++){InDowPersiana[per]=false;InUpPersiana[per]=false;}
    ReiniciarTiempoPersianas();
  #endif 

   #ifdef EXC_DEBUG_MODE   
    Serial.println("Load Security copy");               
  #endif
  for (c=0;c<Number_Circuit;c++){
    byte v=EepromRead( EM_STATECOPY_OFSSET + c );
    if (v==255){v=0;}
    if (circuits[c].Type>100){        
        if (v>=100){circuits[c].Out1_Value=true;v=v-100;}
        if(v==1){circuits[c].Value=1; circuits[c].Out2_Value=true;}else{circuits[c].Value=0; circuits[c].Out2_Value=false;}  
    }else{circuits[c].Value=v;} 
    fcs[c]= circuits[c].Value;
  }
  for (c=0;c<EXC_Condicionados;c++){
    byte b=EepromRead( EM_STATECOPY_OFSSET +30 + c );
    if (b==1){Condicionados[c]=true;}else{Condicionados[c]=false;}
    fcs[30+c]= b;//Condicionados
  }
  /*
  fcs[c]= circuits[c].Value;
  fcs[30+c]= b;//Condicionados
  fcs[40+c]= bl;fcs[50+c]=bh;
  p = (indexstr*2) + EM_SETPOINTS_OFSSET;      
        EepromWrite(p, data1);EepromWrite(p+1, data2);
        Consignas[indexstr]=word(data1, data2) ; 
  */
  
  
  for (c=0; c < EXC_Consignas; c++) {
    int Pos=EM_SETPOINTS_OFSSET + (c*2);
    byte bh=EepromRead(Pos);
    byte bl=EepromRead(Pos+1);
    fcs[40+c]= bl;fcs[50+c]=bh;
    Consignas[c]= word(bh, bl);    
  }
  for (c=0; c < N_ALARMS; c++){Alarms[c]=EepromRead(EM_ALARMS_OFSSET+c);if (Alarms[c]>=AlarmSent){Alarms[c]=WithoutAlarm;}}
  #ifdef EXC_DEBUG_MODE   
    Serial.println("Load Ok");               
 #endif
 #ifdef EXC_WIFI_SHIELD
     if (WiFi.status() == WL_NO_SHIELD) {
       #ifdef EXC_DEBUG_MODE   
          Serial.println("WiFi shield not present"); 
        #endif
      while(true);// don't continue:
    }else{ConexionWifi();}
      
  #else
     Ethernet.init(10);   // mega ETH shield  cs pin
  
     #ifdef EXC_STATIC_IP  
      Ethernet.begin(mac,ip);
     #else    
      Ethernet.begin(mac);
     #endif
     #ifdef EXC_DEBUG_MODE   
        if (Ethernet.hardwareStatus() == EthernetNoHardware) {Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");} else{Serial.println("Ethernet shield running");}
        if (Ethernet.linkStatus() == LinkOFF) {Serial.println("Ethernet cable is not connected.  o tarjeta no compatible");}
     #endif
    #ifdef EXC_SERVER
      if (Udp.begin(localPort)==1){TimUdp=30;}
      else{
        TimUdp=50;
        #ifdef EXC_DEBUG_MODE   
          Serial.println("Fallo Iniciando UDP");  
        #endif
        }
     #else
      if (Udp.begin(localPort)==1){FalloUdp=false;}
      else{
        FalloUdp=true;
         #ifdef EXC_DEBUG_MODE   
              Serial.println("Fallo Iniciando UDP");  
          #endif
        }
     #endif
  #endif
  #ifdef EXC_NumeroPersianas
    for (int per=0; per< EXC_NumeroPersianas; per++){CargaPosicionPersiana(per); }//las persianas ajustar posicion     
  #endif

  //Iniciamos Termostatos
 #ifdef THERMOSTAT_DS18B20_NUMBER
   InitDS18B20();
 #endif 
  #ifdef EXC_IR_RECIVE
   irrecv.enableIRIn();
 #endif
 #ifdef EXC_RECEIVER_433  
    Init433Mhz();// Receiver on inerrupt 0 => that is pin #2
 #endif
 #ifdef EXC_TRANSMITER_433 
  mySwitch.enableTransmit(EXC_433enableTransmit);
  mySwitch.setPulseLength(EXC_433setPulseLength);//Pulse lenght
  mySwitch.setProtocol(EXC_setProtocol);//Tipo Protocolo
  mySwitch.setRepeatTransmit(EXC_433setRepeatTransmit);//Repeticiones
  //mySwitch.enableTransmit(15); //Default pin
  //mySwitch.setPulseLength(320);//Default pin Pulse lenght
  //mySwitch.setProtocol(1);//Default pin Tipo Protocolo
  //mySwitch.setRepeatTransmit(15);//Default pin Repeticiones
 #endif

   #ifdef THERMOSTAT_DTH22 
     dht.begin();  
   #endif 
  
 

  
  unsigned long currenMillis = millis();
  unsigned int reading;
  
  for (c=0;c<Number_Input;c++){
    if ((Inputs[c].Type==Swicth)||(Inputs[c].Type==Retroaviso)){
      reading = digitalRead(PinInput[c]);
      if (reading==LOW){Inputs[c].InState=1;}else{Inputs[c].InState=0;}
      Inputs[c].LastTimeInput=currenMillis;
      if (Inputs[c].Type==Retroaviso){if (reading==LOW){SwicthStateChange(c,HIGH);}else{SwicthStateChange(c,LOW);}}    
    }   
  }

  ExTimer=millis(); 
  
  //analogWriteResolution(8);
  UserSetup();

}

//Set or reset relay output

int ReadInput(int add){
  #ifdef EXC_I2C_BOARD
    if (add<10){if (Inputs[add].InState > 0){return HIGH;}else{return LOW;}}
    else{
      int b=-1;
      while(add>9){b++;add=add-10;}
      if (bitRead(IoBoards[b].Inputs, add)==1){return HIGH;}else{return LOW;}
    }
    
  #else
    if (add<10){if (Inputs[add].InState > 0){return HIGH;}else{return LOW;}}
    else{return LOW;}
  #endif  
}



void loop(){  
  InputState();  
  
   TimNow=millis();   
   if(TimNow < ExTimer ) {ExTimer=TimNow;}
   else{if((TimNow - ExTimer) >= 100) {ExTimer=TimNow;ExcTimer();}}

   #ifdef EXC_I2C_BOARD
    if(TimI2CFilter > TimNow ) {TimI2CFilter=TimNow;}
    else{if((TimNow - TimI2CFilter) >= 30) {TimI2CFilter=TimNow;ReadI2Cinputs();}}
   #endif
   

   #ifdef EXC_WIFI_SHIELD
    if (TimConexion > 34){if ((WiFi.status() == WL_CONNECTED)||(status == WL_AP_LISTENING)){RecepcionPaqueteUDP();} else{ResetConexion();}}  
   #else
    RecepcionPaqueteUDP();
   #endif
   
   #ifdef EXC_NumeroPersianas
     for (int p =0; p< EXC_NumeroPersianas;p++){GestionMovPersianas(p);} //Control de movimiento persianas
   #endif
   
   
   #ifdef EXC_ENABLE_WATCH_DOG
    wdt_reset();
   #endif 
   GestionCircuitos();
   OutGest();
   #ifdef EXC_IR_RECIVE   
    ComprobarInfrarro();
  #endif 
  #ifdef EXC_RECEIVER_433  
    Recepcion433Mhz();
  #endif
   UserLoop();
  
}
#ifdef EXC_I2C_BOARD
  void ReadI2Cinputs(){
    for (byte c=0;c<EXC_I2C_BOARD;c++){
      byte rst =ReadMCP23017(c);
      if (rst!=IoBoards[c].Inputs){
        if (rst!=IoBoards[c].Inputs2){IoBoards[c].Inputs2=rst;}
        else if(rst!=IoBoards[c].Inputs3){IoBoards[c].Inputs3=rst;}
        else{        
          byte v;
          for (byte bt =0;bt<8;bt++){
            v=bitRead(rst,bt);
            if (v!=bitRead(IoBoards[c].Inputs, bt)){if (v==0){SwicthStateChange((c*10) +10 + bt, HIGH);}else{SwicthStateChange((c*10) +10 + bt, LOW);}}
          }
          IoBoards[c].Inputs=rst;
        }
      }else{IoBoards[c].Inputs2=rst;IoBoards[c].Inputs3=rst;}
     }    
  }
#endif

void ExcTimer(){
  byte c=0;
  LoopNew100MillisSg();
  
  #ifdef EXC_RGB_LED
    RgbRandomColor();
  #endif
  CountMsg++;
  #ifdef EXC_I2C_BOARD    
     if ((CountMsg==3)||(CountMsg==8)){for (c=0;c<EXC_I2C_BOARD;c++){WriteMCP23017(c ,IoBoards[c].Outputs);}}
  #endif  
      
  if (CountMsg>=10){
      CountMsg=0;
      CountSg++;
      LoopNewSecond();
      #ifdef EXC_SERVER
        if (SecutityCopy()){TimUdp=12;PaqueteEstados();}
      #else
         SecutityCopy();   
      #endif   
      if (CountSg>=30){
        CountSg=0;
        Loop30Sg();
        #ifndef EXC_SERVER
          if (FalloUdp==true){ResetConexion();} 
        #endif
      } 
      #ifdef EXC_SERVER
        if (IntCom>0){IntCom--;}
        if ((CountSg==1)||(CountSg==11)||(CountSg==21)){PaqueteEstados(); }
      #endif

  
      #ifdef EXC_WIFI_SHIELD        
         if (TimConexion==19){ConexionWifi();}//Inicio conexion
         else if (TimConexion==31){//Comprobar conexion completa

          if ((WiFi.status() != WL_CONNECTED)&&(status != WL_AP_LISTENING)) {ResetConexion();}
          else if (Udp.begin(localPort)==1){        
                TimConexion=35;
                #ifdef EXC_DEBUG_MODE 
                  Serial.println();
                  Serial.println("Wifi connection OK");
                  Serial.print("SSID: ");
                  Serial.println(WiFi.SSID());
                
                  // print your WiFi shield's IP address:
                  IPAddress ip = WiFi.localIP();
                  Serial.print("IP Address: ");
                  Serial.println(ip);
                
                  // print the received signal strength:
                  long rssi = WiFi.RSSI();
                  Serial.print("signal strength (RSSI):");
                  Serial.print(rssi);
                  Serial.println(" dBm");
                #endif            
              }else{ResetConexion();} //Esperando dos 30 sg para reconexion
 
        }else if (TimConexion<35){TimConexion++;}
      #endif
      
      #ifdef THERMOSTAT_DS18B20_NUMBER
        if (CountSg==10){RefreshTemperature();}
      #endif
      if (SegUpdtHora==0){CargaHora();}
      else{SegUpdtHora--;}
      
      #ifdef THERMOSTAT_DTH22 
         if (CountSg==4){ReadDHT();}   
      #endif        
  }  
}
void OutGest(){
  OutControl();
  #ifdef EXC_I2C_BOARD
    for (byte c=0;c<EXC_I2C_BOARD;c++){if (ov[c]!=IoBoards[c].Outputs){WriteMCP23017(c ,IoBoards[c].Outputs);ov[c]=IoBoards[c].Outputs;}}
  #endif
}
void CreateStandarPacket(){
  
  packetBuffer[0]=64;//@
  packetBuffer[1]=63;//?
  packetBuffer[2]=86;//V
  packetBuffer[3]=65;//A

  int c=0;
  for (c=0;c<Number_Circuit;c++){ packetBuffer[4+c]=circuits[c].Value;}
  for (c=0;c<EXC_Condicionados;c++){
    if (Condicionados[c]){packetBuffer[34+c]=1;}else{packetBuffer[34+c]=0;}    
  }
  for (c=0; c < EXC_Consignas; c++) {
    packetBuffer[44+c]=lowByte(Consignas[c]);
    packetBuffer[54+c]=highByte(Consignas[c]);
  } 
  for (c=0; c < Number_Sensor; c++){
    packetBuffer[64+c]=lowByte(Sensors[c].Value);
    packetBuffer[84+c]=highByte(Sensors[c].Value);
  }
  for (c=0; c < N_ALARMS; c++){
     packetBuffer[104+c]=Alarms[c];      
  }
    
}
#ifdef EXC_SERVER
  void PaqueteEstados(){
    #ifdef EXC_WIFI_SHIELD
      if (!EstadoWifiOk()){
         #ifdef EXC_DEBUG_MODE   
            Serial.println("Wifi no disponible");  
          #endif  
          return; 
      }  
     /*
    #else
        if (Ethernet.linkStatus() != LinkON) {
          #ifdef EXC_DEBUG_MODE   
           Serial.println("Fallo cable de red");  
          #endif  
          return;
        }
        */
    #endif

    if (TimUdp>=1){
        TimUdp--;
        if ((TimUdp<=12)&&(IntCom==0)){
        CreateStandarPacket();         
        Udp.beginPacket(ServerIP, SeverPort);
        Udp.write(packetBuffer, 125);
        Udp.endPacket();
        IntCom=3;
        #ifdef EXC_DEBUG_MODE   
          Serial.println("Enviado Paquete de estados");            
        #endif
      }
    }
    else{
      #ifdef EXC_DEBUG_MODE        
        Serial.println("Reiniciando conexion, server timeout"); 
      #endif
      ResetConexion();         
      #ifdef EXC_DEBUG_MODE        
        Serial.println("Completado"); 
      #endif
     }     
    }
#endif


void StopPersiana(byte CirNumber){
  #ifdef EXC_NumeroPersianas
    GestionMovPersianas(circuits[CirNumber].Device_Number);
    circuits[CirNumber].Value=PosicionPersiana[circuits[CirNumber].Device_Number];
    GestionMovPersianas(circuits[CirNumber].Device_Number);
    GestionCircuitos();
    OutControl();
  #endif   
}
void SetCircuits(byte CirNumber,byte  Val){
  #ifdef EXC_NumeroPersianas
    if ((circuits[CirNumber].Type==Persiana)||(circuits[CirNumber].Type==Persiana2)||(circuits[CirNumber].Type==Toldo))
    { 
      if (OutUpPersiana[circuits[CirNumber].Device_Number]||OutDowPersiana[circuits[CirNumber].Device_Number]){StopPersiana(CirNumber);}  //Persiana is running
    }
  #endif     
  circuits[CirNumber].Value=Val;
}
void GestionCircuitos(){
      
  //  ,,,,,,,Persiana,ConsignaTemp,,
  for (int c=0;c<Number_Circuit;c++){
    if ((circuits[c].Type!=SensorView)&&(circuits[c].Type!=Reserva)&&(circuits[c].Type!=Timer)&&(circuits[c].Type!=Riego_Temporizado)){
      if (circuits[c].Type==Ado_3Etapas){
        switch (circuits[c].Value) {
          case 0:    
            circuits[c].Out1_Value=false;circuits[c].Out2_Value=false;
            break;
          case 1:    
            circuits[c].Out1_Value=true;circuits[c].Out2_Value=false;
            break;
          case 2:    
           circuits[c].Out1_Value=false;circuits[c].Out2_Value=true; 
            break;
           case 3:    
             circuits[c].Out1_Value=true;circuits[c].Out2_Value=true; 
            break;
           default:    
            circuits[c].Value=0;circuits[c].Out1_Value=false;circuits[c].Out2_Value=false;
            break;
         }    
      }
      
      else if ((circuits[c].Type==Persiana)||(circuits[c].Type==Toldo)||(circuits[c].Type==Persiana2)){
          #ifdef EXC_NumeroPersianas
           circuits[c].Out1_Value=false;circuits[c].Out2_Value=false;
           if ((OutDowPersiana[circuits[c].Device_Number]==true)||(OutUpPersiana[circuits[c].Device_Number]==true))
           {
             if ((OutDowPersiana[circuits[c].Device_Number]==true)&&(OutUpPersiana[circuits[c].Device_Number]==false)){circuits[c].Out1_Value=true; circuits[c].Out2_Value=true;}
             if ((OutDowPersiana[circuits[c].Device_Number]==false)&&(OutUpPersiana[circuits[c].Device_Number]==true)){circuits[c].Out1_Value=true; circuits[c].Out2_Value=false;}     
           }
          #endif
        }    
      else if ((circuits[c].Type==Frio)||(circuits[c].Type==Calor)){Termostato(c);}
      else if ((circuits[c].Type==High_trigger)||(circuits[c].Type==Low_trigger)){SensorTrigger(c);}
      else if ((circuits[c].Type==SetPoint_200)||(circuits[c].Type==SetPoint_2000)||(circuits[c].Type==SetPoint_20000)){}      
      else if (circuits[c].Type>100){
        boolean b=false;
        if (circuits[c].Value==1){b=true;}
        if (circuits[c].Out2_Value!=b){circuits[c].Out2_Value=b;circuits[c].Out1_Value=!circuits[c].Out1_Value;}    
      }
      else{
        if (circuits[c].Value>=1){circuits[c].Out1_Value=true;circuits[c].Out2_Value=false;}
        else{circuits[c].Out1_Value=false;circuits[c].Out2_Value=false;}
      }
    }
  }      
}
   // else if (circuits[c].Type==High_trigger){HighTrigger(c);}
     // else if (circuits[c].Type==Low_trigger){LowTrigger(c);}
     
void SensorTrigger(byte  CirNumber){
  if ((CirNumber>=Number_Circuit)||(CirNumber<0)){return;}
   if (circuits[CirNumber].Value==0){ circuits[CirNumber].Out1_Value=false;circuits[CirNumber].Out2_Value=false;}
   else{   
 
    Circuit ConsignaCir = circuits[circuits[CirNumber].Device_Number];
    if (Sensors[ConsignaCir.Device_Number].Damaged){circuits[CirNumber].Out1_Value=false;circuits[CirNumber].Value=249;return;}
    short t;  
    switch (ConsignaCir.Type) {
    case SetPoint_100:   
      t=ConsignaCir.Value;
      break;
    case SetPoint_200:
      t=ConsignaCir.Value;
      break;
    case SetPoint_2000:   
      t=ConsignaCir.Value * 10;
      break;   
     case SetPoint_20000:   
      t=ConsignaCir.Value * 100;
      break; 
    default:
      return;
   }
   if ((Sensors[ConsignaCir.Device_Number].Type==Sensor_Temperature)||(Sensors[ConsignaCir.Device_Number].Type==Sensor_Humidity)||(Sensors[ConsignaCir.Device_Number].Type==Sensor_Float)){t=t*10;}
   if ((ConsignaCir.Device_Number<0)||(ConsignaCir.Device_Number >= Number_Sensor)){return;}
   else{
     switch (circuits[CirNumber].Type) {
      case High_trigger:   
        if (Sensors[ConsignaCir.Device_Number].Value > t){circuits[CirNumber].Out1_Value=true;circuits[CirNumber].Value=249;}
        else{circuits[CirNumber].Out1_Value=false;circuits[CirNumber].Value=1;}
        break;
      case Low_trigger:
       if (Sensors[ConsignaCir.Device_Number].Value > t){circuits[CirNumber].Out1_Value=false;circuits[CirNumber].Value=1;}
       else{circuits[CirNumber].Out1_Value=true;circuits[CirNumber].Value=249;}
       break;
      default:
        return;
      }
   }          
  }
}

void Termostato(byte  CirNumber){
  if ((CirNumber>=Number_Circuit)||(CirNumber<0)){return;}
  if (circuits[CirNumber].Value==0){ circuits[CirNumber].Out1_Value=false;circuits[CirNumber].Out2_Value=false;return;}
  if ((circuits[CirNumber].Device_Number > Number_Circuit)||(circuits[CirNumber].Device_Number < 0 )){return;}
  
  Circuit ConsignaCir = circuits[circuits[CirNumber].Device_Number];
  if ((ConsignaCir.Device_Number > Number_Sensor)||(ConsignaCir.Device_Number < 0 )){return;}  
  else{
    
    if (Sensors[ConsignaCir.Device_Number].Damaged){circuits[CirNumber].Out1_Value=false;circuits[CirNumber].Value=249;return;}
    short t;  
    switch (ConsignaCir.Type) {
    case ConsignaTemp:   
      t=ConsignaCir.Value *10;
      break;
    case HomeTemperature:
      t=ConsignaCir.Value + 150;
      break;
    case TempNegative:   
      t=ConsignaCir.Value *  (-10);
      break;   
    default:
      return;
   }
   
   switch (circuits[CirNumber].Type) {
    case Calor:   
      if (Sensors[ConsignaCir.Device_Number].Value > (t + Histeresis)){circuits[CirNumber].Out1_Value=false;}
      else{if (Sensors[ConsignaCir.Device_Number].Value < (t - Histeresis)){circuits[CirNumber].Out1_Value=true;}}
      break;
    case Frio:
      if (Sensors[ConsignaCir.Device_Number].Value < (t - Histeresis)){circuits[CirNumber].Out1_Value=false;}
      else{if (Sensors[ConsignaCir.Device_Number].Value > (t + Histeresis)){circuits[CirNumber].Out1_Value=true;}}
      break;
    default:
      return;
   }   
  }
}

void InputState(){
  unsigned long InputMillis;
  byte v=0;

  InputMillis = millis();

  for (int i=0;i<Number_Input;i++){    
    switch (Inputs[i].Type) {
    case Button: {
      if (digitalRead(PinInput[i])==LOW){v=1;}else{v=0;}
      InputMillis = millis();  
      if(Inputs[i].LastTimeInput>InputMillis){Inputs[i].LastTimeInput=InputMillis;}
      else{
        InputMillis = InputMillis -Inputs[i].LastTimeInput;       
        if ((Inputs[i].InState>=4)||(Inputs[i].InState<0)){Inputs[i].InState=0;}     
        if (v==1){ 
          if ((Inputs[i].InState==0)&&(InputMillis>=60)){Inputs[i].LastTimeInput=millis();Inputs[i].InState=1;}
          if ((Inputs[i].InState==1)&&(InputMillis>=440)){LongInput(i);Inputs[i].InState=2;}
          if (Inputs[i].InState==2){Inputs[i].LastTimeInput=millis();}
          if (Inputs[i].InState==3){Inputs[i].LastTimeInput=millis();Inputs[i].InState=1;}
         }
         else{
            if (Inputs[i].InState==0){Inputs[i].LastTimeInput=millis();}
            if (Inputs[i].InState==1){Inputs[i].LastTimeInput=millis();Inputs[i].InState=3;}
            if ((Inputs[i].InState==2) &&(InputMillis>=60)){Inputs[i].LastTimeInput=millis();Inputs[i].InState=0;LongInputEnd(i);}
            if ((Inputs[i].InState==3)&&(InputMillis>=60)){Inputs[i].LastTimeInput=millis();Inputs[i].InState=0;ShortInput(i);}
        }
      }  
      break;}
  
    default:{
      if (digitalRead(PinInput[i])==LOW){v=1;}else{v=0;}
      InputMillis = millis();
      if (v==Inputs[i].InState){Inputs[i].LastTimeInput=InputMillis;}
      else{
        if(Inputs[i].LastTimeInput>InputMillis){Inputs[i].LastTimeInput=InputMillis;}
        if ((InputMillis-Inputs[i].LastTimeInput)>=60){Inputs[i].LastTimeInput=InputMillis; Inputs[i].InState=v;if (v){SwicthStateChange(i,HIGH);}else{SwicthStateChange(i,LOW);}}    
      }
      break;}
    }
  }
} 


#ifdef EXC_NumeroPersianas
  
  /*************************************************************/
     //Gestion Persianas
  /*************************************************************/ 
  
  void GestionMovPersianas(int NPersiana){
    if (InUpPersiana[NPersiana] || InDowPersiana[NPersiana])
    {  //Funcionamiento Manual
      if (InUpPersiana[NPersiana] && InDowPersiana[NPersiana]){InUpPersiana[NPersiana] =false; InDowPersiana[NPersiana]=false;OutDowPersiana[NPersiana]=false;OutUpPersiana[NPersiana]=false;OutGest();delay(100);}
      else{
        if (InUpPersiana[NPersiana]){SubirPersiana(NPersiana);}else {BajarPersiana(NPersiana);}
        circuits[LocalizadorPersiana[NPersiana]].Value=PosicionPersiana[NPersiana];    
      }    
    }
    else
    {  //Funcionamiento Automatico;
      if (circuits[LocalizadorPersiana[NPersiana]].Value==PosicionPersiana[NPersiana]){OutDowPersiana[NPersiana]=false;OutUpPersiana[NPersiana]=false;}
      else
      {      
         if (circuits[LocalizadorPersiana[NPersiana]].Value > PosicionPersiana[NPersiana]){SubirPersiana(NPersiana);}
         else  {if (circuits[LocalizadorPersiana[NPersiana]].Value< PosicionPersiana[NPersiana])  {BajarPersiana(NPersiana);}}    
      }  
    }
  }
  
  void SubirPersiana(int NPersiana){

    unsigned long TiempoActual = micros();
    if (OutUpPersiana[NPersiana]==false){OutUpPersiana[NPersiana]=true;}
    else{
      unsigned long DiferenciaTiempo;
      if (TiempoActual<TiempoMovPersiana[NPersiana]){DiferenciaTiempo=TiempoActual;}else{DiferenciaTiempo = TiempoActual-TiempoMovPersiana[NPersiana];}
      if ((TiempoPosPersianaUp[NPersiana] + DiferenciaTiempo)<TimUpPersiana[NPersiana]){TiempoPosPersianaUp[NPersiana]=TiempoPosPersianaUp[NPersiana] + DiferenciaTiempo;}
      else{TiempoPosPersianaUp[NPersiana]=TimUpPersiana[NPersiana];}
      byte porcentajeSubida = TiempoPosPersianaUp[NPersiana] / (TimUpPersiana[NPersiana]/100);
      byte porcentajeBajada=100-porcentajeSubida;
      PosicionPersiana[NPersiana]=porcentajeSubida;
      TiempoPosPersianaDown[NPersiana]=porcentajeBajada*(TimDowPersiana[NPersiana]/100);
    }  
    TiempoMovPersiana[NPersiana]=TiempoActual;
  }
  
     
  void BajarPersiana(int NPersiana){
  
    unsigned long TiempoActual = micros();
    if (OutDowPersiana[NPersiana]==false){OutDowPersiana[NPersiana]=true;}
    else{
      unsigned long DiferenciaTiempo;
      if (TiempoActual<TiempoMovPersiana[NPersiana]){DiferenciaTiempo=TiempoActual;}else{DiferenciaTiempo = TiempoActual-TiempoMovPersiana[NPersiana];}
      if ((TiempoPosPersianaDown[NPersiana] + DiferenciaTiempo)<TimDowPersiana[NPersiana]){TiempoPosPersianaDown[NPersiana]=TiempoPosPersianaDown[NPersiana] + DiferenciaTiempo;}else{TiempoPosPersianaDown[NPersiana]=TimDowPersiana[NPersiana];}
     
      byte porcentajeBajada = TiempoPosPersianaDown[NPersiana] / (TimDowPersiana[NPersiana]/100);
      byte porcentajeSubida=100-porcentajeBajada;
      PosicionPersiana[NPersiana]=porcentajeSubida;
      TiempoPosPersianaUp[NPersiana]=porcentajeSubida*(TimUpPersiana[NPersiana]/100);
    }  
    TiempoMovPersiana[NPersiana]=TiempoActual;
  }
  
  
  void ReiniciarPosicionPersiana(int NumPersiana){ TiempoPosPersianaUp[NumPersiana]=0;TiempoPosPersianaDown[NumPersiana]=TimDowPersiana[NumPersiana];circuits[LocalizadorPersiana[NumPersiana]].Value=100;}
  void ReiniciarTiempoPersianas()
  {
    for ( byte c =0; c <  EXC_NumeroPersianas; c++){
      /*
      TimUpPersiana[c]=(EepromRead(EM_UP_TIM_SHUTTER_OFFSET + c))*  1000000; 
      TimDowPersiana[c]=(EepromRead(EM_DO_TIM_SHUTTER_OFFSET + c))* 1000000;
       */
      
      byte Bup=EepromRead(EM_UP_TIM_SHUTTER_OFFSET + c); 
      byte Bdw=EepromRead(EM_DO_TIM_SHUTTER_OFFSET + c);
      if ((Bdw==0)||(Bdw>60)){ Bdw=30;}
      if ((Bup==0)||(Bup>60)){ Bup=30;}
      TimUpPersiana[c]= Bup *  1000000; 
      TimDowPersiana[c]=Bdw * 1000000;  
    }
  }
  void CargaPosicionPersiana(int NPersiana){
    if (circuits[LocalizadorPersiana[NPersiana]].Value>100){ReiniciarPosicionPersiana(NPersiana);}
    else{
      PosicionPersiana[NPersiana]=circuits[LocalizadorPersiana[NPersiana]].Value;
      byte porcentajeBajada=100-PosicionPersiana[NPersiana];
      TiempoPosPersianaDown[NPersiana]=porcentajeBajada*(TimDowPersiana[NPersiana]/100);
      TiempoPosPersianaUp[NPersiana]=PosicionPersiana[NPersiana]*(TimUpPersiana[NPersiana]/100);  
    }
  }
#endif
void PaqueteEstandar(IPAddress ip, uint16_t port)
{
  CreateStandarPacket();
  Udp.beginPacket(ip, port);
  Udp.write(packetBuffer, 125);
  if (Udp.endPacket()!=1){ResetConexion();}         
}
void RecepcionPaqueteUDP(){
  int s = Udp.parsePacket();  

  if(s>0){    
     #ifdef EXC_DEBUG_MODE        
      Serial.print("UDP Packet recive, size"); Serial.println(s);  
    #endif
    if (s>340){
      #ifdef EXC_DEBUG_MODE        
        Serial.println("ERROR  UDP SOBRECARGADO"); 
      #endif
      Udp.stop();
      #ifdef EXC_DEBUG_MODE        
        Serial.println("Desconexion Red ok"); 
      #endif

      #ifdef EXC_DEBUG_MODE        
        Serial.println("Conexion Red"); 
      #endif

     if (Udp.begin(localPort)!=1){ ResetConexion();}
     return;  
    }
    Udp.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE);
    if ((packetBuffer[0]!=64)||(packetBuffer[1]!=35)){
      #ifdef EXC_DEBUG_MODE        
        Serial.println("PAQUETE INCORRECTO!!"); 
      #endif
      return;      
      }
    if  (SecureConnection){
      if (s<10){
          #ifdef EXC_DEBUG_MODE        
            Serial.println("FALTA CONTRASEÑA"); 
          #endif
          return;          
      }
      for(int u=1; u<9;u++){        
        if (ExControlPass[8-u]!=packetBuffer[s -u])
        {
          #ifdef EXC_DEBUG_MODE        
            Serial.println("ERROR DE CONTRASEÑA"); 
          #endif
          return;
        }
      }
    }   
    word code=word(packetBuffer[2], packetBuffer[3]) ;  
    #ifdef EXC_DEBUG_MODE   
      Serial.print("Com Code ");   Serial.println(code); 
    #endif
     switch (code) {
      case 1:{//Set Circuit
        if (packetBuffer[4]<Number_Circuit){
          SetCircuits(packetBuffer[4],packetBuffer[5]);
          OutControl();
        }
        PaqueteEstandar(Udp.remoteIP(), Udp.remotePort());  
        break;         
      }
      //EXC_Condicionados
      case 2:{//Set Condicionados
        if (packetBuffer[4]<EXC_Condicionados){
          if ( packetBuffer[5]==0){Condicionados[packetBuffer[4]] = false;}else{Condicionados[packetBuffer[4]] = true;}
        }
        PaqueteEstandar(Udp.remoteIP(), Udp.remotePort()); 
        break;        
      }
      case 3:{//Set Consignas
        if (packetBuffer[4]<EXC_Consignas){
           Consignas[packetBuffer[4]]=word(packetBuffer[5], packetBuffer[6]) ;
           int cdi=EM_SETPOINTS_OFSSET + (packetBuffer[4]*2);
           EepromWrite(cdi,packetBuffer[5]);   
           EepromWrite(cdi +1,packetBuffer[6]); 
        }
        PaqueteEstandar(Udp.remoteIP(), Udp.remotePort());  
        break;                
      }
      case 4:{//Selecciona Escena        
        SelectScene(packetBuffer[4]+1);
        PaqueteEstandar(Udp.remoteIP(), Udp.remotePort());  
        break;                
      }
      case 5:{//funcion de usuario        
        PersonalFunctions(packetBuffer[4]);
        PaqueteEstandar(Udp.remoteIP(), Udp.remotePort());  
        break;                
      }
       case 6:{//Comandos      
        String kk =RunCommand(Udp.remoteIP(),packetBuffer[4]);
        PaqueteEstandar(Udp.remoteIP(), Udp.remotePort());  
        break;                
      }
      case 7:{//Set Sensors
        if (packetBuffer[4]<Number_Sensor){
           Sensors[packetBuffer[4]].Value= packetBuffer[5] << 8 | packetBuffer[6];           
        }
        PaqueteEstandar(Udp.remoteIP(), Udp.remotePort());  
        break;                
      }
      case 8:{//Estados        
        PaqueteEstandar(Udp.remoteIP(), Udp.remotePort());  
        break;                
      }
      case 9:{//leer escena 
         packetBuffer[0]=86;//'V';
         packetBuffer[1]=69;//'E';
         packetBuffer[2]=83;//'S';
         packetBuffer[3]=67;//'C';
         int p = (packetBuffer[4]  * S_ESCENES ) + EM_ESCENES_OFFSET;  
         for (byte  c = 0; c < S_ESCENES; c++){ packetBuffer[5+c]=EepromRead(p + c); }
         Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
         Udp.write(packetBuffer, 35);
         if (Udp.endPacket()!=1){ResetConexion();}
         break;
      }
       case 10:{//write escena
         int  p = EM_ESCENES_OFFSET + (packetBuffer[4] * S_ESCENES ) ;  
         for (int c = 0; c < S_ESCENES; c++){EepromWrite(p+c, packetBuffer[5+c]);}
         packetBuffer[0]=79;packetBuffer[1]=107;  packetBuffer[2]=83; 
         Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
         Udp.write(packetBuffer, 3);
         if (Udp.endPacket()!=1){ResetConexion();}
         break;
      }
      case 11:{
        if (packetBuffer[4]<Number_Circuit){
          if ((circuits[packetBuffer[4]].Type==Persiana)||(circuits[packetBuffer[4]].Type==Persiana2)||(circuits[packetBuffer[4]].Type==Toldo)){StopPersiana(packetBuffer[4]);}
        }
        PaqueteEstandar(Udp.remoteIP(), Udp.remotePort());  
        break; 
      }
        
      case 12:{
       if (packetBuffer[4]<N_ALARMS){
        Alarms[packetBuffer[4]]=packetBuffer[5];
        EepromWrite(EM_ALARMS_OFSSET + packetBuffer[4],Alarms[packetBuffer[4]]);
       }
       PaqueteEstandar(Udp.remoteIP(), Udp.remotePort());  
       break; 
      }
      case 13:{
        setDateCLOCK(packetBuffer[4] ,packetBuffer[5], packetBuffer[6], packetBuffer[7], packetBuffer[8], packetBuffer[9], packetBuffer[10]);
        CargaHora();
         packetBuffer[0]=100;packetBuffer[1]=101;  packetBuffer[2]=102; 
         Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
         Udp.write(packetBuffer, 3);
         if (Udp.endPacket()!=1){ResetConexion();}
         break;       
      }
      case 14:{
        packetBuffer[0]=100;
        packetBuffer[1]=84;
        packetBuffer[2]=TipoDia ;
        packetBuffer[3]=hour ;  
        packetBuffer[4]=minute;
        packetBuffer[5]=dayOfMonth;
        packetBuffer[6]=month;
        packetBuffer[7]=year;  
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(packetBuffer, 8);
        if (Udp.endPacket()!=1){ResetConexion();}
        break;          
      }
      case 15:{
        for (int c = 0; c<N_EN_TIMETABLE; c++){EepromWrite(EM_EN_TIMETABLE_OFFSET + c, packetBuffer[4+c]); }
        packetBuffer[0]=200;packetBuffer[1]=201;  packetBuffer[2]=202; 
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(packetBuffer, 3);
        if (Udp.endPacket()!=1){ResetConexion();}
        break;
      }
      case 16:{
        packetBuffer[0]=1;packetBuffer[1]=2;  packetBuffer[2]=3;packetBuffer[3]=2;  packetBuffer[4]=3;
        for (int c=0; c<N_EN_TIMETABLE; c++){packetBuffer[5+c]=EepromRead(EM_EN_TIMETABLE_OFFSET+c)+1;}
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(packetBuffer, 60);
        if (Udp.endPacket()!=1){ResetConexion();}
        break;
      }
      case 17:{
        packetBuffer[0]=3;packetBuffer[1]=3;  packetBuffer[2]=3;packetBuffer[3]=3;  packetBuffer[4]=3;       
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(packetBuffer, 5);
        if (Udp.endPacket()!=1){ResetConexion();}
                
     
        #ifdef EXC_ENABLE_WATCH_DOG
          byte paso=0; 
        #endif

        for (int p = EM_TRIGGER_OFFSET; p <= EM_TIME_ESPECIAL2_END; p++){          // Size slot 80 * 4bytes data *7 day of the week + 
          EepromWrite(p, 66);
          #ifdef EXC_ENABLE_WATCH_DOG
            paso++;
            if (paso>200){wdt_reset();paso=0;}          
          #endif
        }
        break;
      }
       case 18:{
        packetBuffer[0]=4;packetBuffer[1]=4;  packetBuffer[2]=4;packetBuffer[3]=4;  packetBuffer[4]=4;       
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(packetBuffer, 5);
        if (Udp.endPacket()!=1){ResetConexion();}  
        #ifdef EXC_ENABLE_WATCH_DOG
          wdt_reset();
        #endif
        for (int c = EM_DATE_ESPECIAL1_OFSSET; c <= EM_DATE_ESPECIAL2_END; c++){ EepromWrite(c, 0);} // size slot (100)  50 * 4bytes data * 2 special days
        break;
      }
      case 19:{
        int p=EM_DATE_ESPECIAL1_OFSSET;
        if (packetBuffer[4]==50){p=EM_DATE_ESPECIAL2_OFSSET;}// Pointer, first address memory slot. Eeprom.
        packetBuffer[0]=4;packetBuffer[1]=4;  packetBuffer[2]=4;packetBuffer[3]=4;  
        for (int c = 0; c<EM_DATE_ESPECIAL_SIZE; c++){packetBuffer[4+c]=EepromRead(p+c); }    
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(packetBuffer, 55);
        if (Udp.endPacket()!=1){ResetConexion();}
        break;
     }
     case 20:{
        int p=EM_DATE_ESPECIAL1_OFSSET;
        if (packetBuffer[4]==2){p=EM_DATE_ESPECIAL2_OFSSET;}
        for (int c=0; c<EM_DATE_ESPECIAL_SIZE; c++){EepromWrite(p+c, packetBuffer[5+c]); }  
        packetBuffer[0]=6;packetBuffer[1]=8;  packetBuffer[2]=6;packetBuffer[3]=8;     
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(packetBuffer, 4);
        if (Udp.endPacket()!=1){ResetConexion();}
        break;
    }
    case 21:{
        int h=4; 
        for (int c=EM_TRIGGER_OFFSET; c <= EM_TRIGGER_END; c++){EepromWrite(c, packetBuffer[h]);h++;}
        packetBuffer[0]=6;packetBuffer[1]=8;  packetBuffer[2]=6;packetBuffer[3]=8;     
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(packetBuffer, 4);
        if (Udp.endPacket()!=1){ResetConexion();}
        break;
    }
      case 22:{
        packetBuffer[0]=9;packetBuffer[1]=9;  packetBuffer[2]=9;packetBuffer[3]=9; 
        int h=4; 
        for (int c=EM_TRIGGER_OFFSET; c <= EM_TRIGGER_END; c++){packetBuffer[h] = EepromRead(c);h++;}
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(packetBuffer, 52);
        if (Udp.endPacket()!=1){ResetConexion();}
        break;
    
    }    
    case 23:{
      int p=(packetBuffer[4] * EM_TIME_DAY_SIZE) + EM_TIME_WEEKLY_OFFSET;
      packetBuffer[0]=10;packetBuffer[1]=11;  packetBuffer[2]=12;packetBuffer[3]=12;
      for (int c = 0; c<EM_TIME_DAY_SIZE; c++){packetBuffer[4+c]=EepromRead(p+c); }
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(packetBuffer, 325);
      if (Udp.endPacket()!=1){ResetConexion();}
      break;
    }
    case 24:{
      int p=(packetBuffer[4] * EM_TIME_DAY_SIZE) + EM_TIME_WEEKLY_OFFSET;
      for (int c=0; c<EM_TIME_DAY_SIZE ;c++){ EepromWrite(p+c, packetBuffer[5+c]);}
      packetBuffer[0]=8;packetBuffer[1]=8;  packetBuffer[2]=8;packetBuffer[3]=8;     
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(packetBuffer, 4);
      if (Udp.endPacket()!=1){ResetConexion();}
      break;  
    }
    case 25:{//Read motors      
      packetBuffer[0]=9;packetBuffer[1]=2;  packetBuffer[2]=1;packetBuffer[3]=3; 
      for (byte c=0; c < (N_UP_TIM_SHUTTER *2) ; c++){packetBuffer[4+c]=EepromRead(EM_UP_TIM_SHUTTER_OFFSET+c); }   
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(packetBuffer, 65);
      if (Udp.endPacket()!=1){ResetConexion();}
      break; 
    }
    case 26:{//write motors
      packetBuffer[0]=19;packetBuffer[1]=12;  packetBuffer[2]=11;packetBuffer[3]=13;       
      for (byte  c = 0; c< (N_UP_TIM_SHUTTER *2) ;c++){EepromWrite(EM_UP_TIM_SHUTTER_OFFSET+c, packetBuffer[4+c]); }      
      #ifdef EXC_NumeroPersianas
        ReiniciarTiempoPersianas();
      #endif
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(packetBuffer, 4);
      if (Udp.endPacket()!=1){ResetConexion();}
      break; 
    }
      

      case 20331:{
            #ifdef EXC_SERVER  
              if (packetBuffer[4]==107){
              IntCom=0;
              TimUdp=35;
              #ifdef EXC_DEBUG_MODE   
                Serial.println("Test Server Com Ok"); 
              #endif
              }
          #endif      
          break;        
        }
        case 20321:{
          #ifdef EXC_SERVER  
            byte num=packetBuffer[4]-1;
            if (num<N_ALARMS){Alarms[num]=AlarmSent;}       
            IntCom=0;
            TimUdp=35;
            #ifdef EXC_DEBUG_MODE   
             Serial.print("Alarma Enviada - ");Serial.println((int)num); 
            #endif
          #endif
          break;      
        }
        case 1031:{
          #ifdef EXC_DEBUG_MODE   
           Serial.print("Envio Packete completo ");Serial.println((int)packetBuffer[4]);
          #endif
          
          #ifdef EXC_SERVER 
            TimUdp=35; 
            IntCom=0;
            RemotePackets[packetBuffer[4]].Send=false;
          #endif
          break;            
          
        }
         case 65280:{//meteo
           #ifdef EXC_DEBUG_MODE   
             Serial.println("NEW METEO"); 
            #endif
            #ifdef EXC_SERVER  
              hour_ocaso=packetBuffer[5];
              minute_ocaso=packetBuffer[6];
              hour_orto=packetBuffer[7];
              minute_horto=packetBuffer[8];
              outside_humidity=packetBuffer[9];
              wind_direction=packetBuffer[10];
              wind_speed=packetBuffer[11];
              rain_now=packetBuffer[12];
              rain_today=packetBuffer[13];        
              outside_temperature=packetBuffer[14];
              
              packetBuffer[0]=77;
              packetBuffer[1]=84;          
              packetBuffer[2]=79;
              packetBuffer[3]=75;//l   
              packetBuffer[4]=0;            
  
              Udp.beginPacket(ServerIP, Udp.remotePort());
              Udp.write(packetBuffer, 5);
              if (Udp.endPacket()!=1){ResetConexion();}
              NewMeteo();
            #endif 
          
          break;            
          
        }
        case 258:{//Recepcion paquete de usuario
           #ifdef EXC_SERVER  
            packetBuffer[0]=85;
            packetBuffer[1]=83;          
            packetBuffer[2]=69;
            packetBuffer[3]=80;//l   
            packetBuffer[4]=79;
            

            Udp.beginPacket(ServerIP, Udp.remotePort());
            Udp.write(packetBuffer, 5);
            if (Udp.endPacket()!=1){ResetConexion();}
            InternalPacketIn(packetBuffer[5],packetBuffer[6],packetBuffer[7],packetBuffer[8],packetBuffer[9],packetBuffer[10],packetBuffer[11],packetBuffer[12],packetBuffer[13],packetBuffer[14]);
          #endif 
          break;    
          
        } 
      
      
      
     }
  }
  else{ //Sin comunicacion, gestionamos otras comunicaciones
    #ifdef EXC_SERVER
      /*
      #ifndef EXC_WIFI_SHIELD
        if (Ethernet.linkStatus() != LinkON) {return;}
      #endif  
      */
      if ((IntCom > 0)||(TimUdp<1)){return;}
   
      for (int a=0;a<N_ALARMS;a++){
        if (Alarms[a]==SendingAlarm){
          packetBuffer[0]=64;//@
          packetBuffer[1]=63;//?          
          packetBuffer[2]=65;//A
          packetBuffer[3]=108;//l   
          packetBuffer[4]=a+1;
          Udp.beginPacket(ServerIP, SeverPort);
          Udp.write(packetBuffer, 5);
          if (Udp.endPacket()!=1){ResetConexion();}
          IntCom=3;
          #ifdef EXC_DEBUG_MODE   
            Serial.print("Send Alarm ");Serial.println((int)a);             
          #endif
          return;
        }  
      } 

    unsigned int t=millis();
    if (InternalComPacket>=RemotePacketNumber){InternalComPacket=0;}
    while (InternalComPacket<RemotePacketNumber){
      if (RemotePackets[InternalComPacket].Send){
          if (RemotePackets[InternalComPacket].LastSend>t){RemotePackets[InternalComPacket].LastSend=t;}
          if ((RemotePackets[InternalComPacket].LastSend +3000)<= t){
            #ifdef EXC_DEBUG_MODE   
              Serial.print("Send User Packet Number ");Serial.println(InternalComPacket);             
            #endif
            RemotePackets[InternalComPacket].LastSend=t;

            packetBuffer[0]=64;//@
            packetBuffer[1]=63;//?          
            packetBuffer[2]=112;//p
            packetBuffer[3]=75;//K   
            packetBuffer[4]=InternalComPacket;
            for (byte dt=0;dt<11;dt++){packetBuffer[5+dt]=RemotePackets[InternalComPacket].Data[dt];}
            Udp.beginPacket(ServerIP, SeverPort);
            Udp.write(packetBuffer, 16);
            if (Udp.endPacket()!=1){ResetConexion();}
            IntCom=3;
            InternalComPacket++; 
            break;
          }
                 
        }else{InternalComPacket++;}
    }
    #endif 
   }
}

void SelectScene(byte Dir)
{
  int p;
  byte c,v;
  if (Dir>=N_ESCENES){return;}
  
  p= (int) Dir;
  p= EM_ESCENES_OFFSET + ((p-1) * S_ESCENES );
  for (c =0 ; c<Number_Circuit; c++){v =EepromRead(p + c); if (v < 250){SetCircuits(c,v);}}
  Scene_Selected(Dir);
  #ifdef EXC_DEBUG_MODE   
      Serial.print("Scene Number ");Serial.print(Dir); Serial.println(" Selected");             
   #endif
  
}
//Envento cada minuto.
void ActualizaMinuto()
{  
  int Reg;
  
 
    //Adelanta la hora.Apartir del dia 25 de Marzo, busca el primer domingo
    //y cuando se han las 2 de la noche adelanta el reloj una hora
    if( minute==0 && Enable_DaylightSavingTime==true ){AutomaticDST();}
    if (hour==4){HoraRetrasa=false;}      

    minutoMemory=minute;
    
    TipoDia=dayOfWeek;
        
        //Verificacion Dia Especial 1
    for (Reg=EM_DATE_ESPECIAL1_OFSSET; Reg <= EM_DATE_ESPECIAL1_END; Reg=Reg +S_DATE_ESPECIAL){
      if (month == EepromRead(Reg) && dayOfMonth== EepromRead(Reg+1)){  TipoDia=8; }
    }
        //Verificacion Dia Especial 2
    for (Reg=EM_DATE_ESPECIAL2_OFSSET; Reg <= EM_DATE_ESPECIAL2_END; Reg=Reg +S_DATE_ESPECIAL){
           if (month == EepromRead(Reg) && dayOfMonth== EepromRead(Reg+1)){ TipoDia=9;}     
    }
    
    int r= ((TipoDia-1)* EM_TIME_DAY_SIZE)+ EM_TIME_WEEKLY_OFFSET;
    
    //Read Timetable for days.

    for (Reg=r; Reg <(r+EM_TIME_DAY_SIZE);Reg=Reg + S_TIME_ESPECIAL){if ( hour==EepromRead(Reg) && minute==EepromRead(Reg+1)){ timeChangeCircuit(Reg+2);}}

    //Read Triggers 
    for (Reg=EM_TRIGGER_OFFSET; Reg < EM_TRIGGER_END ; Reg=Reg + S_TRIGGER){if ( hour==EepromRead(Reg) && minute==EepromRead(Reg+1)){ EepromWrite(Reg, 66);timeChangeCircuit(Reg+2);}}


    
    for (int c=0;c<Number_Circuit;c++){    
      if ((circuits[c].Type==Riego_Temporizado)||(circuits[c].Type==Timer)){
        if (circuits[c].Value>=1) {
          if (circuits[c].Out1_Value==true){circuits[c].Value--;}
          else{circuits[c].Out1_Value=true;}        
        }
        if (circuits[c].Value==0){circuits[c].Out1_Value=false;}
      }
    }
}
void timeChangeCircuit(int addressEE)
{
  byte ci=EepromRead(addressEE);  
  if (EepromRead(EM_EN_TIMETABLE_OFFSET+ci)==0){return;}
  byte val=EepromRead(addressEE+1);
  
  if (ci < Number_Circuit){SetCircuits(ci,val);}
  else if((ci < 40)&&(ci >=30)){SelectScene(ci-29);}  
  else if((ci < 50)&&(ci >= 40)){
    int pos=ci-40;
    if (pos<EXC_Condicionados){
      if (val==1){Condicionados[pos]=true;}else{Condicionados[pos]=false;}
    }    
  } 
  else if((ci < 60)&&(ci >= 50)){PersonalFunctions(ci-50);}  
}

void CargaHora()
{
  getDateClock();
  if (minute != minutoMemory){ActualizaMinuto();NewMinute();  }
}

/******************************************************************/
//  FUNCIONES RELOJ
/*****************************************************************/

// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val){ return ( (val/10*16) + (val%10) );}

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val){return ( (val/16*10) + (val%16) );}


/*void setDateCLOCK(byte second,        // 0-59
                   byte minute,        // 0-59
                   byte hour,          // 1-23
                   byte dayOfWeek,     // 1-7
                   byte dayOfMonth,    // 1-28/29/30/31
                   byte month,         // 1-12
                   byte year)          // 0-99*/

void setDateCLOCK(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)
{
   Wire.beginTransmission(DS_RTC);
   Wire.write( (byte)0);
   //Wire.write(0);
   Wire.write(decToBcd(second));    // 0 to bit 7 starts the clock
   Wire.write(decToBcd(minute));
   Wire.write(decToBcd(hour));      // If you want 12 hour am/pm you need to set
                                   // bit 6 (also need to change readDateDs1307)
   Wire.write(decToBcd(dayOfWeek));
   Wire.write(decToBcd(dayOfMonth));
   Wire.write(decToBcd(month));
   Wire.write(decToBcd(year));
   Wire.endTransmission();
}

// Gets the date and time from the ds1307
void getDateClock()
{
  #ifdef EXC_DEBUG_MODE   
      Serial.print("Get Date Time ");               
  #endif
  // Reset the register pointer
  Wire.beginTransmission(DS_RTC);
  Wire.write( (byte)0);
  //Wire.write(0);
  if (Wire.endTransmission()==0){
    Wire.requestFrom(DS_RTC, 7);
      if (Wire.available()==7){   
      second     = bcdToDec(Wire.read() & 0x7f);
      minute     = bcdToDec(Wire.read());
      hour       = bcdToDec(Wire.read() & 0x3f);  // Need to change this if 12 hour am/pm
      dayOfWeek  = bcdToDec(Wire.read());
      dayOfMonth = bcdToDec(Wire.read());
      month      = bcdToDec(Wire.read());
      year       = bcdToDec(Wire.read());
      #ifdef EXC_DEBUG_MODE   
        Serial.println("OK");               
      #endif
      SegUpdtHora=30;
      return;
    }
  }
  SegUpdtHora=6;
  #ifdef EXC_DEBUG_MODE   
    Serial.println("ERROR");               
  #endif
}


bool SecutityCopy(){

  byte c;
  byte Udt=false;
  
  for (c=0;c<Number_Circuit;c++){
    if (circuits[c].Value!=fcs[c]){
      Udt=true;
      fcs[c]=circuits[c].Value;  
      //      if (circuits[c].Type>100){if (circuits[c].Out1_Value){v=v+100;}}
      if ((circuits[c].Type>100)&&(circuits[c].Out1_Value)){EepromWrite(EM_STATECOPY_OFSSET + c,fcs[c]+100);}
      else{EepromWrite(EM_STATECOPY_OFSSET + c,fcs[c]);}     
    }    
  }    
  for (c=0;c<EXC_Condicionados;c++){
    byte b=0;
    if (Condicionados[c]){b=1;} 
    if (b!= fcs[30+c]){
      fcs[30+c]=b;
      Udt=true;
      EepromWrite(EM_STATECOPY_OFSSET +30 + c,b);      
    }   
  }

   for (c=0; c < EXC_Consignas; c++) {
    word k= word(fcs[40+c], fcs[50+c]);
    if (k!=Consignas[c]){
      Udt=true; 
      fcs[50+c]=lowByte(Consignas[c]);
      fcs[40+c]=highByte(Consignas[c]);
    }    
  }
  
  #ifdef EXC_DEBUG_MODE   
    if (Udt){Serial.println("Update Copia Seguridad"); }  
  #endif    
  return Udt;    
}


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

#ifdef THERMOSTAT_DS18B20_NUMBER
  void InitDS18B20(){
      sensorTemp.begin();
      for (int c=0;c<THERMOSTAT_DS18B20_NUMBER;c++){sensorTemp.setResolution(Ds18B20Addres[c], TEMPERATURE_PRECISION);}
      sensorTemp.setWaitForConversion(true);
      RefreshTemperature();
      //sensorTemp.setWaitForConversion(false);
   }

   void RefreshTemperature(){
     sensorTemp.requestTemperatures();
     float f=0;
     for (int c=0;c<THERMOSTAT_DS18B20_NUMBER;c++){f= sensorTemp.getTempC(Ds18B20Addres[c]); if (f>-100){Temperature[c] =f;}}
    
    /* if (THERMOSTAT_DS18B20_NUMBER==1){Temperature[0] = sensorTemp.getTempCByIndex(0);}
     else{for (int c=0;c<THERMOSTAT_DS18B20_NUMBER;c++){Temperature[c] = sensorTemp.getTempC(Ds18B20Addres[c]);}}*/
  }
#endif 

void ResetConexion() {
  #ifdef EXC_SERVER
    TimUdp=35;
  #endif   
  #ifdef EXC_WIFI_SHIELD
    #ifdef EXC_DEBUG_MODE   
        Serial.println("Reseteando Conexon Wifi");  
    #endif
    #ifdef EXC_ENABLE_WATCH_DOG
      wdt_reset();
    #endif 
     Udp.stop();TimConexion=0;
     #ifdef EXC_ENABLE_WATCH_DOG
       wdt_reset();
      #endif 
     WiFi.disconnect();   

  #else
      #ifdef EXC_DEBUG_MODE   
          Serial.println("Reseteando Conexon");  
      #endif 
      #ifdef EXC_ENABLE_WATCH_DOG
        wdt_reset();
      #endif  
      Udp.stop();
      Ethernet.init(10);   // MKR ETH shield  cs pin
      
      #ifdef EXC_STATIC_IP  
        Ethernet.begin(mac,ip);
      #else    
        Ethernet.begin(mac);
      #endif
  
       #ifdef EXC_SERVER
        if (!Udp.begin(localPort)==1){
          TimUdp=35;
          #ifdef EXC_DEBUG_MODE   
            Serial.println("Fallo Iniciando UDP");  
          #endif
          }
       #else
        if (Udp.begin(localPort)==1){FalloUdp=false;}
        else{
          FalloUdp=true;
           #ifdef EXC_DEBUG_MODE   
                Serial.println("Fallo Iniciando UDP");  
            #endif
          }
       #endif
   /*
    if (Ethernet.linkStatus() == LinkON) {
      #ifdef EXC_DEBUG_MODE   
          Serial.println("Reseteando Conexon");  
      #endif 
      #ifdef EXC_ENABLE_WATCH_DOG
        wdt_reset();
      #endif  
      Udp.stop();
      Ethernet.init(10);   // MKR ETH shield  cs pin
      
      #ifdef EXC_STATIC_IP  
        Ethernet.begin(mac,ip);
      #else    
        Ethernet.begin(mac);
      #endif
  
       #ifdef EXC_SERVER
        if (!Udp.begin(localPort)==1){
          TimUdp=35;
          #ifdef EXC_DEBUG_MODE   
            Serial.println("Fallo Iniciando UDP");  
          #endif
          }
       #else
        if (Udp.begin(localPort)==1){FalloUdp=false;}
        else{
          FalloUdp=true;
           #ifdef EXC_DEBUG_MODE   
                Serial.println("Fallo Iniciando UDP");  
            #endif
          }
       #endif
       
    }
    else
    {
       #ifdef EXC_DEBUG_MODE   
          Serial.println("Fallo Cable de red");  
        #endif
        #ifdef EXC_SERVER
          TimUdp=50;
        #else          
          FalloUdp=false;          
        #endif        
    }   
    */
  #endif
}


#ifdef EXC_WIFI_SHIELD
boolean EstadoWifiOk(){
    if ((TimConexion > 34) && (WiFi.status() == WL_CONNECTED)){return true;}  
    else{return false;}
}

void ConexionWifi() {
    #ifdef EXC_DEBUG_MODE   
        Serial.println("Iniciando Conexon Wifi");  
    #endif
    //WiFi.disconnect(); delay(1000);
    
    #ifdef EXC_STATIC_IP  
        WiFi.config(ip);
    #endif

    
    if (Net_Type == OPEN){status = WiFi.begin(ssid);  }//Open Network
    if (Net_Type == WPA){status = WiFi.begin(ssid, pass); }//WPA NETWORK
    if (Net_Type == WEP){status = WiFi.begin(ssid, keyIndex, pass); }//WEP 
    if (Net_Type == A_POINT){
      status  = WiFi.beginAP(ssid);
      #ifdef EXC_DEBUG_MODE
        if (status != WL_AP_LISTENING) {Serial.println("Creating access point failed");
        }else{Serial.println("Creating access point Ok");}
      #endif    
    }
    TimConexion=20;   
     
}
#endif

//Funciones alarmas
void SetAlarm(int Number){if ((Number<=19)&&(Alarms[Number]==WithoutAlarm)){Alarms[Number]=SendingAlarm;}}
void ResetAlarm(int Number){if ((Number<=19)&&(Alarms[Number]==AlarmSent)){Alarms[Number]=WithoutAlarm;}}

 byte EepromRead ( unsigned short eeaddress){return EEPROM.read(eeaddress);}
 void EepromWrite ( unsigned short eeaddress, byte data ){if(EepromRead(eeaddress) != data) EEPROM.write(eeaddress, data);}
#ifdef EXC_I2C_BOARD
  void SetI2CRelay(byte OutNumber, boolean On){byte b=0;while (OutNumber>7){ OutNumber=OutNumber-10;b++;}if (On){bitWrite(IoBoards[b-1].Outputs, OutNumber, 1);}else{bitWrite(IoBoards[b-1].Outputs, OutNumber, 0);}}
  byte ReadMCP23017(byte Device){
    if (IoBoards[Device].Online==true){
     byte R =0;      
     while (R<4){
      delay(1);
      Wire.beginTransmission(0x20 + Device);  
      Wire.write(0x13);      
      if (Wire.endTransmission()==0){
            Wire.requestFrom(0x20 + Device, 1);//Wire.requestFrom(IC24C32_I2C_ADDRESS,1);
            if (Wire.available() == 1){return Wire.read();}
          }
          #ifdef EXC_DEBUG_MODE   
            Serial.print("i2c INPUT  READ ERROR - ");   
            Serial.println(R);
          #endif 
          #ifdef EXC_ENABLE_WATCH_DOG
            wdt_reset();
           #endif 
          delay(10);
          R++;
          
        }
        IoBoards[Device].Online=false;  
    }else{InitMCP23017(Device);}
    return IoBoards[Device].Inputs;
  }
  void WriteMCP23017(byte Device, byte  regValue){
    if (IoBoards[Device].Online==true){
        byte R =0;
        while (R<4){
          delay(1);
          Wire.beginTransmission(0x20 + Device); 
          Wire.write(0x12);
          Wire.write(regValue);          
          if (Wire.endTransmission()==0) {return;}
          else{
            #ifdef EXC_DEBUG_MODE   
              Serial.print("i2c OUTPUT  WRITE ERROR - "); Serial.println(R);
            #endif   
            #ifdef EXC_ENABLE_WATCH_DOG
              wdt_reset();
            #endif 
            R++;              
          }           
        }
      IoBoards[Device].Online=false;
    }else{InitMCP23017(Device);}
  }
  
  void InitMCP23017(byte Device){

    byte R=0;
    while (R<2){//Configure Output
      Wire.beginTransmission(0x20 + Device);
      Wire.write(0x00); // IODIRA register
      Wire.write(0x00); // set all of port A to outputs
      Wire.write(0xFF); // set all of port b to input    
      if (Wire.endTransmission()==0) {        
        #ifdef EXC_DEBUG_MODE   
          Serial.print("Init i2c setting ok, Device  ");Serial.print(Device);
        #endif
        IoBoards[Device].Online=true;return;}
      else{
        #ifdef EXC_DEBUG_MODE   
          Serial.print("Init i2c config error, Device  ");Serial.print(Device);Serial.print("ERROR COUNT"); Serial.println(R);
        #endif   
        #ifdef EXC_ENABLE_WATCH_DOG
            wdt_reset();
        #endif 
        R++;              
      }           
    }
     IoBoards[Device].Online=false;
  }
#endif
#ifdef EXC_RGB_LED
  void RgbRandomColor(){   
    
     switch (RGBrandomCount) {
          case 2:{
            
            if ((RGBredVal + RGBSpeed)>254){RGBblueVal=0;RGBgreenVal = 0;RGBredVal = 255;RGBrandomCount=0;}
            else{
              RGBredVal += RGBSpeed;
              RGBblueVal -= RGBSpeed; 
              RGBgreenVal=0;
            }
            break;}
          case 1:{  
            if ((RGBblueVal + RGBSpeed)>254){RGBgreenVal=0; RGBredVal = 0;RGBblueVal = 255;RGBrandomCount=2;}
            else{
              RGBblueVal += RGBSpeed;
              RGBgreenVal -= RGBSpeed; 
              RGBredVal=0;
            }
            break;}
          case 0:{ 
            if ((RGBgreenVal + RGBSpeed)>254){RGBredVal=0; RGBblueVal = 0; RGBgreenVal = 255;RGBrandomCount=1;}
            else{
              RGBgreenVal += RGBSpeed;
              RGBredVal -= RGBSpeed; 
              RGBblueVal=0;
            }
            break;}
          } 
  }
  
  void SetRGBLed(byte RedPin, byte GreenPin, byte BluePin,  byte Value, boolean InvertirSalida){  

      byte R,G,B;
      switch (Value) {

        case 199:    //Random
          R=RGBredVal;G=RGBgreenVal;B=RGBblueVal;
          break;
          
        case 1:    //// 8388736 Purple
          R=128;G=0;B=28;
          break;
          
        case 2://15631086 Viole
          R=238;G=130;B=238;      
          break;
          
        case 3: //9055202 BlueViolet
          R=128;G=43;B=226;      
          break;
          
        case 4://6970061 SlateBlue
          R=128;G=43;B=226;
          break;
          
        case 5:// 8087790 MediumSlateBlue
          R=123;G=104;B=238;
          break;
          
        case 6://255 blue
          R=0;G=0;B=255;     
          break;
          
        case 7://65535 Aqua
          R=0;G=255;B=255;
          break;
          
        case 8://11591910 PowderBlue
          R=176;G=224;B=230;
          break;
    
        case 9://49151 DeepSkyBlue
          R=0;G=191;B=255;
          break;
          
        case 10://2142890 LightSeaGreen
          R=32;G=178;B=170;
          break;
        
        case 11: //6737322 MediumAquamarine
          R=102;G=205;B=170;
          break;  
        case 12:  //10025880 PaleGreen
          R=152;G=251;B=152;
          break; 
        case 13: //64154 MediumSpringGreen
          R=0;G=250;B=154;
           break;  
        case 14:  //32768 Green
          R=0;G=128;B=0;
          break;  
        case 15: //65280  green1 
          R=0;G=255;B=0;
         break; 
         
        case 16: //14423100 Crimson
          R=220;G=20;B=60;
          break; 
          
        case 17: //13047173 MediumVioletRed
          R=199;G=21;B=133;
          break;  
        case 18: //16729344 OrangeRed
          R=255;G=69;B=0;      
          break;   
        case 19: //16776960 Yelow
          R=255;G=255;B=0;
          break;  
        case 20: //10824234 Brown
          R=165;G=42;B=42;
          break; 
        case 21: //16711680 Red
          R=255;G=0;B=0;
          break; 
        case 22: //16716947 DeepPink
          R=255;G=20;B=147;
          break;   
        
        case 23: //16747520 DarkOrange
          R=255;G=140;B=0;
          break; 
        case 24: //12092939 DarkGoldenrod
          R=184;G=134;B=11;
          break;
        case 25: //16032864 SandyBrown
          R=244;G=164;B=96;
          break; 
          
        case 26: //16737094 tomato
          R=255;G=99;B=70;
          break; 
         
        
        case 27: //16761035 Pink
          R=255;G=192;B=203;
         break; 
        
        
        case 28: //16752762 LightSalmon
          R=255;G=160;B=122;
          break;      
         
        case 29: //15787660 Khaki
          R=240;G=230;B=140;
          break; 
        
        case 30: //16113331 wheat
          R=245;G=222;B=179;
          break; 
    
        default:   // apagado
          R=0;G=0;B=0;
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

 
