/*
	Este fichero crea las definiciones del proyecto Excontrol.

*/

/****************************************************************************
	Numeración de tipos de circuitos.
        Define circuits type
*****************************************************************************/

#define Reserva       	1
#define Ado_Digital   	2
#define Ado_3Etapas   	3
#define RGB 	        5
#define Dimmer   	6
#define Enchufe		8
#define	Riego		14
#define Riego_Temporizado 15
#define Valvula		16

#define Frio		20
#define Calor		25
#define Radiante	26


#define ConsignaTemp	30
#define HomeTemperature	31
#define TempNegative	32
#define SensorView      34

#define	Persiana	35
#define Toldo		36
#define Persiana2	37
#define	Puerta		40
#define	Ventilador	44
#define	Ventiladorx2	45
#define	Ventiladorx3	46
#define	Piloto  	52
#define	Timer    	53


#define SetPoint_100	54    
#define SetPoint_200	55
#define SetPoint_2000	56
#define SetPoint_20000	57 

#define High_trigger	58 
#define Low_trigger	59 
        
/****************************************************************************
	Numeración de tipos de entradas.
        Define Inputs type
*****************************************************************************/
#define Swicth       	1
#define Button   	2
#define Retroaviso   	3

/****************************************************************************
	Numeración de tipos de Sensores.
        Define Senosors type
*****************************************************************************/
#define Sensor_Temperature      1
#define Sensor_Humidity         2
#define Sensor_Lux   	        3
#define Sensor_Generic          4
#define Sensor_Float          5
/****************************************************************************
	Numeración de tipos de Alarmas.
        Define Senosors Alarms
*****************************************************************************/
#define AlarmDisabled         0
#define WithoutAlarm          1
#define SendingAlarm         2
#define AlarmSent             3


//Definiciones entradas a 433mhz
#define RfInputOn0 6000
#define RfInputOn1 6001
#define RfInputOn2 6002
#define RfInputOn3 6003
#define RfInputOn4 6004
#define RfInputOn5 6005
#define RfInputOn6 6006
#define RfInputOn7 6007
#define RfInputOn8 6008
#define RfInputOn9 6009
#define RfInputOn10 6010
#define RfInputOn11 6011
#define RfInputOn12 6012
#define RfInputOn13 6013
#define RfInputOn14 6014

#define RfInputOff0 6100
#define RfInputOff1 6101
#define RfInputOff2 6102
#define RfInputOff3 6103
#define RfInputOff4 6104
#define RfInputOff5 6105
#define RfInputOff6 6106
#define RfInputOff7 6107
#define RfInputOff8 6108
#define RfInputOff9 6109
#define RfInputOff10 6110
#define RfInputOff11 6111
#define RfInputOff12 6112
#define RfInputOff13 6113
#define RfInputOff14 6114

//Otras configuraciones
#define DS_RTC 0x68  //Direccion Reloj
#define IC24C32_I2C_ADDRESS 0x50 //EEORIN ADD
/*
	#Precompilador calculo posiciones memoria EEProm.
	
	
	# N_name			=> Number de objets.
	# S_name			=> Size de objets in bytes.
	# Name_OFSSET 	=> Direccion inicial bloque de memoria Eeprom
*/

/*
	NUMBER SLOTS.
*/
#define N_ESCENES 	 	10
#define N_EN_TIMETABLE	 	60	//(N_CIRCUITS, N_ESCENES, N_CONDITIONED,FUNCIONES COMUNES)
#define N_CIRCUITS	 	30
#define	N_UP_TIM_SHUTTER	30
#define	N_DO_TIM_SHUTTER	30
#define N_TRIGGER		12
#define N_TIME_DAY		80	//			 daily schedule
#define N_TIME_ESPECIAL  	80
#define N_DATE_ESPECIAL         25	//month ,day
#define N_ALARMS		20	//BYTE ??
#define N_CONDITIONED		10 
#define N_SETPOINTS		10


/*
	SIZE SLOTS.
*/
#define S_ESCENES 	 	30
#define S_EN_TIMETABLE 		1	//(N_CIRCUITS, N_ESCENES, N_CONDITIONED)
#define S_CIRCUITS	 	1
#define	S_UP_TIM_SHUTTER	1
#define	S_DO_TIM_SHUTTER	1
#define S_TRIGGER		4
#define S_TIME_DAY		4
#define S_TIME_ESPECIAL  	4
#define S_DATE_ESPECIAL    	2	//month ,day			
#define S_ALARMS		1	//BYTE ??
#define S_CONDITIONED		1 
#define S_SETPOINTS		2

#define EM_TIME_ESPECIAL_SIZE    (N_TIME_ESPECIAL * S_TIME_ESPECIAL)
#define EM_DATE_ESPECIAL_SIZE    (N_DATE_ESPECIAL * S_DATE_ESPECIAL)
#define EM_TIME_DAY_SIZE         (N_TIME_DAY * S_TIME_DAY)
#define EM_STATECOPY_SIZE 40

#define EM_ESCENES_OFFSET  1
#define EM_ESCENES_END           (N_ESCENES  * S_ESCENES)                                        //300
#define EM_EN_TIMETABLE_OFFSET   (EM_ESCENES_END + 1)                                            //301
#define EM_EN_TIMETABLE_END      (EM_ESCENES_END + (N_EN_TIMETABLE * S_EN_TIMETABLE))            //300 + (60 * 1) = 360
#define EM_CIRCUITS_OFFSET       (EM_EN_TIMETABLE_END + 1)                                       //361
#define EM_CIRCUITS_END          (EM_EN_TIMETABLE_END + ( N_CIRCUITS * S_CIRCUITS))              //360 + (30 * 1) = 390
#define	EM_UP_TIM_SHUTTER_OFFSET (EM_CIRCUITS_END + 1)                                           //391
#define	EM_UP_TIM_SHUTTER_END    (EM_CIRCUITS_END + (N_UP_TIM_SHUTTER * S_UP_TIM_SHUTTER))       //390 + (30 * 1) = 420
#define	EM_DO_TIM_SHUTTER_OFFSET (EM_UP_TIM_SHUTTER_END + 1)                                     //421
#define	EM_DO_TIM_SHUTTER_END    (EM_UP_TIM_SHUTTER_END + (N_DO_TIM_SHUTTER * S_DO_TIM_SHUTTER))// 420 + (30 * 1) = 450
#define EM_ALARMS_OFSSET         (EM_DO_TIM_SHUTTER_END + 1)
#define EM_ALARMS_END            (EM_DO_TIM_SHUTTER_END + (N_ALARMS * S_ALARMS))
#define EM_CONDITIONED_OFSSET    (EM_ALARMS_END + 1)
#define EM_CONDITIONED_END       (EM_ALARMS_END + (N_CONDITIONED * S_CONDITIONED))
#define EM_SETPOINTS_OFSSET 	 (EM_CONDITIONED_END + 1)
#define EM_SETPOINTS_END         (EM_CONDITIONED_END + (N_SETPOINTS * S_SETPOINTS))

// USO de RTC
#define	EM_TRIGGER_OFFSET        (EM_SETPOINTS_END + 1)
#define	EM_TRIGGER_END           (EM_SETPOINTS_END + (N_TRIGGER * S_TRIGGER))
#define	EM_TIME_WEEKLY_OFFSET    (EM_TRIGGER_END + 1)
#define	EM_TIME_WEEKLY_END       (EM_TRIGGER_END + ( 7 * EM_TIME_DAY_SIZE))
#define EM_TIME_ESPECIAL1_OFSSET (EM_TIME_WEEKLY_END + 1)
#define EM_TIME_ESPECIAL1_END    (EM_TIME_WEEKLY_END + EM_TIME_ESPECIAL_SIZE)
#define EM_TIME_ESPECIAL2_OFSSET (EM_TIME_ESPECIAL1_END + 1)
#define EM_TIME_ESPECIAL2_END	 (EM_TIME_ESPECIAL1_END + EM_TIME_ESPECIAL_SIZE)
#define EM_DATE_ESPECIAL1_OFSSET (EM_TIME_ESPECIAL2_END + 1)
#define EM_DATE_ESPECIAL1_END    (EM_TIME_ESPECIAL2_END + EM_DATE_ESPECIAL_SIZE)
#define EM_DATE_ESPECIAL2_OFSSET (EM_DATE_ESPECIAL1_END + 1)
#define EM_DATE_ESPECIAL2_END	 (EM_DATE_ESPECIAL1_END + EM_DATE_ESPECIAL_SIZE)

#define EM_STATECOPY_OFSSET (EM_DATE_ESPECIAL2_END + 1)
#define EM_STATECOPY__END	 (EM_STATECOPY_OFSSET + EM_STATECOPY_SIZE)

#define User_Eeprom                   (EM_STATECOPY__END+ 1)
