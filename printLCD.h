/*********************************************************************
 @description   
   This version only offers minimal wrapping of print lcd. 
 @version 
   0.1.2 para test constantes, en lugar de PRM.
	
	Copyright (C) 2015 Francisco Cosmo "@cosmopaco"

	This program is licensed to you under the Apache License Version 2.0,
    and you may not use this file except in compliance with the Apache License Version 2.0.
	
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.

	Originally developed by Francisco Cosmo @cosmopaco 
	
********************************************************************/  

/*********************************************************************
  Printf personalizado para la escritura en pantalla LCD.

  Los datos de entrada se deben colocar despues de " deben empezar con coma.
  
  writeLCD( "TEMPERATURA %fC %d",variable float, variable int);
  writeLCD( "%D %2/%0/%2 %H:%M:%S ",dayOfMonth, month, year);
  MARCADOR %
  
   ESPECIFICADOR            ENTRADA    POSICIONES/SALIDA   
  %f dato tipo float        float      4 justificado derecha.
                                         Valores en decimal de -9.9 a 99.9
                                         Valor entero -999 a 9999
                                         Valores inferiores o superiores ----
  %d dato decimal           int/byte   4 justificado derecha. Unidades de millar
  %4 dato decimal           int/byte   4 justificado derecha. Unidades de millar
  %3 dato decimal           int/byte   3 justificado derecha. Centenas
  %2 dato decimal           int/byte   2 justificado derecha. Decenas.
  %1 dato decimal           int/byte   1 justificado derecha. Unidades
  %0 dato decimal           int/byte   2 justificado derecha. Decenas con cero izquierda.  
  %c caracter              ,'o' " "    1 Caracter
  %s cadena texto flash.    Puntero F  ? Cadena de texto en flash.
  %b Imprime bitmap         int/byte   1 valor=0 bombilla apagada en caso contrario bombilla encendida. 
  
&&& Datos sistema.  Sin datos entrada.
  %% Imprime simbolo %                 %
  %D Imprime Dia Semana     Sistema    3 texto LUN,MAR,MIE,
  %H Imprime hora           Sistema.   2 posiciones.
  %M Imprime minuto         Sistema    Cero a la izquierda 2 posiciones
  %S Imprime segundos       Sistema    Cero a la izquierda 2 posiciones
 
&&& BITMAP 1 POSICION MARCADOR # 
   ESPECIFICADOR  SALIDA.
   
  #0    Enchufe
  #1    Llave
  #2    Termometro.
  #3    Gota humedad.
  #4    Grados centigrados lcd.print((char)223); //signo grados 
  #5    Bombilla encendidad
  #6    Bombilla apagada.
  #7    Campana Alarma.
  ##    Un solo caracter #

  Problemas:
  El simbolo º produce un ascii extraño.
********************************************************************/  

//#include <avr/pgmspace.h>

extern LiquidCrystal_I2C lcd;
extern byte hour,minute,second,dayOfWeek; 
// Creat a set of new characters
const uint8_t charBitmap[][8] = {
  // #0 Bitmap Socket.
  {
    0b01010,
    0b01010,
    0b11111,
    0b11111,
    0b01110,
    0b01110,
    0b00100,
    0b00010
  },
  // #1 Bitmap Key.
  {   
    0b01110,
    0b10001,
    0b10001,
    0b01110,
    0b00100,
    0b00100,
    0b00110,
    0b00111
  },
  // #2 Bitmap Thermometer
  {
    0b00100,
    0b01010,
    0b01010,
    0b01110,
    0b01110,
    0b11111,
    0b11111,
    0b01110
  },
  // #3 Bitmap Humidity
  {
    0b00100,
    0b00100,
    0b01010,
    0b01010,
    0b10001,
    0b10001,
    0b10001,
    0b01110
  },
  // #4 Bitmap Centigrade
  {
    0b11000,
    0b11000,
    0b00000,
    0b00011,
    0b00100,
    0b00100,
    0b00100,
    0b00011
  },
  // #5 Bitmap BulbOn
  {
    0b01110,
    0b11111,
    0b11111,
    0b11111,
    0b01010,
    0b01110,
    0b01110,
    0b00100
  },
  // #6 Bitmap BulbOff
  {
    0b01110,
    0b10001,
    0b10001,
    0b10001,
    0b01010,
    0b01110,
    0b01110,
    0b00100
  },
  // #7 Bitmap Alarm 
  {
    0b00100,
    0b01110,
    0b01110,
    0b01110,
    0b11111,
    0b11111,
    0b00000,
    0b00100
  }
};

//{{{ loadCharacters():
/*Create a news customs characters.
 \param  None.
 \
 \out    Salida en pantalla.
*/
void loadCharsLCD()
{
  for ( int i = 0; i < 8 ; i++ )
  {
    lcd.createChar ( i, (uint8_t *)charBitmap[i] );
  }
}


// Macros, Using expressions
/*

*/
#define lcdPrintBitMap(n)  lcd.write((uint8_t) n)  // Direct use,  n =(0 to 7)


#define lcdPrintSocket       lcd.write((uint8_t) 0) // #1 Bitmap Socket.
#define lcdPrintKey          lcd.write((uint8_t) 1) // #2 Bitmap Key.
#define lcdPrintThermometer  lcd.write((uint8_t) 2) // #3 Bitmap Thermometer.
#define lcdPrintHumidity     lcd.write((uint8_t) 3) // #4 Bitmap Humidity.
#define lcdPrintCentigrade   lcd.write((uint8_t) 4) // #5 Bitmap Centigrade.
#define lcdPrintBulbOn       lcd.write((uint8_t) 5) // #6 Bitmap BulbOn.
#define lcdPrintBulbOff      lcd.write((uint8_t) 6) // #7 Bitmap BulbOff.
#define lcdPrintAlarm        lcd.write((uint8_t) 7) // #8 Bitmap Alarm.


const char DAY[][4] = {
  {"LUN"},
  {"MAR"},
  {"MIE"},
  {"JUE"},
  {"VIE"},
  {"SAB"},
  {"DOM"},
};

//const char string_0[] PROGMEM = "LUN";   // "String 0" etc are strings to store - change to suit.
//const char string_1[] PROGMEM = "MAR";
//const char string_2[] PROGMEM = "MIE";
//const char string_3[] PROGMEM = "JUE";
//const char string_4[] PROGMEM = "VIE";
//const char string_5[] PROGMEM = "SAB";
//const char string_6[] PROGMEM = "DOM";
//
//// Then set up a table to refer to your strings.
//
//const char* const string_table[] PROGMEM = 	   // change "string_table" name to suit
//{   
//  string_0,
//  string_1,
//  string_2,
//  string_3,
//  string_4,
//  string_5,
//  string_6 
//};

  



//{{{ owrite(): Imprime  cadena.
/*
 \param  puntero a cadena a imprimir.
 \out    Salida en pantalla.
*/
void owrite(const char *str)
{
  lcd.print(str);
    
  #ifdef LCDINSERIAL
    Serial.print(str);
  #endif    
}


//{{{ pdec(): print decena,
/*Subfuncion transforma a ascii los valorea a partir de decenas.
 \param val entero valor.
 \param result cadena donde se va a escribir.
 \param len longitud de la cadena
 \return verdadero si ha sucedido.
*/

char *pdec(int val, char *result, signed char len)
{
  while (len >= 0)
  {
    if (val > 0)
      result[len] =val%10+48;
    else
      result[len] =' '; 
      
    val /= 10;
    len--;    
   }   
   return result;  
}


// {{{ itoaR(): integer to string right justified.
/* Transforam integer en cadena justificado derecha 
 \param val Integer value
 \param result String where the result will be written
 \param len Lenght of the string
 \return true if the operation was succeeded
*/

char *itoaR(int val, char *result, signed char len)
{
  result[len--] = 0;		   //NULL TERMINATION.
  result[len--] = (val%10)+48;   //Guarda Unidades.
  val /= 10;                     //Eliminamos unidades.
  pdec(val, result, len);

  return result;  
}
//{{ sftoaR(): float to string right justified whit signed.
/* Transforma float en cadena justificado derecha añade signo negativo. 
 \param val entero valor.
 \param result cadena donde se va a escribir.
 \param len longitud de la cadena
 \return verdadero si ha sucedido.
*/
char *sftoaR(float val, char *result, signed char len)
{
  float v;
  signed int i;
  boolean neg = false;
  result[len--] = 0;            	//null termination.
  

  if(val > 100){
    i= val;
    pdec(i, result, len);
  }
  else if(val < -10){      
    i = val;                           
    i=-i;
    neg=true;
    pdec(i, result, len);
  }
  else{      
    v= val * 10;			// Ejemplo entrada 32.5 v=325
    i = v;                            // Pasamos float a un entero 325
    
    if(i < 0){                        // Si es negativo 
      neg=true;
      i = - i;
    }
    
    result[len--] = ((i%10)+48); 	//Decimal a array.
    result[len--] = '.';		//Coma decimal.
    i /= 10;                          //Borramos unidades.
    pdec(i, result, len);
  }
  
  if(neg)
    result[0]='-';
  
  return result;
}

// {{{ writeLCD(): personalized printf for LCD
void writeLCD(const unsigned char line,const char *fmt, ...) 
{
  va_list ap;// ap2;
  short d;
  char c, *s;
  double f;
  char i[10];
  
  if (line < 5)
  {
    lcd.setCursor ( 0, line );
  }
   
  va_start(ap, fmt);
  while (*fmt) {
    if (*fmt == '%') 
    {
      *fmt++;
      switch (*fmt){
        case 's':
          s = va_arg(ap, char*);
          owrite (s);
          break;
          
        case 'H':                      
         itoaR(hour, i, 2);
         owrite (i);
          break;
        case 'M':
          itoaR(minute +100, i, 2);
          owrite (i);
          break;
        case 'S':
          itoaR(second +100, i, 2);
          owrite (i);
          break;
        case 'D':
          //strcpy_P(i, (char*)pgm_read_word(&(string_table[dayOfWeek])-1)); QUE DESASTRE.....
	  //	  strcpy_P(i, (char*)pgm_read_word(& string_table[dayOfWeek -1]));
          owrite (DAY[dayOfWeek -1]);
        break;
        
        case 'd':
        case '4':
          d = va_arg(ap, int);
          itoaR(d, i, 4);
          owrite(i);
          break;
        case '3':
          d = va_arg(ap, int);
          itoaR(d, i, 3);
          owrite(i);
          break;
        case '2':
          d = va_arg(ap, int);
          itoaR(d, i, 2);
          owrite(i);
          break;
        case '1':
          d = va_arg(ap, int);
          itoaR(d, i, 1);
          owrite(i);
          break;
        case '0':
          d = va_arg(ap, int);
          itoaR(d +100, i, 2);
          owrite(i);
          break;
        case 'c':
          c = va_arg(ap, int);
          i[0]=char(c);
          i[1]=0;
          owrite(i);
          break;
        case 'f':
          f = va_arg(ap, double);
          if(f > 10000 || f < -999.9){
            owrite("----");
          }
          else{
            sftoaR((float)f, i, 4);
            owrite(i);
          }
          break;
        case '%':
          owrite("%");
          break;
        case 'b':
          if (va_arg(ap, int))
            lcdPrintBulbOn;
          else
            lcdPrintBulbOff;
            break;
          }
          *fmt++;
        }
        else if(*fmt == '#')
        {
          *fmt++;
          if(*fmt >= '0' && *fmt < '8' ){
            lcd.write((uint8_t) (*fmt - '0')); //Corregido texto ascii a valor numerico.
          }
          else if(*fmt == '#'){
            owrite("#");
          }
          *fmt++;
        }
        else
        {
          i[0]=char(*fmt);
          i[1]=0;
          owrite(i);
          *fmt++;
        }
  }
  va_end(ap);
}
