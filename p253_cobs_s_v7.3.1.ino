
//https://hackaday.com/2015/12/10/embed-with-elliot-debounce-your-noisy-buttons-part-ii/

#include <math.h>

//#include <p253.h>
//#include "p253.h"
#include <TimerOne.h>


extern "C" {
#include "p253.h"
}

#define NOF_8BIT 5
#define NOF_16BIT 3
#define NOF_FRAC 2
#define NOF_FRAC16 1

#define  size_of_packet_macro  NOF_8BIT+NOF_16BIT*2  + NOF_FRAC+  (NOF_8BIT+NOF_16BIT-1)/8+1  +(( NOF_FRAC-1)/8)+1 +NOF_FRAC16+  (( NOF_FRAC16-1)/8)+1
#define  size_of_cobs_macro size_of_packet_macro+2


#define CHECK_MSEC    5    // Read hardware every 5 msec 
#define PRESS_MSEC    10   // Stable time before registering pressed
#define RELEASE_MSEC  100  // Stable time before registering released
 
bool DebouncedKeyPress = false;

#define MASK   0b11000111
  //                                 //  5  +6+2+    (8/8+1 )    +   1+1+1
const unsigned long  size_of_packet=NOF_8BIT+NOF_16BIT*2  + NOF_FRAC+  (NOF_8BIT+NOF_16BIT-1)/8+1  +(( NOF_FRAC-1)/8)+1 +NOF_FRAC16+  (( NOF_FRAC16-1)/8)+1;

 p253_t p253 ;
 //NOF_8BIT+NOF_16BIT*2  + NOF_FRAC+  (NOF_8BIT+NOF_16BIT)/8+1  +(( NOF_FRAC-1)/8)+1 +NOF_FRAC16+  (( NOF_FRAC16-1)/8)+1

uint8_t stuffed_example[19]={18,255,1,2,3,40,5,232,3,208,7,184,11,10,50,30,17,4,0}; 
 uint8_t example_received_package[17]={255 ,1 ,2, 3, 40 ,5, 232 ,3 ,208, 7 ,184, 11, 10 ,50 ,30,17 ,4}; 
void StuffData(const unsigned char *ptr, unsigned long length, unsigned char *dst);
uint8_t dest[size_of_packet+2] ;
char* target ;
          
 /*************degişkenler*****************************/
 uint8_t  n0f8 ;uint8_t n0f16;uint8_t noff ;     uint8_t total ; uint8_t noff16 ;uint8_t sbai ;  uint8_t fbai; uint8_t fbai16;
 
  uint8_t  number_of_packet_reset=0 ; 
  uint16_t  number_of_packet=0 ; 
  uint8_t  emergency_shut_down=0 ;        
          uint8_t frac_act_indx[NOF_8BIT]={1,0,0,0,1};
    
             uint8_t frac_act_indx16[NOF_16BIT]={0,0,1};

    

 
          uint8_t sign_bytes[30]; 
          uint8_t fraction_index_for_8bit_values[30]; 
           uint8_t fraction_index_for_16bit_values[30]; 
          uint8_t fractionpart_8bit[30];   // storage array for 8bit data to be sent over usart ....
          uint8_t decimalpart_8bit[30];   // storage array for 8bit data to be sent over usart ....

             uint8_t fractionpart_16bit[30];   // storage array for 8bit data to be sent over usart ....
          uint16_t decimalpart_16bit[30];   // storage array for 8bit data to be sent over usart ....

          
         int8_t raw_packet_8bit[30];   // storage array for 8bit data to be sent over usart ....
            uint8_t debug_array[30]; 
    
           float data_raw8bit[30]={0};   
            float data_raw16bit[30]={0};   
int8_t  dbg=0;
int16_t  dbg1=0;
int16_t  dbg2=0;


int xPin = A0; // A0-A5 analog pinlerinden herhangi birine bağlanabilir.
int yPin = A1; // A0-A5 analog pinlerinden herhangi birine bağlanabilir.

int x2Pin = A2; // A0-A5 analog pinlerinden herhangi birine bağlanabilir.
int y2Pin = A3; // A0-A5 analog pinlerinden herhangi birine bağlanabilir.


float dir_a=0.0;int dir_b=0;  int dir_c=0; int dir_d=0;
 int dir_a2=0;int dir_b2=0;  int dir_c2=0; int dir_d2=0;




int buttonState;             // the current reading from the input pin
int lastButtonState = 0;   // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 500;    // the debounce time; increase if the output flickers

int buton_basildi;             // the current reading from the input pin
int old_reading = 0;

/*****************BUTTON RELATED VARIABLES  ******************************************/
int press_count=0; bool Key_changed = false;  bool Key_pressed = false;

void DebounceSwitch1(bool *Key_changed, bool *Key_pressed)
{
 static uint8_t Count = RELEASE_MSEC / CHECK_MSEC;
 bool RawState;
 *Key_changed = false;
 *Key_pressed = DebouncedKeyPress;
 RawState = digitalRead(2);
 //RawKeyPressed();
 if (RawState == DebouncedKeyPress) {
 // Set the timer which allows a change from current state.
 if (DebouncedKeyPress) Count = RELEASE_MSEC / CHECK_MSEC;
 else Count = PRESS_MSEC / CHECK_MSEC;
 } else {
       // Key has changed - wait for new state to become stable.
       if (--Count == 0) {
       // Timer expired - accept the change.
       DebouncedKeyPress = RawState;
       *Key_changed=true;
       *Key_pressed=DebouncedKeyPress;
     // if (DebouncedKeyPress) Count = RELEASE_MSEC / CHECK_MSEC;
       
       // And reset the timer.
       if (DebouncedKeyPress){ Count = RELEASE_MSEC / CHECK_MSEC;
      press_count++;
       }
       else Count = PRESS_MSEC / CHECK_MSEC;
       }
   }
} 






        
void setup() {
pinMode(2, INPUT);
Serial.begin(115200);
Timer1.initialize(1000); // set timer  for every ... ms and calculate pid again in each period
Timer1.attachInterrupt( timerIsr ); // attach the service routine here

     
  
        /*some test data is appointed to the data arrays */
    //    data_raw8bit[0]=-1.10;  data_raw8bit[1]=-2.2097;   data_raw8bit[2]=-3.30;  data_raw8bit[3]=-40.407;   data_raw8bit[4]=-5.50;  data_raw8bit[5]=-6.6097;   data_raw8bit[6]=-7.70;  data_raw8bit[7]=-8.807; ;  data_raw8bit[8]=-9.907; 
    //    data_raw16bit[0]=-1000.100;  data_raw16bit[1]=-2000.200;   data_raw16bit[2]=-3000.30;  data_raw16bit[3]=-4000.407;   data_raw16bit[4]=-5000.50;  data_raw16bit[5]=-6000.60;   data_raw16bit[6]=-7000.70;  data_raw16bit[7]=-8000.807; ;  data_raw16bit[8]=-9000.907; 
        
    



 
p253_init3(&p253,  NOF_8BIT,  NOF_16BIT,  NOF_FRAC, NOF_FRAC16,frac_act_indx,frac_act_indx16);  
  
//p253_perform_packing(  p253 ,data_raw8bit,data_raw16bit , debug_array );//  StuffData(debug_array,(size_of_packet),dest); 
//Serial.print("Raw_data: ")  ;       for(int i =0 ; i<size_of_packet; i++)        {  Serial.print(debug_array[i]);   Serial.print(",");        }   Serial.println(); Serial.print("StuffedData: ")  ;       for(int i =0 ; i<size_of_packet+2; i++)        {     Serial.print(dest[i]);   Serial.print(",");        }   Serial.println();Serial.print("solution: ")  ;       for(int i =0 ; i<17; i++)        {     Serial.print(example_received_package[i]);   Serial.print(",");        }   Serial.println();
//   for(int i =0 ; i<length_of_package; i++)        { Serial.write(dest[i]);}



}

void loop() {

int xPozisyonu = 0;
int yPozisyonu = 0;
int x2Pozisyonu = 0;

  xPozisyonu = analogRead(xPin);
  yPozisyonu = analogRead(yPin);
  x2Pozisyonu= analogRead(x2Pin); //potantiometer  0 1023
  int reading = digitalRead(2);


convert_from_physical( xPozisyonu, yPozisyonu , xPozisyonu, yPozisyonu )   ;

if (emergency_shut_down==1)      {press_count=0 ;emergency_shut_down=0;}

    
//  dir_a= dir_a*(2.43);

  number_of_packet++;
  if(number_of_packet>1000){ number_of_packet    =   0; number_of_packet_reset++;}


 /*some test data is appointed to the data arrays */
     //   data_raw8bit[0]=-1.10;  data_raw8bit[1]=press_count;   data_raw8bit[2]=emergency_shut_down;  data_raw8bit[3]=number_of_packet_reset;   data_raw8bit[4]=-5.50; // data_raw8bit[5]=-6.6097;   data_raw8bit[6]=-7.70;  data_raw8bit[7]=-8.807; ;  data_raw8bit[8]=-9.907; 
     //   data_raw16bit[0]=dir_a;  data_raw16bit[1]=number_of_packet;   data_raw16bit[2]=x2Pozisyonu; // data_raw16bit[3]=-4000.407;   data_raw16bit[4]=-5000.50;  data_raw16bit[5]=-6000.60;   data_raw16bit[6]=-7000.70;  data_raw16bit[7]=-8000.807; ;  data_raw16bit[8]=-9000.907; 
        
 data_raw8bit[0]=-9.10;  data_raw8bit[1]=press_count;   data_raw8bit[2]=emergency_shut_down;  data_raw8bit[3]=number_of_packet_reset;   data_raw8bit[4]=-5.50;
 data_raw16bit[0]=dir_a;  data_raw16bit[1]=number_of_packet;   data_raw16bit[2]=x2Pozisyonu; 
  
 p253_perform_packing(  p253 ,data_raw8bit,data_raw16bit , debug_array );
 StuffData(debug_array,(size_of_packet),dest); 



  
Serial.print(dir_a)  ;  Serial.print(" ");  Serial.println(xPozisyonu) ;    //    for(int i =0 ; i<size_of_packet; i++)        {  Serial.print(debug_array[i]);   Serial.print(",");        }   Serial.println();//Serial.print("StuffedData: ")  ;       for(int i =0 ; i<size_of_packet+2; i++)        {     Serial.print(dest[i]);   Serial.print(",");        }   Serial.println();

//for(int i =0 ; i<size_of_packet+2; i++)        {     Serial.write(debug_array[i]);        }Serial.write(253); // Serial.print(x2Pozisyonu);    Serial.print(" ");  Serial.print(dest[12]);  Serial.print(" "); Serial.println(dest[11]); 
  
//for(int i =0 ; i<size_of_packet+2; i++)        {     Serial.write(dest[i]);        }  // Serial.print(x2Pozisyonu);    Serial.print(" ");  Serial.print(dest[12]);  Serial.print(" "); Serial.println(dest[11]); 
delay(200);


    
 }


void timerIsr(){


DebounceSwitch1( &Key_changed,  &Key_pressed);


}



void convert_from_physical( int joystick1_x , int joystick1_y , int joystick2_x , int joystick2_y  )            
{         
 /* 1 joystickte 4 farklı yön var. 1. yön yukarı  2. yön aşagı 3.yön sag 4. yön sol olarak tanımlandı
  * 1.yönün degerlerini ilk önce 0 -512 arasında değişen hale getirip sonra 2 ile çarpacagız.
  * eger gelen deger 512den büyükse 512 çıkar.bu  benim dir_a degerimdir.
  * 
  */
  
      dir_a=0; dir_b=0;   dir_c=0;  dir_d=0;
      dir_a2=0; dir_b2=0;   dir_c2=0;  dir_d2=0;

//dir_a=joystick1_x-xofset;

 if (joystick1_x>420)      { dir_a=  joystick1_x-420;}
  if (joystick1_y<100)      { emergency_shut_down=1;}

     //joystick1_x=joystick1_x-40;
 //if (joystick1_x-xofset)      { dir_a=  joystick1_x-543;}
  //   dir_a=  joystick1_x;
   //  if (joystick1_x>552)      { dir_a=  joystick1_x-543;}
//     else if(joystick1_x<552) {  dir_a= 0; dir_b=  joystick1_x;} 
//
//      if (joystick1_y>512)      { dir_c=  joystick1_y-512;}
//     else if(joystick1_y<512) {   dir_d=  joystick1_y;} 
//
//     
//     if (joystick2_x>512)      { dir_a2=  joystick2_x-512;}
//     else if(joystick2_x<512) {   dir_b2=  joystick2_x;} 
//
//      if (joystick2_y>512)      { dir_c2=  joystick2_y-512;}
//     else if(joystick2_y<512) {   dir_d2=  joystick2_y;} 




   //  if (joystick1_x>552)      { dir_a=  joystick1_x-552;}







} 









#define FinishBlock(X) (*code_ptr = (X),  code_ptr = dst++,   code = 0x01  )
void StuffData(const unsigned char *ptr, unsigned long length, unsigned char *dst) 
{


 unsigned char *zero=0;
 unsigned char *dst2;
 
  const unsigned char *end = ptr + length; //başlangıç adresi + uzunluk ile dizideki son degerin adresini belirle
  unsigned char *code_ptr = dst++;  // dst nin 1. elemanının adresini code_ptr ye at
  unsigned char code = 0x01; //counter for 255 adet data
while (ptr < end) // ptr dizisinin sonuna ulaşana dek...
{ 
   if (*ptr == 0) FinishBlock(code); 
  else { 
    *dst++ = *ptr;
    code++; 
     if (code == 0xFF)FinishBlock(code);
     } 
    
    ptr++; 
   // if (ptr - end==0)  *dst++ = zero;
  dst2  =dst;
  } 
   
FinishBlock(code);
dst=dst2;
 //*dst++ = zero; 
//  *dst++=*zero;
}
