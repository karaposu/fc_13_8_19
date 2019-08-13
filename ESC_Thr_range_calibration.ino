
//bu kod çalıştırılmadan önce esc_kalinrasyon kodu ile max ve min throttle degerleri belirlenmiş olmalıdır.
//Standart degerler sırası ile 2000 ve 1000 olarak kullanılmaktadır. 

#include <Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN 10
int temp=0;
Servo motor;

void setup() {
  Serial.begin(9600); motor.attach(MOTOR_PIN);
 
  Serial.println("lutfen gucu acin ve bir tusa basin");  motor.writeMicroseconds(MAX_SIGNAL);
   while (!Serial.available());motor.writeMicroseconds(MAX_SIGNAL);
 Serial.println("Baslangictaki 3 adet beep ten sonra 2sn bekleniyor "); delay(4000);
 Serial.println("2 sn beklendi.max thr secili oldugunu gösteren 2 adet beep duyuldu ");
 Serial.println("MIN THR DEGERİ YAZDIRILIYOR ");motor.writeMicroseconds(MIN_SIGNAL);delay(1000);
  Serial.println("UZUN BEEEP DUYULDU YANI MIN THR DEGERI TANINDI ");motor.writeMicroseconds(MIN_SIGNAL);delay(1000);
 
   
 

  
//  Serial.println("program moduna giris icin max throttle veriliyor");
//  motor.writeMicroseconds(MAX_SIGNAL);
//  delay(2000);
//  motor.writeMicroseconds(MAX_SIGNAL);
//  Serial.println("wait 5 seconds to hear programming tune.");
//  delay(3000);
//   motor.writeMicroseconds(MAX_SIGNAL);
//
//delay(2000);

   
//Serial.println(" ");Serial.println(" ");Serial.println("Girmek istediginiz ayarın sesini duyunca herhangibi bir tuşa basınız");
//   Serial.println("-------------------------------------------------------------------------------------------------");
//    Serial.println("Ayarlarin  alt seçenekleri parantez içinde belirtilmiştir");
//        Serial.println("1. seçenek  beep 2. seçenek beep-beep ve varsa 3. seçenek için beep-beep-beep sesini bekleyip bir tuşa basmanız gerekir ");
//Serial.println(" ");
//Serial.println(" ");
//
//Serial.println(" beep                         Brake , (Enabled / Disabled)");
//Serial.println(" beep-beep                     Batarya Tipi ,(Li-ion / Li-poly or NiMh / NiCd)");
//Serial.println(" beep-beep-beep                 cutoff modu , (Soft Cut-off/Hard Cut-off)  ");
//Serial.println(" beep-beep-beep-beep            cutoff threshold degeri, (Low / Medium / High) *default is medium  ");
//Serial.println(" beeeeeeep                     start-up modu ,   (Normal / Soft / Super Soft (Default is normal)) ");
//Serial.println(" beeeeeeeep-beep                timing ayari , Low / Medium / High (Default is Low)  ");
//Serial.println(" beeeeeeeep-beep-beep            fabrika ayarlari ");
//Serial.println(" beeeeeeeep-beeeeeeeep            çıkış");
//
// Serial.println("secim icin bir tusa basın");//temp=  Serial.read();
// 
//  while (Serial.available()==0){
//    }   
//  motor.writeMicroseconds(MIN_SIGNAL); Serial.println("SECİM 1 yapildi");
//
//
// Serial.println("secim icin bir tusa basın");temp=  Serial.read();
//
//       
//  while (!Serial.available());Serial.read();motor.writeMicroseconds(MAX_SIGNAL);  Serial.println("SECİM 2 yapildi ");
//
// Serial.println("secim icin bir tusa basın");temp=  Serial.read();
// while (!Serial.available());Serial.read();motor.writeMicroseconds(MIN_SIGNAL);   Serial.println("SECİM 3 yapildi");
//
// Serial.println("secim icin bir tusa basın");temp=  Serial.read();
// while (!Serial.available());Serial.read();motor.writeMicroseconds(MAX_SIGNAL);   Serial.println("SECİM 4 yapildi");


}

void loop() {  

}
