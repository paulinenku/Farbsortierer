#include <Servo.h>



const int s0 = 2;
const int s1 = 3;
const int s2 = 4;
const int s3 = 5;
const int out = 6;

int rot = 0;
int gruen = 0;
int blau = 0;
// Define servo pins
int servoPinRot = 9;
int servoPinGruen = 10;
int servoPinBlau = 11;

// Declare Servo Objects
Servo ServoRot;
Servo ServoGruen;
Servo ServoBlau;


void setup() {
  Serial.begin(9600);   //Serielle Kommunikation starten
  pinMode(s0, OUTPUT);  //Die Kontakte des Farbsensors werden als Output oder...
  pinMode(s1, OUTPUT);  // (Warum als Output? Die Frequenzwerte (s0-s3) sollen im
  pinMode(s2, OUTPUT);  // seriellen Monitor ausgegeben werden.)
  pinMode(s3, OUTPUT);
  pinMode(out, INPUT);  // ...Input festgelegt

  digitalWrite(s0, HIGH);  //Die vier weißen LEDs am Farbsensor sollen leuchten
  digitalWrite(s1, HIGH);

  ServoRot.attach(servoPinRot);
  ServoGruen.attach(servoPinGruen);
  ServoBlau.attach(servoPinBlau);
}
void loop() {
  color();                        //Diese Funktion wird am Ende des Codes festgelegt (s.“void color();“)
  Serial.print(" Wert Rot: ");    //Auf dem seriellen Monitor soll jeweils „Wert“
  Serial.print(rot, DEC);         //mit der entsprechenden Farbe angezeigt
  Serial.print(" Wert Gruen: ");  //werden und dahinter der Wert, welcher in der
  Serial.print(gruen, DEC);       //void color(); Funktion ausgelesen wurde.
  Serial.print(" Wert Blau: ");
  Serial.print(blau, DEC);
  //Hier folgen die Befehle für die LEDs
  if (rot < blau && rot < gruen && rot < 20)  //Wenn der Filterwert für rot //kleiner ist als alle anderen Werte..
  {
    Serial.println(" – (Rote Farbe)");  //..soll “Rote Farbe“ am seriellen //Monitor angezeigt werden und..
    ServoRot.write(120);
    delay(1000);
    ServoRot.write(60);
    delay(1000);
    ServoRot.write(90);

  } else if (blau < rot && blau < gruen && rot > 25)  //Das gleiche bei Blau und Grün
  {
    Serial.println(" – (Blaue Farbe)");

    ServoBlau.write(60);
    delay(1000);
    ServoBlau.write(120);
    delay(1000);
    ServoBlau.write(90);

  } else if (gruen < rot && gruen < blau) {
    Serial.println(" – (Gruene Farbe)");

    ServoGruen.write(60);
    delay(1000);
    ServoGruen.write(120);
    delay(1000);
    ServoGruen.write(90);



  } else {             //Wenn keine Werte vorhanden sind..
    Serial.println();  //..nichts auf dem seriellen Monitor anzeigen und..
  }
  delay(300);
}
void color()  //Hier werden die Werte vom Farbsensor ausgelesen und unter den
//entsprechenden Variablen gespeichert
{
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  rot = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  digitalWrite(s3, HIGH);
  blau = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  digitalWrite(s2, HIGH);
  gruen = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
}