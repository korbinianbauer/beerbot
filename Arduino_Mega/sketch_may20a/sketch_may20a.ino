// PWM - Testprogramm
//Code fuer Arduino
//Author Retian
//Version 1.0

#define potiPin 2 //Eingang Poti ist A2

int potiWert;

void setup() {
  Serial.begin(115200);
  //Löschen der Timer/Counter Control Register A und B

  TCCR1A = 0;
  TCCR1B = 0;

  //Modus Fast PWM-Mode 10 Bit einstellen
  TCCR1A |= (1 << WGM10) | (1 << WGM11);
  TCCR1B |= (1 << WGM12);

  //Vorteiler auf 1 setzen
  TCCR1B |= (1 << CS10);

  //Nichtinvertiertes PWM-Signal setzen
  TCCR1A |= (1 << COM1A1);

  //PWM-Pin 9 als Ausgang definieren
  DDRB |= (1 << DDB1);
}



void loop() {
  potiWert = 250; //Potiwert einlesen (Auflösung Analogeingang = 10 Bit)

  //Ist die Auflösung des PWM-Signals z.B 8 Bit, muss der potiWert angepasst werden:
  //potiWert = map(potiWert, 0, 1023, 0, 255);

  OCR1A = potiWert; // Setzen des Impuls-Pausenverhältnis

  Serial.println(potiWert);

  delay(15);
}

