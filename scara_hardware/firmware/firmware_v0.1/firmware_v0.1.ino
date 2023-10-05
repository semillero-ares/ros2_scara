/////// Dedo
#define DedoPulso  34
#define DedoDirec  35
#define DedoSensor 16

volatile unsigned int DedoCount     = 0;
volatile unsigned int DedoPeriod    = 0;
volatile long         DedoPosition  = 0;
volatile bool         DedoOut       = false;
volatile bool         DedoDir       = false;

/////// Muneca
#define MunecaPulso  32
#define MunecaDirec  33

volatile unsigned int MunecaCount     = 0;
volatile unsigned int MunecaPeriod    = 0;
volatile long         MunecaPosition  = 0;
volatile bool         MunecaOut       = false;
volatile bool         MunecaDir       = false;

/////// Codo
#define CodoPulso  30
#define CodoDirec  31
#define CodoSensor 17

volatile unsigned int CodoCount     = 0;
volatile unsigned int CodoPeriod    = 0;
volatile long         CodoPosition  = 0;
volatile bool         CodoOut       = false;
volatile bool         CodoDir       = false;

/////// Hombro
#define HombroPulso  7
#define HombroDirec  8
#define HombroSensor 9

volatile unsigned int HombroCount     = 0;
volatile unsigned int HombroPeriod    = 0;
volatile long         HombroPosition  = 0;
volatile bool         HombroOut       = false;
volatile bool         HombroDir       = false;


volatile unsigned long updateCount = 0;

/////////////////////////////////////////////////////////
void setup_timer2() { //set timer2 interrupt at 1kHz
  cli();

  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 1khz increments
  //OCR2A = 249;// = (16e6) / (1000*64) - 1 (must be <256)
  OCR2A = 99;// = (16e6) / (250*256) - 1 (must be <256)
  //OCR2A = 155;// = (16e6) / (100*1024) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Prescalers https://microcontrollerslab.com/arduino-timer-interrupts-tutorial/
  // CS = 001 for 1
  // CS = 010 for 8
  // CS = 011 for 32
  // CS = 100 for 64
  // CS = 101 for 128
  // CS = 110 for 256
  // CS = 111 for 1024
  //TCCR2B |= (1 << CS22);
  TCCR2B |= (1 << CS21);
  //TCCR2B |= (1 << CS20);
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  sei();//allow interrupts
}
ISR(TIMER2_COMPA_vect) {
  timed_loop();
}

void timed_loop() {

  updateCount++;

  // Dedo
  DedoCount++;
  if (DedoCount >= DedoPeriod && DedoPeriod > 0) {
    DedoOut = !DedoOut;
    DedoCount = 0;
    digitalWrite(DedoDirec, DedoDir);
    digitalWrite(DedoPulso, DedoOut);
    DedoPosition = DedoPosition + DedoDir * 1 - !DedoDir * 1;
  }

  // Muneca
  MunecaCount++;
  if (MunecaCount >= MunecaPeriod && MunecaPeriod > 0) {
    MunecaOut = !MunecaOut;
    MunecaCount = 0;
    digitalWrite(MunecaDirec, !MunecaDir);
    digitalWrite(MunecaPulso, MunecaOut);
    MunecaPosition = MunecaPosition + MunecaDir * 1 - !MunecaDir * 1;
  }

  // Codo
  CodoCount++;
  if (CodoCount >= CodoPeriod && CodoPeriod > 0) {
    CodoOut = !CodoOut;
    CodoCount = 0;
    digitalWrite(CodoDirec, !CodoDir);
    digitalWrite(CodoPulso, CodoOut);
    CodoPosition = CodoPosition + CodoDir * 1 - !CodoDir * 1;
  }

  // Hombro
  HombroCount++;
  if (HombroCount >= HombroPeriod && HombroPeriod > 0) {
    HombroOut = !HombroOut;
    HombroCount = 0;
    digitalWrite(HombroDirec, HombroDir);
    digitalWrite(HombroPulso, HombroOut);
    HombroPosition = HombroPosition + HombroDir * 1 - !HombroDir * 1;
  }

}
void setup_robot() {

  // Dedo /////////////////////////////
  pinMode(DedoSensor, INPUT);
  pinMode(DedoPulso, OUTPUT);
  pinMode(DedoDirec, OUTPUT);

  DedoPeriod = 1;
  DedoDir = true;
  while (!digitalRead(DedoSensor)) {}
  DedoPosition = 0;
  DedoPeriod = 0;

  // Muneca /////////////////////////////
  pinMode(MunecaPulso, OUTPUT);
  pinMode(MunecaDirec, OUTPUT);
  MunecaPosition = 0;
  MunecaPeriod = 0;

  // Codo /////////////////////////////
  pinMode(CodoSensor, INPUT);
  pinMode(CodoPulso, OUTPUT);
  pinMode(CodoDirec, OUTPUT);

  CodoPeriod = 10;
  CodoDir = true;
  while (!digitalRead(CodoSensor)) {}
  CodoPosition = 0;
  CodoPeriod = 0;

  // Hombro ////////////////////////////
  pinMode(HombroSensor, INPUT);
  pinMode(HombroPulso, OUTPUT);
  pinMode(HombroDirec, OUTPUT);

  HombroPeriod = 1;
  while (!digitalRead(HombroSensor)) {}
  HombroPosition = 0;
  HombroPeriod = 0;

}

void setup() {
  Serial.begin(115200);
  setup_timer2();
  setup_robot();
}

void loop() {

  if (updateCount > 1000) {
    Serial.print(HombroPosition);
    Serial.print(",");
    Serial.print(CodoPosition);
    Serial.print(",");
    Serial.print(MunecaPosition);
    Serial.print(",");
    Serial.print(DedoPosition);
    //Serial.print(",");
    Serial.println("");
    updateCount = 0;
  }
  actualizarParametros();

  if(digitalRead(HombroSensor)) HombroPosition = 0;
  if(digitalRead(CodoSensor)) CodoPosition = 0;
  if(digitalRead(DedoSensor)) DedoPosition = 0;

}

/////////////////////////////////////// COMUNICACION

String inputString;
char inputBuffer[256];
bool stringComplete = false;

void actualizarParametros()
{
  if (stringComplete) {

    inputString.toCharArray(inputBuffer,inputString.length()+1);
    char * part;
    part = strtok (inputBuffer,",");  HombroDir = atoi(part) > 0;     
    part = strtok (NULL,",");         HombroPeriod = atoi(part);     
    part = strtok (NULL,",");         CodoDir = atoi(part) > 0;     
    part = strtok (NULL,",");         CodoPeriod = atoi(part);     
    part = strtok (NULL,",");         MunecaDir = atoi(part) > 0;     
    part = strtok (NULL,",");         MunecaPeriod = atoi(part);     
    part = strtok (NULL,",");         DedoDir = atoi(part) > 0;     
    part = strtok (NULL,",");         DedoPeriod = atoi(part);     

    inputString = "";
    stringComplete = false;
  }
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
