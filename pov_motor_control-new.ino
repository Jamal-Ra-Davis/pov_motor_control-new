#define MOTOR_PIN 9
#define MOTOR_PIN_A 10
#define MOTOR_PIN_B 11
#define POT_PIN A0

#define N 10

int pot_vals[N];

int pot_val;
int motor_val;

unsigned long ms;
unsigned long us;
 
void setup()
{
  /*
  DDRB |= (1 << PB3);

  TCCR2A = (1 << COM2A1) | (1 << WGM22) | (1 << WGM21) | (1 << WGM20);
  TCCR2B = (1 << CS20);

  OCR2A = 0;
  */
  //Setup timer1 for pwm
  DDRB |= (1 << PB1);
  TCCR1A = (1 << COM1A1) | (1 << WGM12) | (1 << WGM11) | (1 << WGM10);
  TCCR1B = (1 << CS10);
  OCR1A = 0;

  
 Serial.begin(9600);
 //pinMode(MOTOR_PIN, OUTPUT);
 pinMode(MOTOR_PIN_A, OUTPUT);
 pinMode(MOTOR_PIN_B, OUTPUT);
 pinMode(POT_PIN, INPUT);  

 pinMode(6, OUTPUT);
 


 digitalWrite(MOTOR_PIN_A, HIGH);
 digitalWrite(MOTOR_PIN_B, LOW);
 
 digitalWrite(6, HIGH);

 for (int i=0; i<N; i++)
  pot_vals[i] = 0;



 
 
 //OCR2A = 150;
 OCR1A = 800;
 delay(1000);
 for (int i=150; i>=130; i--)
 {
  OCR2A = i;
  delay(50);
 }
 //return;
 ms = millis();
 pinMode(2, INPUT);
 attachInterrupt(digitalPinToInterrupt(2), getMS, FALLING);
}
int targ = 34;//50;
int targ_us = 33400;
void loop()
{
 /*
 if (Serial.available() > 0)
 {
  int targ_ = Serial.parseInt();
  if (targ_ >= 33 && targ_ )
  {
    
  }
 }
 */
 return;
 pot_val = 0;
 for (int i=N-1; i>=1; i--)
 {
  pot_vals[i] = pot_vals[i-1];
  pot_val += pot_vals[i]; 
 }
 pot_vals[0] = analogRead(POT_PIN);
 pot_val += pot_vals[0]; 
 pot_val /= N;

 motor_val = map(pot_val, 0, 1023, 0, 255);
 //Serial.println(motor_val/*pot_vals[0]*/);
 OCR2A = motor_val;
  delay(20);
}

float val = 800;//150;

float Kp = 0.001;
void getMS()
{
  
  unsigned long ms_ = millis();
  long curr = ms_ - ms;
  
  unsigned long us_ = micros();
  long curr_us = us_ - us;
  if (curr < 10)
    return;
  Serial.print("Period (us): ");
  Serial.println(curr_us);
  ms = ms_;
  us = us_;

  //int error = targ - curr;
  int error = targ_us - curr_us;
  
  val += -1*Kp*error;
  /*
  if (val > 255)
    val = 255;
  if (val < 140)
    val = 140;
  */
  if (val > 1023)
    val = 1023;
  if (val < 750)
    val = 750;
    
  //Serial.print("Val: ");
  //Serial.println(val);
  //Serial.println();
  //OCR2A = (uint8_t)(val+0.5);
  OCR1A = (uint16_t)(val+0.5);
  //Serial.println(ms_ - ms);
  //ms = ms_;
}

