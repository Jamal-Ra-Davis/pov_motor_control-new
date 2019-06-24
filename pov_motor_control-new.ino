#define MOTOR_PIN 11
#define MOTOR_PIN_A 10
#define MOTOR_PIN_B 9
#define POT_PIN A0

#define N 10

int pot_vals[N];

int pot_val;
int motor_val;

unsigned long ms;
 
void setup()
{
  DDRB |= (1 << PB3);

  TCCR2A = (1 << COM2A1) | (1 << WGM22) | (1 << WGM21) | (1 << WGM20);
  TCCR2B = (1 << CS20);

  OCR2A = 0;

  
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



 
 
 OCR2A = 150;
 delay(1000);
 for (int i=150; i>=130; i--)
 {
  OCR2A = i;
  delay(50);
 }
 //return;
 ms = millis();
 pinMode(2, INPUT);
 attachInterrupt(digitalPinToInterrupt(2), getMS, RISING);
}
int targ = 33;//50;
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

float val = 150;

float Kp = 0.1;
void getMS()
{
  
  unsigned long ms_ = millis();
  long curr = ms_ - ms;
  if (curr < 10)
    return;
  Serial.print("Period (ms): ");
  Serial.println(curr);
  ms = ms_;

  int error = targ - curr;
  
  val += -1*Kp*error;
  if (val > 255)
    val = 255;
  if (val < 140)
    val = 140;
    
  Serial.print("Val: ");
  Serial.println(val);
  Serial.println();
  OCR2A = (uint8_t)(val+0.5);
  
  //Serial.println(ms_ - ms);
  //ms = ms_;
}

