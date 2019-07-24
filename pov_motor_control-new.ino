#define PWM_PIN 6
#define TARGET_US 33400

unsigned long us;
float Kp = 0.0005;

float val;

void setup()
{
  Serial.begin(115200);
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  
  //Setup timer1 for pwm
  DDRB |= (1 << PB1);
  TCCR1A = (1 << COM1A1) | (1 << WGM12) | (1 << WGM11) | (1 << WGM10);
  TCCR1B = (1 << CS10);
  OCR1A = 0;

  delay(1000);
  for (uint16_t i=0; i<350; i+=5)
  {
    val = i;
    OCR1A = (uint16_t)val;
    delay(20);
  }
  delay(1000);

  us = micros();
  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), getMS, FALLING);
}

int targ_us = 33400;
void loop()
{

}

void getMS()
{
  unsigned long us_ = micros();
  long curr_us = us_ - us;
  if (curr_us < 10000)
    return;
  us = us_;

  int error = targ_us - curr_us;
  
  val += -1*Kp*error;

  if (val > 1023)
    val = 1023;
  if (val < 300)
    val = 300;

  OCR1A = (uint16_t)(val+0.5);

  Serial.print(curr_us);
  Serial.print(",");
  Serial.println(val*100);
}

