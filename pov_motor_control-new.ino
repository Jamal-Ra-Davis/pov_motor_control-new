#define MOTOR_PIN 11
#define MOTOR_PIN_A 10
#define MOTOR_PIN_B 9
#define POT_PIN A0

#define N 10

int pot_vals[N];

int pot_val;
int motor_val;

void setup()
{
 Serial.begin(9600);
 pinMode(MOTOR_PIN, OUTPUT);
 pinMode(MOTOR_PIN_A, OUTPUT);
 pinMode(MOTOR_PIN_B, OUTPUT);
 pinMode(POT_PIN, INPUT);  


 digitalWrite(MOTOR_PIN_A, HIGH);
 digitalWrite(MOTOR_PIN_B, LOW);
 

 for (int i=0; i<N; i++)
  pot_vals[i] = 0;
}

void loop()
{
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
 Serial.println(motor_val/*pot_vals[0]*/);
 analogWrite(MOTOR_PIN, motor_val);
 delay(20);


 //pot_val = analogRead(POT_PIN);
 //int motor_val = map(pot_val, 0, 1023, 0, 255);
 //analogWrite(MOTOR_PIN, motor_val);
 //delay(25); 
}
