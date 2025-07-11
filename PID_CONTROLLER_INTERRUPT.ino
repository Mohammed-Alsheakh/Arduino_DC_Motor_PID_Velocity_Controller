float kp = 0.2;
float ki = 0.0005;
float kd = 0.01;
unsigned long t, t_prev = 0, count_prev = 0;
const byte interruptPinA = 2, interruptPinB = 3;
const PWMPin = 11, DirPin1 = 12, DirPin2 = 13;
volatile long EncoderCount = 0;
volatile unsigned long count = 0;
float Theta, RPM, RPM_d, Theta_prev = 0.0, RPM_max = 400.0;
float inte, inte_prev = 0.0,  Vmin = -12.0;
float V = 0.1, e, e_prev = 0.0; Vmax = 12.0;
int dt;
#define pi 3.1416
#define pot A0

void ISR_EncoderA()                            {
  bool PinB = digitalRead(interruptPinB);
  bool PinA = digitalRead(interruptPinA);
  if (PinB == LOW)      {
    if (PinA == HIGH)
      EncoderCount++;
    else
      EncoderCount--;
                        }
  else                  {
    if (PinA == HIGH)
      EncoderCount--;
    else
      EncoderCount++;   }                      }

void ISR_EncoderB()                       {
  bool PinB = digitalRead(interruptPinA);
  bool PinA = digitalRead(interruptPinB);

  if (PinA == LOW)     {
    if (PinB == HIGH)
      EncoderCount--;
    else
      EncoderCount++;  }

  else                 {
    if (PinB == HIGH)
      EncoderCount++;
    else
      EncoderCount--;
                       }                  }

/*float sign(float x) {
  if (x > 0) {
    return 1;
  } else if (x < 0) {
    return -1;
  } else {
    return 0; }}*/

void WriteDriverVoltage(float V, float Vmax) {
  int PWMval = int(255 * abs(V) / Vmax);
  if (PWMval > 255)
    PWMval = 255;
  if (V > 0)                     {
    digitalWrite(DirPin1, HIGH);
    digitalWrite(DirPin2, LOW);  }
  else if (V < 0)                {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, HIGH); }
  else                           {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, LOW);  }
  analogWrite(PWMPin, PWMval);               }

void setup() {
  Serial.begin(115200);
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);
  //attachInterrupt((interruptPinA), ISR_EncoderA, CHANGE);
  attachInterrupt((interruptPinB), ISR_EncoderB, CHANGE);
  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);
  pinMode(pot,INPUT);

  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 12499; //Prescaler = 64
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11 | 1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

void loop()      {
  if (count > count_prev)    {
    t = millis();
    Theta = EncoderCount / 900.0;
    dt = (t - t_prev);
    //RPM_d = RPM_max * (sin(2 * pi * 0.005 * t / 1000.0))* 
    //sign(sin(2 * pi * 0.05 * t / 1000.0));
    RPM_d = analogRead(pot)*RPM_max/1023.0;
    if (t / 1000.0 > 100) 
      RPM_d = 0;
    RPM = (Theta - Theta_prev) / (dt / 1000.0) * 60.0;
    e = RPM_d - RPM;
    inte = inte_prev + (dt * (e + e_prev) / 2);
    V = kp * e + ki * inte + (kd * (e - e_prev) / dt) ;
    if (V > Vmax)         {
      V = Vmax;
      inte = inte_prev;   }
    if (V < Vmin)         {
      V = Vmin;
      inte = inte_prev;   }

    WriteDriverVoltage(V, Vmax);

    Serial.print(RPM_d); Serial.print(" \t  ");
    Serial.print(RPM); Serial.print(" \t  ");
    Serial.print(e); Serial.println(" \t  ");
    Theta_prev = Theta;
    count_prev = count;
    t_prev = t;
    inte_prev = inte;
    e_prev = e;  } }

ISR(TIMER1_COMPA_vect) {
  count++;
  //Serial.print(count * 0.05); Serial.print(" \t");
}