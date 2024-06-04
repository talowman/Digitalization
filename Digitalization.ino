

int trigPin1 = 2;    // TRIG pin 
int echoPin1 = 3;    // ECHO pin

int trigPin2 = 8;    // TRIG pin
int echoPin2 = 9;    // ECHO pin

int trigPin3 = 12;    // TRIG pin
int echoPin3 = 13;    // ECHO pin

const float threshold = 8.0;

float duration_us1, duration_us2, duration_us3, distance_cm1, distance_cm2, distance_cm3;
float distance;


void setup() {
  // begin serial port
  Serial.begin (9600);

  // configure the trigger pin to output mode
  pinMode(trigPin1, OUTPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(trigPin3, OUTPUT);
  // configure the echo pin to input mode
  pinMode(echoPin1, INPUT);
  pinMode(echoPin2, INPUT);
  pinMode(echoPin3, INPUT);
}

float getDistance (int trig, int echo)  {
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  float duration = pulseIn(echo, HIGH);
  float distance = 0.017 * duration;
  return distance;
}

bool isWithinDistance(float distance, float threshold) {
  return distance <= threshold;
}

void loop() {
  //Serial.print("distance 1: ");
  //Serial.print(getDistance(trigPin1, echoPin1));
  //Serial.print(" cm, ");
  //Serial.print("distance 2: ");
  //Serial.print(getDistance(trigPin2, echoPin2));
  //Serial.print(" cm, ");
  //Serial.print("distance 3: ");
  //Serial.print(getDistance(trigPin3, echoPin3));
  //Serial.println(" cm");

  // Use a boolean variable to store the result
  bool withinThreshold = isWithinDistance(getDistance(trigPin1, echoPin1), threshold);
  //Serial.print("1 is being picked: ");
  Serial.print(withinThreshold);
  withinThreshold = isWithinDistance(getDistance(trigPin2, echoPin2), threshold);
  Serial.print(" ");
  //Serial.print("2 is being picked: ");
  Serial.print(withinThreshold);
  withinThreshold = isWithinDistance(getDistance(trigPin3, echoPin3), threshold);
  Serial.print(" ");
  //Serial.print("3 is being picked: ");
  Serial.println(withinThreshold);

  delay(500);
}
