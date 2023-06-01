int led = 13;

void setup() {
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(led, HIGH);   
  delay(50);               
  Serial.println("blink...");
  digitalWrite(led, LOW);    
  delay(50);    
}           
