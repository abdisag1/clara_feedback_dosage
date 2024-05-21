const int pin3 = 3;
char buffer[32];
int delay_ms =100;
float flow_rate;
unsigned long lastStateChange = 0;
volatile float period = 0; // 2 seconds for a 0.5Hz frequency
void setup() {
  Serial.begin(9600);
  pinMode(pin3,OUTPUT);
  Serial.println("Enter the flow rate you want to generate a wave form and press enter");
  // put your setup code here, to run once:
}

void loop() {
  if (Serial.available()) {
    // Read the user input until the newline character
    String value_user = Serial.readString();
    flow_rate = value_user.toFloat(); 
    // delay_ms = 104/flow_rate*1000;
    period = (1/(4.8*flow_rate))*1000000;
    Serial.print("period: ");
    Serial.println(period);
    Serial.print("Flow rate is set to: ");
    Serial.print(flow_rate);
    Serial.println(" l/min ");

  }
  // put your main code here, to run repeatedly:
   unsigned long currentMillis = micros();

  if (currentMillis - lastStateChange >= period / 2) {
    digitalWrite(pin3, !digitalRead(pin3));
    lastStateChange = currentMillis;
  }
  
}
