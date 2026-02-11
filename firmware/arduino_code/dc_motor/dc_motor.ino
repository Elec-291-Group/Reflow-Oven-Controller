const int dc_output_pin = 13;
const int dc_input_pin = 2;
void setup() {
  // put your setup code here, to run once:
  pinMode(dc_input_pin, INPUT);
  pinMode(dc_output_pin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  if(digitalRead(dc_input_pin)){
    digitalWrite(dc_output_pin, HIGH);
  }
  else{
    digitalWrite(dc_output_pin, LOW); 
  }

  delay(10);
}
