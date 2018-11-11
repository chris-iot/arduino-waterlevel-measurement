const int pumppin = 4;
const int valvepin = 3;
const int sensorpin = 0; //A0
const int onewireslavepin = 2; //this can either be D2 or D3 on the Arduino Nano as those are the only pins that can handle interrupts
const int lcd_rs = A2;
const int lcd_enable = A3;
const int lcd_d4 = A4;
const int lcd_d5 = A5;
const int lcd_d6 = A6;
const int lcd_d7 = A7;

const int REF_2V56 = 0;
const int REF_5V  = 1;
void switch_reference(int ref){
  if(ref==REF_2V56){
    //With the Arduino Micro the "internal" reference voltage is 2.56 volts... and not 1.1 volts as stated in the docs.
    //http://forum.arduino.cc/index.php?topic=174688.0
    //TODO: As i just had a Micro around for testing this part (and the multipliers in do_measure_kPa) needs to be cleaned up when using a Arduino Nano...
    analogReference(INTERNAL);
    Serial.println("REF internal");
  }
  if(ref==REF_5V){
    analogReference(DEFAULT);
    Serial.println("REF default");
  }
  //As the first few readings after changing the reference voltage might not be accurate we do some dummy readings here
  analogRead(sensorpin);
  delay(10);
  analogRead(sensorpin);
  delay(10);
  analogRead(sensorpin);
  delay(10);
  analogRead(sensorpin);
  delay(10);
  analogRead(sensorpin);
  delay(10);
}

void pump_on(){
  digitalWrite(pumppin,HIGH);
}
void pump_off(){
  digitalWrite(pumppin,LOW);
}
void valve_close(){
  digitalWrite(valvepin,HIGH); //This assumes a "normally open" valve
}
void valve_open(){
  digitalWrite(valvepin,LOW);
}

int sensor_mean_read(){
  int val = 0;
  for(unsigned int i=0; i<10; i++){
	val += analogRead(sensorpin);
  }
  val = val/10;
  Serial.print("sensor_mean_read:");
  Serial.println(val);
  return val;
}

float do_measure_kPa(){
  //first we need to check that the pressure from any previous measurements is really and fully gone away
  //Therefore we
  // - check if the pump is off
  // - open the valve and wait some time
  pump_off();
  valve_open();
  delay(5000);
  //As previous measurements might needed to switch the reference voltage we now restore our default.
  switch_reference(REF_2V56);
  //next thing is to do some measurements for the normal atmosphere pressure. Normally this should result in 0V
  //but as each sensor has an individual offset (that might also depend on temperature, etc.) we need to measure
  //this offset.
  float offset_mV = 0; //this holds the offset voltage in milli volts
  for(unsigned int i=0;i<10;i++){
	offset_mV += (float)(analogRead(sensorpin));
  Serial.print("Offset (0..1023):");
  Serial.println(offset_mV);
	delay(10);
  }
  offset_mV = offset_mV / 1023.0;
  offset_mV = (offset_mV / 10.0) * 2560.0;
  Serial.print("Offset (mV)");
  Serial.println(offset_mV);
  //after that we can start the pump. During that we continuously monitor the sensor value. 2 things are relevant here:
  // - that we get the point in time when the value saturates, because this is when we need to switch off the pump
  //   The saturation is defined that the mean values between 2 measurements do not differ more then 20mV if those
  //   measurements were taken 500ms apart from each other.
  // - that we stay below 95% of the ADC range as otherwise the value might saturate because the ADC clips. If we
  //   exceed the 95% range the reference voltage must be switched to the next higher one.
  bool stop = false;
  int adc_value = 0;
  float ref_value = 2560.0;
  int delay_10ms_counter = 0;
  float last_value_mV = 0;
  float current_mV = 0;
  float delta_mV = 0;
  while(!stop){
  	pump_on();
  	valve_close();
  	adc_value = analogRead(sensorpin);
  	if(adc_value > 972){ //972 = 95% of 1024
      Serial.println("ADC range limit reached");
      switch_reference(REF_5V);
  	  ref_value = 5000.0;
  	}
  	if(delay_10ms_counter % 100 == 0){
      Serial.println("Measuring...");
  	  //50*10ms are done, so we take a new set of averaged values
  	  current_mV = (float)(sensor_mean_read()/1023.0)*ref_value;
      Serial.print("current (mV):");
      Serial.println(current_mV);
  	  //let's compare if the current value differs less then 20mV from the last value.
      Serial.print("last_value_mV:");
      Serial.println(last_value_mV);
  	  if(current_mV > last_value_mV){
    		delta_mV = current_mV - last_value_mV;
  	  }else{
    		delta_mV = last_value_mV - current_mV;
  	  }
  	  if(delta_mV < 20.0){
  		stop = true;
        Serial.println("Done with measuring");
  	  }else{
  		//overwrite the last with the current value
  		last_value_mV = current_mV;
  	  }
  	}
  	delay(10);
  	delay_10ms_counter++;
  	//Add a timeout here
    if(delay_10ms_counter > 2000){
      //if we didn't get a valid result within 20 seconds exit this loop
      stop = true;
    }
  }
  //now we know that we have reached a steady state, but the pump is still on and causing a lot of noise.
  pump_off();
  //we also need to wait a bit until the pump really is off and the noise is gone.
  delay(200);
  //Before the level starts to fall due to leakages we quickly measure the level
  current_mV = (float)(sensor_mean_read()/1023.0)*ref_value;
  valve_open();
  Serial.print("current_mV:");
  Serial.println(current_mV);
  Serial.print("Result:");
  Serial.println((current_mV - offset_mV)/90.0);
  return (current_mV - offset_mV) / 90.0; //90.0 is the sensitivity of the sensor MPX5050 in mV/kPa
}

void setup() {
  switch_reference(REF_2V56);
  Serial.begin(9600);
  pinMode(pumppin,OUTPUT);
  pump_off();
  pinMode(valvepin,OUTPUT);
  valve_open();
  delay(100); //give the serial port some time to connect
}

void loop() {
  //TODO: Add a task scheduler (e.g. https://github.com/arkhipenko/TaskScheduler) that calls do_measure_kPa if
  //requested by the user (e.g. via the OneWire Slave library https://gitea.youb.fr/youen/OneWireArduinoSlave)
  Serial.println(do_measure_kPa());
  delay(1000);
}
