// includes
#include <LiquidCrystal.h>

// Version string
#define VERSION_STR "V1.0"

// pin definitions
const int pumppin = 4;
const int valvepin = 3;
const int sensorpin = 0; //A0
const int onewireslavepin = 2; //this can either be D2 or D3 on the Arduino Nano as those are the only pins that can handle interrupts
const int lcd_rs = A2;
const int lcd_enable = A3;
const int lcd_d4 = A4;
const int lcd_d5 = A5;
const int lcd_d6 = 6; //D6
const int lcd_d7 = 7; //D7
const int valuepin = 10; //D10
const int statuspin = 11; //D11
const int triggerpin = 12; //D12

// LCD object creation
LiquidCrystal lcd(lcd_rs, lcd_enable, lcd_d4, lcd_d5, lcd_d6, lcd_d7);
int animation_counter = 0;

// time keeping variable(s)
unsigned long millis_last_update = 0;

// measurement values and their scaling
float kPa = 0;
#define HEIGTH_TO_ANALOG_SCALING 1.0
// The factor defined in HEIGTH_TO_ANALOG_SCALING controls how a heigth (in cm) maps to a PWM output value of the value pin.
// E.g. a factor of 1.0 means that 10cm result in 10% PWM and therefore in 10% * 3V3

// reference voltage stuff
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

float do_measure_kPa(bool dont_stop){
  //first we need to check that the pressure from any previous measurements is really and fully gone away
  //Therefore we
  // - check if the pump is off
  // - open the valve and wait some time
  unsigned int i;
  pump_off();
  valve_open();
  Serial.println("Pump off and valve open (to release pressure before measurement");
  for(i=0;i<15;i++){
    update_lcd(0.0,0,true);
    delay(500);
    Serial.print("Waiting for pressure to be released (");
    Serial.print(i);
    Serial.println(")");
  }
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
  Serial.println("Pump on and valve closed");
  while(!stop){
  	pump_on();
  	valve_close();
  	adc_value = analogRead(sensorpin);
  	if(adc_value > 972){ //972 = 95% of 1024
      Serial.println("ADC range limit reached");
      switch_reference(REF_5V);
  	  ref_value = 5000.0;
  	}
    if(delay_10ms_counter % 50 == 0){
      update_lcd(0.0,0,true);
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
  	  if(delta_mV < 10.0){
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
    if(delay_10ms_counter > 3000){
      //if we didn't get a valid result within 60 seconds exit this loop
      stop = true;
    }else{
      if(dont_stop){
        stop = false;
      }
    }
  }
  //now we know that we have reached a steady state, but the pump is still on and causing a lot of noise.
  pump_off();
  //we also need to wait a bit until the pump really is off and the noise is gone.
  delay(200);
  //Before the level starts to fall due to leakages we quickly measure the level
  current_mV = (float)(sensor_mean_read()/1023.0)*ref_value;
  valve_open();
  Serial.println("Pump off and valve open (to release pressure after measurement)");
  Serial.print("current_mV:");
  Serial.println(current_mV);
  Serial.print("Result:");
  Serial.println((current_mV - offset_mV)/90.0);
  millis_last_update = millis();
  valve_open();
  for(i=0;i<10;i++){
    update_lcd(0.0,0,true);
    delay(500);
    Serial.print("Waiting for pressure to be released (");
    Serial.print(i);
    Serial.println(")");
  }
  return (current_mV - offset_mV) / 90.0; //90.0 is the sensitivity of the sensor MPX5050 in mV/kPa
}

void setup() {
  switch_reference(REF_2V56);
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.print("Waterlevel Meas.");
  lcd.setCursor(0,1);
  lcd.print(VERSION_STR);
  Serial.begin(9600);
  pinMode(statuspin,OUTPUT);
  digitalWrite(statuspin, LOW);
  pinMode(triggerpin,INPUT);
  pinMode(pumppin,OUTPUT);
  pump_off();
  pinMode(valvepin,OUTPUT);
  valve_open();
  delay(100); //give the serial port some time to connect
  delay(2000); //give the user some time to look at the display
  //start the measurement
  kPa = do_measure_kPa(false);
  update_lcd(kPa*10.2, calc_hours_since_last_update(millis_last_update), false); //float heigth, int hours_since_last_update, bool meas_ongoing
}

void update_lcd(float heigth, int hours_since_last_update, bool meas_ongoing){
  //The LCD has 16x2 characters. The layout is as follows in idle state (when no measurement is ongoing):
  //>999cm, V1.0
  //last meas.:>99h
  //123456789ABCDEF -> 16 characters
  //When a measurement is ongoing the layout should be the following:
  // ---cm, V1.0
  //last meas.: * h
  //where * is a animation of the characters *,.,*,. and so on
  if(!meas_ongoing){
    animation_counter = 0; //reset the counter for the next animation
  }
  lcd.clear();
  lcd.setCursor(0,0);
  if(meas_ongoing){
    digitalWrite(statuspin,HIGH);
    analogWrite(valuepin,0);
    lcd.print(" ---cm, ");
    lcd.print(VERSION_STR);
    lcd.setCursor(0,1);
    lcd.print("last meas.: ");
    switch(animation_counter){
      case 0: lcd.print("*"); break;
      case 1: lcd.print("."); break;
      default: lcd.print(" "); break;
    }
    if(animation_counter++ > 1){
      animation_counter = 0;
    }
    lcd.print(" h");
  }else{
    if(heigth < 100){
      lcd.print(" ");
    }
    if(heigth < 10){
      lcd.print(" ");
    }
    if(heigth > 999){
      heigth = 999;
      lcd.print(">");
    }else{
      lcd.print(" ");
    }
    if(heigth < 0){
      heigth = 0;
    }
    lcd.print(heigth, 0);
    digitalWrite(statuspin,LOW);
    analogWrite(valuepin,(int)(heigth*HEIGTH_TO_ANALOG_SCALING));
    lcd.print("cm, ");
    lcd.print(VERSION_STR);
    lcd.setCursor(0,1);
    lcd.print("last meas.:");
    if(hours_since_last_update > 999){
      hours_since_last_update = 99;
      lcd.print(">");
    }else{
      lcd.print(" ");
    }
    if(hours_since_last_update < 0){
      hours_since_last_update = 0;
    }
    lcd.print(hours_since_last_update);
    lcd.print("h");
  }
  Serial.print("Height: ");
  Serial.print(heigth);
  Serial.print("cm, Last measurement done (>)");
  Serial.print(hours_since_last_update);
  Serial.print("h ago\n");
}

int calc_hours_since_last_update(unsigned long last_update){
  unsigned long now = millis();
  unsigned long delta = 0;
  delta = now - last_update;
  delta = (delta / 1000) / 3600;
  return (int)(delta);
}

void loop() {
  if(!digitalRead(triggerpin)){
    //trigger pin is low, check a few ms later again (debouncing)
    delay(200);
    if(!digitalRead(triggerpin)){
      //start the measurement
      kPa = do_measure_kPa(false);
    }
  }
  update_lcd(kPa*10.2, calc_hours_since_last_update(millis_last_update), false); //float heigth, int hours_since_last_update, bool meas_ongoing
  delay(50);
}
