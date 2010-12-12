//example use of LCD4Bit_mod library

#include <LCD4Bit_mod.h> 
//create object to control an LCD.  
//number of lines in display=1
LCD4Bit_mod lcd = LCD4Bit_mod(2); 

//Key message
char msgs[5][15] = {"Right Key OK ", 
                    "RED          ", 
                    "BLUE         ", 
                    "Left Key OK  ", 
                    "SELECTED   " };
int  adc_key_val[5] ={30, 150, 360, 535, 760 };
int NUM_KEYS = 5;
int adc_key_in;
int key=-1;
int oldkey=-1;
int oldcommand=-1;
int selected_color = -1;
int serial_command;

void setup() { 
  pinMode(13, OUTPUT);  //we'll use the debug LED to output a heartbeat

  lcd.init();
  //optionally, now set up our application-specific display settings, overriding whatever the lcd did in lcd.init()
  //lcd.commandWrite(0x0F);//cursor on, display on, blink on.  (nasty!)
   lcd.clear();
  //lcd.printIn("KEYPAD testing..");
  lcd.printIn("Choose color ...");
  
  Serial.begin(9600);
    
}

void loop() {

  adc_key_in = analogRead(0);    // read the value from the sensor  
  digitalWrite(13, HIGH);  
  key = get_key(adc_key_in);		        // convert into key press
	
if(selected_color == -1) { // Color not selected yet
  if (key != oldkey)				    // if keypress is detected
  {
    delay(50);		// wait for debounce time
    adc_key_in = analogRead(0);    // read the value from the sensor  
    key = get_key(adc_key_in);		        // convert into key press
    if (key != oldkey)				
    {			
      oldkey = key;
      if (key >=0){
        if(key < 4){
          lcd.cursorTo(2, 0);  //line=2, x=0
  	  lcd.printIn(msgs[key]);
          oldcommand = key;
        }
        else {
          if(oldcommand == -1) {
          }
          else
          {
            lcd.cursorTo(2, 5);  //line=2, x=5
  	    lcd.printIn(msgs[key]);
            selected_color = oldcommand;  
          }        
        }
      }
    }
  }
}
else { // Color selected
  
}

if (Serial.available()) {

        serial_command = Serial.read();
        switch (serial_command) {
        case 'C':
            if(selected_color == 1) {// RED
              Serial.print('R');
            }
            else {
              if(selected_color == 2) { // BLUE
                Serial.print('B');
              }
              else {
                Serial.print('N');
              }
            }
            break;
        case 'B':
            break;
        }
          
}
  //delay(1000);
  digitalWrite(13, LOW);
  
}

// Convert ADC value to key number
int get_key(unsigned int input)
{
	int k;
    
	for (k = 0; k < NUM_KEYS; k++)
	{
		if (input < adc_key_val[k])
		{
                  return k;
                }
	}
    
        if (k >= NUM_KEYS)
            k = -1;     // No valid key pressed
    
    return k;
}
