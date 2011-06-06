/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * JBot wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.
 * ----------------------------------------------------------------------------
 */
/*****************************************************
Project : Maximus
Version : 1
Date : 10/12/2010
Author : JBot
Company :
Comments: Application to choose the color (RED or BLUE).


Program type : Application
Clock frequency : 16,00 MHz
 *****************************************************/

#include <LCD4Bit_mod.h>
#include "HL1606strip.h"

#include <NewSoftSerial.h>
#include <RogueMP3.h>
#include <RogueSD.h>

NewSoftSerial rmp3_serial(12, 13);
RogueMP3 rmp3(rmp3_serial);
RogueSD filecommands(rmp3_serial);

int numberOfSongs;
int lastSong;
char path[96];
 
const char *directory = "";

//create object to control an LCD.  
//number of lines in display=1
LCD4Bit_mod lcd = LCD4Bit_mod(2);

#define START_PIN  2

// use -any- 3 pins!
#define STRIP_D 13
#define STRIP_C 12
#define STRIP_L 3

//HL1606strip strip = HL1606strip(STRIP_D, STRIP_L, STRIP_C, 32);

//Key message
char msgs[5][15] = { "Right Key OK ",
    "RED          ",
    "BLUE         ",
    "Left Key OK  ",
    "SELECTED   "
};
int adc_key_val[5] = { 30, 150, 360, 535, 760 };

int NUM_KEYS = 5;
int adc_key_in;
int key = -1;
int oldkey = -1;
int oldcommand = -1;
int selected_color = -1;
int serial_command;

int start_robot = 0;

int check_complete = 0;



void setup()
{
    pinMode(13, OUTPUT);                                   //we'll use the debug LED to output a heartbeat

    lcd.init();
    //optionally, now set up our application-specific display settings, overriding whatever the lcd did in lcd.init()
    //lcd.commandWrite(0x0F);//cursor on, display on, blink on.  (nasty!)
    lcd.clear();
    //lcd.printIn("KEYPAD testing..");
    lcd.printIn("Choose color ...");

    Serial.begin(57600);

    rmp3_serial.begin(9600);

    rmp3.sync();

    filecommands.sync();
   
    // mix up our random number generator
    randomSeed(analogRead(0));
   
    // get the number of songs available
    strcpy(path, directory);
    strcat(path, "/");
    strcat(path, "*.mp3");  // we have to do this because the IDE thinks that "/*" needs to be terminated everywhere
   
    numberOfSongs = filecommands.filecount(path);
   
    lastSong = -1;


    rmp3.setvolume(0);

    rmp3.playfile("/goliath_online.mp3");
    //delay(3000);
    //rmp3.playfile("/levelone_diagnostics.mp3");
    //delay(3000);
    //rmp3.playfile("/checklist_completed.mp3");
    //rmp3.playfile("/01 Rock Dust Light Star.mp3");

    //delay(3000);

    //  chaseSingle(WHITE, 40);

/*
    analogWrite(9, 255);
    analogWrite(10, 0);
    analogWrite(11, 0);
  */
}

void loop()
{

    adc_key_in = analogRead(0);                            // read the value from the sensor  
    digitalWrite(13, HIGH);
    key = get_key(adc_key_in);                             // convert into key press

    if (selected_color == -1) {                            // Color not selected yet
        if (key != oldkey)                                 // if keypress is detected
        {
            delay(50);                                     // wait for debounce time
            adc_key_in = analogRead(0);                    // read the value from the sensor  
            key = get_key(adc_key_in);                     // convert into key press

            /*        if (key == 1) {
               colorWipe(RED, 30);
               } else if (key == 2) {
               colorWipe(BLUE, 30);
               }
             */
             
           if (key == 0) {
             playNextSong();
             while(1) {  
               int mp3_status = rmp3.getplaybackstatus();
 
                if (mp3_status == 'S')
                    playNextSong();

               delay(200);
             }
           }
             
            if (key != oldkey) {
                oldkey = key;
                if (key >= 0) {
                    if (key < 4) {
                        lcd.cursorTo(2, 0);                //line=2, x=0
                        lcd.printIn(msgs[key]);
                        oldcommand = key;
                    } else {
                        if (oldcommand == -1) {
                        } else {
                            lcd.cursorTo(2, 5);            //line=2, x=5
                            lcd.printIn(msgs[key]);
                            selected_color = oldcommand;
                            rmp3.playfile("/levelone_diagnostics.mp3");
                            /*if(selected_color == 1) {
                               analogWrite(9, 0);
                               analogWrite(10, 0);
                               analogWrite(11, 255);
                               }
                               else {
                               analogWrite(9, 0);
                               analogWrite(10, 255);
                               analogWrite(11, 0);  
                               } */
                        }
                    }
                }
            }
        }
    } else {                                               // Color selected

    }

    delay(10);
    if ((digitalRead(START_PIN) == 1) && (start_robot == 0)) {
        start_robot = 1;
        /*      if ((selected_color == 1) || (selected_color == 2)) {
           if (selected_color == 1) {
           chaseSingle(RED, 30);
           } else {
           chaseSingle(BLUE, 30);
           }
           }
         */
        rmp3.playfile("/pizza.mp3");
    } else {
        //start_robot = 0;
    }


    if (Serial.available()) {

        serial_command = Serial.read();
        switch (serial_command) {
        case 'C':
            if (selected_color == 1) {                     // RED
                Serial.print('R');
            } else {
                if (selected_color == 2) {                 // BLUE
                    Serial.print('B');
                } else {
                    Serial.print('N');
                }
            }
            break;
        case 'S':
            if (check_complete == 0) {
                rmp3.playfile("/checklist_completed.mp3");
                check_complete = 1;
            }
            if (start_robot == 1) {                        // START !!!
                Serial.print('O');
            } else {
                Serial.print('N');                         // DO NOT START !!!
            }
            break;

        case 'o':
            rmp3.playfile("/step_away.mp3");
            break;
        case 'e':
            rmp3.playfile("/smell_like_victory.mp3");
            break;
        case 'h':
            rmp3.playfile("/heavy_metal.mp3");
            break;

        }

    }
    //delay(1000);
    //digitalWrite(13, LOW);

}

// Convert ADC value to key number
int get_key(unsigned int input)
{
    int k;

    for (k = 0; k < NUM_KEYS; k++) {
        if (input < adc_key_val[k]) {
            return k;
        }
    }

    if (k >= NUM_KEYS)
        k = -1;                                            // No valid key pressed

    return k;
}

/*
// scroll a rainbow!
void rainbowParty(uint8_t wait)
{
    uint8_t i, j;

    for (i = 0; i < strip.numLEDs(); i += 6) {
        // initialize strip with 'rainbow' of colors
        strip.setLEDcolor(i, RED);
        strip.setLEDcolor(i + 1, YELLOW);
        strip.setLEDcolor(i + 2, GREEN);
        strip.setLEDcolor(i + 3, TEAL);
        strip.setLEDcolor(i + 4, BLUE);
        strip.setLEDcolor(i + 5, VIOLET);

    }
    strip.writeStrip();

    for (j = 0; j < strip.numLEDs(); j++) {

        // now set every LED to the *next* LED color (cycling)
        uint8_t savedcolor = strip.getLEDcolor(0);
        for (i = 1; i < strip.numLEDs(); i++) {
            strip.setLEDcolor(i - 1, strip.getLEDcolor(i));     // move the color back one.
        }
        // cycle the first LED back to the last one
        strip.setLEDcolor(strip.numLEDs() - 1, savedcolor);
        strip.writeStrip();
        delay(wait);
    }
}


// turn everything off (fill with BLACK)
void stripOff(void)
{
    // turn all LEDs off!
    for (uint8_t i = 0; i < strip.numLEDs(); i++) {
        strip.setLEDcolor(i, BLACK);
    }
    strip.writeStrip();
}

// have one LED 'chase' around the strip
void chaseSingle(uint8_t color, uint8_t wait)
{
    uint8_t i;

    // turn everything off
    for (i = 0; i < strip.numLEDs(); i++) {
        strip.setLEDcolor(i, BLACK);
    }

    for (i = 0; i < strip.numLEDs(); i++) {
        strip.setLEDcolor(i, color);
        if (i != 0) {
            // make the LED right before this one OFF
            strip.setLEDcolor(i - 1, BLACK);
        }
        strip.writeStrip();
        delay(wait);
    }
    // turn off the last LED before leaving
    strip.setLEDcolor(strip.numLEDs() - 1, BLACK);
}

// fill the entire strip, with a delay between each pixel for a 'wipe' effect
void colorWipe(uint8_t color, uint8_t wait)
{
    uint8_t i;

    for (i = 0; i < strip.numLEDs(); i++) {
        strip.setLEDcolor(i, color);
        strip.writeStrip();
        delay(wait);
    }
}
*/

void playNextSong()
{
  uint8_t s, i;
  char filename[80];
  char path[96];
  int nextSong = 0;
 
  if (numberOfSongs > 0)
  {
    // Select our next song randomly
    if (numberOfSongs > 2)
    {
      do
        nextSong = random(numberOfSongs);
      while (nextSong == lastSong);
    }
    else if (numberOfSongs == 2)
    {
      // we only have two songs
      if (lastSong == 0)
        nextSong = 1;
      else
        nextSong = 0;
    }
 
    // now, get our file name from file list
 
    filecommands.opendir(directory);
 
    for (i = 0; i <= nextSong; i++)
    {
      filecommands.readdir(filename, "*.mp3");
    }
 
    strcpy(path, directory);
    strcat(path, "/");
    strcat(path, filename);
 
    rmp3.playfile(path);
 
    Serial.print("Playing: ");
    Serial.println(path);
 
    lastSong = nextSong;
  }
  else
  {
    Serial.println("No files to play.");
  }
}

