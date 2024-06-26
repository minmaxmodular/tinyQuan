
///WORKKKIING
#include <timer.h>
#include "STM32encoder.h"
STM32encoder enc(TIM2, 0x07);

//#include <avr/pgmspace.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "definitions.h"

//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define OLED_MOSI   PB3
#define OLED_CLK   PB4
#define OLED_DC    PB5
#define OLED_CS    PB6
#define OLED_RESET PB7
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels


//bool tuning_dac = false;
//bool tuning_adc = false;
bool direct_scale = false;
bool direct_root = true;
bool enc_scale_mode = false;

bool volatile refresh_little_indicator = false;

uint8_t volatile layout_index = 0;
bool volatile refresh_layout = false;

uint8_t pixel_loc = 0;

char key_buffer[10];
int16_t key0 = 0;

char short_scale_name_buffer[15];
uint8_t current_scale = 1; // Force refresh at start
uint16_t current_scale_mask = MAJOR;
uint8_t nb_notes_in_scale = 12;

uint8_t current_root_semitone = 1; // Force refresh at start

// in_scale_cv_mode = 0 ->  1 semitone / 83 mV  (non-constant CV difference to change note in scale)
//                  = 1 ->  1 note in_scale / 83 mV (constant CV difference to change note in scale
bool volatile in_scale_cv_mode = true;

uint8_t last_semitone_index = 0;
long int trigger_out_width = 10;
bool reset_trigger_out = false;
long int last_trigger_out_millis = millis();


/////////// ROTARY ENCODERS //////////

uint8_t volatile D10D11_state = 0b1111;
uint8_t volatile D6D7_state = 0b1111;

//int8_t volatile new_root = 0;
//int8_t volatile new_scale = 0;
int16_t enc_value = 0;
int16_t new_scale = 0;
int16_t new_root = 0;
bool volatile enc_mode = true;

HardwareTimer pwmtimer3(3);
//  HardwareTimer *pwmtimer3 = new HardwareTimer(TIM3); 
////////////////////////////////////////////////////////////////////////////

void setup() {

  pwmtimer3.pause();
  pwmtimer3.setPrescaleFactor(1);
  pwmtimer3.setOverflow(120);
  pwmtimer3.refresh();
  pwmtimer3.resume();

  //////// ROTARY ENCODER INTERRUPTS AND PINS ////////

  enc.setButton(PA2);					// set the button pin
  enc.attachButton(&change_layout_ISR);		// attach button isr. Must be called after setButton
  enc.attach(&change_root_ISR);
  enc.bind(&enc_value, 1, 0, NUM_SCALES);


  ////////////////////////////////////////////////////

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
 
 // display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    if(!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(2000);


  
  pinMode(TRIGGER_OUT_PIN, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);

//  pinMode(CHANGE_CV_MODE_PIN, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(CHANGE_CV_MODE_PIN),
//                  change_cv_mode_ISR, FALLING);

//  pinMode(CHANGE_LAYOUT_PIN, INPUT_PULLUP);
 // attachInterrupt(digitalPinToInterrupt(CHANGE_LAYOUT_PIN),
 //                 change_layout_ISR, FALLING);


  digitalWrite(TRIGGER_OUT_PIN, LOW);


  keyboard_layout = keyboard_layouts[layout_index];

//  if (tuning_dac) {
//    input_and_play_semitone();
//  }

//  if (tuning_adc) {
//    print_adc();
//  }

}

void loop() {
  ////////////////////////////////////
  ////////////////////////////////////
  ////////////////////////////////////
  // Here is where the magic happen //
  ////////////////////////////////////
  ////////////////////////////////////
  ////////////////////////////////////
  // Serial.println();
  // Serial.print("cv_to_quantize:");
  // Serial.print(cv_to_quantize);
  // Serial.print("  ");


  if (enc.isUpdated()) {
	Serial.printf("new_scale: %i\n", new_scale);
    if (enc_mode) {
      new_scale = enc_value;
      } else {
      new_root = enc_value;
    }

    if (new_root == NUM_NOTES) {
      new_root = 0;
    }
    if (new_root == -1) {
      new_root = NUM_NOTES - 1;
    }
    if (new_scale == NUM_SCALES) {
      new_scale = 0;
    }
    if (new_scale == -1) {
      new_scale = NUM_SCALES - 1;
    }
  }
  switch(enc.button()){     // read button and reset state
    case BTN_EVT_CLICK:
      enc_mode = !enc_mode;
      Serial.printf("new scale ID: %u\n", enc_mode);
    break;
  }


  uint8_t semitones_above_root_in_scale;

  uint16_t temp_scale_mask = current_scale_mask;

  uint8_t note_in_scale;
  uint8_t root_semitone;

//  int16_t cv_to_quantize = ADS.readADC(0);
    int16_t cv_to_quantize = 20000;

  //Serial.println(cv_to_quantize);

  if (reset_trigger_out && millis() - last_trigger_out_millis >= trigger_out_width) {
    digitalWrite(TRIGGER_OUT_PIN, LOW);
    reset_trigger_out = false;
  }

  if (in_scale_cv_mode) {
    // This doesn't work as well as I wanted,
    // sometimes the first note of an octave is
    // trigger_outed above a round volt probably due
    // to the tiny non-linearity of the ADC.
    note_in_scale = map(cv_to_quantize,
                        CV_0V_BOUNDARY_INCLUSIVE, CV_ABOVE_5V_BOUNDARY_EXCLUSIVE,
                        0, 5 * nb_notes_in_scale + 1);
    // 5V spans 5 octave



    semitones_above_root_in_scale = 0;
    // The upper boundary is a bit janky if root_semitone != 0, we'll constrain it later.
    for (uint8_t semitones_above_root = 0, note = 0; semitones_above_root < MAX_DAC_SEMITONE; semitones_above_root++) {

      if (bitRead(temp_scale_mask, 0)) {
        semitones_above_root_in_scale = semitones_above_root;
        note += 1;
        if (note >= note_in_scale) {
          break;
        }
      }

      temp_scale_mask = rotate12Right(temp_scale_mask, 1);

    }

  } else {
    semitones_above_root_in_scale = 0;

    // The upper boundary is a bit janky if root_semitone != 0, we'll constrain it later.
    for (uint8_t semitones_above_root = 0; semitones_above_root < MAX_DAC_SEMITONE; semitones_above_root++) {

      int16_t semitone_cv = (int16_t)pgm_read_word_near(inter_semitonecv_to_ADC16read + semitones_above_root);
      if (semitone_cv >= cv_to_quantize) {
        break;
      }

      if (bitRead(temp_scale_mask, 0)) {
        semitones_above_root_in_scale = semitones_above_root;
      }

      temp_scale_mask = rotate12Right(temp_scale_mask, 1);

    }
  }

  uint8_t semitone_index = current_root_semitone + semitones_above_root_in_scale;
//  semitone_index = min(semitone_index, MAX_DAC_SEMITONE - 1);
    semitone_index = semitone_index;
//  MCP.setValue((int16_t)pgm_read_word_near(semitone_cvs_dac + semitone_index));


  if (last_semitone_index != semitone_index) {
    digitalWrite(TRIGGER_OUT_PIN, HIGH);
    reset_trigger_out = true;

    last_semitone_index = semitone_index;
    last_trigger_out_millis = millis();
  }

  ////////////////////////////////////
  ////////////////////////////////////
  ////////////////////////////////////
  ////////////////////////////////////
  ////////////////////////////////////


  if (refresh_layout) {
    refresh_layout = false;
    keyboard_layout = keyboard_layouts[layout_index];
    drawKeyboard(current_scale, current_root_semitone, 0);
    display.display();
  }


  /////////////////////////////////////////////
  /////////////////////////////////////////////
  /////////////////////////////////////////////

  if ((new_root != current_root_semitone) || (new_scale != current_scale)) {

    //    if (new_root != current_root_semitone) {
    //      Serial.print("change key ");
    //      Serial.print(current_root_semitone);
    //      Serial.print(" -> ");
    //      Serial.println(new_root);
    //    }
    //    if (new_scale != current_scale) {
    //      Serial.print("change scale ");
    //      Serial.print(current_scale);
    //      Serial.print(" -> ");
    //      Serial.println(new_scale);
    //    }

    current_root_semitone = new_root;
    current_scale = new_scale;
    current_scale_mask = pgm_read_word_near(scales + current_scale);


    nb_notes_in_scale = 0;
    for (uint8_t i = 0; i < 12; i++) {
      nb_notes_in_scale += bitRead(current_scale_mask, i);
    }

    display.clearDisplay();

    printScale(current_scale, 2);

    drawKeyboard(current_scale, current_root_semitone, 0);

    drawLittleIndicator(current_scale);
    display.display();

  }

  if (refresh_little_indicator) {
    digitalWrite(LED_BUILTIN, in_scale_cv_mode);
    drawLittleIndicator(current_scale);
    display.display();
    refresh_little_indicator = false;
  }

}


///////////////////////////////////////////
///////////////// Graphics ////////////////
///////////////////////////////////////////

void drawLittleIndicator(uint8_t scale) {
  pixel_loc = map(scale, 0, NUM_SCALES - 1, 0, SCREEN_WIDTH - 8);
  //    display.fillRect(pixel_loc, 15, 8, 1, SSD1306_WHITE);
  if (in_scale_cv_mode) {
    display.fillRoundRect(pixel_loc, 16, 8, 6, 1, SSD1306_WHITE);
  } else {
    display.fillRoundRect(pixel_loc, 16, 8, 6, 1, SSD1306_BLACK);
    display.drawRoundRect(pixel_loc, 16, 8, 6, 1, SSD1306_WHITE);

  }

}

void printKey(uint8_t key, uint8_t text_size) {
  display.fillRect(0, 0, SCREEN_WIDTH / 2, 8 * text_size, SSD1306_BLACK);
  display.setTextSize(text_size);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  strcpy_P(key_buffer, (char *)pgm_read_word(&(notes[key])));
  display.println(key_buffer);
}



void printScale(uint16_t scale, uint8_t text_size) {
  display.fillRect(0, 16, SCREEN_WIDTH, 8 * text_size, SSD1306_BLACK);
  display.setTextSize(text_size);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);//16);
  strcpy_P(short_scale_name_buffer, (char *)pgm_read_word(&(short_scale_names[scale])));
  display.println(short_scale_name_buffer);
}

void drawKeyboard(int16_t scale, int8_t root, int8_t origin_key) {
  uint16_t on_keys = 0;

  static int16_t x0 = 0;
  //  static int16_t y0 = SCREEN_HEIGHT - PROP * KEYBOARD_LAYOUT_HEIGHT;
  static int16_t y0 = SCREEN_HEIGHT - PROP * KEYBOARD_LAYOUT_HEIGHT - 8;

  uint8_t root_dot_y_offset;

  display.fillRect(x0, y0, SCREEN_WIDTH, PROP * KEYBOARD_LAYOUT_HEIGHT, SSD1306_BLACK);

  static uint8_t keys_to_display = ceil(12.0 * float(SCREEN_WIDTH) / float(PROP * KEYBOARD_LAYOUT_WIDTH));

  //  uint16_t on_keys = scales[scale];
  on_keys = pgm_read_word_near(scales + scale);



  // Align with the key at origin of display
  if (root > origin_key) {
    on_keys = rotate12Left(on_keys, root - origin_key);
  }
  else if (root < origin_key) {
    on_keys = rotate12Right(on_keys, origin_key - root);
  }
  // Last nudge to print _k = -1
  on_keys = rotate12Left(on_keys, 1);


  //////////////////////////////////////////////
  ////////////// Draw white notes //////////////
  //////////////////////////////////////////////
  int8_t x_offset = -keyboard_layout[mod(origin_key - 1, 12)][1] / 2;
  for (int16_t _k = -1, k; _k < keys_to_display + 1; _k++) {

    k = mod(_k + origin_key, 12);

    if (keyboard_layout[k][0] == 0) {
      if (on_keys & 0b1) {
        display.fillRoundRect(x0 + PROP * x_offset,
                              y0 + PROP * keyboard_layout[k][2],
                              PROP * keyboard_layout[k][3], PROP * keyboard_layout[k][4],
                              keyboard_layout[k][5], SSD1306_WHITE);
      }
      else
      {
        display.drawRoundRect(x0 + PROP * x_offset,
                              y0 + PROP * keyboard_layout[k][2],
                              PROP * keyboard_layout[k][3], PROP * keyboard_layout[k][4],
                              keyboard_layout[k][5], SSD1306_WHITE);
      }


      // Draw root note marker
      if (k == current_root_semitone) {
        root_dot_y_offset = (layout_index == 0) ? (2 * keyboard_layout[k][4]) / 3 : (keyboard_layout[k][4] - ROOT_DOT_SIZE) / 2;
        display.fillRoundRect(x0 + PROP * (x_offset + keyboard_layout[k][3] / 2 - 1),
                              y0 + PROP * (keyboard_layout[k][2] + root_dot_y_offset),
                              PROP * ROOT_DOT_SIZE, PROP * ROOT_DOT_SIZE, 1,
                              SSD1306_BLACK);
      }

    }
    x_offset += keyboard_layout[k][1];
    on_keys = rotate12Right(on_keys, 1);
  }


  //////////////////
  on_keys = pgm_read_word_near(scales + scale);
  if (root > origin_key) {
    on_keys = rotate12Left(on_keys, root - origin_key);
  }
  else if (root < origin_key) {
    on_keys = rotate12Right(on_keys, origin_key - root);
  }
  on_keys = rotate12Left(on_keys, 1);
  //////////////////


  //////////////////////////////////////////////
  ////////////// Draw black notes //////////////
  //////////////////////////////////////////////
  x_offset = -keyboard_layout[mod(origin_key - 1, 12)][1] / 2;
  for (int8_t _k = -1, k; _k < keys_to_display + 1; _k++) {

    k = mod(_k + origin_key, 12);

    if (keyboard_layout[k][0] == 1) {
      if (on_keys & 0b1) {
        display.fillRoundRect(x0 + PROP * x_offset,
                              y0 + PROP * keyboard_layout[k][2],
                              PROP * keyboard_layout[k][3], PROP * keyboard_layout[k][4],
                              keyboard_layout[k][5], SSD1306_WHITE);

        // Draw a black line around black notes that are ON
        // only when the layout is keyboard because black notes
        // in the BSP layout do not overlap white notes
        if (layout_index == 0) {
          display.drawRoundRect(x0 + PROP * x_offset,
                                y0 + PROP * keyboard_layout[k][2],
                                PROP * keyboard_layout[k][3], PROP * keyboard_layout[k][4],
                                keyboard_layout[k][5], SSD1306_BLACK);
        }

      } else {
        display.fillRoundRect(x0 + PROP * x_offset,
                              y0 + PROP * keyboard_layout[k][2],
                              PROP * keyboard_layout[k][3], PROP * keyboard_layout[k][4],
                              keyboard_layout[k][5], SSD1306_BLACK);

        display.drawRoundRect(x0 + PROP * x_offset,
                              y0 + PROP * keyboard_layout[k][2],
                              PROP * keyboard_layout[k][3], PROP * keyboard_layout[k][4],
                              keyboard_layout[k][5], SSD1306_WHITE);
      }

      // Draw root note marker
      if (k == current_root_semitone) {
        root_dot_y_offset = (layout_index == 0) ? (2 * keyboard_layout[k][4]) / 3 : (keyboard_layout[k][4] - ROOT_DOT_SIZE) / 2;

        display.fillRoundRect(x0 + PROP * (x_offset + keyboard_layout[k][3] / 2 - 1),
                              y0 + PROP * (keyboard_layout[k][2] + root_dot_y_offset),
                              PROP * ROOT_DOT_SIZE, PROP * ROOT_DOT_SIZE, 1,
                              SSD1306_BLACK);

      }
    }

    x_offset += keyboard_layout[k][1];
    on_keys = rotate12Right(on_keys, 1);

  }

}

///////////////////////////////////////////
////////////// End graphics ///////////////
///////////////////////////////////////////

///////////// KEY OF ISR
///////////// KEY OF ISR
/*
void change_root_ISR() {
    directr = enc.dir();

    if (enc_scale_mode == 1) {
        if (directr == 1) {
          new_root += 1;
        } else if (directr == 0) {
          new_root -= 1;
        }
        if (new_root == NUM_NOTES) {
          new_root = 0;
        }
        if (new_root == -1) {
          new_root = NUM_NOTES - 1;
        }

        
 
    ) else if (enc_scale_mode == 0) {
        if (directr == 1) {
          new_scale += 1;
        } else if (directr == 0) {
          new_scale -= 1;
        }
        if (new_scale == NUM_SCALES) {
          new_scale = 0;
        }
        if (new_scale == -1) {
          new_scale = NUM_SCALES - 1;
        }
    }
)

*/

void change_root_ISR() {
//    direct_root = enc.dir();
/*
if (enc_scale_mode == true) {
  direct_root = enc.dir();
} else if (enc_scale_mode == false) {
  direct_scale = enc.dir();
}
*/
/*
  Serial.println(direct_root);
    Serial.println(direct_scale);
  if (direct_root == 1) {
    new_root += 1;
  } else if (direct_root == 0) {
    new_root -= 1;
  }
  if (new_root == NUM_NOTES) {
    new_root = 0;
  }
  if (new_root == -1) {
    new_root = NUM_NOTES - 1;
  }
*/
/*
    Serial.println(direct_scale);
  if (direct_scale == 1) {
    new_scale += 1;
  } else if (direct_scale == 0) {
    new_scale -= 1;
  }
*/

 // if (new_scale == NUM_SCALES) {
 //   new_scale = 0;
 // }
 // if (new_scale == -1) {
 //   new_scale = NUM_SCALES - 1;
 // }

}


/*
//////////// NEW SCALE ENCODER ISR
void change_scale_ISR() {
//    direct_scale = enc.dir();
if (enc_scale_mode == 1) {
  direct_scale = enc.dir();
}
    Serial.println(direct_scale);
  if (direct_scale == 1) {
    new_scale += 1;
  } else if (direct_scale == 0) {
    new_scale -= 1;
  }
  if (new_scale == NUM_SCALES) {
    new_scale = 0;
  }
  if (new_scale == -1) {
    new_scale = NUM_SCALES - 1;
  }
}
*/

void change_layout_ISR() {
//  layout_index = (layout_index + 1) % 2;
//  refresh_layout = true;
//  enc_mode = !enc_mode;
}

void change_cv_mode_ISR() {
  in_scale_cv_mode = !in_scale_cv_mode;
  refresh_little_indicator = true;
}



///////////// Utilities //////////////

int mod(int x, int m) {
  return (x % m + m) % m;
}

uint16_t rotate12Left(uint16_t n, uint16_t d) {
  return 0xfff & ((n << (d % 12)) | (n >> (12 - (d % 12))));
}

uint16_t rotate12Right(uint16_t n, uint16_t d) {
  return 0xfff & ((n >> (d % 12)) | (n << (12 - (d % 12))));
}


void print_adc() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("    Tuning");
  display.println("       ADC");
  display.println("dval/d83mV");
  display.display();
//  for (;;) {
//    Serial.println(ADS.readADC(0));
//  }
}


void input_and_play_semitone() {
  static String inData;

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("");
  display.println("    Tuning");
  display.println("       DAC");
  display.println("d83mV/semi");
  display.display();

  Serial.println("");
  for (;;) {
    char received = ' '; // Each character received
    inData = ""; // Clear recieved buffer
    Serial.print("DAC value (0-4095): ");

    while (received != '\n') { // When new line character is received (\n = LF, \r = CR)
      if (Serial.available() > 0) // When character in serial buffer read it
      {
        received = Serial.read();
        Serial.print(received); // Echo each received character, terminal dont need local echo
        inData += received; // Add received character to inData buffer
      }
    }
    inData.trim(); // Eliminate \n, \r, blank and other not “printable”
    Serial.println();
//    MCP.setValue(inData.toInt());

  }

}
