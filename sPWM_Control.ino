/* ใช้ D8 D9 เป็น Out PUT
 * ใช้ A0 เป็น SW input
 * ใช้ D7 เป็น interrupt input
 * ใช้ A4 เป็น SDA
 * ใช้ A5 เป็น SCL
 * 
 * ใช้ D5 เป็นตัว Test ระบบ
 * ใช้ D13 เป็นตัวแสดงว่า PWM กำลังทำงาน
 * 
 */

#include <Wire.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,2); 

#include <avr/io.h>
#include <avr/interrupt.h>

 byte set_start_time = 0;
 byte run_pwm = 0;

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}


ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
 {
     if(set_start_time == 1){  ///set_start_time จะใช้เป็นตัวระบุว่าระบบได้หาจุดเริ่มต้นของ 50Hz เจอแล้วยัน หรือ ใช้เป็นตัว Reset ระบบ
      run_pwm = 1;              // run_pwm เป็นตัวแปรที่จะเป็นตัวกำหนกว่าระบบพร้อมที่จะผลิต PWM แล้วหรือยัง
     set_start_time = 0;
     digitalWrite(5,run_pwm);
     
     digitalWrite(4,digitalRead(7));
     }
     
     
     
 }   

//---------------------------sPWM------------------------

#define SinDivisions (200)// Sub divisions of sisusoidal wave.

static int microMHz = 16; // Micro clock frequency
static int freq = 50;     // Sinusoidal frequency
static long int period;   // Period of PWM in clock cycles.
static unsigned int lookUp[SinDivisions];
static char theTCCR1A = 0b10000010; //varible for TCCR1A

//---------------------------------------------------------


//----------------------------KEY SW-----------------------
#define BUTTON_ADC_PIN           A0  // A0 is the button ADC input
#define LCD_BACKLIGHT_PIN         3  // D3 controls LCD backlight
// ADC readings expected for the 5 buttons on the ADC input
#define RIGHT_10BIT_ADC           0  // right
#define UP_10BIT_ADC            145  // up
#define DOWN_10BIT_ADC          350  // down 329
#define LEFT_10BIT_ADC          505  // left
#define SELECT_10BIT_ADC        741  // right
#define BUTTONHYSTERESIS         10  // hysteresis for valid button sensing window
//return values for ReadButtons()
#define BUTTON_NONE               0  // 
#define BUTTON_RIGHT              1  // 
#define BUTTON_UP                 2  // 
#define BUTTON_DOWN               3  // 
#define BUTTON_LEFT               4  // 
#define BUTTON_SELECT             5  // 

/*--------------------------------------------------------------------------------------
  Variables
  --------------------------------------------------------------------------------------*/
byte buttonJustPressed  = false;         //this will be true after a ReadButtons() call if triggered
byte buttonJustReleased = false;         //this will be true after a ReadButtons() call if triggered
byte buttonWas          = BUTTON_NONE;   //used by ReadButtons() for detection of button events


void setup(void) {
  Serial.begin(9600);
  lcd.init();  
  lcd.backlight();
  lcd.print("Hello, world!");
  pinMode(13,OUTPUT);
  pinMode(5,OUTPUT);
  

    pciSetup(7); /// เปิด Interrupt fn.
  //----------------------------------

  //-----------------------------------------------------------------

  //--------------------Key SW --------------------------------------
  pinMode( BUTTON_ADC_PIN, INPUT );         //ensure A0 is an input
  digitalWrite( BUTTON_ADC_PIN, LOW );      //ensure pullup is off on A0
  //lcd backlight control

}

void loop(void) {

  //-----------------OLED-----------------------------
  unsigned int buttonVoltage;
  byte button = BUTTON_NONE;   // return no button pressed if the below checks don't write to btn

  button = ReadButtons();

  //------------------------------------------------
if(run_pwm ==1){
   double temp; //Double varible for <math.h> functions.

    period = microMHz * 1e6 / freq / SinDivisions; // Period of PWM in clock cycles

    for (int i = 0; i < SinDivisions / 2; i++) { // Generating the look up table.
      temp = sin(i * 2 * M_PI / SinDivisions) * period;
      lookUp[i] = (int)(temp + 0.5);     // Round to integer.
    }
    // Register initilisation, see datasheet for more detail.
    TCCR1A = theTCCR1A; // 0b10000010;
    /*10 clear on match, set at BOTTOM for compA.
      00 compB disconected initially, toggled later to clear on match, set at BOTTOM.
      00
      10 WGM1 1:0 for waveform 15.
    */
    TCCR1B = 0b00011001;
    /*000
      11 WGM1 3:2 for waveform 15.
      001 no prescale on the counter.
    */
    TIMSK1 = 0b00000001;
    /*0000000
      1 TOV1 Flag interrupt enable.
    */
    ICR1   = period;   /* Period for 16MHz crystal, for a switching frequency of 100KHz for 200 subdivisions per 50Hz sin wave cycle. */
    sei();             // Enable global interrupts.
    // Set outputs pins.
    DDRB = 0b00000110; // Set PB1 and PB2 as outputs.
}
    
  if (button == BUTTON_UP) {
        Serial.print("UP");
        set_start_time = 1;
        run_pwm=0;
  }


  if (button == BUTTON_DOWN) {
    Serial.print("DOWN");
    digitalWrite(5,0);
    digitalWrite(4,0);
  }



  if (button == BUTTON_RIGHT) {
    Serial.print("Right");
  }




  if (button == BUTTON_LEFT) {
    Serial.print("Left");
  }



  if (button == BUTTON_SELECT) {
    Serial.print("Select");
  }


  lcd.setCursor(2,1);
  lcd.print(run_pwm);


}


byte ReadButtons()
{
  unsigned int buttonVoltage;
  byte button = BUTTON_NONE;   // return no button pressed if the below checks don't write to btn

  //read the button ADC pin voltage
  buttonVoltage = analogRead( BUTTON_ADC_PIN );

  //sense if the voltage falls within valid voltage windows
  if ( buttonVoltage < ( RIGHT_10BIT_ADC + BUTTONHYSTERESIS ) )
  {
    button = BUTTON_RIGHT;
  }
  else if (   buttonVoltage >= ( UP_10BIT_ADC - BUTTONHYSTERESIS )
              && buttonVoltage <= ( UP_10BIT_ADC + BUTTONHYSTERESIS ) )
  {
    button = BUTTON_UP;
  }
  else if (   buttonVoltage >= ( DOWN_10BIT_ADC - BUTTONHYSTERESIS )
              && buttonVoltage <= ( DOWN_10BIT_ADC + BUTTONHYSTERESIS ) )
  {
    button = BUTTON_DOWN;
  }
  else if (   buttonVoltage >= ( LEFT_10BIT_ADC - BUTTONHYSTERESIS )
              && buttonVoltage <= ( LEFT_10BIT_ADC + BUTTONHYSTERESIS ) )
  {
    button = BUTTON_LEFT;
  }
  else if (   buttonVoltage >= ( SELECT_10BIT_ADC - BUTTONHYSTERESIS )
              && buttonVoltage <= ( SELECT_10BIT_ADC + BUTTONHYSTERESIS ) )
  {
    button = BUTTON_SELECT;
  }
  //handle button flags for just pressed and just released events
  if ( ( buttonWas == BUTTON_NONE ) && ( button != BUTTON_NONE ) )
  {
    //the button was just pressed, set buttonJustPressed, this can optionally be used to trigger a once-off action for a button press event
    //it's the duty of the receiver to clear these flags if it wants to detect a new button change event
    buttonJustPressed  = true;
    buttonJustReleased = false;
  }
  if ( ( buttonWas != BUTTON_NONE ) && ( button == BUTTON_NONE ) )
  {
    buttonJustPressed  = false;
    buttonJustReleased = true;
  }

  //save the latest button value, for change event detection next time round
  buttonWas = button;

  return ( button );
}



ISR(TIMER1_OVF_vect) {
  static int num;
  static int delay1;
  static char trig;

    if (delay1 == 1) { /*delay by one period because the high time loaded into OCR1A:B values are buffered but can be disconnected immediately by TCCR1A. */
      theTCCR1A ^= 0b10100000;// Toggle connect and disconnect of compare output A and B.
      TCCR1A = theTCCR1A;
      delay1 = 0;             // Reset delay1
    } else if (num >= SinDivisions / 2) {
      num = 0;                // Reset num
      delay1++;
    }
    // change duty-cycle every period.
    OCR1A = OCR1B = lookUp[num];
    num++;
  }


