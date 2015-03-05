// TODO make stop function to reset back to default settings

int pinTimer1 = 9; // 9 or 10
int pinTimer2 = 3; // 11 or 3


// variables for writing to timers

// volatile directs the compiler to load the variable form RAM
// as opposed to from a storage register.  Under certain conditions,
// the value for a variable stored in registers can be innacurate.
// Variable should be volatile whenever its value can be changed by 
// something beyond the control of the code section in which it appears.
// in arduino, the main association is with interupts.

// uint is a byte
// int32_t is a signed long integer
volatile int32_t timer1_toggle_count;
volatile uint8_t *timer1_pin_port;
volatile uint8_t timer1_pin_mask;
volatile int32_t timer2_toggle_count;
volatile uint8_t *timer2_pin_port;
volatile uint8_t timer2_pin_mask;


void setup() {
  begin_timer1();
  begin_timer2();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void begin_timer1() {
  // timer 1 - 16 bit timer
  TCCR1A = 0;
  TCCR1B = 0;
  
  // bitWrite(variable to write to, which bit to write, value to write 0 or 1)
  // WGM = Waveform Generation Mode - bits control overall mode of timer
  // CS = Clock Select - bits control clock prescaler
  
  bitWrite(TCCR1B, WGM12, 1); // mode
  bitWrite(TCCR1B, CS10, 1); // prescaler
  
  // initializes port to write to timer1
  timer1_pin_port = portOutputRegister(digitalPinToPort(pinTimer1));
  
  // initializes bit mask for timer1
  // bitmask is used to access specific bits of data
  timer1_pin_mask = digitalPinToBitMask(pinTimer1);
}
      
void begin_timer2() {
  // 8 timer 2 - bit timer
  TCCR2A = 0;
  TCCR2B = 0;
  
  bitWrite(TCCR2A, WGM21, 1); // mode
  bitWrite(TCCR2B, CS20, 1); // prescaler
  
  // initializes port to write to timer2
  timer2_pin_port = portOutputRegister(digitalPinToPort(pinTimer2));
  
  // initializes bit mask for timer2
  timer2_pin_mask = digitalPinToBitMask(pinTimer2);
}


// frequency (in hertz) and duration (in milliseconds).
void play_timer1(uint16_t frequency, uint32_t duration)
{
  // OCR - Output Compare Registers  - sets the level at which outputs A and B will be affected.
  // When the timer value matches the register value, the cooresponding output will be modified
  // as specified by the mode
  
  uint8_t prescalarbits = 0b001;
  int32_t toggle_count = 0;
  uint32_t ocr = 0;
  
  // Set the pinMode as OUTPUT
  pinMode(pinTimer1, OUTPUT);
    
  // TODO figure out what this ends of being and reduce code
  // two choices for the 16 bit timers: ck/1 or ck/64
  ocr = F_CPU / frequency / 2 - 1; // F_CPU returns clock speed
  prescalarbits = 0b001;
  if (ocr > 0xffff) {
    ocr = F_CPU / frequency / 2 / 64 - 1;
    prescalarbits = 0b011;
  }

  TCCR1B = (TCCR1B & 0b11111000) | prescalarbits;
    
  // Calculate the toggle count
  if (duration > 0) {
    toggle_count = 2 * frequency * duration / 1000;
  } else {
    toggle_count = -1;
  }

  // Set the OCR for the given timer,
  // set the toggle count,
  // then turn on the interrupts
  OCR1A = ocr;
  timer1_toggle_count = toggle_count;
  bitWrite(TIMSK1, OCIE1A, 1);
}
