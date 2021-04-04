#include <LenBucUart.h>
#include <Arduino.h>

/*
  Prototypes for private functions
*/
void LBDebug(char debugStr[], unsigned long dbgValue);
void LBPrepareToTalk(void);
void LBPrepareToListen(void);
void LBStop(void);
void LBToggleDebug(void);
void LBNextReadBufferPos(void);

/*
  Private variables
*/
const byte lbRxPin = PIND3; // Used because it is attached to interrupt INT1
const byte lbRxPinMsk = 1 << PIND3;
const byte lbTxPin = PIND4;
const byte lbTxPinMsk = 1 << PIND4;
const byte lbWriteBufferSize = 32; // Amount of bytes for the write buffer.
const byte lbReadBufferSize = 32; // Amount of bytes for the read buffer.
const boolean lbDebugging = true;
volatile u32 lbBaudRate = 9600;
volatile bool lbReady = false;
volatile byte lbWriteBuffer[lbWriteBufferSize]; // The write buffer.
volatile byte lbWriteBufferPos = 0; // The current position of the write buffer.
volatile byte lbWriteBufferRemaining = 0; // How many bytes are left to be written.
volatile byte lbWriteNextBit = 0; // What bit is to send next.
volatile byte lbWritePositiveBits = 0; // Amount of positive bit that have been sent during the current package.
byte lbWriteTimerSpeed = 0; // Counter match value for OCR register.
volatile byte lbReadBuffer[lbReadBufferSize]; // The read buffer.
volatile byte lbReadBufferPos = 0; // The current position of the read buffer for the reading function. If lbReadBufferRemaining > 0 this position contains the first entry.
volatile byte lbReadBufferRemaining = 0; // How many bytes are left to be read.
volatile byte lbReadPacketCounter = 0; // The counter for the current packet being received.
volatile byte lbReadPositiveBits = 0; // Amount of positive bits that have been received.
volatile byte lbReadActiveCount = 0; // What index of the buffer array is currently being written to.
uint16_t lbReadTimerSpeed = 0; // Counter match value for OCR register.

void (*lbTimerFunctionP)(void); // Function pointer for the ISR TIMER2.

/*
  Public functions
*/
void LBBegin(int baudrate) {
  if (lbDebugging) {
    Serial.begin(9600);
  }
  LBStop(); // First stop everything, in case LBBegin is called more than once.

  lbBaudRate = baudrate;

  pinMode(7, OUTPUT); // Debug line.

  LBPrepareToTalk();
  LBPrepareToListen();

  lbReady = true;
}

void LBWrite(byte data) {
  if (!lbReady) {
    LBDebug("Cant write data because not ready. ", 0);
    return; // Cant write data if not ready.
  }
  if (lbWriteBufferRemaining >= lbWriteBufferSize) {
    LBDebug("Cant write data because buffer is full. ", lbWriteBufferRemaining);
    return; // Buffer is full.
  }
  cli();
  byte availablePosition = lbWriteBufferPos + lbWriteBufferRemaining;
  if (availablePosition >= lbWriteBufferSize) {
    availablePosition = availablePosition - lbWriteBufferSize;
  }
  lbWriteBuffer[availablePosition] = data;
  lbWriteBufferRemaining++;
  if (lbWriteBufferRemaining == 1) {
    TIMSK2 |= 0b10;  // Enable Timer2 Compare Match A Interrupt.
  }
  sei();
}

boolean LBAvailable(void) {
  return lbReadBufferRemaining > 0;
}

byte LBRead(void) {
  cli();
  byte data = lbReadBuffer[lbReadBufferPos];
  LBNextReadBufferPos();
  lbReadBufferRemaining--;
  sei();

  return data;
}

/*
  Private functions
*/
void LBDebug(char debugStr[], unsigned long debugValue) {
  if (lbDebugging) {
    Serial.print(debugStr);
    Serial.println(debugValue);
  }
}

void LBPrepareToTalk(void) {
  LBDebug("Preparing to talk..", 0);
  cli(); // Prevent interrupt from interfering during setup.
  pinMode(lbTxPin, OUTPUT);
  TIMSK2 &= (~0b11); // Disable Timer2 Compare Match A Interrupt.
  PORTD |= lbTxPinMsk; // Set pin to HIGH for idle.
  TCCR2A = 0b00000010; // Set to CTC mode so we can use counter matching.
  TCCR2B = 0; // Reset register
  TCCR2B |= 0b010; // Prescaler to 8.
  lbWriteTimerSpeed = (byte)(16000000L / (8 * lbBaudRate)) - 1; // Set counter match target. (16mhz / (prescaler * baudrate))
  OCR2A = TCNT2 + lbWriteTimerSpeed;
  sei(); // Enable interrupts.
}

void LBPrepareToListen(void) {
  LBDebug("Preparing to listen..", 0);
  // cli(); // Prevent interrupt from interfering during setup.
  // Setup pin interrupt for start bit.
  pinMode(lbRxPin, INPUT_PULLUP);
  EICRA &= (~0b1100); // Reset INT1 control register.
  EICRA |= 0b1000; // Set to interrupt on falling edge to indicate a start bit.
  EIMSK |= 0b10; // Enable interrupt for starting bit.
  // Setup timer for after the start bit.
  TIMSK1 &= (~0b11); // Disable timer 1.
  TCCR1A = 0; // Reset register.
  TCCR1B = 0; // Reset register.
  TCCR1B |= (1 << CS10); // Prescaler to 0.
  TCCR1B |= (1 << WGM12); // Set to CTC mode so we can use counter matching.
  lbReadTimerSpeed = (uint16_t)(16000000L / lbBaudRate / 2) - 30; // Set counter match target. (16mhz / (prescaler * baudrate) / 2).
  // The divide by 2 is so that the timer is twice as fast as required so we can 'Read' between the bits.
  OCR1A = TCNT1 + lbReadTimerSpeed;
  sei(); // Enable interrupts.
}

void LBStop(void) {
  // Stop the timer used for sending the bits on the Tx pin.
  TIMSK2 &= (~0b10);  // Disable Timer2 interrupt.

  // Disable interrupt INT1 of the Rx pin
  EIMSK &= (~0b10);

  lbReady = false;
}

// Debug function to toggle output pin 7 whenever I want to see if code is being executed.
void LBToggleDebug(void) {
  const byte pin7Msk = 0b10000000;
  if ((PORTD & pin7Msk) == pin7Msk) {
    PORTD &= (~pin7Msk);
  }
  else {
    PORTD |= pin7Msk;
  }
}

void LBNextReadBufferPos() {
  if (lbReadBufferPos == (lbReadBufferSize - 1)) {
    // Wrap around and go to 0 again.
    lbReadBufferPos = 0;
  }
  else {
    // Just increase the current position by one.
    lbReadBufferPos++;
  }
}

// ISR for detecting start bit
ISR(INT1_vect, ISR_NOBLOCK) {
  if (lbReadPacketCounter == 0) {
    // This is the start bit. Enable the timer so we can poll and save the incoming bits.
    lbReadPacketCounter = 1;
    lbReadPositiveBits = 0;
    cli();
    TCNT1 = 0;
    OCR1A = lbReadTimerSpeed;
    TIFR1 = (1 << OCF1A); // Clear flag.
    TIMSK1 |= 0b10; // Enable timer 1 interrupt.
    sei();

    byte availableBufferPos;

    if (lbReadBufferRemaining >= lbReadBufferSize) {
      // Handle full buffer, as in remove the oldest entry.
      availableBufferPos = lbReadBufferPos;
      LBNextReadBufferPos();
    }
    else {
      availableBufferPos = lbReadBufferPos + lbReadBufferRemaining;
      if (availableBufferPos > (lbReadBufferSize - 1)) {
        // Lets say the pos is 30 and remaining is 5. The the availablePos is 5, but until pos 3 is filled
        availableBufferPos -= lbReadBufferSize;
      }
    }
    
    lbReadActiveCount = availableBufferPos;
    lbReadBuffer[lbReadActiveCount] = 0;
  }
}

// ISR for receiving bit by bit
ISR(TIMER1_COMPA_vect, ISR_NOBLOCK) {
  // Counter values and what they represent:
  // 0 == waiting for startbit
  // 1 == between start bit and first data bit
  // 2 == expect this bit to be around when the first data bit changes.
  // 3 == in between data bit 0 and 1, so we save this state as the value of bit 0.
  // 4 == expect this bit to be around when the second data bit changes.
  // 5 == in between data bit 1 and 2, so we save this state as the value of bit 1.
  // 7 == data bit 2.
  // 9 == data bit 3.
  // 11 == data bit 4.
  // 13 == data bit 5.
  // 15 == data bit 6.
  // 17 == data bit 7.
  // 19 == parity bit.
  // 21 == stop bit 1.
  
  if (lbReadPacketCounter <= 2 || lbReadPacketCounter == 20) {
    // ignore these counts due to the start and stop bits.
  }
  else if (lbReadPacketCounter == 19) {
    // Calc parity bit and handle errors.
    volatile boolean measuredParityBit = ((PIND & lbRxPinMsk) == lbRxPinMsk);
    volatile boolean calculatedParity = ((lbReadPositiveBits % 2) == 0);
    if (measuredParityBit == calculatedParity) {
      // The parity bit is true whilst we already have even parity on the data bits. So a mismatch, discard the read byte.
      LBToggleDebug(); 
      LBToggleDebug(); 
    }
    else {
      // Reported parity matches calculated parity, approve the read byte.
      lbReadBufferRemaining++;
    }
  }
  else if (lbReadPacketCounter == 21) {
    // Disable timer and reset to wait for the next interrupt on the rx pin.
    cli();
    TIMSK1 &= (~0b10); // Disable timer 0 interrupt.
    sei();
    lbReadPacketCounter = 0;
    return;
  }
  else {
    if (lbReadPacketCounter % 2 == 0) {
      // Every even count should be ignored.
    }
    else {
      if ((PIND & lbRxPinMsk) == lbRxPinMsk) {
        // Pin is high.
        byte dataBitOffsetHelper = ((lbReadPacketCounter + 1) / 2) - 2;
        // Count 0 is now bit 0.
        // Count 2 is now bit 1.
        // Count 4 is now bit 2.
        // Etc...
        lbReadBuffer[lbReadActiveCount] |= (1 << dataBitOffsetHelper);
        lbReadPositiveBits++;
      }
    }
  }
  lbReadPacketCounter++;
}

// ISR for sending bit by bit
ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
  OCR2A = TCNT2 + lbWriteTimerSpeed; // If not set as the first thing once the ISR is called the timing gets messed up.
  // Bit 0 = start bit
  // Bit 1..8 = data bits
  // Bit 9 = parity bit
  // Bit 10..11 stop bits

  if (lbWriteBufferRemaining == 0) {
    return;
  }
  if (lbWriteNextBit == 0) {
    // Send start bit.
    PORTD &= (~lbTxPinMsk);
  }
  else if (lbWriteNextBit == 9) {
    // Send parity bit
    if (lbWritePositiveBits % 2 == 0) {
      // Parity bit is even.
      PORTD &= (~lbTxPinMsk);
    }
    else {
      // Parity bit is odd.
      PORTD |= lbTxPinMsk;
    }
    lbWritePositiveBits = 0;
  }
  else if (lbWriteNextBit == 10 || lbWriteNextBit == 11) {
    // Send stop bits
    PORTD |= lbTxPinMsk;
  }
  else if (lbWriteNextBit == 12) {
    // Handle final stuff
    lbWriteNextBit = 0;
    if (lbWriteBufferPos == lbWriteBufferSize - 1) {
      lbWriteBufferPos = 0;
    }
    else {
      lbWriteBufferPos++;
    }
    lbWriteBufferRemaining--;
    return;
  }
  else {
    // Send data bit
    byte nextBitMsk = 1 << (lbWriteNextBit - 1);
    if ((lbWriteBuffer[lbWriteBufferPos] & nextBitMsk) == nextBitMsk) {
      PORTD |= lbTxPinMsk;
      lbWritePositiveBits++;
    }
    else {
      PORTD &= (~lbTxPinMsk);
    }
  }
  lbWriteNextBit++;
}

