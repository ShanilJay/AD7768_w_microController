// Variables Required by SPISlave_T4
uint32_t spiRx[10];
volatile int spiRxIdx;
volatile int spiRxComplete = 0;

// ADC pins
const uint32_t DCLK         = 2;
const uint32_t DRDY         = 3;
const uint32_t DOUT0        = 4;
const uint32_t DRDY_for_RPi = 5;

// Code Variables
const int nChannels = 8;
const int nBytes    = 4;

// Variables
int bitCntr = 7;                        // Counter for number of bits we've received
int bytCntr = 0;                        // Counter for number of bytes we've received
int chnCntr = 0;                        // Counter for number of channels we've received
bool DRDY_flag = false;                 // Flag for knowing if we need to read DOUT pin
byte ADCDataBuffer[nChannels][nBytes];  // Matrix for storing data (each elem is one byte)
byte read_byte = 0x00;

// Initialise Teensy
#include "SPISlave_T4.h"
SPISlave_T4<&SPI, SPI_8_BITS> mySPI;

void setup() {

  // Wait until serial monitor initialised
  while (!Serial) {}

  Serial.print("START...");

  // Disable interrupts for setup
  cli();

  // For debugging
  Serial.begin(115200); //baudrate does not matter (is USB VCP anyway)

  // Set pins
  pinMode(DCLK, INPUT);
  pinMode(DRDY, INPUT);
  pinMode(DOUT0, INPUT);
  pinMode(DRDY_for_RPi, OUTPUT);

  // Start Pin High
  digitalWriteFast(DRDY_for_RPi, HIGH);

  // Configure Teensy as SPI Slave
  mySPI.begin();
  mySPI.swapPins(); 

  // Data ready interrupt (get ready to read data)
  attachInterrupt(digitalPinToInterrupt(DRDY), readyFunction, FALLING);

  // Data clock interrupt (start listening for clock signals)
  attachInterrupt(digitalPinToInterrupt(DCLK), pinReadFunction, FALLING);

  // Allow interrupts
  sei();
}


// // Function to read value of pin and once received 'n' channels worth of data send to RPi sending function
void pinReadFunction() {

  if(DRDY_flag){
    // Read value of data pin
    read_byte |= digitalReadFast(DOUT0) << bitCntr;
    bitCntr--;
    if (bitCntr == -1) {
      ADCDataBuffer[chnCntr][bytCntr] = read_byte;
      read_byte = 0x00;
      bitCntr = 7;
      bytCntr++;
      if (bytCntr == nBytes) {
        chnCntr++;
        bytCntr = 0;
      }
    }

    // If we are finished reading all channels, send signal to RPi 
    if (chnCntr == nChannels) {
      DRDY_flag = false;
      cli();
      digitalWriteFast(DRDY_for_RPi, LOW);

      // Loop through all channels
      for (int i = 0; i < nChannels; i++) {
        // Loop through all bytes stored for i-th channel
        for (int m = 0; m < nBytes; m++) {
          // Load into spi MISO register
          mySPI.pushr(ADCDataBuffer[i][m]);
          // Ensure data is read
          // while(!mySPI.available()){}
        }
      }
      chnCntr = 0;
      digitalWriteFast(DRDY_for_RPi, HIGH);
      sei();
    }
  }
}

// Function change DRDY flag once received interrupt on DRDY pin
void readyFunction() {
  DRDY_flag = true;
}

void loop() {
}
