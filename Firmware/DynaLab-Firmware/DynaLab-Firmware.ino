#define VERSION_MAJOR 0
#define VERSION_MINOR 1

// Create version string using preprocessor stringification
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define VERSION_STRING TOSTRING(VERSION_MAJOR) "." TOSTRING(VERSION_MINOR)

#include <Arduino.h>
#include "Adafruit_TinyUSB.h"
#include "ramdisk.h"
#include "gpio.h"
#include "LedControl.h"

LedControl lc = LedControl(19, 18, 20, 1);

#define CAPTURE_TIMEOUT_MS 5000  // maximum possible recording duration
#define MOTION_TIMEOUT_MS 500    // timeout when mass has stopped moving
#define USB_DETACH_DELAY_MS 10
#define SERIAL_WAIT_MS 100
#define POST_CAPTURE_DELAY_MS 1000

// Volatile variables shared between ISR and main loop
volatile bool stateChanged = false;
volatile int positionCounter = 0;

// State tracking
uint8_t currentState = 0;
uint8_t previousState = 0;

unsigned long timestamp = 0;

Adafruit_USBD_MSC usb_msc;

// Data collection structure
#define MAX_DATAPOINTS 5000
struct DataPoint {
  uint32_t millisecond;
  int displacement_mm;
};

DataPoint timeseries[MAX_DATAPOINTS];
int datapoint_count = 0;

// Function to add datapoint
void addDataPoint(uint32_t ms, int disp) {
  if (datapoint_count < MAX_DATAPOINTS) {
    timeseries[datapoint_count].millisecond = ms;
    timeseries[datapoint_count].displacement_mm = disp;
    datapoint_count++;
  }
}


// Generate CSV content from timeseries data
void generateCSV() {
  uint32_t current_block = 4;  // Data blocks start at block 4 (Block0: Boot, Block1: FAT1, Block2: FAT2, Block3: Root Dir)
  uint32_t block_offset = 0;
  char line_buffer[20];  // Single line buffer

  // Write header
  uint32_t len = sprintf(line_buffer, "Time [us],Displacement [mm]\n");
  writeToBlocks(line_buffer, len, current_block, block_offset);

  // Write data rows
  for (int i = 0; i < datapoint_count; i++) {
    len = sprintf(line_buffer, "%lu,%d\n",
                  timeseries[i].millisecond,
                  timeseries[i].displacement_mm);
    writeToBlocks(line_buffer, len, current_block, block_offset);
  }

  // Calculate file metrics
  uint32_t file_size = (current_block - 4) * DISK_BLOCK_SIZE + block_offset;
  uint32_t blocks_needed = current_block - 4 + (block_offset > 0 ? 1 : 0);

  // Zero remainder of final block. Partial blocks must be zeroed to avoid garbage data.
  if (block_offset > 0) {
    memset(msc_disk[current_block] + block_offset, 0, DISK_BLOCK_SIZE - block_offset);
  }

  // Update BOTH FAT Tables (Windows requires both copies to match)
  for (int fat_copy = 0; fat_copy < 2; fat_copy++) {
    uint8_t* fat = msc_disk[1 + fat_copy];  // Block 1 (FAT1) and Block 2 (FAT2)

    // Chain logic:
    // if not last block: next_cluster = cluster + 1 (points to next block)
    // if last block: next_cluster = 0xFFF (EOF marker)
    for (uint32_t cluster = 2; cluster < 2 + blocks_needed; cluster++) {
      uint16_t next_cluster = (cluster < 2 + blocks_needed - 1) ? (cluster + 1) : 0xFFF;
      uint32_t byte_offset = (cluster * 3) / 2;

      // FAT12 Packing (12 bits per entry). Two entries = 3 bytes.
      if (cluster % 2 == 0) {
        fat[byte_offset] = next_cluster & 0xFF;
        fat[byte_offset + 1] = (fat[byte_offset + 1] & 0xF0) | ((next_cluster >> 8) & 0x0F);
      } else {
        fat[byte_offset] = (fat[byte_offset] & 0x0F) | ((next_cluster << 4) & 0xF0);
        fat[byte_offset + 1] = (next_cluster >> 4) & 0xFF;
      }
    }
  }

// Update Block3 directory entry
// Block3, starting byte 32 defines Data_.csv
// Bytes 37-39: Characters 6-8 of filename (currently spaces) - generate three random alphanumeric characters to append to this file (helps avoid file-overwrites when user copies data onto PC)
// Bytes 60-63: 4-bytes little-endian file size
#define ROOT_DIR_BLOCK 3
#define FILENAME_RANDOM_OFFSET 37  // Start of the 3 spaces in "DATA_   "
#define FILE_SIZE_OFFSET 60

  // Generate 3 random alphanumeric characters
  const char charset[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
  const int charset_size = sizeof(charset) - 1;  // Exclude null terminator
  randomSeed(micros());
  for (int i = 0; i < 3; i++) {
    msc_disk[ROOT_DIR_BLOCK][FILENAME_RANDOM_OFFSET + i] = charset[random(charset_size)];
  }

  // Update file size
  msc_disk[ROOT_DIR_BLOCK][FILE_SIZE_OFFSET] = file_size & 0xFF;
  msc_disk[ROOT_DIR_BLOCK][FILE_SIZE_OFFSET + 1] = (file_size >> 8) & 0xFF;
  msc_disk[ROOT_DIR_BLOCK][FILE_SIZE_OFFSET + 2] = (file_size >> 16) & 0xFF;
  msc_disk[ROOT_DIR_BLOCK][FILE_SIZE_OFFSET + 3] = (file_size >> 24) & 0xFF;
}

static void writeToBlocks(const char* data, uint32_t len,
                          uint32_t& current_block, uint32_t& block_offset) {
  for (uint32_t i = 0; i < len; i++) {
    msc_disk[current_block][block_offset++] = data[i];
    if (block_offset >= DISK_BLOCK_SIZE) {
      current_block++;
      block_offset = 0;
    }
  }
}

void updateBootSectorForDiskSize() {
  // Update total sectors in boot sector (bytes 19-20)
  msc_disk[0][19] = DISK_BLOCK_NUM & 0xFF;
  msc_disk[0][20] = (DISK_BLOCK_NUM >> 8) & 0xFF;

  // Calculate required FAT size (12 bits per entry, 8 entries per 3 bytes)
  uint32_t fat_entries = DISK_BLOCK_NUM + 2;  // +2 for reserved entries
  uint32_t fat_bytes = (fat_entries * 3 + 1) / 2;
  uint32_t sectors_per_fat = (fat_bytes + DISK_BLOCK_SIZE - 1) / DISK_BLOCK_SIZE;

  // Update sectors per FAT (bytes 22-23)
  msc_disk[0][22] = sectors_per_fat & 0xFF;
  msc_disk[0][23] = (sectors_per_fat >> 8) & 0xFF;
}


enum modes {
  INITIALISED,
  STANDBY,
  CAPTURE,
  MOUNT_DRIVE
};
modes mode = STANDBY;
modes last_mode = INITIALISED;

// the setup function runs once when you press reset or power the board
void setup() {
  // Manual begin() is required on core without built-in support e.g. mbed rp2040
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  Serial.begin(115200);
  static uint8_t serial_initialiser_counter = 0;
  while (!Serial && serial_initialiser_counter < 5) {
    delay(100);
    serial_initialiser_counter++;
  }

  gpio_initialise();

  // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  usb_msc.setID("DntPanic", "Mass Storage", VERSION_STRING);

  // Set disk size
  usb_msc.setCapacity(DISK_BLOCK_NUM, DISK_BLOCK_SIZE);
  updateBootSectorForDiskSize();
  // Set callback
  usb_msc.setReadWriteCallback(msc_read_callback, msc_write_callback, msc_flush_callback);
  usb_msc.setStartStopCallback(msc_start_stop_callback);
  usb_msc.setReadyCallback(msc_ready_callback);

  // Set Lun ready (RAM disk is always ready)
  usb_msc.setUnitReady(true);

  // 7-segment display
  lc.shutdown(0, false);  // wakeup
  lc.setIntensity(0, 8);  // medium brightness
  lc.clearDisplay(0);


  Serial.println("=== DynaLab Initialised ===");
}



void loop() {
  switch (mode) {
    case STANDBY:
      handleStandbyMode();
      break;
    case CAPTURE:
      handleCaptureMode();
      break;
    case MOUNT_DRIVE:
      handleMountDriveMode();
      break;
  }
}


void handleStandbyMode() {
  if (mode != last_mode) {
    Serial.println("Standing By");

    enableEncoderInterrupts();


    last_mode = mode;

    // throw away first reading - zeros the device and updates encoder-last-state
    processEncoderChange();
    positionCounter = 0;
  }
  if (digitalRead(btnRec) == 0) mode = CAPTURE;

  // Display live displacement
  processEncoderChange();
  long units = abs(positionCounter) % 10;
  long tens = (abs(positionCounter) / 10) % 10;
  lc.setDigit(0, 0, units, false);
  if (tens != 0) lc.setDigit(0, 1, tens, false);
  else lc.setChar(0, 1, ' ', false);

  if (positionCounter < 0) lc.setChar(0, 2, '-', false);
  else lc.setChar(0, 2, ' ', false);
}

void handleCaptureMode() {
  static unsigned long time_begin;
  static unsigned int num_points = 0;

  // Mode entry
  if (mode != last_mode) {
    Serial.println("Capturing");
    digitalWrite(ledRec, HIGH);

    // display "rEC" (recording)
    lc.setRow(0, 2, 0x05);         // r
    lc.setChar(0, 1, 'E', false);  // E
    lc.setRow(0, 0, 0x4E);         // c
    last_mode = mode;

    stateChanged = false;
    time_begin = micros();
  }
  static unsigned long lastMotion = 0;
  // Process state changes
  if (stateChanged) {
    num_points++;

    stateChanged = false;
    processEncoderChange();
    if (num_points == 1) time_begin = timestamp;  // first timestamp starts at zero

    lastMotion = millis();

    if (num_points > MAX_DATAPOINTS) {
      mode = MOUNT_DRIVE;
      return;
    }

    addDataPoint(timestamp - time_begin, positionCounter);
    Serial.printf("%8d, %4d\n", timestamp, positionCounter);
  }


  // Timeout check

  bool motionTimeout = (millis() - lastMotion > MOTION_TIMEOUT_MS) && num_points > 100;

  if (micros() - time_begin > 1000 * CAPTURE_TIMEOUT_MS || motionTimeout) {
    disableEncoderInterrupts();
    Serial.println("Done");
    digitalWrite(ledRec, LOW);
    mode = MOUNT_DRIVE;
  }
}

void handleMountDriveMode() {
  if (mode != last_mode) {
    last_mode = MOUNT_DRIVE;

    // Display 'USB' on 7-seg
    lc.setRow(0, 2, 0x3E);         // U
    lc.setDigit(0, 1, 5, false);   // S
    lc.setChar(0, 0, 'b', false);  // b

    generateCSV();
    usb_msc.begin();

    if (TinyUSBDevice.mounted()) {
      TinyUSBDevice.detach();
      delay(1000);  // Longer delay for Windows to forget the device
      TinyUSBDevice.attach();
    }
  }

#ifdef TINYUSB_NEED_POLLING_TASK
  TinyUSBDevice.task();
#endif
}



void encoderISR() {
  timestamp = micros();
  stateChanged = true;
}

// Process encoder state change and update position
void processEncoderChange() {
  currentState = readEncoderState();
  // Only process if state actually changed
  if (currentState != previousState) {
    int direction = getDirection(previousState, currentState);

    if (direction != 0) {
      // Disable interrupts briefly while updating position
      noInterrupts();
      positionCounter += direction;
      interrupts();
    }

    previousState = currentState;
  }
}

// Encoder state transition lookup table
// each state is encoded by the 3-bit value of the optical sensors.
// Index: [previous_state][current_state] -> direction
// Returns: +1 forward, -1 reverse, 0 invalid/no-change
const int8_t ENCODER_TRANSITION_TABLE[8][8] = {
  { 0, -1, 0, 0, 1, 0, 0, 0 },  // 0 (0b000) previous
  { 1, 0, 0, -1, 0, 0, 0, 0 },  // 1 (0b001)
  { 0, 0, 0, 0, 0, 0, 0, 0 },   // 2 (0b010) invalid
  { 0, 1, 0, 0, 0, 0, 0, -1 },  // 3 (0b011)
  { -1, 0, 0, 0, 0, 0, 1, 0 },  // 4 (0b100)
  { 0, 0, 0, 0, 0, 0, 0, 0 },   // 5 (0b101) invalid
  { 0, 0, 0, 0, -1, 0, 0, 1 },  // 6 (0b110)
  { 0, 0, 0, 1, 0, 0, -1, 0 },  // 7 (0b111)
};

// Simplified direction function - single lookup, no searching
int getDirection(uint8_t prevState, uint8_t currState) {
  // Bounds check for safety
  if (prevState > 7 || currState > 7) return 0;

  return ENCODER_TRANSITION_TABLE[prevState][currState];
}


// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
int32_t msc_read_callback(uint32_t lba, void* buffer, uint32_t bufsize) {
  uint8_t const* addr = msc_disk[lba];
  memcpy(buffer, addr, bufsize);

  return bufsize;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
int32_t msc_write_callback(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {
  uint8_t* addr = msc_disk[lba];
  memcpy(addr, buffer, bufsize);

  return bufsize;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
void msc_flush_callback(void) {
  // nothing to do
}

bool msc_start_stop_callback(uint8_t power_condition, bool start, bool load_eject) {
  Serial.printf("Start/Stop callback: power condition %u, start %u, load_eject %u\n", power_condition, start, load_eject);
  return true;
}

// Invoked when received Test Unit Ready command.
// return true allowing host to read/write this LUN e.g SD card inserted
bool msc_ready_callback(void) {
#ifdef BTN_EJECT
  // button not active --> medium ready
  return digitalRead(BTN_EJECT) != activeState;
#else
  return true;
#endif
}


void enableEncoderInterrupts() {
  attachInterrupt(digitalPinToInterrupt(chA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(chB), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(chC), encoderISR, CHANGE);
}

void disableEncoderInterrupts() {
  detachInterrupt(digitalPinToInterrupt(chA));
  detachInterrupt(digitalPinToInterrupt(chB));
  detachInterrupt(digitalPinToInterrupt(chC));
}