// TODO: ramdisk definition mismatches the FAT16 block number
// Block0: 0x10, 0x00 (bytes 19-20): 16 total sectors = MISMATCH - should be 64 (0x40) to match DISK_BLOCK_NUM, current value wastes ~48KB


/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/
#include <Arduino.h>
#include "Adafruit_TinyUSB.h"
#include "ramdisk.h"
#include "gpio.h"
#include "LedControl.h"

LedControl lc=LedControl(19,18,20,1);

#define CAPTURE_TIMEOUT_MS 5000
#define USB_DETACH_DELAY_MS 10
#define SERIAL_WAIT_MS 100
#define POST_CAPTURE_DELAY_MS 1000

// Volatile variables shared between ISR and main loop
volatile bool stateChanged = false;
volatile long positionCounter = 0;

// State tracking
uint8_t currentState = 0;
uint8_t previousState = 0;

// Gray code state sequence (forward direction)
const uint8_t graySequence[6] = { 0b000, 0b100, 0b110, 0b111, 0b011, 0b001 };

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
  uint32_t current_block = 3;  // Data blocks start at block 4 (Block0: Boot sector, Block1: FAT table, Block3: Readme File, Block4: Data)
  uint32_t block_offset = 0;
  char line_buffer[20];  // Single line buffer
  
  // Helper lambda function to write data across blocks
  //   - copy one byte to disk
  //   - If block full (512 bytes): increment current_block, reset block_offset to 0
  auto writeToBlocks = [&](const char* data, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
      msc_disk[current_block][block_offset++] = data[i];
      if (block_offset >= DISK_BLOCK_SIZE) {
        current_block++;
        block_offset = 0;
      }
    }
  };
  
  // Write header
  uint32_t len = sprintf(line_buffer, "Time [ms],Displacement [mm]\n");
  writeToBlocks(line_buffer, len);
  
  // Write data rows
  for (int i = 0; i < datapoint_count; i++) {
    len = sprintf(line_buffer, "%lu,%d\n", 
                  timeseries[i].millisecond, 
                  timeseries[i].displacement_mm);
    writeToBlocks(line_buffer, len);
  }
  
  // Calculate file metrics
  uint32_t file_size = (current_block - 3) * DISK_BLOCK_SIZE + block_offset;
  uint32_t blocks_needed = current_block - 3 + (block_offset > 0 ? 1 : 0);
  
  // Zero remainder of final block. Partial blocks must be zeroed to avoid garbage data.
  if (block_offset > 0) {
    memset(msc_disk[current_block] + block_offset, 0, DISK_BLOCK_SIZE - block_offset);
  }
  
  // Update FAT Table
  uint8_t* fat = msc_disk[1];
  // for all blocks, Block3 and onwards...
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
  
  // Update file size in Block2 directory entry. Block2, starting byte 32 defines DataFile.csv
  // Bytes 60-63: 4-bytes little-endian file size.
  msc_disk[2][60] = file_size & 0xFF;
  msc_disk[2][61] = (file_size >> 8) & 0xFF;
  msc_disk[2][62] = (file_size >> 16) & 0xFF;
  msc_disk[2][63] = (file_size >> 24) & 0xFF;
}

void updateBootSectorForDiskSize() {
  // Update total sectors in boot sector (bytes 19-20)
  msc_disk[0][19] = DISK_BLOCK_NUM & 0xFF;
  msc_disk[0][20] = (DISK_BLOCK_NUM >> 8) & 0xFF;
  
  // Calculate required FAT size (12 bits per entry, 8 entries per 3 bytes)
  uint32_t fat_entries = DISK_BLOCK_NUM + 2; // +2 for reserved entries
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
  while (!Serial) delay(100);

  gpio_initialise();

  // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  usb_msc.setID("DntPanic", "Mass Storage", "1.0");

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
  lc.shutdown(0,false); // wakeup
  lc.setIntensity(0,8); // medium brightness
  lc.clearDisplay(0);


  Serial.println("=== DynaLab Initialised ===");
}



void loop() {
  switch(mode) {
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
  if (mode != last_mode){
    Serial.println("Standing By");

    attachInterrupt(digitalPinToInterrupt(chA), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(chB), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(chC), encoderISR, CHANGE);
    

    last_mode = mode;
  }
  if (digitalRead(btnRec) == 0) mode = CAPTURE;

  // Display live displacement
  processEncoderChange();
  long units = abs(positionCounter) % 10;
  long tens = (abs(positionCounter) / 10) % 10;
  lc.setDigit(0,0,units,false);
  if(tens != 0) lc.setDigit(0,1,tens,false);
  else lc.setChar(0,1,' ',false);

  if (positionCounter < 0) lc.setChar(0,2,'-',false);
  else lc.setChar(0,2,' ',false);
}

void handleCaptureMode() {
  static uint32_t time_begin;
  static unsigned int num_points = 0;
  
  // Mode entry
  if (mode != last_mode) {
    Serial.println("Capturing");
    digitalWrite(ledRec, HIGH);
    last_mode = mode;
    currentState = readEncoderState();
    previousState = currentState;
    time_begin = millis();
    num_points = 0;

  }
  
  // Process state changes
  if (stateChanged) {
    num_points++;
    stateChanged = false;
    processEncoderChange();
    
    if (num_points < MAX_DATAPOINTS) {
      addDataPoint(timestamp - time_begin, positionCounter);
      Serial.printf("%8d, %4d\n", timestamp, positionCounter);
    } else {
      mode = MOUNT_DRIVE;
    }
  }
  
  Serial.printf("%d  %d\n", millis(), time_begin);

  // Timeout check
  if (millis() - time_begin > CAPTURE_TIMEOUT_MS) {
    Serial.println("Done");
    delay(1000);
    disable_linear_encoder();
    digitalWrite(ledRec, LOW);
    mode = MOUNT_DRIVE;
  }
}

void handleMountDriveMode() {
  if (mode != last_mode) {
    last_mode = MOUNT_DRIVE;
    lc.setRow(0,2,0x3E);        // U
    lc.setDigit(0,1,5,false);   // S
    lc.setChar(0,0,'b',false);  // b

    generateCSV();
    usb_msc.begin();
    
    if (TinyUSBDevice.mounted()) {
      TinyUSBDevice.detach();
      delay(10);
      TinyUSBDevice.attach();
    }
  }
  
  #ifdef TINYUSB_NEED_POLLING_TASK
    TinyUSBDevice.task();
  #endif
}



void encoderISR() {
  timestamp = millis();
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


// Determine direction based on Gray code sequence
// Returns: +1 for forward, -1 for reverse, 0 for invalid/noise
int getDirection(uint8_t prevState, uint8_t currState) {
  // Find current and previous state indices in Gray sequence
  int prevIndex = findStateIndex(prevState);
  int currIndex = findStateIndex(currState);

  // Invalid states
  if (prevIndex == -1 || currIndex == -1) {
    return 0;
  }

  // Calculate direction based on sequence position
  int diff = currIndex - prevIndex;
  
  // Handle wraparound cases
  if (diff == 1 || diff == -5) {
    return 1;  // Forward
  } else if (diff == -1 || diff == 5) {
    return -1;  // Reverse
  } else if (diff == 0) {
    return 0;  // No change (noise/bounce)
  } else {
    // Multiple steps jumped - likely noise, ignore
    return 0;
  }
}

// Find index of state in Gray code sequence
int findStateIndex(uint8_t state) {
  for (int i = 0; i < 6; i++) {
    if (graySequence[i] == state) {
      return i;
    }
  }
  return -1;  // Invalid state
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
