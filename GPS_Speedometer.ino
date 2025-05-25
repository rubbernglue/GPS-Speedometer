#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <U8g2lib.h>
#include <Wire.h> // Required for I2C communication (LM75A and DS1307)
#include <RTClib.h> // Re-including for DS1307 RTC NVRAM storage

// === OLED Display (U8g2) ===
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
  U8G2_R1,     // rotation: Set to U8G2_R1 for portrait mode (90 degrees clockwise)
  /* reset=*/ 16, // Reset pin (or U8X8_PIN_NO_RESET if not used/connected)
  /* clock=*/ 5,  // SCL (D1)
  /* data=*/ 4    // SDA (D2)
);

// === GPS ===
#define GPS_RX 12  // GPS TX to ESP GPIO12 (D6)
#define GPS_TX 13  // GPS RX to ESP GPIO13 (D7)
#define GPS_BAUD 9600

TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

// === Timing ===
unsigned long lastGpsUpdate = 0;
const unsigned long gpsTimeout = 5000; // ms
float lastKnownSpeed = 0.0; // Stores the last valid speed

// Define a maximum plausible speed to filter out junk readings
const float MAX_VALID_SPEED_KMPH = 300.0; // Speeds above this will be treated as invalid

// === LM75A Temperature Sensor ===
#define LM75A_ADDRESS 0x48 // Default I2C address for LM75A

// Variables for temperature update timing
static unsigned long lastTempUpdateTime = 0;
float temperature = -999.9; // Initialize to an error value

// Define a reasonable temperature range for filtering junk values
const float MIN_VALID_TEMP_C = -30.0;
const float MAX_VALID_TEMP_C = 80.0;


// === Button Pins ===
#define BUTTON_SET 2     // Connected to GPIO 2 (D4)
#define BUTTON_MODE 14   // Connected to GPIO 14 (D5)

// Clock adjustment variables
volatile bool setButtonPressed = false;
volatile bool modeButtonPressed = false;
int clockAdjustMode = 0; // 0: normal, 1: adjust hour offset (increment), 2: adjust hour offset (decrement)

// Variables for time adjustment timeout
static unsigned long lastAdjustmentActivityTime = 0;
const unsigned long adjustmentTimeout = 5000; // 5 seconds for timeout

// --- GPS Time Offset Variables ---
static int timeOffsetHours = 0;   // Offset in hours from GPS (UTC) time

// --- RTC for NVRAM Storage ---
RTC_DS1307 rtc;
bool rtcFound = false; // Flag to track if RTC is initialized

// RTC NVRAM addresses for storing offset
const byte RTC_NVRAM_OFFSET_HOURS_ADDR = 0x08;
const byte RTC_NVRAM_MAGIC_ADDR = 0x09;
const byte RTC_NVRAM_MAGIC_VALUE = 0xA5; // A unique value to indicate initialized data

// Flag to signal offset needs saving (set in ISR, processed in loop)
volatile bool offsetNeedsSaving = false;


// Interrupt Service Routines for buttons
void IRAM_ATTR setButtonISR() {
  setButtonPressed = true;
  // If we are in an adjustment mode, set the flag to signal saving
  if (clockAdjustMode == 1 || clockAdjustMode == 2) {
    offsetNeedsSaving = true; 
  }
}

void IRAM_ATTR modeButtonISR() {
  modeButtonPressed = true;
}

// Function to read temperature from LM75A
float readTemperatureLM75A() {
  Wire.beginTransmission(LM75A_ADDRESS);
  Wire.write(0x00); // Temperature register
  Wire.endTransmission();

  Wire.requestFrom(LM75A_ADDRESS, 2); // Request 2 bytes
  if (Wire.available() == 2) {
    int rawTemp = Wire.read() << 8 | Wire.read(); // Combine two bytes
    rawTemp >>= 5; // Shift right by 5 bits to remove unused LSBs

    // Convert to Celsius (LM75A outputs 0.125 degrees per bit)
    return rawTemp * 0.125;
  }
  return -999.9; // Error value
}

void setup() {
  Serial.begin(115200); // Initialize Serial Monitor for debugging
  gpsSerial.begin(GPS_BAUD);

  Wire.begin(); // Initialize I2C for OLED, LM75A and RTC
   
  // --- RTC Initialization and Offset Loading ---
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC! Time offset will not be saved/loaded.");
    rtcFound = false;
  } else {
    Serial.println("RTC found. Checking for saved offset.");
    rtcFound = true;

    byte magicByte = rtc.readnvram(RTC_NVRAM_MAGIC_ADDR);
    if (magicByte == RTC_NVRAM_MAGIC_VALUE) {
      timeOffsetHours = rtc.readnvram(RTC_NVRAM_OFFSET_HOURS_ADDR);
      // Validate loaded offset to ensure it's within our expected range (-11 to +12)
      if (timeOffsetHours > 12 || timeOffsetHours < -11) {
        timeOffsetHours = 0; // Reset if invalid
        Serial.println("Loaded offset out of range, resetting to 0 and re-saving.");
        rtc.writenvram(RTC_NVRAM_OFFSET_HOURS_ADDR, (byte)timeOffsetHours);
        rtc.writenvram(RTC_NVRAM_MAGIC_ADDR, RTC_NVRAM_MAGIC_VALUE); // Re-write magic
      }
      Serial.print("Loaded time offset: ");
      Serial.println(timeOffsetHours);
    } else {
      // First boot or corrupted NVRAM, initialize
      timeOffsetHours = 0; // Default to UTC
      rtc.writenvram(RTC_NVRAM_OFFSET_HOURS_ADDR, (byte)timeOffsetHours);
      rtc.writenvram(RTC_NVRAM_MAGIC_ADDR, RTC_NVRAM_MAGIC_VALUE);
      Serial.println("No saved offset found, initialized to 0 and saved.");
    }
  }

  u8g2.begin();
  u8g2.setFontMode(0); // transparent mode - this means only foreground pixels are drawn

  pinMode(BUTTON_SET, INPUT_PULLUP);
  pinMode(BUTTON_MODE, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_SET), setButtonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_MODE), modeButtonISR, FALLING);

  // Initialize last activity time on boot, so it doesn't immediately timeout if in mode 0
  lastAdjustmentActivityTime = millis();
}

void loop() {
  // --- GPS Data Reading ---
  while (gpsSerial.available()) {
    if (gps.encode(gpsSerial.read())) {
      // Check for speed validity and filter out super high values
      if (gps.speed.isValid()) {
        float currentRawSpeed = gps.speed.kmph();
        if (currentRawSpeed <= MAX_VALID_SPEED_KMPH) { // Within reasonable limits
          lastKnownSpeed = currentRawSpeed;
        } else {
          // If speed is valid but suspiciously high (junk), set to 0.0 to indicate invalidity
          lastKnownSpeed = 0.0;
          Serial.print("GPS: Filtering out suspicious speed: ");
          Serial.println(currentRawSpeed);
        }
        lastGpsUpdate = millis(); // Still update `lastGpsUpdate` to keep `hasSignal` true
      }
      // If gps.speed.isValid() is false, `lastKnownSpeed` will hold its previous value,
      // and `hasSignal` will eventually turn false if no other valid data comes in.
    }
  }

  // Check if GPS has updated recently for general signal (speed and time)
  bool hasSignal = (millis() - lastGpsUpdate < gpsTimeout); 

  // --- Read Temperature (Updated only every 5 seconds) ---
  const unsigned long tempUpdateInterval = 5000; // 5 seconds
  if (millis() - lastTempUpdateTime >= tempUpdateInterval) {
    temperature = readTemperatureLM75A();
    lastTempUpdateTime = millis();
    Serial.print("Temperature Updated: "); // Debug print
    Serial.println(temperature);            // Debug print
  }

  // --- Handle Button Presses ---
  static unsigned long lastButtonPressTime = 0;
  const unsigned long debounceDelay = 100; // ms

  if (setButtonPressed) {
    setButtonPressed = false; // Reset the flag immediately
    if (millis() - lastButtonPressTime > debounceDelay) { // Debounce check
      Serial.print("SET Button Pressed. Current mode: "); Serial.println(clockAdjustMode); // DIAGNOSTIC
      
      if (clockAdjustMode == 0) { // From normal to adjust hour offset
        clockAdjustMode = 1; // Enter adjustment mode
        Serial.println("SET button pressed: Entered Offset Adjustment Mode.");
      } else if (clockAdjustMode == 1) { // In adjustment mode, SET button now DECREMENTS
        Serial.println("Decrementing hour offset."); // DIAGNOSTIC
        timeOffsetHours--;
        if (timeOffsetHours < -11) { // Cycle offset from -11 to +12
            timeOffsetHours = 12;
        }
        Serial.print("SET button pressed: Hour offset decremented. Current offset: "); 
        Serial.println(timeOffsetHours);
        offsetNeedsSaving = true; // Signal save on offset change
      }
      // Update last activity time when SET button is used for adjustment
      lastAdjustmentActivityTime = millis();
      lastButtonPressTime = millis(); // Update last press time after successful processing
    }
  }

  if (modeButtonPressed) {
    modeButtonPressed = false; // Reset the flag immediately
    if (millis() - lastButtonPressTime > debounceDelay) { // Debounce check
      Serial.print("MODE Button Pressed. Current mode: "); Serial.println(clockAdjustMode); // DIAGNOSTIC
      
      if (clockAdjustMode == 0) { // From normal to adjust hour offset
        clockAdjustMode = 1; // Enter adjustment mode
        Serial.println("MODE button pressed: Entered Offset Adjustment Mode.");
      } else if (clockAdjustMode == 1) { // In adjustment mode, MODE button now INCREMENTS
        Serial.println("Incrementing hour offset."); // DIAGNOSTIC
        timeOffsetHours++;
        if (timeOffsetHours > 12) { // Cycle offset from +12 to -11
            timeOffsetHours = -11;
        }
        Serial.print("MODE button pressed: Hour offset incremented. Current offset: "); 
        Serial.println(timeOffsetHours);
        offsetNeedsSaving = true; // Signal save on offset change
      }
      // Update last activity time when MODE button is used for adjustment
      lastAdjustmentActivityTime = millis(); // Update even if just cycling through non-zero modes
      lastButtonPressTime = millis(); // Update last press time after successful processing
    }
  }

  // --- Time Adjustment Timeout ---
  if (clockAdjustMode == 1) { // Only apply timeout if currently in adjustment mode (mode 1)
    if (millis() - lastAdjustmentActivityTime >= adjustmentTimeout) {
      Serial.println("Time adjustment timed out. Exiting adjustment mode."); // DIAGNOSTIC
      clockAdjustMode = 0; // Revert to normal mode
      // Ensure offset is saved if timeout occurred during adjustment
      offsetNeedsSaving = true; 
    }
  }

  // --- Save Offset to RTC NVRAM if changed and RTC is found ---
  // This is done in loop() to avoid blocking ISR
  if (rtcFound && offsetNeedsSaving) {
    rtc.writenvram(RTC_NVRAM_OFFSET_HOURS_ADDR, (byte)timeOffsetHours);
    // The magic byte doesn't need to be written repeatedly unless corruption is detected
    Serial.print("Saved time offset to RTC NVRAM: ");
    Serial.println(timeOffsetHours);
    offsetNeedsSaving = false; // Clear the flag
  }


  // --- Display on OLED ---
  u8g2.firstPage(); // Clears the entire display buffer to BLACK for a new frame
  do {
    // Speed in Top Half (64 pixels high, from Y=0 to Y=63)
    char speedStr[10];

    // Blinking logic for speed if no signal
    static unsigned long lastBlinkTime = 0;
    const unsigned long blinkInterval = 500; // ms (500ms visible, 500ms invisible)
    static bool blinkShouldBeVisible = true; // State for blinking

    if (!hasSignal) {
      if (millis() - lastBlinkTime > blinkInterval) {
        lastBlinkTime = millis();
        blinkShouldBeVisible = !blinkShouldBeVisible; // Toggle visibility
      }
    } else {
      blinkShouldBeVisible = true; // Always visible when signal is present
    }

    // Format speed: Aim for 3 characters, right-aligned
    if (lastKnownSpeed >= 100.0) { // Speeds 100 or more
        sprintf(speedStr, "%3d", (int)lastKnownSpeed); // "123"
    } else if (lastKnownSpeed >= 10.0) { // Speeds 10.0 to 99.9
        sprintf(speedStr, "%3d", (int)lastKnownSpeed); // " 12", " 99" (pads with leading space)
    } else { // Speeds 0.0 to 9.9
        sprintf(speedStr, "%3d", (int)lastKnownSpeed); // "  5", "  0" (pads with leading spaces)
    }
    
    u8g2.setFont(u8g2_font_7_Seg_41x21_mn); 

    // --- Right-align the speed string ---
    int desiredRightEdge = u8g2.getWidth() - 2; // 2 pixels from the right edge
    int speedX = desiredRightEdge - u8g2.getStrWidth(speedStr);
    
    // --- Fix Vertical Alignment (Shifted 12 pixels higher) ---
    int fontHeight = u8g2.getFontAscent() - u8g2.getFontDescent(); 
    int speedY = (64 - fontHeight) / 2 + u8g2.getFontAscent() - 24; 
                                                                    
    // Always fill the entire top half of the screen with black.
    u8g2.setDrawColor(0); // Set draw color to black
    u8g2.drawBox(0, 0, u8g2.getWidth(), u8g2.getHeight() / 2); // Fill the entire top half (Y=0 to Y=63) with black.

    // ONLY draw the speed string in white if it should be visible
    if (blinkShouldBeVisible) {
      u8g2.setDrawColor(1); // Set draw color to white (foreground)
      u8g2.drawStr(speedX, speedY, speedStr); // Draw speed
    }

    u8g2.setDrawColor(1); // Reset draw color to white for subsequent drawings

    // Bottom Half: Clock and Temperature (starts from Y=64)
    // Clear the entire bottom half with black for a clean slate
    u8g2.setDrawColor(0);
    u8g2.drawBox(0, u8g2.getHeight() / 2, u8g2.getWidth(), u8g2.getHeight() / 2); // Fill bottom half
    u8g2.setDrawColor(1); // Reset to white for drawing content


    // Clock
    // Check if GPS time is valid AND we have a recent signal
    if (gps.time.isValid() && hasSignal) { 
      u8g2.setFont(u8g2_font_ncenB12_tn); // Smaller font for clock

      // --- Calculate Adjusted Time from GPS and Offset ---
      int gpsHour = gps.time.hour();
      int gpsMinute = gps.time.minute();

      int displayHour = gpsHour + timeOffsetHours;
      int displayMinute = gpsMinute; // Minute offset is no longer used for adjustment

      // Normalize minutes (robustness, even if no offset here)
      if (displayMinute >= 60) {
          displayHour += displayMinute / 60;
          displayMinute %= 60;
      } else if (displayMinute < 0) {
          displayMinute = (displayMinute % 60 + 60) % 60;
          displayHour--; 
      }

      // Normalize hours
      if (displayHour >= 24) {
          displayHour %= 24;
      } else if (displayHour < 0) {
          displayHour = (displayHour % 24 + 24) % 24;
      }

      char timeStr[6]; // HH:MM\0
      sprintf(timeStr, "%02d:%02d", displayHour, displayMinute);

      int timeX = (u8g2.getWidth() - u8g2.getStrWidth(timeStr)) / 2;
      // Position for clock in bottom half (from Y=64 to Y=127)
      fontHeight = u8g2.getFontAscent() - u8g2.getFontDescent(); // Recalculate for current font
      int clockY = u8g2.getHeight()/2 + (64 - fontHeight) / 2 + u8g2.getFontAscent() - 2; // Adjusted Y slightly
      
      // --- Draw the UTC Offset string if in adjustment mode ---
      if (clockAdjustMode == 1) { // Only check for mode 1 now
          u8g2.setFont(u8g2_font_6x10_tf); // Smaller font for the message
          int symbolFontAscent = u8g2.getFontAscent(); 
          // Position relative to the top of the bottom half (Y=64)
          int symbolY = u8g2.getHeight() / 2 + symbolFontAscent + 3; // +3 for a small margin

          char offsetDisplayStr[15]; 
          if (timeOffsetHours >= 0) {
              sprintf(offsetDisplayStr, "UTC +%d", timeOffsetHours);
          } else {
              sprintf(offsetDisplayStr, "UTC %d", timeOffsetHours); 
          }
          int offsetStrWidth = u8g2.getStrWidth(offsetDisplayStr);
          int offsetDisplayX = (u8g2.getWidth() - offsetStrWidth) / 2;
          u8g2.setDrawColor(1); // Explicitly set to white for this string
          u8g2.drawStr(offsetDisplayX, symbolY, offsetDisplayStr);

          u8g2.setFont(u8g2_font_ncenB12_tn); // Revert to main clock font
      }

      // --- Draw the main time string (and highlight if in adjustment mode) ---
      u8g2.setDrawColor(0); // Set draw color to black
      if (clockAdjustMode == 1) { // Highlight only if in mode 1
          u8g2.drawBox(timeX, clockY - u8g2.getFontAscent(), u8g2.getStrWidth(timeStr), u8g2.getFontAscent() + 2); 
      }
      u8g2.setDrawColor(1); // Set draw color to white
      u8g2.drawStr(timeX, clockY, timeStr); // Always draw the time string
    } else {
      // If GPS time not valid or no recent signal, display a message instead of time
      u8g2.setFont(u8g2_font_6x13_tr);
      // Center "No GPS Time" messages in the bottom half
      u8g2.drawStr((u8g2.getWidth() - u8g2.getStrWidth("No GPS Time")) / 2, u8g2.getHeight()/2 + 30, "No GPS Time");
    }

    // Temperature (now only uses 6x13_tr)
    u8g2.setFont(u8g2_font_6x13_tr);

    char tempStr[10];
    // Check if the temperature is within a reasonable range AND not the error value
    if (temperature != -999.9 && temperature >= MIN_VALID_TEMP_C && temperature <= MAX_VALID_TEMP_C) {
      dtostrf(temperature, 4, 1, tempStr);
      strcat(tempStr, " C");
    } else {
      // Display a dash " - " followed by " C" for error value or out-of-range "junk" values
      strcpy(tempStr, " - C");
    }
    
    // Position temperature in the bottom right corner of the bottom half
    int tempY = u8g2.getHeight() - u8g2.getFontAscent() - 2; // Near bottom edge
    int tempX = u8g2.getWidth() - u8g2.getStrWidth(tempStr) - 5; // Align right, 5px margin
    u8g2.drawStr(tempX, tempY, tempStr);
  } while (u8g2.nextPage());

  delay(200); // update rate
}
