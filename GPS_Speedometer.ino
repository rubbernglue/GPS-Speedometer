#include <TinyGPSPlus.h>
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

// --- ADDED: RTC Synchronization Variables ---
bool initialRTCSyncDone = false; // Flag to ensure RTC is synced at least once
unsigned long lastRTCSyncTime = 0;
const unsigned long rtcSyncInterval = 3600000; // Sync RTC every 1 hour (3600 * 1000 ms)


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
    Serial.println("Couldn't find RTC! Time offset will not be saved/loaded. RTC will not sync.");
    rtcFound = false;
  } else {
    Serial.println("RTC found. Checking for saved offset.");
    rtcFound = true;

    // --- MODIFIED: Removed rtc.lostPower() check as it's not a member of RTC_DS1307 in this RTClib version. ---
    // If your RTC DS1307 loses power and time is garbage, it will be corrected by GPS sync
    // once a valid GPS signal is acquired, or it will display "RTC Not Set".

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
    Serial.println(temperature);             // Debug print
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


    // --- Clock Display & RTC Sync ---
    u8g2.setFont(u8g2_font_ncenB12_tn); // Set font for time (and main temp part)
    DateTime displayDateTime; // This will hold the time to display
    
    // Check if RTC is actually running and has a plausible time (e.g., year > 2000)
    bool rtcTimePlausible = false;
    if(rtcFound && rtc.now().year() > 2000) { 
      rtcTimePlausible = true;
    }

    // --- ADDED: RTC Synchronization Logic ---
    if (gps.time.isValid() && hasSignal) { 
      // GPS time is valid: Get UTC from GPS, apply offset, then synchronize RTC
      DateTime gpsUtcDateTime(gps.date.year(), gps.date.month(), gps.date.day(), 
                              gps.time.hour(), gps.time.minute(), gps.time.second());
      
      // Calculate local time by adding the offset
      // Using unixtime() makes adding/subtracting hours easier and handles date/time rollovers
      displayDateTime = DateTime(gpsUtcDateTime.unixtime() + timeOffsetHours * 3600); 

      if (rtcFound) {
        // Sync if initial sync not done, OR interval passed, OR time difference is more than 60 seconds
        if (!initialRTCSyncDone || (millis() - lastRTCSyncTime >= rtcSyncInterval) || (abs((long)displayDateTime.unixtime() - (long)rtc.now().unixtime()) > 60) ) {
          rtc.adjust(displayDateTime); // Set RTC to calculated local time
          lastRTCSyncTime = millis();
          initialRTCSyncDone = true; // Mark that initial sync has occurred
          Serial.println("RTC adjusted to local GPS time."); // For debugging
        }
      }
    } 
    // --- END ADDED ---
    
    // Fallback to RTC if GPS is not valid OR if RTC is already running and plausible after initial sync
    // MODIFIED: Simplified conditions
    if (rtcTimePlausible && (!gps.time.isValid() || initialRTCSyncDone)) {
      displayDateTime = rtc.now(); // Use RTC time if plausible and GPS isn't valid, or if initially synced
    } else if (gps.time.isValid() && hasSignal && gps.date.isValid()) { // Ensure date is also valid for robust DateTime construction
      // If RTC is not plausible or found, but GPS is valid, rely on GPS time (already calculated above)
      // This path is primarily for the first few moments before RTC sync
      DateTime gpsUtcDateTime(gps.date.year(), gps.date.month(), gps.date.day(), 
                              gps.time.hour(), gps.time.minute(), gps.time.second());
      displayDateTime = DateTime(gpsUtcDateTime.unixtime() + timeOffsetHours * 3600);
    } else {
      // If neither GPS nor RTC is plausible/running, display a "No Time" message.
      // displayDateTime will remain at its default (e.g., year 0, which is handled by display check).
    }

    char timeStr[6]; // HH:MM\0
    // Only format timeStr if displayDateTime is valid (i.e., its year is plausible)
    if (displayDateTime.year() > 2000) { // Check for plausible year (e.g., not 0000)
        sprintf(timeStr, "%02d:%02d", displayDateTime.hour(), displayDateTime.minute());
    } else {
        strcpy(timeStr, "--:--"); // Display dashes if time is not valid
    }

    int timeX = (u8g2.getWidth() - u8g2.getStrWidth(timeStr)) / 2;
    // Position for clock in bottom half (from Y=64 to Y=127)
    // fontHeight = u8g2.getFontAscent() - u8g2.getFontDescent(); // Recalculate for current font, already done for speed
    // MODIFIED: Adjusted Y positioning for better centering with temperature
    int clockY = u8g2.getHeight()/2 + (64 - (u8g2.getFontAscent() - u8g2.getFontDescent())) / 2 + u8g2.getFontAscent() - 25; 

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
    // MODIFIED: Ensure box is drawn if in adjustment mode for highlighting
    if (clockAdjustMode == 1) { // Highlight only if in mode 1
        u8g2.setDrawColor(0); // Set draw color to black
        u8g2.drawBox(timeX, clockY - u8g2.getFontAscent(), u8g2.getStrWidth(timeStr), u8g2.getFontAscent() + 2); 
        u8g2.setDrawColor(1); // Set draw color to white
    }
    u8g2.drawStr(timeX, clockY, timeStr); // Always draw the time string
    
    // --- MODIFIED: "No GPS Time" message now also checks RTC plausibility ---
    if (strcmp(timeStr, "--:--") == 0) { // If time couldn't be determined from GPS or RTC
        u8g2.setFont(u8g2_font_6x13_tr);
        const char* message = "No Time"; // Default message if no GPS & no RTC
        if (rtcFound && rtc.now().year() < 2000) { // If RTC found but its time is garbage
            message = "RTC Not Set";
        } else if (!gps.time.isValid() && !rtcFound) { // If no GPS & no RTC module
            message = "No Time";
        }
        u8g2.drawStr((u8g2.getWidth() - u8g2.getStrWidth(message)) / 2, u8g2.getHeight()/2 + 30, message);
    }
    // --- END MODIFIED ---

    // --- MODIFIED: Temperature display positioning and font ---
    // Prepare temperature string
    char tempNumStr[7]; // e.g., "25.0\0"
    if (temperature != -999.9 && temperature >= MIN_VALID_TEMP_C && temperature <= MAX_VALID_TEMP_C) {
      dtostrf(temperature, 4, 1, tempNumStr); // Format to 1 decimal place, e.g., "25.0", " -1.5"
    } else {
      strcpy(tempNumStr, "---.-"); // Error display
    }
    
    const char* tempUnitStr = " C"; // The unit string
    
    // Set font for numeric part of temperature
    u8g2.setFont(u8g2_font_ncenB12_tn); 
    int tempNumWidth = u8g2.getStrWidth(tempNumStr);

    // Set font for unit part
    u8g2.setFont(u8g2_font_6x10_tf); // Smaller font for " C"
    int tempUnitWidth = u8g2.getStrWidth(tempUnitStr);

    // Calculate total width and center X for the combined string
    int totalTempWidth = tempNumWidth + tempUnitWidth;
    int tempStartX = (u8g2.getWidth() - totalTempWidth) / 2;

    // Calculate Y position: Below the clock, with some padding.
    // The clock's baseline is 'clockY'. We'll place temp below it, using clock font's descent
    // and then some extra margin.
    int clockFontDescent = u8g2.getFontDescent(); // Use clock font's descent
    int tempY = clockY + abs(clockFontDescent) + 25;  // Increased margin to 25 pixels below clock baseline

    // Draw the numeric part
    u8g2.setFont(u8g2_font_ncenB12_tn); 
    u8g2.drawStr(tempStartX, tempY, tempNumStr);

    // Draw the unit part (" C")
    u8g2.setFont(u8g2_font_6x10_tf); // Revert to smaller font for unit
    u8g2.drawStr(tempStartX + tempNumWidth, tempY, tempUnitStr);
    // --- END MODIFIED ---

  } while (u8g2.nextPage());

  delay(200); // update rate
}
