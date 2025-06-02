// --- INCLUDES ---
#include <Wire.h>
#include <U8g2lib.h>
#include <SoftwareSerial.h> // For GPS
#include <TinyGPSPlus.h>    // For GPS data parsing
#include <RTClib.h>         // For RTC
#include <LM75.h>           // Using LM75.h
#include <EEPROM.h>         // For persistent storage of time offset on ESP8266


// --- PIN DEFINITIONS ---
const int GPS_RX_PIN = D5; // D1 Mini D5 (GPIO14) to GPS TX
const int GPS_TX_PIN = D6; // D1 Mini D6 (GPIO12) to GPS RX
const long GPS_BAUD_RATE = 9600;

// Button pins are no longer used for timezone adjustment but remain if you have other uses
const int SET_BUTTON_PIN = D3;  // D1 Mini D3 (GPIO0) for SET button
const int MODE_BUTTON_PIN = D4; // D1 Mini D4 (GPIO2) for MODE button


// --- GLOBAL OBJECTS ---
// --- ROTATION SETTING: Display is rotated 90 degrees clockwise (U8G2_R1) ---
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R1, /* reset=*/ U8X8_PIN_NONE);

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;
RTC_DS3231 rtc;
LM75 lm75(0x48); // LM75 library constructor, default address 0x48


// --- GLOBAL VARIABLES ---
float lastKnownSpeed = 0.0; // Last known speed in km/h
unsigned long lastGpsUpdate = 0; // Timestamp of last GPS data update
const unsigned long gpsTimeout = 5000; // 5 seconds without GPS update

float temperature = -999.9; // Stores the temperature
unsigned long lastTempUpdateTime = 0; // Last time temperature was read

// Time adjustment variables
// --- HARDCODED TIMEZONE OFFSET: ---
// Change this value to adjust the local time.
// For example:
// +2 for Central European Summer Time (CEST)
// +1 for Central European Time (CET)
// -4 for Eastern Daylight Time (EDT)
signed char timeOffsetHours = 2; // Defaulted to +2 hours (e.g., for CEST/EET)

// RTC status
bool rtcFound = false;
bool initialRTCSyncDone = false; // Flag to ensure RTC is synced at least once
unsigned long lastRTCSyncTime = 0;
const unsigned long rtcSyncInterval = 3600000; // Sync RTC every 1 hour (3600 * 1000 ms)

// Constants for EEPROM storage (only for initial save of hardcoded offset)
const int EEPROM_SIZE = 2; // Need 2 bytes: 1 for magic byte, 1 for offset
const byte EEPROM_MAGIC_BYTE = 0xAF; // A recognizable byte to indicate valid data
const int EEPROM_OFFSET_ADDR = 0; // Address to store the magic byte (0), and offset (1)


// --- TEMPERATURE CONSTANTS ---
const float MIN_VALID_TEMP_C = -50.0; // Minimum plausible temperature
const float MAX_VALID_TEMP_C = 125.0; // Maximum plausible temperature
const float MAX_VALID_SPEED_KMPH = 250.0; // Max plausible speed for filtering (e.g., car max speed)


// --- HELPER FUNCTIONS ---

// Function to read temperature from LM75A sensor
// Returns temperature in Celsius, or -999.9 if read fails
float readTemperatureLM75A() {
  if (!lm75.begin()) { // Check if sensor is connected/responding
    Serial.println("LM75 sensor not found!");
    return -999.9; // Return error value
  }
  // This line might need adjustment based on your specific LM75 library.
  // Common methods are: lm75.getTempC(), lm75.readTemp(), or lm75.getTemperature().
  // If 'getTemperature()' still fails, you might need to check your LM75.h file
  // or try a different LM75 library.
  float tempC = lm75.getTemperature(); 
  if (isnan(tempC)) { // Check for NaN (Not a Number) which can indicate a read error
    Serial.println("Failed to read temperature from LM75 sensor!");
    return -999.9; // Return error value
  }
  return tempC;
}


// --- INTERRUPT SERVICE ROUTINES (ISRs) ---
// Button ISRs are no longer needed for timezone adjustment, but kept as placeholders
// if you decide to use them for other features. Otherwise, they can be removed
// along with their attachInterrupt calls in setup().
void IRAM_ATTR isr_set_button() {
  // setButtonPressed = true; // Not used for time offset anymore
}

void IRAM_ATTR isr_mode_button() {
  // modeButtonPressed = true; // Not used for time offset anymore
}


// --- setup() function ---
void setup() {
  // Serial setup for debugging
  Serial.begin(115200);
  Serial.println("Starting up...");

  // Initialize U8g2 (OLED)
  u8g2.begin();
  u8g2.setFontMode(1); // Set font mode to solid (opaque background)
  u8g2.setFontDirection(0); // Set font direction to 0 (default)

  // Initialize GPS serial
  gpsSerial.begin(GPS_BAUD_RATE);

  // Initialize LM75A sensor
  lm75.begin();
  Serial.println("LM75A sensor initialized.");

  // Initialize RTC
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    rtcFound = false;
  } else {
    rtcFound = true;
    Serial.println("RTC found.");
    if (rtc.lostPower()) {
      Serial.println("RTC lost power, let's set the time!");
      // This will be synchronized by GPS later if available,
      // or you can set a default time here.
      // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  }

  // --- Initialize EEPROM and store hardcoded offset ---
  EEPROM.begin(EEPROM_SIZE); // Allocate 2 bytes for EEPROM emulation
  
  // Directly set and store the desired offset (timeOffsetHours is already set to 2 at global scope)
  EEPROM.write(EEPROM_OFFSET_ADDR + 1, (byte)timeOffsetHours); // Store the hardcoded 2
  EEPROM.write(EEPROM_OFFSET_ADDR, EEPROM_MAGIC_BYTE); // Write magic byte last
  EEPROM.commit(); // Save changes to flash
  Serial.print("Hardcoded and stored time offset to EEPROM: ");
  Serial.println(timeOffsetHours);


  // Button interrupts are no longer necessary for timezone adjustment,
  // but keeping the pinMode lines to ensure pins are set up.
  pinMode(SET_BUTTON_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(SET_BUTTON_PIN), isr_set_button, FALLING); // No longer needed for time offset

  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(MODE_BUTTON_PIN), isr_mode_button, FALLING); // No longer needed for time offset

  Serial.println("Setup complete.");
}


// --- loop() function ---
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
      // If gps.time.isValid() is false, `lastKnownSpeed` will hold its previous value,
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

  // --- Handle Button Presses (Not used for timezone adjustment anymore) ---
  // You can repurpose these buttons for other features if needed.


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

    u8g2.setDrawColor(1); // Reset draw color to white for subsequent drawings
    // ONLY draw the speed string in white if it should be visible
    if (blinkShouldBeVisible) {
      u8g2.drawStr(speedX, speedY, speedStr); // Draw speed
    }


    // Bottom Half: Clock and Temperature (starts from Y=64)
    // Clear the entire bottom half with black for a clean slate
    u8g2.setDrawColor(0);
    u8g2.drawBox(0, u8g2.getHeight() / 2, u8g2.getWidth(), u8g2.getHeight() / 2); // Fill bottom half
    u8g2.setDrawColor(1); // Reset to white for drawing content


    // --- Clock Display & RTC Sync ---
    // Make sure to set the font for the main time string here
    u8g2.setFont(u8g2_font_ncenB14_tf); // Set font for time (and main temp part)
    DateTime displayDateTime; // This will hold the time to display
    
    // Check if RTC is actually running and has a plausible time (e.g., year > 2000)
    bool rtcTimePlausible = false;
    if(rtcFound && rtc.now().year() > 2000) { 
      rtcTimePlausible = true;
    }

    if (gps.time.isValid() && hasSignal) { 
      // GPS time is valid: Get UTC from GPS, apply offset, then synchronize RTC
      DateTime gpsUtcDateTime(gps.date.year(), gps.date.month(), gps.date.day(), 
                              gps.time.hour(), gps.time.minute(), gps.time.second());
      
      // Calculate local time by adding the offset
      // Using unixtime() makes adding/subtracting hours easier and handles date/time rollovers
      displayDateTime = DateTime(gpsUtcDateTime.unixtime() + timeOffsetHours * 3600); 

      // --- RTC Synchronization Logic ---
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
    
    // Fallback to RTC if GPS is not valid OR if RTC is already running and plausible after initial sync
    if (rtcTimePlausible && (!gps.time.isValid() || initialRTCSyncDone)) {
      displayDateTime = rtc.now();
    } else if (gps.time.isValid() && hasSignal) {
      // If RTC is not plausible or found, but GPS is valid, rely on GPS time.
      // This case is already handled by the first `if`, but added for clarity.
    } else {
      // If neither GPS nor RTC is plausible/running, display a "No Time" message.
      // This will be handled by the display section below.
    }

    char timeStr[6]; // HH:MM\0
    // Only format timeStr if displayDateTime is valid (i.e., its year is plausible)
    if (displayDateTime.year() > 2000) { 
        sprintf(timeStr, "%02d:%02d", displayDateTime.hour(), displayDateTime.minute());
    } else {
        strcpy(timeStr, "--:--"); // Display dashes if time is not valid
    }

    // --- Temperature String Formatting and Drawing ---
    char fullTempNumStr[10]; // e.g., " 25.6", "-5.1"
    char intPartStr[5]; // e.g., "25", "-5", " - "
    char decAndUnitPartStr[5]; // Stores ".X°C" or "°C" for error.

    if (temperature != -999.9 && temperature >= MIN_VALID_TEMP_C && temperature <= MAX_VALID_TEMP_C) {
      dtostrf(temperature, 4, 1, fullTempNumStr); // e.g., "25.6", " 5.1", "-5.1"
      
      char* dot = strchr(fullTempNumStr, '.');
      if (dot) {
        // Copy integer part (everything before the dot)
        strncpy(intPartStr, fullTempNumStr, dot - fullTempNumStr);
        intPartStr[dot - fullTempNumStr] = '\0'; // Null-terminate
        
        // Copy decimal part and unit
        sprintf(decAndUnitPartStr, "%s%cC", dot, (char)176); // ".6°C", ".1°C"
      } else {
        // Fallback if no dot (shouldn't happen with 1 decimal place)
        strcpy(intPartStr, fullTempNumStr);
        sprintf(decAndUnitPartStr, "%cC", (char)176); // Just "°C"
      }
    } else {
      strcpy(intPartStr, "-");
      sprintf(decAndUnitPartStr, "%cC", (char)176); // "- °C" as two parts
    }

    // Calculate widths for combined centering
    u8g2.setFont(u8g2_font_ncenB14_tf); // Main font for integer part
    int intPartWidth = u8g2.getStrWidth(intPartStr);

    u8g2.setFont(u8g2_font_6x10_tf); // Smaller font for decimal and unit
    int decAndUnitPartWidth = u8g2.getStrWidth(decAndUnitPartStr);

    int combinedTempWidth = intPartWidth + decAndUnitPartWidth;
    int startX_temp = (u8g2.getWidth() - combinedTempWidth) / 2;

    // --- Calculate Positioning for Time and Temperature ---
    // (Still based on main font height for overall layout)
    int currentFontAscent = u8g2.getFontAscent(); // From u8g2_font_ncenB14_tf (14)
    int currentFontDescent = u8g2.getFontDescent(); // (-3)
    int currentFontHeight = currentFontAscent - currentFontDescent; // Height of u8g2_font_ncenB14_tf (17)

    const int paddingBetweenLines = 5; // Padding between time and temperature
    
    // Total height of the two lines (time and temp) + padding
    int totalContentHeight = currentFontHeight * 2 + paddingBetweenLines; 

    // Calculate the top Y coordinate for the entire block (time + temp)
    int startY = u8g2.getHeight() / 2 + (u8g2.getHeight() / 2 - totalContentHeight) / 2;

    // Calculate baseline for time string
    int timeY_baseline = startY + currentFontAscent;
    
    // Calculate baseline for the main integer part of temperature
    int tempY_baseline_main = timeY_baseline + currentFontHeight + paddingBetweenLines + 10; 

    // --- Draw the main time string ---
    u8g2.setFont(u8g2_font_ncenB14_tf); // Ensure this font is set BEFORE drawing the time!
    u8g2.setDrawColor(1); 
    u8g2.drawStr((u8g2.getWidth() - u8g2.getStrWidth(timeStr)) / 2, timeY_baseline, timeStr); 
    
    // --- Draw the Temperature (two parts) ---
    // Draw integer part
    u8g2.setFont(u8g2_font_ncenB14_tf); // Use the main font for integer part
    u8g2.drawStr(startX_temp, tempY_baseline_main, intPartStr);

    // Draw decimal part and unit
    u8g2.setFont(u8g2_font_6x10_tf); // Switch to smaller font for decimal and unit
    int decAndUnitX = startX_temp + intPartWidth;
    
    u8g2.drawStr(decAndUnitX, tempY_baseline_main + -3, decAndUnitPartStr); 


    // --- Display "No Time" or "RTC Not Set" messages ---
    if (strcmp(timeStr, "--:--") == 0) { 
        u8g2.setFont(u8g2_font_6x13_tr); // Using a slightly different small font for these messages
        
        const char* message = "";
        if (!gps.time.isValid() && !rtcFound) { 
          message = "No Time";
        } else if (rtcFound && !rtcTimePlausible && !gps.time.isValid()) { 
          message = "RTC Not Set";
        }

        if (strlen(message) > 0) {
            int msgX = (u8g2.getWidth() - u8g2.getStrWidth(message)) / 2;
            int msgY = timeY_baseline; 
            u8g2.drawStr(msgX, msgY, message);
        }
        // Revert font to the main time/temp font after drawing the message
        u8g2.setFont(u8g2_font_ncenB14_tf); 
    }

  } while (u8g2.nextPage());

  delay(200); // update rate
}
