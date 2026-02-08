# ESP32 Lathe Controller - WebUI and Display Implementation

## Project Context
This is an ESP32-based lathe controller that includes:
- A hardware display using u8g2 library (256x48 pixels)
- A web-based user interface for monitoring and testing
- WiFi connectivity for time synchronization and web interface

## Implementation Requirements

### 1. WebUI Testing Controls
Add the following interactive controls to the web interface for testing purposes (they will later be removed again for the final implementation):

**Sliders:**
- RPM: Range 0-6000, step 1
- CSS (Constant Surface Speed): Range 0-500, step 1
- X Coordinate: Range -999.99 to 999.99, step 0.01
- Z Coordinate: Range -99.99 to 99.99, step 0.01

**Radio Buttons/Toggle:**
- WiFi Status: Connected / Not Connected (for testing the WiFi indicator display)

**Requirements:**
- Display current values next to each slider
- Use WebSocket or AJAX to update the ESP32 in real-time
- Style the controls to be clear and easy to use
- Add labels and units (RPM, CSS, mm for coordinates)

### 2. Network Time Synchronization
Implement a function to fetch and maintain current time from an NTP server:

**Requirements:**
- Use NTP (Network Time Protocol) to get accurate time
- Format time as HH:MM:SS for the `Time_text` variable
- Update time regularly (every second on display, sync from NTP every few minutes)
- Handle timezone appropriately (configure for user's timezone)
- Gracefully handle network failures

**Suggested NTP servers:**
- pool.ntp.org
- time.nist.gov

### 3. Hardware Display Implementation

#### Display Specifications
- Resolution: 256x48 pixels
- Library: u8g2
- Mode: Font mode 1, Bitmap mode 1

#### Dynamic Variables to Display
```cpp
// Measurement values
const char* CSS_value_text;      // Current CSS value (0-500)
const char* RPM_value_text;      // Current RPM value (0-6000)
const char* x_reading_text;      // X coordinate reading (-999.99 to 999.99)
const char* z_reading_text;      // Z coordinate reading (-99.99 to 99.99)

// Visual indicators
int CSS_indicator_frame_w;       // CSS indicator bar width (0-130 pixels)
int RPM_indicator_frame_w;       // RPM indicator bar width (0-130 pixels)

// Status information
const char* Time_text;           // Current time (HH:MM:SS)
bool wifi_connected;             // WiFi connection status
```

#### Static Display Elements (Bitmaps)
```cpp
// Clock icon (15x16 pixels)
static const unsigned char image_clock_bits[] = {
  0xe0,0x03,0x18,0x0c,0x94,0x14,0x82,0x20,0x86,0x30,0x81,0x40,
  0x81,0x40,0x87,0x70,0x01,0x41,0x01,0x42,0x06,0x34,0x02,0x20,
  0x94,0x14,0x98,0x0c,0xe0,0x03,0x00,0x00
};

// WiFi connected icon (19x16 pixels)
static const unsigned char image_wifi_full_bits[] = {
  0x80,0x0f,0x00,0xe0,0x3f,0x00,0x78,0xf0,0x00,0x9c,0xcf,0x01,
  0xee,0xbf,0x03,0xf7,0x78,0x07,0x3a,0xe7,0x02,0xdc,0xdf,0x01,
  0xe8,0xb8,0x00,0x70,0x77,0x00,0xa0,0x2f,0x00,0xc0,0x1d,0x00,
  0x80,0x0a,0x00,0x00,0x07,0x00,0x00,0x02,0x00,0x00,0x00,0x00
};

// WiFi disconnected icon (19x16 pixels)
static const unsigned char image_wifi_not_connected_bits[] = {
  0x84,0x0f,0x00,0x68,0x30,0x00,0x10,0xc0,0x00,0xa4,0x0f,0x01,
  0x42,0x30,0x02,0x91,0x40,0x04,0x08,0x85,0x00,0xc4,0x1a,0x01,
  0x20,0x24,0x00,0x10,0x4a,0x00,0x80,0x15,0x00,0x40,0x20,0x00,
  0x00,0x42,0x00,0x00,0x85,0x00,0x00,0x02,0x01,0x00,0x00,0x00
};
```

#### Display Layout Reference Code
```cpp
u8g2.setFontMode(1);
u8g2.setBitmapMode(1);

// === FRAMES ===
// Time frame (top left)
u8g2.drawRFrame(0, 0, 79, 23, 3);

// WiFi frame (bottom left)
u8g2.drawRFrame(0, 25, 25, 23, 3);

// Coordinate frame (bottom middle)
u8g2.drawRFrame(27, 25, 52, 23, 3);

// RPM mode indicator frame (top middle-right)
u8g2.drawRFrame(81, 0, 42, 23, 3);

// CSS mode indicator frame (bottom middle-right)
u8g2.drawRFrame(81, 25, 42, 23, 3);

// RPM value frame (top right)
u8g2.drawRFrame(124, 0, 132, 23, 3);

// CSS value frame (bottom right)
u8g2.drawRFrame(124, 25, 132, 23, 3);

// === ICONS ===
// Clock icon
u8g2.drawXBM(2, 3, 15, 16, image_clock_bits);

// WiFi icon (conditional - see WiFi indicator section)
// Position: (3, 29)

// === TEXT ELEMENTS ===
// Time display
u8g2.setFont(u8g2_font_profont15_tr);
u8g2.drawStr(20, 15, Time_text);

// Coordinate labels
u8g2.setFont(u8g2_font_profont12_tr);
u8g2.drawStr(30, 35, "X");
u8g2.drawStr(30, 45, "Z");

// Coordinate values
u8g2.drawStr(35, 35, x_reading_text);
u8g2.drawStr(41, 45, z_reading_text);

// RPM label
u8g2.setFont(u8g2_font_profont22_tr);
u8g2.drawStr(84, 18, "RPM");

// CSS label
u8g2.drawStr(84, 43, "CSS");

// RPM value
u8g2.drawStr(165, 18, RPM_value_text);

// CSS value
u8g2.drawStr(165, 43, CSS_value_text);

// === INDICATOR BARS ===
// RPM indicator bar
u8g2.drawBox(125, 1, RPM_indicator_frame_w, 21);

// CSS indicator bar
u8g2.drawFrame(125, 26, CSS_indicator_frame_w, 21);

u8g2.sendBuffer();
```

### 4. WiFi Connection Indicator
Display the appropriate WiFi icon based on connection status:

**Implementation:**
```cpp
// Show connected icon when WiFi is connected
if (wifi_connected) {
  u8g2.drawXBM(3, 29, 19, 16, image_wifi_full_bits);
} else {
  // Show disconnected icon when WiFi is not connected
  u8g2.drawXBM(3, 29, 19, 16, image_wifi_not_connected_bits);
}
```

### 5. Mode Animation System
Implement visual feedback for RPM vs CSS mode using inverted/filled rectangles.

#### RPM Mode (Active)
- **RPM mode indicator frame (81, 0, 42x23):**
  - Use `u8g2.setDrawColor(2)` (XOR/inverted)
  - Use `u8g2.drawRBox()` - filled rounded box
  - Result: Inverted filled box with white "RPM" text on black background

- **RPM indicator bar (125, 1):**
  - Use `u8g2.setDrawColor(1)` (normal)
  - Use `u8g2.drawBox()` - filled rectangle
  - Result: Solid black filled bar

- **CSS mode indicator frame (81, 25, 42x23):**
  - Use `u8g2.setDrawColor(1)` (normal)
  - Use `u8g2.drawRFrame()` - outline only
  - Result: Empty frame with black "CSS" text

- **CSS indicator bar (125, 26):**
  - Use `u8g2.setDrawColor(1)` (normal)
  - Use `u8g2.drawFrame()` - outline only
  - Result: Empty outline rectangle

#### CSS Mode (Active)
- **CSS mode indicator frame (81, 25, 42x23):**
  - Use `u8g2.setDrawColor(2)` (XOR/inverted)
  - Use `u8g2.drawRBox()` - filled rounded box
  - Result: Inverted filled box with white "CSS" text on black background

- **CSS indicator bar (125, 26):**
  - Use `u8g2.setDrawColor(1)` (normal)
  - Use `u8g2.drawBox()` - filled rectangle
  - Result: Solid black filled bar

- **RPM mode indicator frame (81, 0, 42x23):**
  - Use `u8g2.setDrawColor(1)` (normal)
  - Use `u8g2.drawRFrame()` - outline only
  - Result: Empty frame with black "RPM" text

- **RPM indicator bar (125, 1):**
  - Use `u8g2.setDrawColor(1)` (normal)
  - Use `u8g2.drawFrame()` - outline only
  - Result: Empty outline rectangle

#### Implementation Approach
**Suggested structure:**
```cpp
enum OperatingMode {
  MODE_RPM,
  MODE_CSS
};

OperatingMode current_mode = MODE_RPM;

void drawDisplay(OperatingMode mode) {
  u8g2.clearBuffer();
  u8g2.setFontMode(1);
  u8g2.setBitmapMode(1);
  
  // Draw static frames...
  
  if (mode == MODE_RPM) {
    // RPM mode active
    u8g2.setDrawColor(2);
    u8g2.drawRBox(81, 0, 42, 23, 3);
    u8g2.setDrawColor(1);
    u8g2.drawRFrame(81, 25, 42, 23, 3);
    u8g2.drawBox(125, 1, RPM_indicator_frame_w, 21);
    u8g2.drawFrame(125, 26, CSS_indicator_frame_w, 21);
  } else {
    // CSS mode active
    u8g2.setDrawColor(1);
    u8g2.drawRFrame(81, 0, 42, 23, 3);
    u8g2.setDrawColor(2);
    u8g2.drawRBox(81, 25, 42, 23, 3);
    u8g2.setDrawColor(1);
    u8g2.drawFrame(125, 1, RPM_indicator_frame_w, 21);
    u8g2.drawBox(125, 26, CSS_indicator_frame_w, 21);
  }
  
  // Draw text with appropriate draw colors...
  // Draw icons...
  
  u8g2.sendBuffer();
}
```

### 6. Additional Implementation Notes

#### Indicator Bar Width Calculation
Convert RPM/CSS values to bar width (0-130 pixels):
```cpp
// For RPM (0-6000 → 0-130 pixels)
RPM_indicator_frame_w = map(current_rpm, 0, 6000, 0, 130);

// For CSS (0-500 → 0-130 pixels)
CSS_indicator_frame_w = map(current_css, 0, 500, 0, 130);
```

#### Text Formatting
Ensure numeric values are formatted correctly:
```cpp
// Format RPM value (e.g., "2950")
sprintf(RPM_value_text, "%d", rpm_value);

// Format CSS value (e.g., "230")
sprintf(CSS_value_text, "%d", css_value);

// Format coordinates (e.g., "-002.54", "-00.89")
sprintf(x_reading_text, "%+07.2f", x_coord);  // Sign, 7 total width, 2 decimals
sprintf(z_reading_text, "%+06.2f", z_coord);  // Sign, 6 total width, 2 decimals
```

#### Mode Switching
Add a way to toggle between RPM and CSS modes (could be via WebUI button or hardware input).

## Expected Deliverables

1. **WebUI HTML/JavaScript** with:
   - Sliders for RPM, CSS, X, and Z with live value display
   - WiFi status toggle
   - Real-time updates sent to ESP32

2. **NTP Time Function** that:
   - Fetches time from NTP server
   - Formats as HH:MM:SS
   - Updates Time_text variable
   - Handles connection failures gracefully

3. **Display Update Function** that:
   - Implements the mode-based animation system
   - Conditionally displays WiFi icons
   - Updates all dynamic text elements
   - Calculates and displays indicator bars

4. **Integration Code** showing how these components work together in the main loop.

## Design Principles
- Keep display updates efficient (u8g2 buffer operations)
- Separate concerns: WebUI, time sync, display rendering
- Make mode switching smooth and immediate
- Ensure values are always readable with appropriate formatting
- Handle edge cases (WiFi disconnection, invalid values, etc.)
