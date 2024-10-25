#include <Adafruit_GFX.h>      // Core graphics library
#include <Adafruit_ST7789.h>   // Hardware-specific library for ST7789
#include <SPI.h>

// Uncomment the following line to enable serial printouts for debugging
#define SHOW_CALCULATIONS

// Define the size of the screen
#define LCD_WIDTH  135
#define LCD_HEIGHT 240

// Define the pins of the ESP32 connected to the LCD
#define LCD_MOSI 23  // SDA Pin on ESP32 D23
#define LCD_SCLK 18  // SCL Pin on ESP32 D18
#define LCD_CS   15  // Chip select control pin on ESP32 D15
#define LCD_DC    2   // Data Command control pin on ESP32 D2
#define LCD_RST   4   // Reset pin (could connect to RST pin) on ESP32 D4
#define LCD_BLK   32  // Back Light Pin on ESP32 D32

// Create the Adafruit_ST7789 object
Adafruit_ST7789 lcd = Adafruit_ST7789(LCD_CS, LCD_DC, LCD_RST);

// Define graph parameters
#define GRAPH_WIDTH LCD_WIDTH   // Adjusted for landscape mode
#define GRAPH_HEIGHT LCD_HEIGHT  // Adjusted for landscape mode
#define Y_AXIS_COLOR ST77XX_WHITE
#define X_AXIS_COLOR ST77XX_YELLOW
#define BG_COLOR ST77XX_BLACK

// Number of samples to display at once
#define POINT_SPACING 10  // Control the spacing between points
#define MAX_POINTS (GRAPH_HEIGHT / POINT_SPACING)  // Calculate based on spacing

// Colors for different variables
#define COLOR_CURRENT ST77XX_RED
#define COLOR_VIBRATION ST77XX_GREEN
#define COLOR_RPM ST77XX_BLUE
#define COLOR_TEMPERATURE ST77XX_YELLOW
#define COLOR_OUTPUT ST77XX_MAGENTA

// Store the current and last values for different inputs
float currentValue = 0.0, lastCurrentValue = 0.0;
float vibrationValue = 0.0, lastVibrationValue = 0.0;
float rpmValue = 0.0, lastRpmValue = 0.0;
float temperatureValue = 0.0, lastTemperatureValue = 0.0;

int currentIndex = 0;  // Tracks where we are in the graph

void setup() {
  // Initialize the serial monitor if debugging is enabled
  #ifdef SHOW_CALCULATIONS
  Serial.begin(115200);
  Serial.println("Starting setup...");
  #endif

  // Initialize the LCD
  lcd.init(LCD_WIDTH, LCD_HEIGHT);  // Initialize the display with the correct width and height
  lcd.setRotation(1);  // Set rotation to landscape mode (1 or 3 for landscape)

  // Turn on the backlight
  pinMode(LCD_BLK, OUTPUT);
  digitalWrite(LCD_BLK, HIGH);

  lcd.fillScreen(BG_COLOR);  // Clear the screen initially
  drawAxis();  // Draw the initial axis
  
  #ifdef SHOW_CALCULATIONS
  Serial.println("Setup complete.");
  #endif
}

void loop() {
  // Simulated data input (replace with actual sensor data in practice)
  static unsigned long lastUpdate = 0;
  unsigned long currentTime = millis();
  
  // Update data every 500 ms (adjust this value to slow down or speed up updates)
  if (currentTime - lastUpdate > 500) {  // 500 ms delay between updates
    // Simulate data
    lastCurrentValue = currentValue;
    lastVibrationValue = vibrationValue;
    lastRpmValue = rpmValue;
    lastTemperatureValue = temperatureValue;
    
    // Simulate values for different inputs (replace with actual sensor values)
    currentValue = random(-300, 300) / 100.0;  // Simulate current data
    vibrationValue = random(-300, 300) / 100.0;  // Simulate vibration data
    rpmValue = random(-300, 300) / 100.0;  // Simulate RPM data
    temperatureValue = random(-300, 300) / 100.0;  // Simulate temperature data

    // Plot the new data points (partial update)
    plotNewData();

    lastUpdate = currentTime;
  }
  delay(5000);  // Optional delay to control processing speed
}
  
// Function to draw the axis
void drawAxis() {
  lcd.drawLine(0, GRAPH_WIDTH / 2, GRAPH_HEIGHT, GRAPH_WIDTH / 2, X_AXIS_COLOR);  // X-axis
  lcd.drawLine(0, 0, 0, GRAPH_WIDTH, Y_AXIS_COLOR);  // Y-axis
}

// Function to plot new data points for all variables
void plotNewData() {
  int currentX = currentIndex * POINT_SPACING;  // Set current X based on currentIndex
  int lastX = (currentIndex - 1 + MAX_POINTS) % MAX_POINTS * POINT_SPACING; // Last X calculation with wrap-around

  // Check if we need to clear the screen
  if (currentIndex == 0) {
    lastX = 0;
  }

  // Plot Current
  plotLine(lastX, currentX, lastCurrentValue, currentValue, COLOR_CURRENT);
  
  // Plot Vibration
  plotLine(lastX, currentX, lastVibrationValue, vibrationValue, COLOR_VIBRATION);
  
  // Plot RPM
  plotLine(lastX, currentX, lastRpmValue, rpmValue, COLOR_RPM);
  
  // Plot Temperature
  plotLine(lastX, currentX, lastTemperatureValue, temperatureValue, COLOR_TEMPERATURE);

  // Check if we need to clear the screen and restart the graph
  if (currentIndex == MAX_POINTS - 1) {
    clearScreenAndRestart();
    lastX = 0;
  }

  // Increment the current index and wrap it around
  currentIndex = (currentIndex + 1) % MAX_POINTS; // Increment index and wrap
}

// Helper function to plot a line for a specific variable
void plotLine(int lastX, int currentX, float lastValue, float currentValue, uint16_t color) {
  int lastY = mapDataToScreen(lastValue);
  int currentY = mapDataToScreen(currentValue);
  lcd.drawLine(lastX, lastY, currentX, currentY, color);
}

// Function to clear the screen and reset the graph
void clearScreenAndRestart() {
  #ifdef SHOW_CALCULATIONS
  Serial.println("Reached the end of the screen. Clearing and restarting...");
  #endif

  lcd.fillScreen(BG_COLOR);  // Clear the screen
  drawAxis();  // Draw the axis again
  currentIndex = 0;  // Reset the index to 0
}

// Function to map data values from -3 to 3 to screen coordinates
int mapDataToScreen(float value) {
  // Define the data range you're working with
  float valueMin = -3.0;  // Minimum expected value
  float valueMax = 3.0;   // Maximum expected value

  // Define the screen range with a 10-pixel margin at top and bottom
  int screenMin = 10;  // Top margin
  int screenMax = GRAPH_WIDTH - 10;  // Bottom margin

  // Map the value to the screen height (Y-axis range from screenMin to screenMax)
  int screenY = screenMax - ((value - valueMin) / (valueMax - valueMin)) * (screenMax - screenMin);

  // Ensure the value stays within bounds (screenMin to screenMax)
  if (screenY < screenMin) screenY = screenMin;
  if (screenY > screenMax) screenY = screenMax;

  return screenY;
}
