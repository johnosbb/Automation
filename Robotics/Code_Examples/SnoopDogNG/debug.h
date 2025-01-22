#define DEBUG_SERIAL  // Uncomment to enable debugging

// Debugging Levels
#define DEBUG_LEVEL_NONE       0  // No debugging
#define DEBUG_LEVEL_ERROR      1  // Errors only
#define DEBUG_LEVEL_WARN       2  // Warnings and Errors
#define DEBUG_LEVEL_INFO       3  // Info, Warnings, and Errors
#define DEBUG_LEVEL_DEBUG      4  // Basic Debugging
#define DEBUG_LEVEL_VERBOSE    5  // Verbose Debugging
#define DEBUG_LEVEL_VERY_VERBOSE 6  // Very Verbose Debugging
#define LONG_TAP 500

// Set the desired debug level here
#define DEBUG_LEVEL DEBUG_LEVEL_DEBUG

#ifdef DEBUG_SERIAL
  // Error
  #define DEBUG_PRINT_ERROR(x)  if (DEBUG_LEVEL >= DEBUG_LEVEL_ERROR) { Serial.print("[ERROR] "); Serial.print(__FILE__); Serial.print(":"); Serial.print(__LINE__); Serial.print(" - "); Serial.println(x); }

  // Warning
  #define DEBUG_PRINT_WARN(x)   if (DEBUG_LEVEL >= DEBUG_LEVEL_WARN) { Serial.print("[WARNING] "); Serial.print(__FILE__); Serial.print(":"); Serial.print(__LINE__); Serial.print(" - "); Serial.println(x); }

  // Info
  #define DEBUG_PRINT_INFO(x)   if (DEBUG_LEVEL >= DEBUG_LEVEL_INFO) { Serial.print("[INFO] "); Serial.print(__FILE__); Serial.print(":"); Serial.print(__LINE__); Serial.print(" - "); Serial.println(x); }

  // Basic Debug
  #define DEBUG_PRINT(x)        if (DEBUG_LEVEL >= DEBUG_LEVEL_DEBUG) { Serial.print("[DEBUG] "); Serial.print(x); }
  #define DEBUG_PRINTLN(x)      if (DEBUG_LEVEL >= DEBUG_LEVEL_DEBUG) { Serial.print("[DEBUG] "); Serial.println(x); }

  // Verbose Debug
  #define DEBUG_PRINT_VERBOSE(x)   if (DEBUG_LEVEL >= DEBUG_LEVEL_VERBOSE) { Serial.print("[VERBOSE] "); Serial.print(__FILE__); Serial.print(":"); Serial.print(__LINE__); Serial.print(" - "); Serial.println(x); }

  // Very Verbose Debug
  #define DEBUG_PRINT_VERY_VERBOSE(x) if (DEBUG_LEVEL >= DEBUG_LEVEL_VERY_VERBOSE) { Serial.print("[VERY_VERBOSE] "); Serial.print(__FILE__); Serial.print(":"); Serial.print(__LINE__); Serial.print(" - "); Serial.println(x); }
#else
  #define DEBUG_PRINT_ERROR(x)
  #define DEBUG_PRINT_WARN(x)
  #define DEBUG_PRINT_INFO(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT_VERBOSE(x)
  #define DEBUG_PRINT_VERY_VERBOSE(x)
#endif