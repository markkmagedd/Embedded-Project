/**
 * @file config.h
 * @brief Configuration file for Symbion Station 8
 * 
 * Contains all pin definitions, constants, and configuration parameters
 * for the puzzle sorting robot.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Keypad pins (4 rows, 4 columns)
const uint8_t KEYPAD_ROW_PINS[4] = {2, 3, 4, 5};      // GPIO 2-5
const uint8_t KEYPAD_COL_PINS[4] = {6, 7, 8, 9};      // GPIO 6-9

// Ultrasonic sensor pins
const uint8_t ULTRASONIC_TRIGGER_PIN = 10;  // GPIO 10
const uint8_t ULTRASONIC_ECHO_PIN = 11;     // GPIO 11

// Motor driver pins (L298N)
const uint8_t MOTOR_IN1_PIN = 12;   // GPIO 12 - Direction control 1
const uint8_t MOTOR_IN2_PIN = 13;   // GPIO 13 - Direction control 2
const uint8_t MOTOR_ENA_PIN = 14;   // GPIO 14 - PWM speed control

// Servo pins
const uint8_t SERVO_BOX_PIN = 15;        // GPIO 15 - Opens piece box bottom (MG996)
const uint8_t SERVO_BOARD_LID_PIN = 16;  // GPIO 16 - Opens game board bottom lid

// Game button pins (6 buttons total)
const uint8_t BUTTON_COLUMN_1_PIN = 17;   // GPIO 17 - Select Column 1
const uint8_t BUTTON_COLUMN_2_PIN = 18;   // GPIO 18 - Select Column 2
const uint8_t BUTTON_COLUMN_3_PIN = 19;   // GPIO 19 - Select Column 3
const uint8_t BUTTON_DROP_PIN = 20;       // GPIO 20 - Execute drop
const uint8_t BUTTON_CONFIRM_PIN = 21;    // GPIO 21 - Confirm selection
const uint8_t BUTTON_START_OVER_PIN = 22; // GPIO 22 - Start over/Reset game

// Buzzer pin
const uint8_t BUZZER_PIN = 26;  // GPIO 26

// ============================================================================
// GAME CONFIGURATION
// ============================================================================

// Keypad unlock code (4 digits from Station 7)
const char UNLOCK_CODE[5] = "1111";  // Temporary: Changed due to keypad reading 4x per press

// Column positions (distance from ultrasonic sensor in cm)
// NOTE: Adjust these values based on your physical setup!
const float COLUMN_1_DISTANCE_CM = 5.0f;   // Distance to Column 1 (5cm from sensor)
const float COLUMN_2_DISTANCE_CM = 10.0f;  // Distance to Column 2 (5cm from Col1)
const float COLUMN_3_DISTANCE_CM = 15.0f;  // Distance to Column 3 (5cm from Col2)
const float HOME_POSITION_CM = 2.0f;       // Home position distance

// Distance tolerance for positioning (cm)
const float DISTANCE_TOLERANCE_CM = 0.5f;

// Motor calibration
const uint8_t MOTOR_SPEED = 70;      // Motor speed (0-100%)
const uint32_t MOTOR_TIMEOUT_MS = 10000;  // Safety timeout for motor movement

// Servo #1 - Piece box bottom (drop gate)
const float BOX_OPEN_ANGLE = 90.0f;       // Box gate open (piece drops)
const float BOX_CLOSED_ANGLE = 0.0f;      // Box gate closed
const uint32_t BOX_DROP_TIME_MS = 5000;   // Keep gate open for 5 seconds

// Servo #2 - Board bottom lid (reset/clear all pieces)
const float LID_OPEN_ANGLE = 90.0f;       // Board lid open (clear all pieces)
const float LID_CLOSED_ANGLE = 0.0f;      // Board lid closed

// Game rules
const uint8_t MAX_PIECES_PER_COLUMN = 3;  // 3 pieces per column
const uint8_t TOTAL_COLUMNS = 3;          // 3 columns total
const uint8_t TOTAL_PIECES = 9;           // Total 9 pieces (3x3)

// Timing constants
const uint32_t DEBOUNCE_TIME_MS = 50;
const uint32_t BUZZER_SUCCESS_DURATION_MS = 2000;

// ============================================================================
// STATE MACHINE
// ============================================================================

enum GameState {
    STATE_INIT,              // Initial setup
    STATE_LOCKED,            // Web interface locked, waiting for keypad code
    STATE_UNLOCKED,          // Code entered, waiting for Start button
    STATE_IDLE,              // Ready for column selection
    STATE_MOVING_TO_COLUMN,  // Motor moving box to selected column
    STATE_POSITIONED,        // Box positioned, waiting for Drop button
    STATE_DROPPING,          // Drop servo opening/closing
    STATE_COMPLETE,          // All 9 pieces dropped, waiting for Confirm/Start Over
    STATE_WIN,               // Win sequence (buzzer playing)
    STATE_RESET,             // Reset servo clearing board
    STATE_ERROR              // Error state
};

#endif // CONFIG_H
