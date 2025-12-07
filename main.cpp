/**
 * @file main.cpp
 * @brief Symbion Station 8 - Connect Four Style 3x3 Game
 * 
 * Game Flow:
 * Phase 1: Access Control
 *   - Enter 4-digit code on keypad to unlock web interface
 * 
 * Phase 2: Assembly (9 drops total, 3 per column)
 *   - Player places puzzle piece in delivery box
 *   - Press Column button (1, 2, or 3) to select column
 *   - DC motor moves box to selected column (ultrasonic verifies position)
 *   - Press Drop button → Servo opens box gate for 5 seconds → piece drops
 *   - Counter increments (C1, C2, or C3)
 *   - Column button disables when counter reaches 3
 *   - Repeat until all 9 pieces dropped (C1=3, C2=3, C3=3)
 * 
 * Phase 3: Final Validation
 *   - Press Confirm → "You Win!" + buzzer success sequence
 *   - Press Start Over → Reset servo opens board bottom, all pieces fall out, counters reset
 * 
 * Hardware:
 *   - 4x4 Keypad (unlock code entry)
 *   - HC-SR04 Ultrasonic (position verification via distance measurement)
 *   - DC Motor + L298N (horizontal box movement)
 *   - Servo #1 (opens/closes piece box bottom - drop gate)
 *   - Servo #2 (opens board bottom lid for reset)
 *   - 6 Buttons (Column 1/2/3, Drop, Confirm, Start Over)
 *   - Buzzer (audio feedback)
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <cstring>

#include "Keypad4x4.h"
#include "Ultrasonic.h"
#include "MotorDriver.h"
#include "ServoController.h"
#include "Buzzer.h"
#include "PushButton.h"
#include "config.h"

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

Keypad4x4 keypad(KEYPAD_ROW_PINS, KEYPAD_COL_PINS);
Ultrasonic ultrasonic(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN);
MotorDriver motor(MOTOR_IN1_PIN, MOTOR_IN2_PIN, MOTOR_ENA_PIN);
ServoController boxServo(SERVO_BOX_PIN);           // Servo #1 - Drop gate
ServoController boardLidServo(SERVO_BOARD_LID_PIN); // Servo #2 - Board reset

Buzzer buzzer(BUZZER_PIN);

PushButton column1Button(BUTTON_COLUMN_1_PIN);
PushButton column2Button(BUTTON_COLUMN_2_PIN);
PushButton column3Button(BUTTON_COLUMN_3_PIN);
PushButton dropButton(BUTTON_DROP_PIN);
PushButton confirmButton(BUTTON_CONFIRM_PIN);
PushButton startOverButton(BUTTON_START_OVER_PIN);

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

GameState currentState = STATE_INIT;
uint8_t columnCounters[3] = {0, 0, 0};  // C1, C2, C3
uint8_t selectedColumn = 0;              // 0=none, 1=col1, 2=col2, 3=col3
float currentPosition = HOME_POSITION_CM; // Track current box position
char enteredCode[5] = "";                // Keypad input buffer
uint8_t codeIndex = 0;
bool isUnlocked = false;
bool gameStarted = false;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

void initializeHardware();
void updateButtons();
void printGameStatus();
bool allColumnsComplete();
bool isColumnEnabled(uint8_t column);
float getTargetDistance(uint8_t column);
bool moveToColumn(uint8_t column);
void executeDropSequence();
void executeWinSequence();
void executeResetSequence();
void returnToHome();
void handleKeypadInput();
void handleButtonInput();
void updateStateMachine();

// ============================================================================
// MAIN FUNCTION
// ============================================================================

int main() {
    // Initialize standard I/O for debugging
    stdio_init_all();
    sleep_ms(2000);  // Wait for USB serial connection
    
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║     SYMBION STATION 8 - CORE AWAKENING              ║\n");
    printf("║     3x3 Connect Four Assembly Challenge             ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    // Initialize all hardware
    initializeHardware();
    
    // Play startup sequence
    buzzer.playStartupSequence();
    printf("✓ System initialized!\n\n");
    
    currentState = STATE_LOCKED;
    
    // Main loop
    while (true) {
        // Update all button states
        updateButtons();
        
        // Update servos
        boxServo.update();
        boardLidServo.update();
        
        // Update buzzer
        buzzer.update();
        
        // Run state machine
        updateStateMachine();
        
        // Small delay
        sleep_ms(10);
    }
    
    return 0;
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void initializeHardware() {
    printf("Initializing hardware components...\n");
    
    keypad.init();
    printf("  ✓ Keypad\n");
    
    ultrasonic.init();
    printf("  ✓ Ultrasonic sensor\n");
    
    motor.init();
    printf("  ✓ Motor driver\n");
    
    boxServo.init();
    boardLidServo.init();
    printf("  ✓ Servos\n");
    
    buzzer.init();
    printf("  ✓ Buzzer\n");
    
    column1Button.init();
    column2Button.init();
    column3Button.init();
    dropButton.init();
    confirmButton.init();
    startOverButton.init();
    printf("  ✓ Buttons\n");
    
    // Set initial servo positions (closed)
    boxServo.setAngle(BOX_CLOSED_ANGLE);
    boardLidServo.setAngle(LID_CLOSED_ANGLE);
    
    printf("Hardware initialization complete!\n");
}

void updateButtons() {
    column1Button.update();
    column2Button.update();
    column3Button.update();
    dropButton.update();
    confirmButton.update();
    startOverButton.update();
}

// ============================================================================
// GAME LOGIC HELPERS
// ============================================================================

void printGameStatus() {
    printf("\n┌─────────────────────────────┐\n");
    printf("│     GAME STATUS             │\n");
    printf("├─────────────────────────────┤\n");
    printf("│ Column 1: %d/3 pieces       │\n", columnCounters[0]);
    printf("│ Column 2: %d/3 pieces       │\n", columnCounters[1]);
    printf("│ Column 3: %d/3 pieces       │\n", columnCounters[2]);
    printf("│ Total:    %d/9 pieces       │\n", columnCounters[0] + columnCounters[1] + columnCounters[2]);
    printf("└─────────────────────────────┘\n\n");
}

bool allColumnsComplete() {
    return (columnCounters[0] == MAX_PIECES_PER_COLUMN &&
            columnCounters[1] == MAX_PIECES_PER_COLUMN &&
            columnCounters[2] == MAX_PIECES_PER_COLUMN);
}

bool isColumnEnabled(uint8_t column) {
    if (column < 1 || column > 3) return false;
    return columnCounters[column - 1] < MAX_PIECES_PER_COLUMN;
}

float getTargetDistance(uint8_t column) {
    switch (column) {
        case 1: return COLUMN_1_DISTANCE_CM;
        case 2: return COLUMN_2_DISTANCE_CM;
        case 3: return COLUMN_3_DISTANCE_CM;
        default: return HOME_POSITION_CM;
    }
}

// ============================================================================
// MOVEMENT FUNCTIONS
// ============================================================================

bool moveToColumn(uint8_t column) {
    float targetDistance = getTargetDistance(column);
    
    printf("→ Moving to Column %d (current: %.1f cm, target: %.1f cm)...\n", 
           column, currentPosition, targetDistance);
    
    // Determine direction based on current vs target position
    MotorDriver::Direction direction;
    if (targetDistance > currentPosition) {
        direction = MotorDriver::FORWARD;  // Move away from sensor (increase distance)
        printf("  Direction: FORWARD (moving right)\n");
    } else if (targetDistance < currentPosition) {
        direction = MotorDriver::REVERSE;  // Move toward sensor (decrease distance)
        printf("  Direction: REVERSE (moving left)\n");
    } else {
        printf("  Already at target position!\n");
        return true;
    }
    
    // Start motor in appropriate direction
    motor.run(MOTOR_SPEED, direction);
    
    uint32_t startTime = to_ms_since_boot(get_absolute_time());
    bool positionReached = false;
    
    while (!positionReached) {
        // Check timeout
        if ((to_ms_since_boot(get_absolute_time()) - startTime) > MOTOR_TIMEOUT_MS) {
            motor.stop();
            printf("✗ Movement timeout!\n");
            buzzer.playErrorBeep();
            return false;
        }
        
        // Measure current distance
        float measuredDistance = ultrasonic.measureDistance();
        
        if (measuredDistance >= 0) {
            printf("  Current: %.1f cm | Target: %.1f cm\n", measuredDistance, targetDistance);
            
            // Check if we've reached the target (within tolerance)
            if (measuredDistance >= targetDistance - DISTANCE_TOLERANCE_CM &&
                measuredDistance <= targetDistance + DISTANCE_TOLERANCE_CM) {
                positionReached = true;
                currentPosition = measuredDistance;  // Update tracked position
            }
            // Check if we've overshot (moved past target)
            else if (direction == MotorDriver::FORWARD && 
                     measuredDistance > targetDistance + DISTANCE_TOLERANCE_CM) {
                printf("  Overshot target, stopping.\n");
                positionReached = true;
                currentPosition = measuredDistance;
            }
            else if (direction == MotorDriver::REVERSE && 
                     measuredDistance < targetDistance - DISTANCE_TOLERANCE_CM) {
                printf("  Overshot target, stopping.\n");
                positionReached = true;
                currentPosition = measuredDistance;
            }
        }
        
        sleep_ms(100);  // Check every 100ms
    }
    
    motor.stop();
    printf("✓ Position reached! Box at %.1f cm\n", currentPosition);
    buzzer.playConfirmBeep();
    return true;
}

void returnToHome() {
    printf("→ Returning to home position...\n");
    
    // Move in reverse to home
    motor.run(MOTOR_SPEED, MotorDriver::REVERSE);
    
    uint32_t startTime = to_ms_since_boot(get_absolute_time());
    bool homeReached = false;
    
    while (!homeReached) {
        if ((to_ms_since_boot(get_absolute_time()) - startTime) > MOTOR_TIMEOUT_MS) {
            motor.stop();
            printf("✗ Home timeout!\n");
            buzzer.playErrorBeep();
            return;
        }
        
        float currentDistance = ultrasonic.measureDistance();
        
        if (currentDistance >= 0) {
            if (currentDistance <= HOME_POSITION_CM + DISTANCE_TOLERANCE_CM) {
                homeReached = true;
            }
        }
        
        sleep_ms(100);
    }
    
    motor.stop();
    printf("✓ Home position reached!\n");
}

// ============================================================================
// GAME SEQUENCES
// ============================================================================

void executeDropSequence() {
    printf("\n▼ EXECUTING DROP SEQUENCE ▼\n");
    printf("Opening box gate...\n");
    
    // Open the box servo
    boxServo.moveToAngle(BOX_OPEN_ANGLE, 500);
    while (boxServo.isMoving()) {
        boxServo.update();
        sleep_ms(10);
    }
    
    printf("Gate open - piece dropping for %lu seconds...\n", BOX_DROP_TIME_MS / 1000);
    
    // Keep gate open for specified time
    sleep_ms(BOX_DROP_TIME_MS);
    
    // Close the box servo
    printf("Closing gate...\n");
    boxServo.moveToAngle(BOX_CLOSED_ANGLE, 500);
    while (boxServo.isMoving()) {
        boxServo.update();
        sleep_ms(10);
    }
    
    // Increment counter
    columnCounters[selectedColumn - 1]++;
    
    printf("✓ Drop complete!\n");
    printf("Column %d counter: %d/%d\n", selectedColumn, columnCounters[selectedColumn - 1], MAX_PIECES_PER_COLUMN);
    
    buzzer.playConfirmBeep();
    
    // Check if column is now full
    if (columnCounters[selectedColumn - 1] >= MAX_PIECES_PER_COLUMN) {
        printf("⚠ Column %d is now FULL (disabled)\n", selectedColumn);
    }
}

void executeWinSequence() {
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║                                                       ║\n");
    printf("║              ★★★ YOU WIN! ★★★                        ║\n");
    printf("║                                                       ║\n");
    printf("║      SYMBION Core Successfully Activated!            ║\n");
    printf("║                                                       ║\n");
    printf("╚═══════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    // Play success sequence on buzzer
    buzzer.playSuccessBeep();
    sleep_ms(BUZZER_SUCCESS_DURATION_MS);
    
    printf("The final escape is yours! 🎉\n\n");
}

void executeResetSequence() {
    printf("\n▼ EXECUTING RESET SEQUENCE ▼\n");
    printf("Opening board bottom lid...\n");
    
    // Open the board lid servo
    boardLidServo.moveToAngle(LID_OPEN_ANGLE, 1000);
    while (boardLidServo.isMoving()) {
        boardLidServo.update();
        sleep_ms(10);
    }
    
    printf("Lid open - all pieces falling out...\n");
    sleep_ms(3000);  // Wait for pieces to fall
    
    // Close the board lid
    printf("Closing board lid...\n");
    boardLidServo.moveToAngle(LID_CLOSED_ANGLE, 1000);
    while (boardLidServo.isMoving()) {
        boardLidServo.update();
        sleep_ms(10);
    }
    
    // Reset all counters
    columnCounters[0] = 0;
    columnCounters[1] = 0;
    columnCounters[2] = 0;
    selectedColumn = 0;
    
    printf("✓ Reset complete! Counters cleared.\n");
    buzzer.playConfirmBeep();
    
    printGameStatus();
}

// ============================================================================
// INPUT HANDLERS
// ============================================================================

void handleKeypadInput() {
    char key = keypad.getKey();
    
    if (key != '\0') {
        if (currentState == STATE_LOCKED) {
            // Collecting unlock code
            if (key >= '0' && key <= '9') {
                enteredCode[codeIndex] = key;
                codeIndex++;
                printf("*");  // Show asterisk for security
                
                if (codeIndex >= 4) {
                    enteredCode[4] = '\0';  // Null terminate
                    printf("\n");
                    
                    // Check if code is correct
                    if (strcmp(enteredCode, UNLOCK_CODE) == 0) {
                        printf("✓ Code correct! Interface unlocked.\n");
                        isUnlocked = true;
                        buzzer.playSuccessBeep();
                        currentState = STATE_UNLOCKED;
                    } else {
                        printf("✗ Incorrect code. Try again.\n");
                        buzzer.playErrorBeep();
                        codeIndex = 0;
                        memset(enteredCode, 0, sizeof(enteredCode));
                    }
                }
            }
        }
    }
}

void handleButtonInput() {
    // Column selection buttons (available in IDLE state after unlock)
    if (currentState == STATE_IDLE && selectedColumn == 0) {
        if (column1Button.wasPressed() && isColumnEnabled(1)) {
            selectedColumn = 1;
            printf("\n► Column 1 selected\n");
            buzzer.playConfirmBeep();
            currentState = STATE_MOVING_TO_COLUMN;
        }
        else if (column2Button.wasPressed() && isColumnEnabled(2)) {
            selectedColumn = 2;
            printf("\n► Column 2 selected\n");
            buzzer.playConfirmBeep();
            currentState = STATE_MOVING_TO_COLUMN;
        }
        else if (column3Button.wasPressed() && isColumnEnabled(3)) {
            selectedColumn = 3;
            printf("\n► Column 3 selected\n");
            buzzer.playConfirmBeep();
            currentState = STATE_MOVING_TO_COLUMN;
        }
        else if (column1Button.wasPressed() && !isColumnEnabled(1)) {
            printf("✗ Column 1 is full!\n");
            buzzer.playErrorBeep();
        }
        else if (column2Button.wasPressed() && !isColumnEnabled(2)) {
            printf("✗ Column 2 is full!\n");
            buzzer.playErrorBeep();
        }
        else if (column3Button.wasPressed() && !isColumnEnabled(3)) {
            printf("✗ Column 3 is full!\n");
            buzzer.playErrorBeep();
        }
    }
    
    // Drop button
    if (currentState == STATE_POSITIONED && dropButton.wasPressed()) {
        currentState = STATE_DROPPING;
    }
    
    // Confirm button (win)
    if (currentState == STATE_COMPLETE && confirmButton.wasPressed()) {
        currentState = STATE_WIN;
    }
    
    // Start Over button (reset)
    if (currentState == STATE_COMPLETE && startOverButton.wasPressed()) {
        currentState = STATE_RESET;
    }
}


// ============================================================================
// STATE MACHINE
// ============================================================================

void updateStateMachine() {
    switch (currentState) {
        case STATE_INIT:
            // Initialization complete, move to locked state
            currentState = STATE_LOCKED;
            break;
            
        case STATE_LOCKED:
            // Display locked message (once)
            {
                static bool messagePrinted = false;
                if (!messagePrinted) {
                    printf("\n╔═══════════════════════════════════════════════════════╗\n");
                    printf("║          SYSTEM LOCKED                               ║\n");
                    printf("║  Enter 4-digit code from Station 7 to unlock         ║\n");
                    printf("╚═══════════════════════════════════════════════════════╝\n");
                    printf("\nEnter code: ");
                    messagePrinted = true;
                }
            }
            
            // Handle keypad input for unlock
            handleKeypadInput();
            break;
            
        case STATE_UNLOCKED:
            printf("\n╔═══════════════════════════════════════════════════════╗\n");
            printf("║          WELCOME TO SYMBION CORE                     ║\n");
            printf("║                                                       ║\n");
            printf("║  Mission: Complete the 3x3 puzzle assembly           ║\n");
            printf("║  - Drop 3 pieces in each of the 3 columns            ║\n");
            printf("║  - Total: 9 pieces required                          ║\n");
            printf("║                                                       ║\n");
            printf("║  Game starting now...                                ║\n");
            printf("╚═══════════════════════════════════════════════════════╝\n");
            printf("\n");
            
            gameStarted = true;
            printGameStatus();
            
            printf("Instructions:\n");
            printf("  1. Place puzzle piece in delivery box\n");
            printf("  2. Press Column button (1, 2, or 3) to select\n");
            printf("  3. Wait for box to move and position\n");
            printf("  4. Press DROP to release piece\n");
            printf("  5. Repeat until all 9 pieces are placed\n\n");
            
            currentState = STATE_IDLE;
            break;
            
        case STATE_IDLE:
            // Wait for column selection
            handleButtonInput();
            break;
            
        case STATE_MOVING_TO_COLUMN:
            // Move box to selected column
            if (moveToColumn(selectedColumn)) {
                printf("✓ Ready to drop into Column %d\n", selectedColumn);
                printf("Press DROP button to release piece...\n");
                currentState = STATE_POSITIONED;
            } else {
                printf("✗ Failed to reach column position\n");
                selectedColumn = 0;
                returnToHome();
                currentState = STATE_IDLE;
            }
            break;
            
        case STATE_POSITIONED:
            // Wait for drop button
            handleButtonInput();
            break;
            
        case STATE_DROPPING:
            // Execute drop sequence
            executeDropSequence();
            
            // Box stays at current column for efficiency
            // No need to return home between drops
            selectedColumn = 0;
            
            // Check if all columns are complete
            if (allColumnsComplete()) {
                printf("\n");
                printf("╔═══════════════════════════════════════════════════════╗\n");
                printf("║     ALL 9 PIECES PLACED!                             ║\n");
                printf("╚═══════════════════════════════════════════════════════╝\n");
                printf("\n");
                printGameStatus();
                printf("Press CONFIRM to complete the mission!\n");
                printf("Press START OVER to reset and try again.\n");
                currentState = STATE_COMPLETE;
            } else {
                // Continue game
                printGameStatus();
                currentState = STATE_IDLE;
            }
            break;
            
        case STATE_COMPLETE:
            // Wait for Confirm or Start Over
            handleButtonInput();
            break;
            
        case STATE_WIN:
            // Execute win sequence
            executeWinSequence();
            
            // Stay in win state (game over)
            printf("Game complete! Power cycle to play again.\n");
            while (true) {
                buzzer.update();
                sleep_ms(100);
            }
            break;
            
        case STATE_RESET:
            // Execute reset sequence
            executeResetSequence();
            
            // Return to idle state
            currentState = STATE_IDLE;
            printf("\nGame reset! Ready for new assembly.\n");
            printf("Place puzzle piece in box and select a column...\n\n");
            break;
            
        case STATE_ERROR:
            printf("System error! Please restart.\n");
            buzzer.playErrorBeep();
            while (true) {
                sleep_ms(1000);
            }
            break;
    }
}
