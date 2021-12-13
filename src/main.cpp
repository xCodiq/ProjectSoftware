// Include all the libraries used in this program
#include <Arduino.h>
#include <EEPROM.h>

// Define constant values, which will be used in our program
#define SAFE_CLOSED_POSITION_DEGREES 0
#define SAFE_OPEN_POSITION_DEGREES 90

/**
 * This enum type definition will be used to determine the current state of the safe. The save
 * can be in several states which are explain below.
 *
 * - SAFE_OPEN: The safe is currently opened, which means a correct password is entered.
 * - SAFE_CLOSED: The safe is currently closed, which means a password is yet to be entered.
 * - BLOCKED: The safe is in a blocked state, which means the user is not able to re-enter a password at the time.
 */
typedef enum State {
    SAFE_OPEN, SAFE_CLOSED, BLOCKED
} State;

/**
 * The StateController structure will be used to control the current state of the safe.
 * Using the controller, we are able to properly change the state defined by an enumeration.
 *
 * @see State
 */
typedef struct StateController {
    State currentState;

    State getState() {
        return currentState;
    }

    void setState(State newState) {
        this->currentState = newState;
    }
} StateController;

/**
 * The ServoController structure will be used to control the Servo motor with ease.
 * Using the controller, we are able to properly save the current position of the Servo and adjust the angle.
 *
 * Every operation done will be saved, calling the ServoController#rotate() function will
 * flush the latest changes to the Servo motor
 *
 * @see Servo - The servo instance used to write
 * @See StateController - The state controller used to change the current safe state
 */
typedef struct ServoController {
    Servo servo;
    StateController stateController;
    int position;

    /**
     * Calling this function will result in a closed phase of the Servo followed by a closed state change
     * The angle is an constant angle, which is defined at the top of the source code
     */
    void setClosed() {
        this->position = SAFE_CLOSED_POSITION_DEGREES;
        this->stateController.setState(SAFE_CLOSED);
    }

    /**
     * Calling this function will result in a open phase of the Servo followed by a open state change
     * The angle is an constant angle, which is defined at the top of the source code
     *
     */
    void setOpen() {
        this->position = SAFE_OPEN_POSITION_DEGREES;
        this->stateController.setState(SAFE_OPEN);
    }

    /**
     * Calling this function will let you manually adjust the angle of the Servo motor.
     *
     * @param addition the amount of degrees to add to the current angle
     */
    void changePosition(int addition) {
        this->position += addition;
    }

    /**
     * Calling this function will flush/execute the latest changes done to the position,
     * it will write the data to the Servo motor.
     */
    void rotate() {
        this->servo.write(this->position);
    }
} ServoController;

// Define global variables which are mandatory for the program to function properly.
ServoController servoController;
StateController stateController;

/**
 * Setup function
 */
void setup() {
    // Initialize a new Servo instance, and attach it to a pin
    Servo localServo;
    localServo.attach(7);

    // Initialize the State controller by creating a new StateController structure
    stateController = {
            .currentState = SAFE_CLOSED
    };

    // Initialize the Servo controller by creating a new ServoController structure
    servoController = {
            .servo = localServo,
            .position = 0
    };
}

/**
* Loop function
*/
void loop() {
    // Open and wait 1 second
    servoController.setOpen();
    servoController.rotate();
    delay(1000);

    // Close and wait 1 second
    servoController.setClosed();
    servoController.rotate();
    delay(1000);
}
