// Include all the libraries used in this program
#include <Arduino.h>
#include <EEPROM.h>
#include <Servo.h>

// Define constant values, which will be used in our program
#define SERIAL_BOUND 9600
#define SAFE_CLOSED_POSITION_DEGREES 0
#define SAFE_OPEN_POSITION_DEGREES 90

// Pin settings
#define SETTING__SEVEN_SEGMENT_ONE_PIN 2
#define SETTING__SEVEN_SEGMENT_TWO_PIN 3
#define SETTING__SEVEN_SEGMENT_THREE_PIN 4

#define SETTING__DATA_PIN 5
#define SETTING__LATCH_PIN 6
#define SETTING__CLOCK_PIN 7

#define SETTING__ENCODER_PIN_A 8
#define SETTING__ENCODER_PIN_B 9
#define SETTING__ENCODER_BUTTON_PIN 13

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

typedef enum Display {
    ONE = 1, TWO = 2, THREE = 3
} Display;

typedef struct Password {
private:
    int code;

public:
    Password(int defaultCode) {
        this->code = defaultCode;
    }

    int getCode() const {
        return this->code;
    }

    void setCode(const int newCode) {
        this->code = newCode;
    }
} Password;

namespace Segment {
    class Pair {
    private:
        int decimalValue;
        int binaryValue;

    public:
        Pair(const int decimalValue, const int binaryValue) {
            this->decimalValue = decimalValue;
            this->binaryValue = binaryValue;
        }

    public:
        int getDecimalValue() const {
            return this->decimalValue;
        }

        int getBinaryValue() const {
            return this->binaryValue;
        }
    };

    const auto ZERO = Pair(0, B0);
    const auto ONE = Pair(1, B1);
    const auto TWO = Pair(2, B10);
    const auto THREE = Pair(3, B11);
    const auto FOUR = Pair(4, B100);
    const auto FIVE = Pair(5, B101);
    const auto SIX = Pair(6, B110);
    const auto SEVEN = Pair(7, B111);
    const auto EIGHT = Pair(8, B1000);
    const auto NINE = Pair(9, B1001);

    static const Pair All[] = {ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE};

    static int convertDecimal(const int decimal) {
        // Make sure the decimal value is between 0 and 9
        if (decimal < 0 || decimal > 9) return -1;

        // Loop through all the pairs
        for (const auto pair: Segment::All) {
            const int decimalValue = pair.getDecimalValue(),
                    binaryValue = pair.getBinaryValue();

            // If the two decimal values are equal, return the corresponding binary value
            if (decimalValue == decimal) return binaryValue;
        }

        // Return -1 if no matching decimal value has been found
        return -1;
    }
}

/**
 * The StateController structure will be used to control the current state of the safe.
 * Using the controller, we are able to properly change the state defined by an enumeration.
 *
 * @see State
 */
typedef struct StateController {
public:
    State currentState;
    Display currentDisplay;

    StateController(State currentState, Display currentDisplay) {
        this->currentState = currentState;
        this->currentDisplay = currentDisplay;
    }

    StateController() = default;

public:
    State getState() const {
        return currentState;
    }

    void setState(State newState) {
        this->currentState = newState;
    }

    Display getDisplay() const {
        return currentDisplay;
    }

    void setDisplay(Display newDisplay) {
        this->currentDisplay = newDisplay;
    }

    void nextDisplayMode() {
        switch (currentDisplay) {
            case Display::ONE:
                this->setDisplay(Display::TWO);
                break;
            case Display::TWO:
                this->setDisplay(Display::THREE);
                break;
            case Display::THREE:
                this->setDisplay(Display::ONE);
                break;
        }
    }
} StateController;

/**
 *
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
public:
    Servo servo;
    StateController stateController{};
    int position{};

    ServoController(Servo servo, StateController stateController, int position) {
        this->servo = servo;
        this->stateController = stateController;
        this->position = position;

    }

    ServoController() = default;

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

typedef struct DigitalController {
public:
    DigitalController() = default;

private:
    void write(int pin, int value) {
        digitalWrite(pin, value);
    }

    int read(int pin) {
        return digitalRead(pin);
    }

public:
    void updateSevenSegments(int valueOne, int valueTwo, int valueThree) {
//        this->write(SETTING__SEVEN_SEGMENT_ONE_PIN, valueOne);
//        this->write(SETTING__SEVEN_SEGMENT_TWO_PIN, valueTwo);
//        this->write(SETTING__SEVEN_SEGMENT_THREE_PIN, valueThree);

        digitalWrite(SETTING__SEVEN_SEGMENT_ONE_PIN, valueOne);
        digitalWrite(SETTING__SEVEN_SEGMENT_TWO_PIN, valueTwo);
        digitalWrite(SETTING__SEVEN_SEGMENT_THREE_PIN, valueThree);
    }

    int readRotaryEncoderAPosition() {
        return this->read(SETTING__ENCODER_PIN_A);
    }

    int readRotaryEncoderBPosition() {
        return this->read(SETTING__ENCODER_PIN_B);
    }

    int readRotaryEncoderButton() {
        return this->read(SETTING__ENCODER_BUTTON_PIN);
    }
} DigitalController;

typedef struct ShiftRegisterController {
public:
    ShiftRegisterController() = default;

public:
    void shift(int decimal) {
        digitalWrite(SETTING__LATCH_PIN, LOW);
        shiftOut(SETTING__DATA_PIN, SETTING__CLOCK_PIN, MSBFIRST, Segment::convertDecimal(decimal));
        digitalWrite(SETTING__LATCH_PIN, HIGH);
    }
} ShiftRegisterController;

typedef struct SegmentController {
public:
    DigitalController digitalController;
    ShiftRegisterController shiftRegisterController;

    SegmentController(DigitalController digitalController, ShiftRegisterController shiftRegisterController) {
        this->digitalController = digitalController;
        this->shiftRegisterController = shiftRegisterController;
    }

    SegmentController() = default;

private:
    int selectedSegment = 1, currentValueOne = 0, currentValueTwo = 0, currentValueThree = 0;

public:
    void display(Display display, int numberToDisplay) {
        shiftRegisterController.shift(numberToDisplay);
        switch (display) {
            case Display::ONE:
                digitalController.updateSevenSegments(HIGH, LOW, LOW);
                break;
            case Display::TWO:
                digitalController.updateSevenSegments(LOW, HIGH, LOW);
                break;
            case Display::THREE:
                digitalController.updateSevenSegments(LOW, LOW, HIGH);
                break;
        }
        delay(1);
    }

    void displayAll() {
        this->display(Display::ONE,this->getCurrentOne());
        this->display(Display::TWO,this->getCurrentTwo());
        this->display(Display::THREE,this->getCurrentThree());

        Serial.println("t");
    }

    int getSelectedSegment() const {
        return this->selectedSegment;
    }

    int getCurrentOne() const {
        return this->currentValueOne;
    }

    int getCurrentTwo() const {
        return this->currentValueTwo;
    }

    int getCurrentThree() const {
        return this->currentValueThree;
    }

    void setSelectedSegment(const int newSelectedSegment) {
        this->selectedSegment = newSelectedSegment;
    }

    void setCurrentOne(const int newValueOne) {
        this->currentValueOne = newValueOne;
    }

    void setCurrentTwo(const int newValueTwo) {
        this->currentValueTwo = newValueTwo;
    }

    void setCurrentThree(const int newValueThree) {
        this->currentValueThree = newValueThree;
    }
} SegmentController;

typedef struct RotaryController {
public:
    int test = 0;
    DigitalController digitalController;
    SegmentController segmentController;

public:
    RotaryController(const DigitalController digitalController, const SegmentController segmentController) {
        this->digitalController = digitalController;
        this->segmentController = segmentController;
    }

    RotaryController() = default;

private:
    int encoderPosition = -1, internalRotaryState = LOW, rotaryEncoderPinALast = LOW;

public:
    int getEncoderPosition() const {
        return this->encoderPosition;
    }

    void setEncoderPosition(const int newEncoderPosition) {
        this->encoderPosition = newEncoderPosition;
    }

    int readEncoderPosition() {
//        int test = 0;
        this->internalRotaryState = this->digitalController.readRotaryEncoderAPosition();
//        Serial.print("internalRotaryState = ");
//        Serial.println(internalRotaryState);
        if (this->rotaryEncoderPinALast == LOW && this->internalRotaryState == HIGH) {


            if(digitalRead(SETTING__ENCODER_PIN_B) == LOW)
            {
                test--;
                if(test == -1)
                {
                    test = 9;
                }
            }
            else
            {
                test++;
                if(test == 10)
                {
                    test = 0;
                }
            }
//            if (this->digitalController.readRotaryEncoderBPosition() == LOW) {
//                this->setEncoderPosition((this->getEncoderPosition() - 1) == -1 ? 9 : this->getEncoderPosition());
//                test = (test - 1) == -1 ? 9 : test;
//            } else {
//                this->setEncoderPosition((this->getEncoderPosition() + 1) == 10 ? 0 : this->getEncoderPosition());
//                test = (test + 1) == 10 ? 0 : test;
//            }

//            Serial.print(">>>> getEncoderPosition = ");
//            Serial.println(this->getEncoderPosition());
        }

        this->rotaryEncoderPinALast = this->internalRotaryState;
        return test;
    }

    void checkInputEncoderPositionAndButton() {
        static int selectedSegment = this->segmentController.getSelectedSegment();
        if (this->digitalController.readRotaryEncoderButton() == LOW) {
            switch (selectedSegment) {
                case 1:
                    this->setEncoderPosition(this->segmentController.getCurrentOne());
                    break;
                case 2:
                    this->setEncoderPosition(this->segmentController.getCurrentTwo());
                    break;
                case 3:
                    this->setEncoderPosition(this->segmentController.getCurrentThree());
                    break;
                default:
                    break;
            }

            this->segmentController.setSelectedSegment(this->segmentController.getSelectedSegment() + 1);
            while (this->digitalController.readRotaryEncoderButton() == LOW) {
                this->segmentController.displayAll();
            }
        }

        switch (selectedSegment) {
            case 1:
                this->segmentController.setCurrentOne(this->getEncoderPosition());
                break;
            case 2:
                this->segmentController.setCurrentTwo(this->getEncoderPosition());
                break;
            case 3:
                this->segmentController.setCurrentThree(this->getEncoderPosition());
                break;
            default:
                break;
        }
    }
} RotaryController;

typedef struct PasswordController {
private:
    Password currentPassword = Password(0000);

private:
    void put(Password password) {
        EEPROM.put(0, password);
    }

    void reset() {
        EEPROM.put(0, 0);
    }

    Password read() {
        Password password = Password(0000);
        EEPROM.get(0, password);
        return password;
    }

public:
    void savePassword(int code) {
        this->currentPassword.setCode(code);
        this->put(this->currentPassword);
    }

    Password readPassword() {
        return this->read();
    }

    Password getPassword() const {
        return this->currentPassword;
    }

    void resetPassword() {
        this->currentPassword = Password(0000);
        this->reset();
    }
} PasswordController;

// Define global variables which are mandatory for the program to function properly.
ServoController servoController;
StateController stateController;
DigitalController digitalController;
ShiftRegisterController shiftRegisterController;
SegmentController segmentController;
RotaryController rotaryController;
PasswordController passwordController;

/**
 * Setup function
 */
void setup() {
    // Initialize a new Servo instance, and attach it to a pin
    Servo localServo;
    localServo.attach(7);

    // Initialize the State controller by creating a new StateController structure
    stateController = StateController(SAFE_CLOSED, ONE);

    // Initialize the Servo controller by creating a new ServoController structure
    servoController = ServoController(localServo, stateController, 0);

    // Initialize the Digital controller by creating a new DigitalController structure
    digitalController = DigitalController();

    // Initialize the ShiftRegister controller by creating a new ShiftRegisterController structure
    shiftRegisterController = ShiftRegisterController();

    // Initialize the Segment controller by creating a new SegmentController structure
    segmentController = SegmentController(digitalController, shiftRegisterController);

    // Initialize the Rotary controller by creating a new RotaryController structure
    rotaryController = RotaryController(digitalController, segmentController);

    // Initialize the Password controller by creating a new PasswordController structure
    passwordController = PasswordController();

    // Start a new Serial instance
    Serial.begin(SERIAL_BOUND);

    // Set all the output pins to the correct mode
    pinMode(SETTING__DATA_PIN, OUTPUT);
    pinMode(SETTING__CLOCK_PIN, OUTPUT);
    pinMode(SETTING__LATCH_PIN, OUTPUT);
    pinMode(SETTING__SEVEN_SEGMENT_ONE_PIN, OUTPUT);
    pinMode(SETTING__SEVEN_SEGMENT_TWO_PIN, OUTPUT);
    pinMode(SETTING__SEVEN_SEGMENT_THREE_PIN, OUTPUT);

    // Set all the input pins to the correct mode
    pinMode(SETTING__ENCODER_PIN_A, INPUT);
    pinMode(SETTING__ENCODER_PIN_B, INPUT);
    pinMode(SETTING__ENCODER_BUTTON_PIN, INPUT);
}

/**
 * testing function, delete if done
 */
void testing() {
    shiftOut(0, 0, MSBFIRST, Segment::convertDecimal(1));
    int binary = Segment::convertDecimal(1);

    printf("%d", binary);

    // Open and wait 1 second
    servoController.setOpen();
    servoController.rotate();
    delay(1000);

    // Close and wait 1 second
    servoController.setClosed();
    servoController.rotate();
    delay(1000);
}

/**
* The main thread function of the program
*/
void loop() {
    const State currentState = stateController.getState();
    const int currentEncoderPosition = rotaryController.readEncoderPosition();

//    Serial.print("pos = ");
//    Serial.println(currentEncoderPosition);

//    segmentController.displayAll();

    segmentController.display(Display::ONE,6);

    switch (currentState) {
        case State::SAFE_OPEN:

            break;

        case State::SAFE_CLOSED:

            break;

        case State::BLOCKED:

            break;

    }
}