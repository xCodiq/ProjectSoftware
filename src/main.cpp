// Include all the libraries used in this program
#include <Arduino.h>
#include <EEPROM.h>
#include <Servo.h>

// Define constant values, which will be used in our program
#define SERIAL_BOUND 9600
#define SAFE_CLOSED_POSITION_DEGREES 0
#define SAFE_OPEN_POSITION_DEGREES 90

// Pin settings
#define SETTING__SEVEN_SEGMENT_ONE_PIN 2 //2
#define SETTING__SEVEN_SEGMENT_TWO_PIN 3 //3
#define SETTING__SEVEN_SEGMENT_THREE_PIN 4 //4

#define SETTING__DATA_PIN 5 //5
#define SETTING__LATCH_PIN 6 //6
#define SETTING__CLOCK_PIN 7 //7

#define SETTING__SENSOR_PIN A3
#define SETTING__SERVO_PIN 12
#define SETTING__BUZZER_PIN A0
#define SETTING__RED_LED_PIN A2
#define SETTING__GREEN_LED_PIN A1

#define SETTING__ENCODER_PIN_A 8 //8
#define SETTING__ENCODER_PIN_B 9 //9
#define SETTING__ENCODER_BUTTON_PIN 13 //13

#define SETTING__RESET_BUTTON_PIN 10
#define SETTING__CONFIRM_BUTTON_PIN 11

/**
 * This enum type definition will be used to determine the current state of the safe. The save
 * can be in several states which are explain below.
 *
 * - SAFE_OPEN: The safe is currently opened, which means a correct password is entered.
 * - SAFE_CLOSED: The safe is currently closed, which means a password is yet to be entered.
 * - SAFE_RESET: The safe is currently resetting, which means a password is being entered at this moment.
 * - BLOCKED: The safe is in a blocked state, which means the user is not able to re-enter a password at the time.
 */
typedef enum State {
    SAFE_OPEN, SAFE_CLOSED, SAFE_RESET, BLOCKED
} State;

/**
 * This enum type definition will be used to have a clear separation of our 7Segement Displays.
 *
 * - ONE: The first 7Segment Display
 * - TWO: The second 7Segment Display
 * - THREE: The third 7Segment Display
 */
typedef enum Display {
    ONE = 1, TWO = 2, THREE = 3
} Display;

/**
 * The password structure is a model we use to save the actual integer code into the Arduino memory using EEPROM
 *
 * @field code The code of the saved password structure
 */
typedef struct Password {
private:
    int code;

public:
    Password(int defaultCode) {
        this->code = defaultCode;
    }

    /**
     * Get the current integer code of the password instance
     * @return the code as an integer type
     */
    int getCode() const {
        return this->code;
    }

    /**
     * Set the current integer code of the password instance (This will only change it in local memory, not EEPROM)
     * @param newCode the new code as integer type
     */
    void setCode(const int newCode) {
        this->code = newCode;
    }
} Password;

/**
 * The segment namespace will be used as an utility to mainly convert a decimal number to a binary value
 *
 * Example:
 *
 * ZERO (decimal=0, binary=B0)
 */
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

    /**
     * A static method to iteratively search for a valid conversion pair
     *
     * @param decimal the decimal integer value
     * @return the binary value which was paired with the decimal value
     */
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
    State getState() {
        return this->currentState;
    }

    void setState(State newState) {
        this->currentState = newState;
    }

    Display getDisplay() {
        return this->currentDisplay;
    }

    void setDisplay(Display newDisplay) {
        this->currentDisplay = newDisplay;
    }

    /**
     * This method will be used to switch to the next display, using a switch statement for simplicity
     */
    void nextDisplayMode() {
        switch (this->currentDisplay) {
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
    }

    /**
     * Calling this function will result in a open phase of the Servo followed by a open state change
     * The angle is an constant angle, which is defined at the top of the source code
     *
     */
    void setOpen() {
        this->position = SAFE_OPEN_POSITION_DEGREES;
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

/**
 * The DigitalController structure will be used to control all the pins for both INPUT and OUTPUT pinModes
 * There are two private methods which are used to call the internal arduino write/read methods
 */
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
        this->write(SETTING__SEVEN_SEGMENT_ONE_PIN, valueOne);
        this->write(SETTING__SEVEN_SEGMENT_TWO_PIN, valueTwo);
        this->write(SETTING__SEVEN_SEGMENT_THREE_PIN, valueThree);
    }

    void writeRedLed(int value) {
        this->write(SETTING__RED_LED_PIN, value);
    }

    void writeGreenLed(int value) {
        this->write(SETTING__GREEN_LED_PIN, value);
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

    unsigned long lastConfirmButtonRead = millis();

    int readConfirmButton() {
        int read = this->read(SETTING__CONFIRM_BUTTON_PIN);
        if (millis() - lastConfirmButtonRead > 250 && read == HIGH) {
            lastConfirmButtonRead = millis();
            return HIGH;
        }

        return LOW;
    }

    unsigned long lastResetButtonRead = millis();

    int readResetButton() {
        int read = this->read(SETTING__RESET_BUTTON_PIN);
        if (millis() - lastResetButtonRead > 250 && read == HIGH) {
            lastResetButtonRead = millis();
            return HIGH;
        }

        return LOW;
    }


    int readSensorButton() {
        return this->read(SETTING__SENSOR_PIN);
    }
} DigitalController;

/**
 * The ShiftRegisterController will control the shift register based on the Arduino's shiftOut function.
 */
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

/**
 * The SegmentController structure will control the actual 7segment displays themselves.
 * Using the display function, you can simply provide a Display enum type and a decimal value to update a segment.
 */
typedef struct SegmentController {
public:
    DigitalController digitalController;
    ShiftRegisterController shiftRegisterController;

    SegmentController(DigitalController digitalController,
                      ShiftRegisterController shiftRegisterController) {
        this->digitalController = digitalController;
        this->shiftRegisterController = shiftRegisterController;
    }

    SegmentController() = default;

private:
    int selectedSegment = 1, blockedCountdown = 5, currentValueOne = 0, currentValueTwo = 0, currentValueThree = 0;
    unsigned long lastBlockedCountdownUpdate = 0;

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

    void updateCurrentBasedOnDisplay(Display display, int numberToDisplay) {
        switch (display) {
            case Display::ONE:
                this->setCurrentOne(numberToDisplay);
                break;
            case Display::TWO:
                this->setCurrentTwo(numberToDisplay);
                break;
            case Display::THREE:
                this->setCurrentThree(numberToDisplay);
                break;
        }
    }

    void displayManual(int one, int two, int three) {
        this->display(Display::ONE, one);
        this->display(Display::TWO, two);
        this->display(Display::THREE, three);
    }

    void displayAll() {
        this->display(Display::ONE, this->getCurrentOne());
        this->display(Display::TWO, this->getCurrentTwo());
        this->display(Display::THREE, this->getCurrentThree());
    }

    int updateBlockedCountdown() {
        if (millis() - lastBlockedCountdownUpdate > 1000) {
            this->setCurrentOne(0);
            this->setCurrentTwo(this->blockedCountdown / 10 % 10);
            this->setCurrentThree(this->blockedCountdown % 10);

            if (this->blockedCountdown-- < 0) return -1;
            lastBlockedCountdownUpdate = millis();
        }

        return this->blockedCountdown;
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

/**
 * The RotaryController stucture will control everything that has something to do with the rotary encoder. Combined
 * with the DigitalController, it's able to perform actions to flawlessly read and write input/ouput
 */
typedef struct RotaryController {
public:
    DigitalController digitalController;

public:
    RotaryController(DigitalController digitalController) {
        this->digitalController = digitalController;
    }

    RotaryController() = default;

private:
    int encoderPosition = -2, internalRotaryState = LOW, rotaryEncoderPinALast = LOW;

public:
    int getEncoderPosition() const {
        return this->encoderPosition;
    }

    void setEncoderPosition(const int newEncoderPosition) {
        this->encoderPosition = newEncoderPosition;
    }

    void recoverEncoderValueOfDisplay(Display display, int currentOne, int currentTwo, int currentThree) {
        switch (display) {
            case Display::ONE:
                this->setEncoderPosition(currentOne);
                break;
            case Display::TWO:
                this->setEncoderPosition(currentTwo);
                break;
            case Display::THREE:
                this->setEncoderPosition(currentThree);
                break;
            default:
                break;
        }
    }

    int readEncoderPosition() {
        this->internalRotaryState = this->digitalController.readRotaryEncoderAPosition();
        if (this->rotaryEncoderPinALast == LOW && this->internalRotaryState == HIGH) {

            if (this->digitalController.readRotaryEncoderBPosition() == LOW) {
                this->setEncoderPosition(this->getEncoderPosition() - 1);
                if (this->getEncoderPosition() == -1) this->setEncoderPosition(9);
            } else {
                this->setEncoderPosition(this->getEncoderPosition() + 1);
                if (this->getEncoderPosition() == 10) this->setEncoderPosition(0);
            }
        }

        this->rotaryEncoderPinALast = this->internalRotaryState;
        return this->getEncoderPosition();
    }

    unsigned long lastDisplayModeCheck = 0;

    int checkDisplayModeChange() {
        if (this->digitalController.readRotaryEncoderButton() == LOW
            && millis() - lastDisplayModeCheck > 300) {
            lastDisplayModeCheck = millis();
            return 1;
        }
        return 0;
    }
} RotaryController;

/**
 * The BuzzerController structure will control the buzzer used to play certain tones. The buzzer is programmed to
 * always play at a frequency of 1100Hz.
 */
typedef struct BuzzerController {
public:
    BuzzerController() = default;

public:
    void play(int timeInSeconds) {
        tone(SETTING__BUZZER_PIN, 1100, timeInSeconds * 1000);
    }

    void stop() {
        noTone(SETTING__BUZZER_PIN);
    }
} BuzzerController;

/**
 * The PasswordController structure is the controller of the Password structure to handle actions like, saving, reading,
 * and resetting the safe password.
 *
 * @see Password
 */
typedef struct PasswordController {
private:
    Password currentPassword = Password(000);
    int passwordTries = 3;

private:
    void put(Password password) {
        EEPROM.put(0, password);
    }

    void reset() {
        EEPROM.put(0, 0);
    }

    Password read() {
        Password password = Password(000);
        EEPROM.get(0, password);
        return password;
    }

public:
    void savePassword(int code) {
        this->currentPassword.setCode(code);
        this->put(this->currentPassword);

        Serial.print("Password saved: ");
        Serial.println(code);
    }

    Password readPassword() {
        return this->read();
    }

    Password getPassword() const {
        return this->currentPassword;
    }

    int convertDigits(int first, int second, int third) {
        int one = first * 100;
        int two = second * 10;
        return one + two + third;
    }

    void resetPassword() {
        this->currentPassword = Password(000);
        this->reset();
    }

    void decreasePasswordTries() {
        this->passwordTries--;
    }

    int getPasswordTries() const {
        return this->passwordTries;
    }

    void resetPasswordTries() {
        this->passwordTries = 3;
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
BuzzerController buzzerController;

/**
 * Setup function
 */
void setup() {
    // Initialize a new Servo instance, and attach it to a pin
    Servo localServo;
    localServo.attach(SETTING__SERVO_PIN);

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
    rotaryController = RotaryController(digitalController);

    // Initialize the Password controller by creating a new PasswordController structure
    passwordController = PasswordController();

    // Initialize the Buzzer controller by creating a new BuzzerController structure
    buzzerController = BuzzerController();

    // Start a new Serial instance
    Serial.begin(SERIAL_BOUND);

    // Set all the output pins to the correct mode
    pinMode(SETTING__DATA_PIN, OUTPUT);
    pinMode(SETTING__CLOCK_PIN, OUTPUT);
    pinMode(SETTING__LATCH_PIN, OUTPUT);
    pinMode(SETTING__SEVEN_SEGMENT_ONE_PIN, OUTPUT);
    pinMode(SETTING__SEVEN_SEGMENT_TWO_PIN, OUTPUT);
    pinMode(SETTING__SEVEN_SEGMENT_THREE_PIN, OUTPUT);
    pinMode(SETTING__RED_LED_PIN, OUTPUT);
    pinMode(SETTING__GREEN_LED_PIN, OUTPUT);

    // Set all the input pins to the correct mode
    pinMode(SETTING__ENCODER_PIN_A, INPUT);
    pinMode(SETTING__ENCODER_PIN_B, INPUT);
    pinMode(SETTING__ENCODER_BUTTON_PIN, INPUT);
    pinMode(SETTING__CONFIRM_BUTTON_PIN, INPUT);
    pinMode(SETTING__RESET_BUTTON_PIN, INPUT);

    // Boot up reset
    servoController.setClosed();
    servoController.rotate();
}

/**
* The main thread function of the program
*/
void loop() {
    State currentState = digitalController.readSensorButton() == HIGH ? State::SAFE_CLOSED : stateController.getState();
    Display currentDisplay = stateController.getDisplay();

    int currentDigitOne = segmentController.getCurrentOne();
    int currentDigitTwo = segmentController.getCurrentTwo();
    int currentDigitThree = segmentController.getCurrentThree();

    // Open state
    if (currentState == State::SAFE_OPEN) {
        digitalController.writeGreenLed(HIGH);

        if (digitalController.readResetButton() == HIGH) {
            stateController.setState(State::SAFE_RESET);
            Serial.println("Switched to resetting state");
            return;
        }

        if (digitalController.readConfirmButton() == HIGH) {
            stateController.setState(SAFE_CLOSED);
            servoController.setClosed();
            servoController.rotate();
            digitalController.writeGreenLed(LOW);
            return;
        }

        // Closed state
    } else if (currentState == State::SAFE_CLOSED) {
        // Check if the push-button is pressed
        if (rotaryController.checkDisplayModeChange() == 1) {
            stateController.nextDisplayMode();

            Serial.print("Switched to display #");
            Serial.println(stateController.getDisplay());
        }

        // Check if the reset button is pressed
        if (digitalController.readResetButton() == HIGH) {
            segmentController.updateCurrentBasedOnDisplay(Display::ONE, 0);
            segmentController.updateCurrentBasedOnDisplay(Display::TWO, 0);
            segmentController.updateCurrentBasedOnDisplay(Display::THREE, 0);
            return;
        }

        if (digitalController.readConfirmButton() == HIGH) {
            int convertedPassword = passwordController.convertDigits(currentDigitOne, currentDigitTwo,
                                                                     currentDigitThree);

            // Check if the password is not correct, if so, fail unlocking part
            Serial.println(passwordController.readPassword().getCode());
            if (passwordController.readPassword().getCode() != convertedPassword) {
                Serial.print("Password incorrect (#");
                Serial.print(3 - passwordController.getPasswordTries() + 1);
                Serial.println(" try)");

                // Check if the user tried to enter the password too many times
                passwordController.decreasePasswordTries();
                if (passwordController.getPasswordTries() <= 0) {
                    buzzerController.play(2);
                    segmentController.displayManual(0,0,0);
                    stateController.setState(State::BLOCKED);
                    return;
                } else {
                    buzzerController.play(1);
                    digitalController.writeRedLed(HIGH);
                    segmentController.displayManual(0,0,0);
                    delay(1000);
                    digitalController.writeRedLed(LOW);
                    return;
                }

                // The password is correct, so the safe should be opened
            } else {
                Serial.println("Password correct");
                digitalController.writeGreenLed(HIGH);
                servoController.setOpen();
                servoController.rotate();
                stateController.setState(SAFE_OPEN);
                return;
            }
        }

        // Recover the encoder values of the displays
        rotaryController.recoverEncoderValueOfDisplay(currentDisplay, currentDigitOne, currentDigitTwo,
                                                      currentDigitThree);
        // Update the current displays based on the encoder position
        segmentController.updateCurrentBasedOnDisplay(currentDisplay, rotaryController.readEncoderPosition());

        // Blocked state
    } else if (currentState == State::BLOCKED) {
        int blockedCountdown = segmentController.updateBlockedCountdown();
        digitalController.writeRedLed(HIGH);

        if (blockedCountdown == -1) {
            digitalController.writeRedLed(LOW);
            passwordController.resetPasswordTries();
            stateController.setState(State::SAFE_CLOSED);

            Serial.println("Blocked state has expired");
            return;
        }

        // Resetting state
    } else if (currentState == State::SAFE_RESET) {
        // Check if the confirm button is pressed
        if (digitalController.readConfirmButton() == HIGH) {
            int convertedPassword = passwordController.convertDigits(currentDigitOne, currentDigitTwo,
                                                                     currentDigitThree);
            passwordController.savePassword(convertedPassword);
            servoController.setClosed();
            servoController.rotate();
            stateController.setState(SAFE_CLOSED);
            digitalController.writeGreenLed(LOW);
            return;
        }

        // Check if the display mode change button is pressed
        if (rotaryController.checkDisplayModeChange() == 1) {
            stateController.nextDisplayMode();

            Serial.print("Switched to display ");
            Serial.println(stateController.getDisplay());
            return;
        }

        // Recover the encoder values of the displays
        rotaryController.recoverEncoderValueOfDisplay(currentDisplay, currentDigitOne, currentDigitTwo,
                                                      currentDigitThree);
        // Update the current displays based on the encoder position
        segmentController.updateCurrentBasedOnDisplay(currentDisplay, rotaryController.readEncoderPosition());
    }

    segmentController.displayManual(currentDigitOne, currentDigitTwo, currentDigitThree);
}