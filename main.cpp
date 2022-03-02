/*********************************************/
/*      Team A3 FEH Robot Project Code       */
/*            OSU FEH Spring 2022            */
/*                                           */
/*      Steven Broaddus, Conolly Burgess     */
/*        Joseph Richmond, Jake Chang        */
/*                                           */
/*            Updated 3/2/2022               */
/*       Uses Doxygen for documentation      */
/*********************************************/

/*
 * Game plan for tonight: 
 *
 * 1. Commit current code to GitHub for backup
 * 
 * 1.5. Clean code of any obvious issues.
 * 
 * 2. Check online documentation to see how much slower reversing motors are 
 * than forward Igwan motors.
 * 
 * 3. Get robot to drive straight (calibrate right motor)
 * 
 * 4. See if calibrated right motor can turn degress with accuracy. 
 * If not possible with both wheels, use only one forward wheel.
 * 
 * 5. Take detailed measurements of robot or check for Joe's model.
 * 
 * 5. Map out course with dimensions of robot accounted for. This will use the turn that 
 * I deem most accurate.
 * 
 * 6. Get to a decent spot.
 * 
 */

/************************************************/
// Include preprocessor directives
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHServo.h>
#include <cmath> // abs() 

/************************************************/
// Definitions
#define ROBOT_WIDTH 7.95 // Length of front/back side of OUR robot in inches
#define PI 3.14159265
#define BACKGROUND_COLOR WHITE // Background color of layout
#define FONT_COLOR BLACK // Font color of layout
#define BUTTON_TIME_TO_SLEEP 0.20 // Time to sleep (seconds) after button pressed for accessibility.

// Movement/Dimension calculations
#define DIST_AXIS_CDS (5.375 - 1.25) // Distance from the center of the wheel axis to the CdS cell.
#define COUNT_PER_INCH (318 / (2 * 3.14159265 * 1.25)) // Number of encoder counts per inch ((ENCODER_COUNTS_PER_REV / (2 * PI * WHEEL_RADIUS))) 

// Precise movement calibrations
#define BACKWARDS_CALIBRATOR 2.15 // Percent difference needed to make backward motors move the same as forward motors at 20% 
#define RIGHT_MOTOR_CALIBRATOR 1 

// Servo min/max values
#define BASE_SERVO_MIN 500
#define BASE_SERVO_MAX 2290
#define ON_ARM_SERVO_MIN 500
#define ON_ARM_SERVO_MAX 2400

/************************************************/
// Course numbers. Used in startMenu() and runCourse()
enum { 
            TEST_COURSE_1 = 1, 
            TEST_COURSE_2 = 2, 
            TEST_COURSE_3 = 3, 
            CALIBRATE_SERVOS = 4,
            PERF_COURSE_1 = 5, 
            PERF_COURSE_2 = 6, 
            PERF_COURSE_3 = 7, 
            PERF_COURSE_4 = 8, 
            IND_COMP = 9, 
            FINAL_COMP = 10
         };

/************************************************/
// Function Prototypes
int confirmation(const char prompt[], int xPrompt, int yPrompt); // Prompts the user to confirm a choice
void drawMainMenuScreen(FEHIcon::Icon test_button, FEHIcon::Icon perf_test_button, FEHIcon::Icon competition_button); // Draws the main menu screen
int startMenu(); // Sets up a starting menu for the user.
int readStartLight(); // Waits for the start light
void move_forward_inches(int percent, float inches); // Moves forward number of inches
void turn_right_degrees(int percent, float degrees); // Turns right a specified number of degrees
void turn_left_degrees(int percent, float degrees); // Turns left a specified amount of degrees
int detectColor(int timeToDetect); // Detects the color of the jukebox
void pressJukeboxButtons(); // Presses the jukebox buttons
void writeStatus(const char status[]); // Clears room for a printed string w/o clearing display
void showRPSData(); // Shows basic RPS data for the robot
void runCourse(int courseNumber); // Runs the course specified by startUp()

/************************************************/
// Declarations for encoders/motors
DigitalEncoder right_encoder(FEHIO::P3_2);
DigitalEncoder left_encoder(FEHIO::P3_1);
FEHMotor right_motor(FEHMotor::Motor2,9.0);
FEHMotor left_motor(FEHMotor::Motor3,9.0);

// Declarations for servos
// GROUND FARTHER SIDE
FEHServo base_servo(FEHServo::Servo5);
FEHServo on_arm_servo(FEHServo::Servo7);

// Declaration for CdS cell sensorsad 
AnalogInputPin CdS_cell(FEHIO::P0_7);

/*******************************************************
 * @brief Prompts the user to confirm their choice.
 * 
 * @param prompt Prompt for the user to confirm.
 * @param xPrompt X RC (Rows/Columns, see LCD.WriteRC documentation) to write at.
 * @param yPrompt Same as xPrompt, but y RC coord.
 * @return int The value of the user's decision. 
 *          Yes -> 1 (true)
 *          No  -> 0 (false)
 */
int confirmation(const char prompt[], int xPrompt, int yPrompt) {

    enum { 
            YES = true, 
            NO = false
         };

    // Coords of touch 
    int xTouch, yTouch;

    // Selection to check buttons
    int selection;

    // Initiates decision to invalid value
    int decision = -1;

    // Icons to display choices
    FEHIcon::Icon confirm[2];
    char confirm_labels[2][20] = {"Yes", "No"};

    // Sleeps to show "pressed" status of other buttons
    Sleep(BUTTON_TIME_TO_SLEEP);
    LCD.ClearBuffer();

    // Draws choices and prompt
    LCD.Clear();

    // Writes the passed-in prompt at the x, y RC coords
    LCD.WriteRC(prompt, xPrompt, yPrompt);

    FEHIcon::DrawIconArray(confirm, 1, 2, 100, 50, 50, 50, confirm_labels, FONT_COLOR, FONT_COLOR);

    // Waits a bit to not suddenly allow for a choice
    Sleep(BUTTON_TIME_TO_SLEEP);
    LCD.ClearBuffer();

    // While decision hasn't been made, wait for touch
    while (decision == -1)  
    {
        // Waits for touch and loops through buttons to see if they were pressed
        if (LCD.Touch(&xTouch, &yTouch)) {
            for (int i = 0; i < 2; i++) {
                if (confirm[i].Pressed(xTouch, yTouch, 0)) {
                    selection = i + 1;
                }
            }
        }

        // Checks which button has been pressed
        switch (selection) {
            case 1:
                decision = YES;
                break;
            case 2:
                decision = NO;
                break;
        }
    }

    return decision;
}

/*******************************************************
 * @brief Draws the main menu screen. In a function for re-use.
 * 
 * @param test_button Button that leads to the test menu
 * @param perf_test_button Button that leads to the performance test menu
 * @param competition_button Button that leads to the competition test menu
 */
void drawMainMenuScreen(FEHIcon::Icon test_button, FEHIcon::Icon perf_test_button, FEHIcon::Icon competition_button) {
    
    // Sleeps to show "pressed" status of other buttons
    Sleep(BUTTON_TIME_TO_SLEEP);
    LCD.ClearBuffer();

    LCD.Clear();

    // Prompts the user for selection
    LCD.WriteRC("What do?", 2, 9);

    // Draws the passed-in icons
    test_button.Draw(); 
    perf_test_button.Draw(); 
    competition_button.Draw(); 

    // Waits a bit to not suddenly allow for a choice
    Sleep(BUTTON_TIME_TO_SLEEP);
    LCD.ClearBuffer();
}

/*******************************************************
 * @brief Initializes the starting menu to choose a course.
 * 
 * @return int Course chosen, see defined course number enum.
 */
int startMenu() {
    
    // Coordinates for touch
    float xTouch, yTouch; 

    // Enums for menus
    enum { MAIN_MENU, TEST_MENU, PERFORMANCE_MENU, COMPETITION_MENU };

    // Initializes the screen
    LCD.SetBackgroundColor(BACKGROUND_COLOR);
    LCD.SetFontColor(FONT_COLOR);
    LCD.Clear();

    // Creates main menu icons
    char mainLabels[4][20] = {"Test", "Perf. Tests", "Competition", "Calibrate Servo"};
    FEHIcon::Icon test_button, perf_test_button, competition_button;
    test_button.SetProperties(mainLabels[0], 75, 75, 170, 40, FONT_COLOR, FONT_COLOR);
    perf_test_button.SetProperties(mainLabels[1], 75, 125, 170, 40, FONT_COLOR, FONT_COLOR); 
    competition_button.SetProperties(mainLabels[2], 75, 175, 170, 40, FONT_COLOR, FONT_COLOR); 

    // Creates button to calibrate servos
    FEHIcon::Icon calibrate_servo_button;
    calibrate_servo_button.SetProperties(mainLabels[3], 50, 75, 220, 40, FONT_COLOR, FONT_COLOR);

    // Assigns the labels for the different Tests
    FEHIcon::Icon testButtons[3];
    char test_button_labels[3][20] = {"1", "2", "3"};

    FEHIcon::Icon performanceTests[4];
    char performance_labels[4][20] = {"1", "2", "3", "4"};

    FEHIcon::Icon competitions[2];
    char competition_labels[2][20] = {"Ind.", "Final"};

    // Used in while loop to check for decisions
    int courseChosen = 0;
    int selection;
    int confirmed = false;

    // Sets the screen to the default of the MAIN_MENU
    int screen = MAIN_MENU;

    // Draws the MAIN_MENU
    drawMainMenuScreen(test_button, perf_test_button, competition_button);

    // Repeats until the user hasn't chosen a course and confirmed it
    while (!courseChosen && !confirmed) {
        
        // Checks buttons on MAIN_MENU
        while (screen == MAIN_MENU && !courseChosen) {

            if (LCD.Touch(&xTouch, &yTouch)) {

                if (test_button.Pressed(xTouch, yTouch, 0)) 
                { 
                    Sleep(BUTTON_TIME_TO_SLEEP);
                    LCD.Clear();
                    screen = TEST_MENU;
                    LCD.WriteRC("Tests!", 2, 10);
                    FEHIcon::DrawIconArray(testButtons, 1, 3, 150, 50, 50, 50, test_button_labels, FONT_COLOR, FONT_COLOR);
                    calibrate_servo_button.Draw();

                    // Waits a bit to not suddenly allow for a choice
                    Sleep(BUTTON_TIME_TO_SLEEP);
                    LCD.ClearBuffer();
                } 
                if (perf_test_button.Pressed(xTouch, yTouch, 0)) 
                { 
                    Sleep(BUTTON_TIME_TO_SLEEP);
                    LCD.Clear();
                    screen = PERFORMANCE_MENU;
                    LCD.WriteRC("Performance Tests", 1, 5);
                    FEHIcon::DrawIconArray(performanceTests, 2, 2, 50, 25, 25, 25, performance_labels, FONT_COLOR, FONT_COLOR);

                    // Waits a bit to not suddenly allow for a choice
                    Sleep(BUTTON_TIME_TO_SLEEP);
                    LCD.ClearBuffer();
                } 
                if (competition_button.Pressed(xTouch, yTouch, 0)) 
                { 
                    Sleep(BUTTON_TIME_TO_SLEEP);
                    LCD.Clear();
                    screen = COMPETITION_MENU;
                    LCD.WriteRC("Competitions", 2, 7);
                    FEHIcon::DrawIconArray(competitions, 1, 2, 75, 50, 50, 50, competition_labels, FONT_COLOR, FONT_COLOR);
                    // Waits a bit to not suddenly allow for a choice
                    Sleep(BUTTON_TIME_TO_SLEEP);
                    LCD.ClearBuffer();
                }
            }
        } // End MAIN_MENU button check

        // Checks buttons on TEST_MENU
        while (screen == TEST_MENU && !courseChosen) {
            
            // Initializes selection
            selection = 0;
            
            // Loops through test button array upon touch
            if (LCD.Touch(&xTouch, &yTouch)) {
                for (int i = 0; i < 3; i++) {
                    if (testButtons[i].Pressed(xTouch, yTouch, 0)) {
                        selection = i+1;
                    }
                }

                // Checks if calibrate servo button was pressed
                if (calibrate_servo_button.Pressed(xTouch, yTouch, 0)) {
                    selection = 4;
                }

                // Responds to which button was pressed
                switch (selection) {
                    case 1:
                        if (confirmation("Test 1?", 3, 10)) {
                            courseChosen = TEST_COURSE_1;
                        } else {
                            screen = MAIN_MENU;
                            drawMainMenuScreen(test_button, perf_test_button, competition_button);
                        }
                        break;

                    case 2:
                        if (confirmation("Test 2?", 3, 10)) {
                            courseChosen = TEST_COURSE_2;
                        } else {
                            screen = MAIN_MENU;
                            drawMainMenuScreen(test_button, perf_test_button, competition_button);
                        }
                        break;
                        
                    case 3:
                        if (confirmation("Test 3?", 3, 10)) {
                            courseChosen = TEST_COURSE_3;
                        } else {
                            screen = MAIN_MENU;
                            drawMainMenuScreen(test_button, perf_test_button, competition_button);
                        }
                        break;
                    case 4:
                        if (confirmation("Calibrate Servos?", 3, 5)) {
                            courseChosen = CALIBRATE_SERVOS;
                        } else {
                            screen = MAIN_MENU;
                            drawMainMenuScreen(test_button, perf_test_button, competition_button);
                        }
                        break;
    
                    default:
                        continue;
                }
            }
        } // End TEST_MENU button check

        // Checks buttons on PERFORMANCE_MENU
        while (screen == PERFORMANCE_MENU && !courseChosen) {

            // Initializes selection
            selection = 0;

            // Loops through performance button array upon touch
            if (LCD.Touch(&xTouch, &yTouch)) {
                for (int i = 0; i < 4; i++) {
                    if (performanceTests[i].Pressed(xTouch, yTouch, 0)) {
                        selection = i+1;
                    }
                }

                // Responds to which button was pressed
                switch (selection) {
                    case 1:
                        if (confirmation("Perf. 1?", 3, 9)) {
                            courseChosen = PERF_COURSE_1;
                        } else {
                            screen = MAIN_MENU;
                            drawMainMenuScreen(test_button, perf_test_button, competition_button);
                        }
                        break;

                    case 2:
                        if (confirmation("Perf. 2?", 3, 9)) {
                            courseChosen = PERF_COURSE_2;
                        } else {
                            screen = MAIN_MENU;
                            drawMainMenuScreen(test_button, perf_test_button, competition_button);
                        }
                        break;

                    case 3:
                        if (confirmation("Perf. 3?", 3, 9)) {
                            courseChosen = PERF_COURSE_3;
                        } else {
                            screen = MAIN_MENU;
                            drawMainMenuScreen(test_button, perf_test_button, competition_button);
                        }
                        break;

                    case 4:
                        if (confirmation("Perf. 4?", 3, 9)) {
                            courseChosen = PERF_COURSE_4;
                        } else {
                            screen = MAIN_MENU;
                            drawMainMenuScreen(test_button, perf_test_button, competition_button);
                        }
                        break;
                    default:
                        continue;
                }
            }
        } // End PERFORMANCE_MENU button check

        // Checks buttons on COMPETITION_MENU
        while (screen == COMPETITION_MENU && !courseChosen) {

            // Initializes selection
            selection = 0;

            // Loops through performance button array upon touch
            if (LCD.Touch(&xTouch, &yTouch)) {
                for (int i = 0; i < 3; i++) {
                    if (competitions[i].Pressed(xTouch, yTouch, 0)) {
                        selection = i+1;
                    }
                }

                // Responds to which button was pressed
                switch (selection) {
                    case 1:
                        if (confirmation("Ind. Comp.?", 3, 8)) {
                            courseChosen = IND_COMP;
                        } else {
                            screen = MAIN_MENU;
                            drawMainMenuScreen(test_button, perf_test_button, competition_button);
                        }
                        break;

                    case 2:
                        if (confirmation("Final Comp.?", 3, 8)) {
                            courseChosen = FINAL_COMP;
                        } else {
                            screen = MAIN_MENU;
                            drawMainMenuScreen(test_button, perf_test_button, competition_button);
                        }
                        break;

                    default:
                        continue;
                }
            }
        } // End COMPETITION_MENU button check
        
    }

    return courseChosen;

}

/*******************************************************
 * @brief Waits until the start light to run the course
 * 
 * @return int The status of the light
 *         1 -> ON
 *         0 -> OFF
 */
int readStartLight() {
    LCD.Clear();

    int lightOn = 0;

    writeStatus("Waiting for light");

    // Waits until light is detected
    while (!lightOn) {
        // Writes out CdS value to the screen
        LCD.WriteRC("CdS Value: ", 7, 2);
        LCD.WriteRC(CdS_cell.Value(), 7, 20);

        // If 
        if (CdS_cell.Value() < 0.345) {
            lightOn = 1;
            writeStatus("GO!");
        }
    }

    return lightOn;
}

/*******************************************************
 * @brief Moves CENTER OF ROBOT forward a number of inches using encoders
 * @author Steven Broaddus
 * @param percent - Percent for the motors to run at. Negative for reverse.
 * @param inches - Inches to move forward .
 */
void move_forward_inches(int percent, float inches) {
    // Calculates desired counts based on the radius of the wheels and the robot
    float expectedCounts = COUNT_PER_INCH * inches;

    // Clears space for movement data and status
    LCD.SetFontColor(BACKGROUND_COLOR);
    LCD.FillRectangle(0,100,319,239);
    LCD.SetFontColor(FONT_COLOR);
    LCD.DrawHorizontalLine(100, 0, 319);

    // Writes out status to the screen
    LCD.WriteRC("Moving forward...", 7, 1);

    // Resets encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    // Sets both motors to same percentage, but accounts for one motor moving backwards
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);

    // Keeps running until average motor counts are in proper range
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < expectedCounts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();

    //Print out data
    LCD.WriteRC("Theoretical Counts: ", 9, 1);
    LCD.WriteRC(expectedCounts, 9, 20);
    LCD.WriteRC("Motor Percent: ", 10, 1);
    LCD.WriteRC(percent, 10, 20);
    LCD.WriteRC("Actual LE Counts: ", 11, 1);
    LCD.WriteRC(left_encoder.Counts(), 11, 20);
    LCD.WriteRC("Actual RE Counts: ", 12, 1);
    LCD.WriteRC(right_encoder.Counts(), 12, 20);
}

/*******************************************************
 * @brief Turns right a certain amount of degrees
 * 
 * @param percent - Percent for the motors to run at. Negative for reverse.
 * @param degrees - Degrees to rotate.
 */
void turn_right_degrees(int percent, float degrees) {

    // Calculates desired counts based on the radius of the wheels and the robot
    float expectedCounts = COUNT_PER_INCH * ((degrees * PI) / 180.0) * (ROBOT_WIDTH / 2);

    // Clears space for movement data and status
    LCD.SetFontColor(BACKGROUND_COLOR);
    LCD.FillRectangle(0,100,319,239);
    LCD.SetFontColor(FONT_COLOR);
    LCD.DrawHorizontalLine(100, 0, 319);

    // Writes out status to the screen
    LCD.WriteRC("Turning Right...", 7, 2);

    // Resets encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    // Sets both motors to specific percentage
    right_motor.SetPercent(-percent - BACKWARDS_CALIBRATOR);
    left_motor.SetPercent(percent);

    // Keeps running until average motor counts are in proper range
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < expectedCounts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();

    //Print out data
    LCD.WriteRC("Theoretical Counts: ", 9, 1);
    LCD.WriteRC(expectedCounts, 9, 20);
    LCD.WriteRC("Motor Percent: ", 10, 1);
    LCD.WriteRC(percent, 10, 20);
    LCD.WriteRC("Actual LE Counts: ", 11, 1);
    LCD.WriteRC(left_encoder.Counts(), 11, 20);
    LCD.WriteRC("Actual RE Counts: ", 12, 1);
    LCD.WriteRC(right_encoder.Counts(), 12, 20);
}

/*******************************************************
 * @brief Turns left a certain amount of degrees
 * 
 * @param percent - Percent for the motors to run at. Negative for reverse.
 * @param degrees - Degrees to rotate.
 */
void turn_left_degrees(int percent, float degrees) {

    // Calculates desired counts based on the radius of the wheels and the robot
    float expectedCounts = COUNT_PER_INCH * ((degrees * PI) / 180.0) * (ROBOT_WIDTH / 2);

    // Clears space for movement data and status
    LCD.SetFontColor(BACKGROUND_COLOR);
    LCD.FillRectangle(0,100,319,239);
    LCD.SetFontColor(FONT_COLOR);
    LCD.DrawHorizontalLine(100, 0, 319);

    // Writes out status to the screen
    LCD.WriteRC("Turning Left...", 7, 2);

    // Resets encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    // Sets both motors to specific percentage
    right_motor.SetPercent(percent);
    left_motor.SetPercent(-percent - BACKWARDS_CALIBRATOR);

    // Keeps running until average motor counts are in proper range
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < expectedCounts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
    
    //Print out data
    LCD.WriteRC("Theoretical Counts: ", 9, 1);
    LCD.WriteRC(expectedCounts, 9, 20);
    LCD.WriteRC("Motor Percent: ", 10, 1);
    LCD.WriteRC(percent, 10, 20);
    LCD.WriteRC("Actual LE Counts: ", 11, 1);
    LCD.WriteRC(left_encoder.Counts(), 11, 20);
    LCD.WriteRC("Actual RE Counts: ", 12, 1);
    LCD.WriteRC(right_encoder.Counts(), 12, 20);
}

/*******************************************************
 * @brief Initiates both servos, sets min/max values and 
 * turns it to starting rotation.
 */
void initiateServos() {
    
    // Calibrates base servo
    base_servo.SetMin(BASE_SERVO_MIN);
    base_servo.SetMax(BASE_SERVO_MAX);

    // Calibrate on-arm servo
    on_arm_servo.SetMin(ON_ARM_SERVO_MIN);
    on_arm_servo.SetMax(ON_ARM_SERVO_MAX);

    // Sets base servo to initial degree
    base_servo.SetDegree(85.);
    on_arm_servo.SetDegree(180.);
}

/*******************************************************
 * @brief Detects the color using the CdS cell
 * 
 * @return int color Color detected.
 *          0 -> Red
 *          1 -> Blue
 * @param timeToDetect time the robot takes to detect if it doesn't see the color right away
 */
int detectColor(int timeToDetect) {
    LCD.Clear();

    // Color to be returned
    int color = -1;

    // Initiates variables to find average value
    double startTime = TimeNow();
    double sum = 0.0;
    int numValues = 0;
    double averageValue = sum / numValues; // Average value of CdS cell values

    // 0 if color hasn't been detected yet, 1 if otherwise
    int colorFound = 0;

    // Reads values for four seconds OR until color is found
    while (( (TimeNow() - startTime) < timeToDetect) && !colorFound) {
        
        // Takes the average value read
        sum += CdS_cell.Value();
        numValues++;
        averageValue = sum / numValues;

        // Detects color using CdS_cell values
        if (CdS_cell.Value() < 0.345) {
            color = 0;
            colorFound = 1;
        } else if (CdS_cell.Value() > 0.345) {
            color = 1;
            colorFound = 1;
        } else {
            color = 0; // Default is red
        }

        // Prints out info
        LCD.SetBackgroundColor(BACKGROUND_COLOR);
        LCD.SetFontColor(FONT_COLOR);
        LCD.Clear();

        LCD.WriteRC("Reading color: ", 1, 3);
        LCD.WriteRC(TimeNow() - startTime, 1, 18); // Time elapsed

        LCD.WriteRC("CdS Value: ", 3, 4);
        LCD.WriteRC(CdS_cell.Value(), 3, 15); // CdS cell value

        LCD.WriteRC("Color: ", 5, 7);

        // Prints which color is recognized
        switch (color)
        {
        case 0:
            LCD.SetFontColor(RED);
            LCD.WriteRC("Red", 5, 15);
            break;

        case 1: 
            LCD.SetFontColor(BLUE);
            LCD.WriteRC("Blue", 5, 15);
            break;
        
        default:
            LCD.WriteRC("Other", 5, 15);
            break;
        }

        // Prints out the average value
        LCD.SetFontColor(FONT_COLOR);
        LCD.WriteRC("Average Value: ", 7, 6);
        LCD.WriteRC(averageValue, 7, 15);
        Sleep(0.1);
    }

    return color;
}

/*******************************************************
 * @brief Presses jukebox buttons based on color.
 * @author Steven Broaddus
 */
void pressJukeboxButtons() {
    
    /*
     * Detects the color of the jukebox
     * 
     * 0 -> Red      
     * 1 -> Blue
     */ 
    int color = detectColor(4);

    // Responds to the jukebox light appropriately
    if (color == 0) { // On right path (red light)
        turn_right_degrees(20, 35);
        move_forward_inches(20, 2.75);
        turn_left_degrees(20, 35);

        float startTime = TimeNow();

        while (TimeNow() - startTime < 1.0) {
            // Sets both motors to same percentage
            right_motor.SetPercent(20);
            left_motor.SetPercent(20);
        }

        right_motor.Stop();
        left_motor.Stop();

        Sleep(2.0);

        // Reverses
        startTime = TimeNow();

        while (TimeNow() - startTime < 1.0) {
            // Sets both motors to same percentage
            right_motor.SetPercent(-20);
            left_motor.SetPercent(-20);
        }

        turn_right_degrees(20, 35);
        move_forward_inches(-20, 2.75);
        turn_left_degrees(20, 35);
        

    } else if (color == 1) { // On left path (blue light)

        turn_left_degrees(20, 35);
        move_forward_inches(20, 2.75);
        turn_right_degrees(20, 35);

        float startTime = TimeNow();

        while (TimeNow() - startTime < 1.0) {
            // Sets both motors to same percentage
            right_motor.SetPercent(20);
            left_motor.SetPercent(20);
        }

        right_motor.Stop();
        left_motor.Stop();

        Sleep(2.0);

        // Reverses
        startTime = TimeNow();

        while (TimeNow() - startTime < 1.0) {
            // Sets both motors to same percentage
            right_motor.SetPercent(-20);
            left_motor.SetPercent(-20);
        }


        turn_left_degrees(20, 35);
        move_forward_inches(-20, 2.75);
        turn_right_degrees(20, 35);

    } else {
        LCD.Write("ERROR: COLOR NOT READ SUCCESFULLY");
    }
}

/*******************************************************
 * @brief Clears room for status and prints it to screen 
 * without clearing the screen
 * 
 * @param status Status to be printed
 */
void writeStatus(const char status[]) {
    LCD.SetFontColor(BACKGROUND_COLOR);
    LCD.FillRectangle(0, 17,319,17);
    LCD.SetFontColor(FONT_COLOR);
    LCD.WriteRC(status,1,2);
}

/*******************************************************
 * @brief Shows the current RPS data.
 * 
 * @pre RPS must be initialized.
 * 
 */
void showRPSData() {

    LCD.DrawHorizontalLine(100, 0, 319);

    // Writes the data from the RPS
    writeStatus("Reading RPS Data");

    LCD.WriteRC("Heading: ", 7, 1);
    LCD.WriteRC(RPS.Heading(), 7, 10);

    LCD.WriteRC("X Value: ", 8, 1);
    LCD.WriteRC(RPS.X(), 8, 10);

    LCD.WriteRC("Y Value: ", 9, 1);
    LCD.WriteRC(RPS.Y(), 9, 10);

    LCD.WriteRC("Time: ", 10, 1);
    LCD.WriteRC(RPS.Time(), 10, 10);

    LCD.WriteRC("Course: ", 11, 1);
    LCD.WriteRC(RPS.CurrentRegionLetter(), 11, 10);
}

/*******************************************************
 * @brief Runs the specified course
 * @author Steven Broaddus
 * @param courseNumber Course number to runs
 */
void runCourse(int courseNumber) {

    /*
     * NOTE: Status messages from movement functions only clear the 
     * portion of the screen that they use. They only do this beforehand.
     * 
     * writeStatus() is used to print what the robot is doing without 
     * clearing the movement status (turn left/right etc..)
     */

    // Creates room for status messages
    LCD.Clear();
    
    switch (courseNumber)
    {
    case TEST_COURSE_1: // Test course 1
        writeStatus("Running Test 1");

        int xGarb, yGarb;

        Sleep(1.0);
        while(true) {
            writeStatus("Press to turn left.");
            while(!LCD.Touch(&xGarb, &yGarb));
            turn_left_degrees(40, 90);
            while(!LCD.Touch(&xGarb, &yGarb));
            turn_right_degrees(40, 90);
        }
        writeStatus("Complete.");
        break;

    case TEST_COURSE_2: // Test course 1
        writeStatus("Running Test 2");

        int xTrash2, yTrash2;

        Sleep(1.0);
        writeStatus("Press to move forward");
        
        while(!LCD.Touch(&xTrash2, &yTrash2));
        while(true) {
            move_forward_inches(20, 9999);
        }

        break;

    case TEST_COURSE_3: // Test course 1
        writeStatus("Running Test 3");

        float degreesToTurnBase, degreesToTurnArm;
        int xTrash, yTrash;
        degreesToTurnBase = 90.0;
        degreesToTurnArm = 90.0;

        base_servo.SetDegree(degreesToTurnBase);
        on_arm_servo.SetDegree(degreesToTurnArm);

        LCD.DrawHorizontalLine(40, 0, 319);
        LCD.DrawHorizontalLine(140, 0, 319);
        LCD.DrawVerticalLine(160, 0, 239);
        
        while (true) {

            while(!LCD.Touch(&xTrash, &yTrash));

                LCD.SetFontColor(BACKGROUND_COLOR);
                LCD.FillRectangle(0, 0, 159, 39);
                LCD.SetFontColor(FONT_COLOR);

                // Base Servo
                if (xTrash < 160) {
                    if (yTrash < 120) {
                        if ((degreesToTurnBase + 2.5) >= 0) {
                            degreesToTurnBase += 2.5;
                            base_servo.SetDegree(degreesToTurnBase);
                        }
                    } else {
                        if ((degreesToTurnBase - 2.5) <= 180) {
                            degreesToTurnBase - 2.5;
                            base_servo.SetDegree(degreesToTurnBase);
                        }
                    }
                    
                    LCD.WriteRC("Base:", 1, 0);
                    LCD.WriteRC(degreesToTurnBase, 1, 6);

                } else { // Arm servo
                    if (yTrash < 120) {
                        if ((degreesToTurnArm + 2.5) >= 0) {
                            degreesToTurnArm += 2.5;
                            on_arm_servo.SetDegree(degreesToTurnArm);
                        }
                    } else {
                        if ((degreesToTurnArm - 2.5) <= 180) {
                            degreesToTurnArm - 2.5;
                            on_arm_servo.SetDegree(degreesToTurnArm);
                        }
                    }

                    LCD.WriteRC("Arm:", 1, 14);
                    LCD.WriteRC(degreesToTurnArm, 1, 19);
                }    
        }

        break;

    case CALIBRATE_SERVOS:
        writeStatus("Calibrating Servos");
        Sleep(1.0);
        writeStatus("L -> base | R -> arm");

        int xTrash420, yTrash69;

        // Waits for touch. If touch is on left then base_servo is calibrated, and vice versa
        while(!LCD.Touch(&xTrash420, &yTrash69));

        LCD.DrawVerticalLine(160, 20, 239);

        if (xTrash420 < 160) {
            base_servo.TouchCalibrate();
        } else if (xTrash420 > 160) {
            on_arm_servo.TouchCalibrate();
        }

        break;

    case PERF_COURSE_1: // Performance Test 1

        /*****
         * THIS PSEUDO-CODE NEEDS UPDATING
         * Algorithm for performance test 1:
         * All degree measurements are CCW from +x
         * 
         * 1. Move forward 7.5 inches
         * 
         * 2. Rotate left 45 degrees (until heading is 180 degrees)
         * 
         * 3. Move forward 11.9 inches
         * 
         * 4. Rotate left 90 degrees (until heading is 270 degrees)
         * 
         * 4.5. reverse so that wheel axis is over CdS cell
         * 
         * 5. Read light
         * 
         * 6. Respond to light value (display it too)
         * 
         * Following is in a function: 
         * if (color == red (on right path)) {
         *      Rotate 35 degrees right (heading == 235 degrees)
         *      Move forward 2.75 inches
         *      Rotate 35 degrees left (heading == 270 degrees)
         *      Move forward until button is pressed (1 second)
         *      Reverse that much distance
         *      Rotate 35 right 
         *      Reverse 2.75 inches
         *      Rotate 35 left 
         * 
         * } else if (color == blue (on left path)) {
         *      Rotate 35 degrees left (heading == 305 degrees)
         *      Move forward 2.75 inches
         *      Rotate 35 degrees right (heading == 270 degrees)
         *      Move forward until button is pressed (1 second)
         *      Reverse that much distance
         *      Rotate 35 left 
         *      Reverse 2.75 inches
         *      Rotate 35 degrees right
         * 
         * 6.5. Turn left 85 degrees
         * 
         * 7. Move forward 9 inches to align with ramp
         * 
         * 8. Rotate left 90 degrees (heading == 90 degrees)
         * 
         * 9. Move forward 11 inches (base of ramp)
         * 
         * 10. Move forward 10 inches (across ramp)
         * 
         * 11. Move forward 14 inches (from ramp)
         * 
         * 12. Reverse that much (35 inches)
         * 
         * 13. Rotate 90 degrees right (heading == 0 degrees)
         * 
         * 14. Move forward 2.9 inches 
         * 
         * 15. Rotate 45 degrees right (heading == 315 degrees)
         * 
         * 16. Move forward until button is pressed (7.5 inches?)
         * 
         * 17. Win.
         * 
         */ 

        writeStatus("Running Perf. Test 1");

        Sleep(1.0);
        
        /***************************************************/

        writeStatus("Moving towards jukebox");

        // Heads from button to center
        move_forward_inches(20, 8.0 + DIST_AXIS_CDS); // Direct: 7.5 inches 
        Sleep(1.0);

        // Moves towards jukebox
        turn_left_degrees(20, 43);
        Sleep(1.0);

        move_forward_inches(20, 12);
        Sleep(1.0);

        turn_left_degrees(20, 89);
        Sleep(1.0);

        //Reverses to move CdS cell over jukebox light
        move_forward_inches(-20, 0.75 + DIST_AXIS_CDS);
    
       /***************************************************/

        writeStatus("Pressing jukebox buttons");
        
        // Presses jukebox buttons
        pressJukeboxButtons();
        
        /***************************************************/

        // Moves forward to move wheel axis over jukebox light
        move_forward_inches(20, DIST_AXIS_CDS);

        writeStatus("Moving towards ramp");

        // Moves to center (aligns with ramp)
        turn_left_degrees(20, 85);
        Sleep(1.0);
        move_forward_inches(20, 9);
        Sleep(1.0);
        turn_left_degrees(20, 90);
        Sleep(1.0);
        
        writeStatus("Moving up ramp");

        // Moves up ramp
        move_forward_inches(35, 35); // 11 + 10 + 14
        Sleep(1.0);

        writeStatus("Moving down ramp");
        
        // Moves down ramp
        move_forward_inches(-35, 35);
        Sleep(1.0);

        writeStatus("Towards final button");

        // Heads toward final button
        turn_right_degrees(20, 90);
        Sleep(1.0);
        move_forward_inches(20, 2.9);
        Sleep(1.0);
        turn_right_degrees(20, 45);
        Sleep(1.0);
        move_forward_inches(20, 7.5);
        Sleep(1.0);

        writeStatus("Woo?");
        
        break;
        

    case PERF_COURSE_2: // Performance Test 2

        /****
         * 
         * Algorithm for Performance Test 2 
         * 
         * 1. Move forward 11.55 inches + DIST_AXIS_CDS (Aligns with ramp)
         * 
         * 2. Turn right 45 degrees (Straight at ramp)
         * 
         * 3. Move forward 8.1 inches (Base of ramp)
         * 
         * 4. Move forward 10.3 inches (Top of ramp)
         * 
         * 5. Move forward 13.35 inches + DIST_ACIS_CDS 
         * 
         * 6. Turn right 90 degrees (To reverse towards sink)
         * 
         * 7. Reverse 10.5 inches 
         * 
         * 8. Turn left 90 degrees (To reverse towards sink)
         * 
         * 9. Reverse 7 inches (To sink)
         * 
         * 10. Drop tray
         * 
         * 11. More forward 7 inches (From sink)
         * 
         * 12. Turn left 90 degrees (To reverse towards ticket) 
         * 
         * 13. Reverse 10.5 inches
         * 
         * 14. Reverse 12.15 inches (Towards ticket)
         * 
         * 15. Turn left 90 degrees (front towards ticket)
         * 
         * 16. Move forard 8.1 inches (Front of ticket)
         * 
         * 17. Slide ticket
         * 
         */

        LCD.Write("Running Performance Test 2");

        Sleep(1.0);

    
        writeStatus("Aligning with ramp");
        move_forward_inches(20, 11.55 + DIST_AXIS_CDS);
        turn_right_degrees(20, 45);

        writeStatus("Moving up ramp");
        move_forward_inches(40, 31.75 + DIST_AXIS_CDS);

        writeStatus("Moving towards sink");
        turn_right_degrees(20, 90);
        move_forward_inches(-20, 10.5); // Reverses
        turn_left_degrees(20, 90);
        move_forward_inches(-20, 7);
    
        writeStatus("Dropping tray");

        base_servo.SetDegree(85.);
        Sleep(1.0);
        base_servo.SetDegree(105.);
        Sleep(2.5);
        base_servo.SetDegree(85.);

        writeStatus("Moving away from sink");
        move_forward_inches(20, 7);
        turn_left_degrees(20, 90);
        move_forward_inches(-20, 10.5);

        writeStatus("Moving towards ticket");
        move_forward_inches(-20, 12.15);
        turn_left_degrees(20, 90);
        move_forward_inches(20, 8.1);

        writeStatus("Sliding ticket");
    
        break;

    case PERF_COURSE_3: // Performance Test 3
        LCD.Write("Running Performance Test 3");
        break;

    case PERF_COURSE_4: // Performance Test 4
        LCD.Write("Running Performance Test 4");
        break;

    case IND_COMP: // Individual Competition
        LCD.Write("Running Ind. Competition");
        break;

    case FINAL_COMP: // Final Competition
        LCD.Write("Running Final Competition");
        break;
    
    default:
        LCD.WriteRC("ERROR: NO COURSE SPECIFIED", 1, 0);
        break;
    }
} 

/*****************************************************************
 * main
 */
int main() {

    float xTrash, yTrash;

    // Initiates servos
    initiateServos();

    // Initializes RPS
    //RPS.InitializeTouchMenu();

    // Initializes menu and returns chosen course number
    int courseNumber = startMenu();

    //Waits until start light is read
    //readStartLight();
    //Sleep(1.0);

    // Runs specified course number.
    runCourse(courseNumber);

    return 0;
}