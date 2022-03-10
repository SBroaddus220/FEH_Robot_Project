/*********************************************/
/*      Team A3 FEH Robot Project Code       */
/*            OSU FEH Spring 2022            */
/*                                           */
/*      Steven Broaddus, Conolly Burgess     */
/*        Joseph Richmond, Jake Chang        */
/*                                           */
/*            Updated 3/7/2022               */
/*       Uses Doxygen for documentation      */
/*********************************************/

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
#define INCH_PER_COUNT ((2 * 3.14159265 * 1.25) / 318) // ^ but opposite

// Precise movement calibrations
#define BACKWARDS_CALIBRATOR 2.15 // Percent difference needed to make backward motors move the same as forward motors at 20% 
#define RIGHT_MOTOR_CALIBRATOR 1 

// Servo min/max values
#define BASE_SERVO_MIN 500
#define BASE_SERVO_MAX 2290
#define ON_ARM_SERVO_MIN 500
#define ON_ARM_SERVO_MAX 2400

// RPS pulse values
#define RPS_DELAY_TIME 0.35 // Time that the RPS takes to check again before correcting

#define RPS_TURN_PULSE_PERCENT 20 // Percent at which motors will pulse to correct movement while turning
#define RPS_TURN_PULSE_TIME 0.05 // Time that the wheels pulse for to correct heading
#define RPS_TURN_THRESHOLD 0.5 // Degrees that the heading can differ from before calling it a day

#define RPS_TRANSLATIONAL_PULSE_PERCENT 20 // Percent at which motors will pulse to correct translational movement
#define RPS_TRANSLATIONAL_PULSE_TIME 0.1 // Time that the wheels pulse for to correct translational coords
#define RPS_TRANSLATIONAL_THRESHOLD 0.25 // Coord units that the robot can be in range of

/************************************************/
// Course numbers. Used in start_menu() and run_course()
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
void draw_main_menu_screen(FEHIcon::Icon test_button, FEHIcon::Icon perf_test_button, FEHIcon::Icon competition_button); // Draws the main menu screen
int start_menu(); // Sets up a starting menu for the user.
int read_start_light(); // Waits for the start light
void move_forward_inches(int percent, float inches); // Moves forward number of inches
void move_forward_seconds(float percent, float seconds); // Moves forward for a number of seconds
void turn_right_degrees(int percent, float degrees); // Turns right a specified number of degrees
void turn_left_degrees(int percent, float degrees); // Turns left a specified amount of degrees
void RPS_correct_heading(float heading); // Corrects the heading of the robot using RPS
void RPS_check_x(float x_coord); // Corrects the x-coord of the robot using RPS
void RPS_check_y(float y_coord); // Corrects the y-coord of the robot using RPS
int detect_color(int timeToDetect); // Detects the color of the jukebox
void press_jukebox_buttons(); // Presses the jukebox buttons
void flip_burger(); // Flips the hot plate and burger
void flip_ice_cream_lever(); // Flips the correct ice cream lever
void write_status(const char status[]); // Clears room for a printed string w/o clearing display
void show_RPS_data(); // Shows basic RPS data for the robot
void run_course(int courseNumber); // Runs the course specified by startUp()

/************************************************/
// Declarations for encoders/motors
// WHITE ENCODER -> LEFT MOTOR
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
void draw_main_menu_screen(FEHIcon::Icon test_button, FEHIcon::Icon perf_test_button, FEHIcon::Icon competition_button) {
    
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
int start_menu() {
    
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
    draw_main_menu_screen(test_button, perf_test_button, competition_button);

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
                            draw_main_menu_screen(test_button, perf_test_button, competition_button);
                        }
                        break;

                    case 2:
                        if (confirmation("Test 2?", 3, 10)) {
                            courseChosen = TEST_COURSE_2;
                        } else {
                            screen = MAIN_MENU;
                            draw_main_menu_screen(test_button, perf_test_button, competition_button);
                        }
                        break;
                        
                    case 3:
                        if (confirmation("Test 3?", 3, 10)) {
                            courseChosen = TEST_COURSE_3;
                        } else {
                            screen = MAIN_MENU;
                            draw_main_menu_screen(test_button, perf_test_button, competition_button);
                        }
                        break;
                    case 4:
                        if (confirmation("Calibrate Servos?", 3, 5)) {
                            courseChosen = CALIBRATE_SERVOS;
                        } else {
                            screen = MAIN_MENU;
                            draw_main_menu_screen(test_button, perf_test_button, competition_button);
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
                            draw_main_menu_screen(test_button, perf_test_button, competition_button);
                        }
                        break;

                    case 2:
                        if (confirmation("Perf. 2?", 3, 9)) {
                            courseChosen = PERF_COURSE_2;
                        } else {
                            screen = MAIN_MENU;
                            draw_main_menu_screen(test_button, perf_test_button, competition_button);
                        }
                        break;

                    case 3:
                        if (confirmation("Perf. 3?", 3, 9)) {
                            courseChosen = PERF_COURSE_3;
                        } else {
                            screen = MAIN_MENU;
                            draw_main_menu_screen(test_button, perf_test_button, competition_button);
                        }
                        break;

                    case 4:
                        if (confirmation("Perf. 4?", 3, 9)) {
                            courseChosen = PERF_COURSE_4;
                        } else {
                            screen = MAIN_MENU;
                            draw_main_menu_screen(test_button, perf_test_button, competition_button);
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
                            draw_main_menu_screen(test_button, perf_test_button, competition_button);
                        }
                        break;

                    case 2:
                        if (confirmation("Final Comp.?", 3, 8)) {
                            courseChosen = FINAL_COMP;
                        } else {
                            screen = MAIN_MENU;
                            draw_main_menu_screen(test_button, perf_test_button, competition_button);
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
int read_start_light() {
    LCD.Clear();

    int lightOn = 0;

    write_status("Waiting for light");

    // Waits until light is detected
    while (!lightOn) {
        // Writes out CdS value to the screen
        LCD.WriteRC("CdS Value: ", 7, 2);
        LCD.WriteRC(CdS_cell.Value(), 7, 20);

        // If 
        if (CdS_cell.Value() < 0.345) {
            lightOn = 1;
            write_status("GO!");
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
 * @brief Moves forward for the specified time at the specified percentage.
 * 
 * @param percent Percent that the motors will drive at
 * @param seconds Time that the motors will drive for
 */
void move_forward_seconds(float percent, float seconds) {
    
    // Set both motors to passed percentage
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);

    Sleep(seconds);

    // Turns off motors after elapsed time
    right_motor.Stop();
    left_motor.Stop();
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
 * @brief Corrects the heading using RPS, pulses to correct heading
 * 
 * @param headingDegrees Heading to correct to in degrees
 */
void RPS_correct_heading(float heading) {
    /*
     * Determines the direction to turn to get to the desired heading faster
     * 1 -> CW
     * -1 -> CCW
     */ 
    int direction = 0;

    // Difference between the actual heading and the desired one
    float difference;
    if ((heading - RPS.Heading()) > 180) {
        difference = abs((heading) - (RPS.Heading() + 360)); 
    } else if ((RPS.Heading() - heading) > 180) {
        difference = abs((heading + 360) - (RPS.Heading())); 
    } else {
        difference = abs((heading + 360) - (RPS.Heading() + 360)); 
    }
    
    
    // Determine the direction of the motors based on the orientation of the QR code
    int power = RPS_TURN_PULSE_PERCENT;

    // Check if receiving proper RPS coordinates and whether the robot is within an acceptable range
    while((RPS.Heading() >= 0) && (difference > RPS_TURN_THRESHOLD))
    {
        // Checks which way to turn to turn the least
        if (RPS.Heading() < heading) {
            direction = 1;
        } else {
            direction = -1;
        }

        if (abs(RPS.Heading() - heading) > 180) {
            direction = -direction;
        }

        // Pulses towards the ideal position
        if (direction == 1) {

            // PULSES COUNTERCLOCKWISE
            // Set both motors to desired percent
            right_motor.SetPercent(RPS_TURN_PULSE_PERCENT);
            left_motor.SetPercent(-RPS_TURN_PULSE_PERCENT - BACKWARDS_CALIBRATOR);

            // Wait for the correct number of seconds
            Sleep(RPS_TURN_PULSE_TIME);

            // Turn off motors
            right_motor.Stop();
            left_motor.Stop();
            
        } else if (direction == -1) {

            // PULSES CLOCKWISE
            // Set both motors to desired percent
            right_motor.SetPercent(-RPS_TURN_PULSE_PERCENT - BACKWARDS_CALIBRATOR);
            left_motor.SetPercent(RPS_TURN_PULSE_PERCENT);

            // Wait for the correct number of seconds
            Sleep(RPS_TURN_PULSE_TIME);

            // Turn off motors
            right_motor.Stop();
            left_motor.Stop();

        }

        // Waits a tiny bit before checking RPS again
        Sleep(RPS_DELAY_TIME);

        // Updates direction to turn to
        if ((heading - RPS.Heading()) > 180) {
            difference = abs((heading) - (RPS.Heading() + 360)); 
        } else if ((RPS.Heading() - heading) > 180) {
            difference = abs((heading + 360) - (RPS.Heading())); 
        } else {
            difference = abs((heading + 360) - (RPS.Heading() + 360)); 
        }

        show_RPS_data();
    }
}

/*******************************************************
 * @brief Checks and corrects the x-coord of the robot using RPS. Makes sure the robot is facing 
 * east/west to correct movement
 * 
 * @param x_coord Desired x-coord of the robot
 */
void RPS_check_x(float x_coord) {

    // Directions the robot needs to be facing to correct x-coord
    enum { EAST, WEST };

    // Initial heading of the robot
    float orientation = RPS.Heading();

    // Direction the robot is corrected to
    int direction;

    // Makes sure robot can be seen by RPS
    if (orientation > 0) {

        write_status("Correcting x with RPS");
        
        // Adjusts robot to be facing east/west based on initial orientation
        if ((orientation <= 90) || (orientation >= 270)) {
            RPS_correct_heading(0);
            direction = EAST;
        } else {
            RPS_correct_heading(180);
            direction = WEST;
        }

        // Determine the direction of the motors based on the direction the robot is facing
        int power = RPS_TRANSLATIONAL_PULSE_PERCENT;
        if(direction == WEST){
            power = -RPS_TRANSLATIONAL_PULSE_PERCENT;
        }

        // Check if receiving proper RPS coordinates and whether the robot is within an acceptable range
        while((RPS.X() > 0) && (abs(RPS.X() - x_coord) > RPS_TRANSLATIONAL_THRESHOLD))
        {
            if(RPS.X() > x_coord)
            {
                // Pulse the motors for a short duration in the correct direction
                move_forward_seconds(-power, RPS_TRANSLATIONAL_PULSE_TIME);
            }
            else if(RPS.X() < x_coord)
            {
                // Pulse the motors for a short duration in the correct direction
                move_forward_seconds(power, RPS_TRANSLATIONAL_PULSE_TIME);
            }
            Sleep(RPS_DELAY_TIME);

            show_RPS_data();
        }

        RPS_correct_heading(orientation);


    } else {
        write_status("ERROR. RPS NOT READING.");
    }
}

/*******************************************************
 * @brief Checks and corrects the y-coord of the robot using RPS. Makes sure the robot is facing 
 * north/south to correct movement.
 * 
 * @param y_coord Desired y-coord of the robot
 */
void RPS_check_y(float y_coord) {

    // Directions the robot needs to be facing to correct y-coord
    enum { NORTH, SOUTH };

    // Initial heading of the robot
    float orientation = RPS.Heading();

    // Direction the robot is corrected to
    int direction;

    // Makes sure robot can be seen by RPS
    if (orientation > 0) {

        write_status("Correcting y with RPS");
        
        // Adjusts robot to be facing east/west based on initial orientation
        if ((orientation >= 0) && (orientation <= 180)) {
            RPS_correct_heading(90);
            direction = NORTH;
        } else {
            RPS_correct_heading(270);
            direction = SOUTH;
        }

        // Determine the direction of the motors based on the direction the robot is facing
        int power = RPS_TRANSLATIONAL_PULSE_PERCENT;
        if(direction == SOUTH){
            power = -RPS_TRANSLATIONAL_PULSE_PERCENT;
        }

        // Check if receiving proper RPS coordinates and whether the robot is within an acceptable range
        while((RPS.Y() > 0) && (abs(RPS.Y() - y_coord) > RPS_TRANSLATIONAL_THRESHOLD))
        {
            if(RPS.Y() > y_coord)
            {
                // Pulse the motors for a short duration in the correct direction
                move_forward_seconds(-power, RPS_TRANSLATIONAL_PULSE_TIME);
            }
            else if(RPS.Y() < y_coord)
            {
                // Pulse the motors for a short duration in the correct direction
                move_forward_seconds(power, RPS_TRANSLATIONAL_PULSE_TIME);
            }
            Sleep(RPS_DELAY_TIME);

            show_RPS_data();
        }

        RPS_correct_heading(orientation);


    } else {
        write_status("ERROR. RPS NOT READING.");
    }
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
    on_arm_servo.SetDegree(8.);
}

/*******************************************************
 * @brief Detects the color using the CdS cell
 * 
 * @return int color Color detected.
 *          0 -> Red
 *          1 -> Blue
 * @param timeToDetect time the robot takes to detect if it doesn't see the color right away
 */
int detect_color(int timeToDetect) {
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
void press_jukebox_buttons() {
    
    /*
     * Detects the color of the jukebox
     * 
     * 0 -> Red      
     * 1 -> Blue
     */ 
    int color = detect_color(4);

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
 * @brief Algorithm for flipping the hot plate when robot is ~13 inches in front of it. 
 * Facing directly at it.
 * 
 */
void flip_burger() {

    write_status("Flipping hot plate");

    //***********
    // Initial flip

    // Sets initial arm positions
    base_servo.SetDegree(85);
    on_arm_servo.SetDegree(8);

    Sleep(0.5);

    // Lowers base servo and moves it under hot plate
    base_servo.SetDegree(0);
    move_forward_inches(20, 2.25);
    
    Sleep(1.0);

    // Raises arm and moves forward consecutively
    base_servo.SetDegree(20); // First lift
    move_forward_inches(20, 2);
    Sleep(0.25);

    base_servo.SetDegree(45); // Second lift
    move_forward_inches(20, 1.25);

    turn_right_degrees(20, 15); // Turns right to help flip burger
    Sleep(0.5);

    on_arm_servo.SetDegree(130); // Second arm finishes push

    Sleep(2.0);

    //***********
    // Return flip

    write_status("Moving towards other side");

    // Resets position
    on_arm_servo.SetDegree(8.); // Resets on arm servo position
    turn_left_degrees(20, 5); // Readjusts angle

    // Heads towards other side
    move_forward_inches(-20, 4); // Reverse away from hot plate 
    RPS_correct_heading(90);
    RPS_check_y(55);
    Sleep(0.5);

    base_servo.SetDegree(85); // Raise base arm and head towards other side
    turn_left_degrees(20, 90);
    RPS_correct_heading(180);
    RPS_check_x(23.3);
    move_forward_inches(-20, 5.8); // 6.3 is measured value
    turn_right_degrees(20, 90);
    RPS_correct_heading(90);
    on_arm_servo.SetDegree(180);

    write_status("Returning hot plate");

    // Flips back hot plate
    move_forward_inches(20, 5); 
    base_servo.SetDegree(50);
    Sleep(1.0);
    on_arm_servo.SetDegree(15);
    Sleep(2.0);
    on_arm_servo.SetDegree(180);
    Sleep(1.0);
    base_servo.SetDegree(85);
}

/*******************************************************
 * @brief Flips the correct ice cream lever. 
 * 
 * @pre RPS must be initialized.
 */
void flip_ice_cream_lever() {

    // Makes sure on arm servo is out of the way
    on_arm_servo.SetDegree(180);

    write_status("Pushing lever down");
    base_servo.SetDegree(85);
    move_forward_inches(20, 8);
    base_servo.SetDegree(60);
    Sleep(7.0);

    write_status("Pushing lever up");
    move_forward_inches(-20, 8);
    base_servo.SetDegree(0); 
    move_forward_inches(20, 8);
    base_servo.SetDegree(40);
    move_forward_inches(-20, 8);

    /*
    if (RPS.GetIceCream() == 0) { // VANILLA

        

    } else if (RPS.GetIceCream() == 1) { // TWIST

        write_status("Pushing lever down");
        base_servo.SetDegree(85);
        move_forward_inches(20, 3);
        base_servo.SetDegree(60);
        Sleep(7.0);

        write_status("Pushing lever up");
        move_forward_inches(-20, 3);
        base_servo.SetDegree(0); 
        move_forward_inches(20, 3);
        base_servo.SetDegree(60);
        move_forward_inches(-20, 3);

    } else if (RPS.GetIceCream() == 2) { // CHOCOLATE

    } else {
        write_status("ERROR. ICE CREAM LEVER NOT SPECIFIED.");
    }*/

}

/*******************************************************
 * @brief Clears room for status and prints it to screen 
 * without clearing the screen
 * 
 * @param status Status to be printed
 */
void write_status(const char status[]) {
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
void show_RPS_data() {

    // Clears space for movement data and status
    LCD.SetFontColor(BACKGROUND_COLOR);
    LCD.FillRectangle(0,100,319,239);
    LCD.SetFontColor(FONT_COLOR);
    LCD.DrawHorizontalLine(100, 0, 319);

    // Writes the data from the RPS
    write_status("Reading RPS Data");

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

    Sleep(0.1);
}

/*******************************************************
 * @brief Runs the specified course
 * @author Steven Broaddus
 * @param courseNumber Course number to runs
 */
void run_course(int courseNumber) {

    /*
     * NOTE: Status messages from movement functions only clear the 
     * portion of the screen that they use. They only do this beforehand.
     * 
     * write_status() is used to print what the robot is doing without 
     * clearing the movement status (turn left/right etc..)
     */

    // Creates room for status messages
    LCD.Clear();
    
    switch (courseNumber)
    {
    case TEST_COURSE_1: // Test course 1
        write_status("Running Test 1");

        int xGarb, yGarb;

        Sleep(1.0);
        while(true) {
            write_status("Press to turn left.");
            while(!LCD.Touch(&xGarb, &yGarb));
            turn_left_degrees(40, 90);
            while(!LCD.Touch(&xGarb, &yGarb));
            turn_right_degrees(40, 90);
        }
        write_status("Complete.");
        break;

    case TEST_COURSE_2: // Test course 1
        write_status("Running Test 2");

        int xTrash2, yTrash2;

        Sleep(1.0);
        write_status("Press to move forward");
        
        while(!LCD.Touch(&xTrash2, &yTrash2));
        while(true) {
            move_forward_inches(20, 9999);
        }

        break;

    case TEST_COURSE_3: // Test course 1
        write_status("Running Test 3");

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
        write_status("Calibrating Servos");
        Sleep(1.0);
        write_status("L -> base | R -> arm");

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

        write_status("Running Perf. Test 1");

        Sleep(1.0);
        
        /***************************************************/

        write_status("Moving towards jukebox");

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

        write_status("Pressing jukebox buttons");
        
        // Presses jukebox buttons
        press_jukebox_buttons();
        
        /***************************************************/

        // Moves forward to move wheel axis over jukebox light
        move_forward_inches(20, DIST_AXIS_CDS);

        write_status("Moving towards ramp");

        // Moves to center (aligns with ramp)
        turn_left_degrees(20, 85);
        Sleep(1.0);
        move_forward_inches(20, 9);
        Sleep(1.0);
        turn_left_degrees(20, 90);
        Sleep(1.0);
        
        write_status("Moving up ramp");

        // Moves up ramp
        move_forward_inches(35, 35); // 11 + 10 + 14
        Sleep(1.0);

        write_status("Moving down ramp");
        
        // Moves down ramp
        move_forward_inches(-35, 35);
        Sleep(1.0);

        write_status("Towards final button");

        // Heads toward final button
        turn_right_degrees(20, 90);
        Sleep(1.0);
        move_forward_inches(20, 2.9);
        Sleep(1.0);
        turn_right_degrees(20, 45);
        Sleep(1.0);
        move_forward_inches(20, 7.5);
        Sleep(1.0);

        write_status("Woo?");
        
        break;
        

    case PERF_COURSE_2: // Performance Test 2

        LCD.Write("Running Performance Test 2");

        Sleep(1.0);

        write_status("Aligning with ramp");
        move_forward_inches(20, 11.55 + DIST_AXIS_CDS);
        turn_right_degrees(20, 45);

        write_status("Moving up ramp");
        move_forward_inches(40, 31.75 + DIST_AXIS_CDS);

        write_status("Moving towards sink");
        turn_right_degrees(20, 90);
        move_forward_inches(-20, 10.5); // Reverses
        turn_left_degrees(20, 90);
        move_forward_inches(-20, 8);
    
        write_status("Dropping tray");

        base_servo.SetDegree(85.);
        Sleep(1.0);
        base_servo.SetDegree(105.);
        Sleep(2.5);
        base_servo.SetDegree(85.);

        write_status("Moving away from sink");
        move_forward_inches(20, 8);
        turn_right_degrees(20, 90);
        move_forward_inches(20, 10.5);
        turn_left_degrees(20, 185);

        write_status("Moving towards ticket");
        move_forward_inches(-20, 13.15);
        turn_left_degrees(20, 90);

        write_status("Sliding ticket");
        on_arm_servo.SetDegree(45);
        Sleep(1.0);
        base_servo.SetDegree(0);

        move_forward_inches(20, 5.7);

        on_arm_servo.SetDegree(180);

        Sleep(1.0);

        move_forward_inches(-20, 23);

    
        break;

    case PERF_COURSE_3: // Performance Test 3

        LCD.Write("Running Performance Test 3");
        Sleep(1.0);

        write_status("Aligning with ramp");
        move_forward_inches(20, 11.55 + DIST_AXIS_CDS);
        turn_right_degrees(20, 45);
        RPS_correct_heading(90);

        write_status("Moving up ramp");
        move_forward_inches(40, 33.26 + DIST_AXIS_CDS); // Initially 35.26
        RPS_check_y(55); // On top of ramp y-coord
    
        write_status("Moving towards hot plate");
        turn_right_degrees(20, 90);
        RPS_correct_heading(0);
        RPS_check_x(18.6); // On top of ramp x-coord
    
        // PROBLEM AREA. MOVES PRECISELY IN FRONT OF BURGER PLATE
        move_forward_inches(20, 8); // Initially 5.5
        RPS_check_x(27.8); // In front of burger plate x
        turn_left_degrees(20, 90);
        RPS_correct_heading(90);
        RPS_check_y(55);

        Sleep(2.0);

        // Flips burger when robot is ~13 inches in front, facing towards it
        flip_burger();

        write_status("Moving towards ice cream lever");
        move_forward_inches(-20, 5);
        RPS_correct_heading(90);
        RPS_check_y(55);
        turn_left_degrees(20, 90);
        RPS_correct_heading(180);
        RPS_check_x(29.1);
        move_forward_inches(20, 3); // Moves forward a bit to get in better RPS range
        RPS_correct_heading(180);
        move_forward_inches(20, 3.5 + DIST_AXIS_CDS); // Initially 12.9
        RPS_correct_heading(180);
        turn_right_degrees(20, 45);
        RPS_correct_heading(135);

        // Flips ice cream lever, about 3 inches in front of it (including base servo arm)
        flip_ice_cream_lever();
    
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

    // Initiates servos 25.3 58.3
    initiateServos();
    Sleep(1.0);

    // Initializes RPS
    RPS.InitializeTouchMenu();

    // Clears the screen
    LCD.SetBackgroundColor(BACKGROUND_COLOR);
    LCD.SetFontColor(FONT_COLOR);
    LCD.Clear();

    // Initializes menu and returns chosen course number
    // Commented out since QR code stand is too small to easily navigate over Proteus
    //int courseNumber = start_menu();

    //Waits until start light is read
    read_start_light();
    //Sleep(1.0);

    // Runs specified course number.
    //run_course(courseNumber);
    run_course(PERF_COURSE_3);

    //*****************************************
    // TEST CODE

    //**********************************
    // Show RPS stuff
    LCD.Clear();
    while (true) {
        show_RPS_data();
        Sleep(0.1);
    }
    
    //*****************************************

    return 0;
}