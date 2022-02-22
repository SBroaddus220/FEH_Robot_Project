/*********************************************/
/*      Team A3 FEH Robot Project Code       */
/*            OSU FEH Spring 2022            */
/*                                           */
/*      Steven Broaddus, Conolly Burgess     */
/*        Joseph Richmond, Jake Chang        */
/*                                           */
/*            Updated 2/20/2022              */
/*       Uses Doxygen for documentation      */
/*********************************************/

// Include preprocessor directives
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <cmath> // abs() 

// Definitions
#define TEST_COURSE 0 // Number of course for testing
#define ROBOT_WIDTH 6.5 // Length of front/back side of robot in inches
#define PI 3.14159265
#define BACKGROUND_COLOR WHITE // Background color of layout
#define FONT_COLOR BLACK // Font color of layout

/*
 * Number of encoder counts per inch.
 *
 * ((ENCODER_COUNTS_PER_REV / (2 * PI * WHEEL_RADIUS))) 
 */ 
#define COUNT_PER_INCH (318 / (2 * 3.14159265 * 1.25)) // Number of encoder counts per inch

// Function Prototypes
int startUp(); // Sets the screen and asks the user for the next step
void move_forward_inches(int percent, float inches); // Moves forward number of inches
void turn_right_degrees(int percent, float degrees); // Turns right a specified number of degrees
void turn_left_degrees(int percent, float degrees); // Turns left a specified amount of degrees
int detectColor(int timeToDetect); // Detects the color of the jukebox
void pressJukeboxButtons(); // Presses the jukebox buttons
int yesOrNo(char question[], int xLoc, int yLoc); // Asks a yes/no question
void writeStatus(char status[]); // Clears room for a printed string w/o clearing display
void runCourse(); // Runs the course specified by startUp()


// Declarations for encoders/motors
DigitalEncoder right_encoder(FEHIO::P0_0);
DigitalEncoder left_encoder(FEHIO::P0_1);
FEHMotor right_motor(FEHMotor::Motor0,9.0);
FEHMotor left_motor(FEHMotor::Motor1,9.0);

// Declaration for CdS cell sensorsad 
AnalogInputPin CdS_cell(FEHIO::P0_2);

/*******************************************************
 * @brief Initializes the screen and asks for what to do
 * @author Steven Broaddus
 * @return Course number to run
 */
int startUp() 
{
    // Initializes the screen
    LCD.Clear();
    LCD.SetBackgroundColor(BACKGROUND_COLOR);
    LCD.SetFontColor(FONT_COLOR);
    LCD.WriteRC("Robot Starting Up", 2, 4);
    Sleep(0.25);
    LCD.WriteRC('.', 2, 21);
    Sleep(0.25);
    LCD.WriteRC('.', 2, 22);
    Sleep(0.25);
    LCD.WriteRC('.', 2, 23);
    Sleep(0.25);

    return 0;

}

/*******************************************************
 * @brief Moves forward a number of inches using encoders
 * @author Steven Broaddus
 * @param percent - Percent for the motors to run at. Negative for reverse.
 * @param inches - Inches to move forward 
 */
void move_forward_inches(int percent, float inches)
{
    // Calculates desired counts based on the radius of the wheels and the robot
    float expectedCounts = COUNT_PER_INCH * inches;

    // Clears space for movement data and status
    LCD.SetFontColor(BACKGROUND_COLOR);
    LCD.FillRectangle(0,100,319,239);
    LCD.SetFontColor(FONT_COLOR);
    LCD.DrawHorizontalLine(100, 0, 319);

    // Writes out status to the screen
    LCD.WriteRC("Moving forward...", 7, 2);

    // Resets encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    // Sets both motors to specific percentage
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);

    // Keeps running until average motor counts are in proper range
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < expectedCounts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();

    //Print out data
    LCD.WriteRC("Theoretical Counts: ", 9, 2);
    LCD.WriteRC(expectedCounts, 9, 20);
    LCD.WriteRC("Motor Percent: ", 10, 2);
    LCD.WriteRC(percent, 10, 20);
    LCD.WriteRC("Actual LE Counts: ", 11, 2);
    LCD.WriteRC(left_encoder.Counts(), 11, 20);
    LCD.WriteRC("Actual RE Counts: ", 12, 2);
    LCD.WriteRC(right_encoder.Counts(), 12, 20);
}

/*******************************************************
 * @brief Turns right a certain amount of degrees
 * 
 * @param percent 
 * @param degrees 
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
    right_motor.SetPercent(-percent);
    left_motor.SetPercent(percent);

    // Keeps running until average motor counts are in proper range
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < expectedCounts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();

    //Print out data
    LCD.WriteRC("Theoretical Counts: ", 9, 2);
    LCD.WriteRC(expectedCounts, 9, 20);
    LCD.WriteRC("Motor Percent: ", 10, 2);
    LCD.WriteRC(percent, 10, 20);
    LCD.WriteRC("Actual LE Counts: ", 11, 2);
    LCD.WriteRC(left_encoder.Counts(), 11, 20);
    LCD.WriteRC("Actual RE Counts: ", 12, 2);
    LCD.WriteRC(right_encoder.Counts(), 12, 20);
}

/*******************************************************
 * @brief Turns left a certain amount of degrees
 * 
 * @param percent 
 * @param degrees 
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
    left_motor.SetPercent(-percent);

    // Keeps running until average motor counts are in proper range
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < expectedCounts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
    
    //Print out data
    LCD.WriteRC("Theoretical Counts: ", 9, 2);
    LCD.WriteRC(expectedCounts, 9, 20);
    LCD.WriteRC("Motor Percent: ", 10, 2);
    LCD.WriteRC(percent, 10, 20);
    LCD.WriteRC("Actual LE Counts: ", 11, 2);
    LCD.WriteRC(left_encoder.Counts(), 11, 20);
    LCD.WriteRC("Actual RE Counts: ", 12, 2);
    LCD.WriteRC(right_encoder.Counts(), 12, 20);
    
}

/*******************************************************
 * @brief Detects the color using the CdS cell
 * 
 * @return int color Color detected.
 *          0 -> Red
 *          1 -> Blue
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
        
        // Takes the average value read
        sum += CdS_cell.Value();
        numValues++;
        averageValue = sum / numValues;

        // Detects color using CdS_cell values
        if (std::abs(CdS_cell.Value() - 0.25) < 0.2) {
                color = 0;
                colorFound = 1;
        } else if (std::abs(CdS_cell.Value() - 0.65) < 0.2) {
                color = 1;
                colorFound = 1;
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
        turn_right_degrees(20, 35);
        move_forward_inches(20, 4.5); // Moves until button is pressed
        Sleep(2.0);
        move_forward_inches(20, -4.5); // Reverses
        turn_left_degrees(20, 155);
        move_forward_inches(20, 2.75);
        turn_right_degrees(20, 65);

    } else if (color == 1) { // On left path (blue light)

        turn_left_degrees(20, 35);
        move_forward_inches(20, 2.75);
        turn_right_degrees(20, 35);
        move_forward_inches(20, 4.5); // Moves until button is pressed
        Sleep(2.0);
        move_forward_inches(20, -4.5);
        turn_left_degrees(20, 155);
        move_forward_inches(20, 2.75);
        turn_right_degrees(20, 65);

    } else {
        LCD.Write("ERROR: COLOR NOT READ SUCCESFULLY");
    }
}

/*******************************************************
 * @brief Asks the user a yes or no question, and returns the answer
 * @author Steven Broaddus
 * @param question Question that the user is asked
 * @param xLoc x-coordinate of text
 * @param yLoc y-coordinate of text
 * @return answer Returns 1 if yes, 0 if no
 */
int yesOrNo(char question[], int xLoc, int yLoc) {

    LCD.Clear();

    // User's answer: 1 if yes, 0 if no
    int answer = -1;
    
    // Touch coordinates
    int xCoord, yCoord;

    // Prompts user question
    LCD.WriteRC(question, xLoc, yLoc);

    LCD.WriteRC("Yes", 9, 5);
    LCD.DrawVerticalLine(160, 90, 239);
    LCD.WriteRC("No", 9, 19);
    LCD.DrawHorizontalLine(90,0,319);

    // Waits for an input and sees where it is
    while(!LCD.Touch(&xCoord, &yCoord)) {
        if (xCoord < 160) { // Yes
                answer = 1;
        } else if (xCoord > 160) { // No
                answer = 0;
            }  
        }

    return answer;
}

/*******************************************************
 * @brief Clears room for status and prints it to screen 
 * without clearing the screen
 * 
 * @param status Status to be printed
 */
void writeStatus(char status[]) {
    LCD.SetFontColor(BACKGROUND_COLOR);
    LCD.FillRectangle(0, 17,319,17);
    LCD.SetFontColor(FONT_COLOR);
    LCD.WriteRC(status,1,2);
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
    
    switch (courseNumber)
    {
    case 0: // Test course
        writeStatus("Running Test");

        detectColor(99);
        
        break;

    case 1: // Performance Test 1

        /*****
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
         * 5. Read light
         * 
         * 6. Respond to light value (display it too)
         * 
         * Following is in a function: 
         * if (color == red (on right path)) {
         *      Rotate 35 degrees right (heading == 235 degrees)
         *      Move forward 2.75 inches
         *      Rotate 35 degrees right (heading == 270 degrees)
         *      Move forward until button is pressed (4.5 inches?)
         *      Reverse that much distance
         *      Rotate 155 left (heading == 65 degrees)
         *      Move forward 2.75 inches
         *      Rotate 65 right (heading == 0 degrees)
         * 
         * } else if (color == blue (on left path)) {
         *      Rotate 35 degrees left (heading == 305 degrees)
         *      Move forward 2.75 inches
         *      Rotate 35 degrees right (heading == 270 degrees)
         *      Move forward until button is pressed (4.5 inches?)
         *      Reverse that much distance
         *      Rotate 155 right (heading == 115 degrees)
         *      Move forward 2.75 inches
         *      Rotate 115 degrees right (heading == 0 degrees)
         * 
         * } else {
         *      Resort to RPS to try and find light 
         *      (take a certain amount of time to do this or move on)
         * }
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
         * 12. Rotate 180 degrees right (heading == 270 degrees)
         * 
         * 13. Move forward 14 inches (from ramp)
         * 
         * 14. Move forward (10 inches) (across ramp)
         * 
         * 15. Move forward (11 inches) (base of ramp)
         * 
         * 16. Rotate 90 degrees left (heading == 0 degrees)
         * 
         * 17. Move forward 2.9 inches 
         * 
         * 18. Rotate 45 degrees right (heading == 315 degrees)
         * 
         * 19. Move forward until button is pressed (7.5 inches?)
         * 
         * 20. Win.
         * 
         */ 

        writeStatus("Running Perf. Test 1");

        Sleep(1.0);

        writeStatus("Moving towards jukebox");

        // Heads from button to center
        move_forward_inches(20, 7.5);

        // Moves towards jukebox
        turn_left_degrees(20, 45);
        move_forward_inches(20, 11.9);
        turn_left_degrees(20, 90);

        writeStatus("Pressing jukebox buttons");
        
        // Presses jukebox buttons
        pressJukeboxButtons();

        writeStatus("Moving towards ramp");

        // Moves to center (aligns with ramp)
        move_forward_inches(20, 9);
        turn_left_degrees(20, 90);
        
        writeStatus("Moving up ramp");

        // Moves up ramp
        move_forward_inches(20, 11);
        move_forward_inches(20, 10);
        move_forward_inches(20, 14);
        turn_right_degrees(20, 180);

        writeStatus("Moving down ramp");
        
        // Moves down ramp
        move_forward_inches(20, 14);
        move_forward_inches(20, 10);
        move_forward_inches(20, 11);

        writeStatus("Towards final button");

        // Heads toward final button
        turn_left_degrees(20, 90);
        move_forward_inches(20, 2.9);
        turn_right_degrees(20, 45);
        move_forward_inches(20, 7.5);

        writeStatus("Woo?");

        break;

    case 2: // Performance Test 2
        LCD.Clear();
        LCD.Write("Running Performance Test 2");
        break;

    case 3: // Performance Test 3
        LCD.Clear();
        LCD.Write("Running Performance Test 3");
        break;

    case 4: // Performance Test 4
        LCD.Clear();
        LCD.Write("Running Performance Test 4");
        break;

    case 5: // Individual Competition
        LCD.Clear();
        LCD.Write("Running Ind. Competition");
        break;

    case 6: // Final Competition
        LCD.Clear();
        LCD.Write("Running Final Competition");
        break;
    
    default:
        LCD.Clear();
        LCD.WriteRC("ERROR: NO COURSE SPECIFIED", 1, 0);
        break;
    }
} 

/*****************************************************************
 * main
 */
int main() {

    // Initializes screen
    startUp();

    // Runs specified course number.
    runCourse(1);

    return 0;
}