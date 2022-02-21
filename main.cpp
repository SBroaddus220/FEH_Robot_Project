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

/*
 * Number of encoder counts per inch.
 *
 * ((ENCODER_COUNTS_PER_REV / (2 * PI * WHEEL_RADIUS))) 
 */ 
#define COUNT_PER_INCH (318 / (2 * 3.14159265 * 1.25)) // Number of encoder counts per inch

// Function Prototypes
int startUp(); // Sets the screen and asks the user for the next step
void runCourse(); // Runs the course specified by startUp()
int yesOrNo(char question[], int xLoc, int yLoc); // Asks a yes/no question
void move_forward_inches(int percent, float inches); // Moves forward number of inches
void turn_right_degrees(int percent, float degrees); // Turns right a specified number of degrees
void turn_left_degrees(int percent, float degrees); // Turns left a specified amount of degrees

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
    LCD.Clear(GRAY);
    LCD.SetFontColor(BLACK);
    LCD.WriteRC("Robot Starting Up", 2, 4);
    Sleep(0.5);
    LCD.WriteRC('.', 2, 21);
    Sleep(0.5);
    LCD.WriteRC('.', 2, 22);
    Sleep(0.5);
    LCD.WriteRC('.', 2, 23);
    Sleep(0.5);

    // Asks the user for which step to take
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

    // Writes out data to the screen
    LCD.Clear();
    LCD.WriteRC("Turning Left...", 1, 2);

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
    LCD.WriteRC("Theoretical Counts: ", 3, 2);
    LCD.WriteRC(expectedCounts, 3, 20);
    LCD.WriteRC("Motor Percent: ", 4, 2);
    LCD.WriteRC(percent, 4, 20);
    LCD.WriteRC("Actual LE Counts: ", 5, 2);
    LCD.WriteRC(left_encoder.Counts(), 5, 20);
    LCD.WriteRC("Actual RE Counts: ", 6, 2);
    LCD.WriteRC(right_encoder.Counts(), 6, 20);
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

    // Writes out data to the screen
    LCD.Clear();
    LCD.WriteRC("Turning Left...", 1, 2);

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
    LCD.WriteRC("Theoretical Counts: ", 3, 2);
    LCD.WriteRC(expectedCounts, 3, 20);
    LCD.WriteRC("Motor Percent: ", 4, 2);
    LCD.WriteRC(percent, 4, 20);
    LCD.WriteRC("Actual LE Counts: ", 5, 2);
    LCD.WriteRC(left_encoder.Counts(), 5, 20);
    LCD.WriteRC("Actual RE Counts: ", 6, 2);
    LCD.WriteRC(right_encoder.Counts(), 6, 20);
    
}

/*******************************************************
 * @brief Detects the color using the CdS cell
 * 
 * @return int color Color detected.
 *          0 -> Red
 *          1 -> Blue
 */
int detectColor() {
    LCD.Clear();

    // Color to be returned
    int color = -1;

    // Initiates variables to find average value
    double startTime = TimeNow();
    double sum = 0.0;
    int numValues = 0;

    // Average value of CdS cell values
    double averageValue = sum / numValues;

    // 0 if color hasn't been detected yet, 1 if otherwise
    int colorFound = 0;

    // Reads values for four seconds
    while ((startTime - TimeNow() < 4) || !colorFound) {
        LCD.Clear();
        LCD.WriteRC("Reading color...", 1, 2);
        LCD.WriteRC("Cell Value: ", 3, 2);
        LCD.WriteRC(CdS_cell.Value(), 3, 4);
        
        sum += CdS_cell.Value();
        numValues++;

        // Takes the average value read
        averageValue = sum / numValues;

        // If 1 second of checking has passed, take absolute value to determine color
        if (startTime - TimeNow() > 1) {
            if (std::abs(averageValue - 0.3) < 0.01) {
                color = 0;
                colorFound = 1;
            } else if (std::abs(averageValue - 0.62) < 0.01) {
                color = 1;
                colorFound = 1;
            }
        }
        
    }

    LCD.Clear();
    LCD.WriteRC("Reading color...", 1, 2);
    LCD.WriteRC("Average Value: ", 3, 2);
    LCD.WriteRC(averageValue, 3, 4);

    return color;
}

/*******************************************************
 * @brief Asks the user a yes or no question, and returns the answer
 * @author Steven Broaddus
 * @param question Question that the user is asked
 * @param xLoc x-coordinate of text
 * @param yLoc y-coordinate of text
 */
int yesOrNo(char question[], int xLoc, int yLoc) {

    LCD.Clear();

    // User's answer, 0 if yes 1 if no
    int answer = -1;
    
    // Touch coordinates
    int xCoord, yCoord;

    LCD.WriteRC(question, xLoc, yLoc);

    LCD.WriteRC("Yes", 9, 5);
    LCD.DrawVerticalLine(160, 90, 239);
    LCD.WriteRC("No", 9, 19);
    LCD.DrawHorizontalLine(90,0,319);

    // Waits for an input and sees where it is
    while(!LCD.Touch(&xCoord, &yCoord)) {
        LCD.ClearBuffer();
        if (LCD.Touch(&xCoord, &yCoord)) {
            if (xCoord < 160) {
                answer = 0;
        } else if (xCoord > 160) {
                answer = 1;
            }  
        }
    }

    return answer;
}

/*******************************************************
 * @brief Runs the specified course
 * @author Steven Broaddus
 * @param courseNumber Course number to run
 */
void runCourse(int courseNumber) {
    
    switch (courseNumber)
    {
    case 0: // Test course
        LCD.Clear();
        LCD.Write("Successfully running test course");

        while (true) {
            move_forward_inches(20, 2);
        }
        
        break;

    case 1: // Performance Test 1
        LCD.Clear();
        LCD.Write("Successfully running Performance Test 1");
        break;

    case 2: // Performance Test 2
        LCD.Clear();
        LCD.Write("Successfully running Performance Test 2");
        break;

    case 3: // Performance Test 3
        LCD.Clear();
        LCD.Write("Successfully running Performance Test 3");
        break;

    case 4: // Performance Test 4
        LCD.Clear();
        LCD.Write("Successfully running Performance Test 4");
        break;

    case 5: // Individual Competition
        LCD.Clear();
        LCD.Write("Successfully running Ind. Competition");
        break;

    case 6: // Final Competition
        LCD.Clear();
        LCD.Write("Successfully running Final Competition");
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

    // Prompts user for which course to run
    int courseNumber = yesOrNo("Run Test?", 2, 9);

    // Runs the specified course number
    runCourse(courseNumber);

    return 0;
}