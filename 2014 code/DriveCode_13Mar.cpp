#include "WPILib.h"
#include <iostream.h>
#include "Encoder.h"
#include <AnalogChannel.h>
//include these libraries to tell robot how to read I/O
//WPILib.h contains most of our common stuff like motor controllers
//iostream.h should be included in any program that handles strings and outputs data to a display
//Encoder.h contains code for using optical encoders
//AnalogChannel.h provides code for reading voltages from custom circuits like potentiometers


//Start Variable Definitions

//joystick axis values
float joyLeft_y;
float joyRight_y;
float joyLeft_x;
//end joystick axis values


//JOYSTICK BUTTON ASSIGNMENTS - these should be set to 'read only' hence 'static const' so you don't accidentally reset them
//convenient to assign your buttons through variables so that you need only change the values of the variables to completely remap your controller
static const int shifter = 2;
static const int driveScheme = 10;
//end joystick button assignments


//JOYSTICK BUTTON TOGGLES
//booleans are needed whenever you wish to assign a button to a function where it is critical that the function be performed once and only once or where a toggle state is necessary
bool highGear = false;
bool shifterPressed = false;
bool schemePressed = false;
bool softBallPressed = false;
//end joystick button toggles


//GAMEPAD AXIS VALUES
float override = 0;
float shooterMode = 0;
float gamepad_axis3 = 0;
//end gamepad axis values


//GAMEPAD BUTTON ASSIGNMENTS
//ditto (see comments in JOYSTICK section)
static const int plusFive = 8;
//static const int plusone = 6;
static const int minusFive = 7;
//static const int minusone = 5;
static const int ShootSole = 2;
static const int trussLeft = 5;
static const int trussRight = 6;
//end gamepad button assignments


//GAMEPAD BUTTON TOGGLES
//ditto (see comments in JOYSTICK section)
bool plusFive_pressed = false;
//bool plusone_pressed = false;
bool minusFive_pressed = false;
//bool minusone_pressed = false;
bool adjustAngleUp = false;
bool adjustAngleDown = false;
bool valueChecked = false;
//end gamepad button toggles


//MISCELANEOUS INTEGERS
//these are integer values that will be referenced throughout the code but that don't fall into a greater category
static const int armShootRight = 340;     //read only. value of arm potentiometer when shooting to the left
static const int armShootLeft = 580;    //ditto except to the right
static const int armVertical = 460;      //ditto except for vertical position
static const int armTrussRight = 360;
static const int armTrussLeft = 560;
int armTarget = 0;                       //used in void shooterControl() for varying the arm position
int angleCorrection = 0;                 //ditto for creating an constant arm angle correction should the need arise
long leftCount = 5;                      //left drive encoder count. encoder count will never be negative hence why it is a long class integer
int autoStage = 1;                       //used in void AutonomousPeriodic() to tell which stage of the autonomous program should be running
//end miscelaneous integers


//MISCELANEOUS DECIMALS
//these are decimal values that will be referenced throughout the code but that don't fall into a greater category
float joybuffer = 0.15;                  //joystick buffer zone where any small variations of the joystick will be equated to zero. see void tankDrive() for details on use
float shooterspeed = 0.6;                //used in void speedChange() and void shooterControl to control the speed of the shooter wheels
float current_time = 0;                  //used for timers. constantly updated. see void AutonomousPeriodic and void shooterControl for details on use
float initial_time = 0;                  //ditto except used to determine the start time of a function
static const double PI = 3.141592654;
static const double wheel_circ = (6*PI);
static const double encodingRatio = 1.3;
float rawCounts = 0;
//end miscelaneous decimals


//VICTOR SPEED VARIABLES
//victor speeds are a decimal value between -1 and 1
float victorRight_speed = 0;             //right drive
float victorLeft_speed = 0;              //left drive
float victorShooterFront = 0;            //front shooter wheel
float victorShooterBack = 0;             //back shooter wheel
float victorArm = 0;                     //arm
//end victor speed variables


//MISCELAENOUS BOOLEANS
//these are TRUE/FALSE values that will be referenced throughout the code but that don't fall into a greater category
bool tankMode = false;                   //controls the drive scheme used (tank or arcade). see void TeleopPeriodic and void changeDrive()
bool shootAllowed = false;               //tells the computer when it is allowed to shoot a ball. see void shooterControl()
bool ballShot = false;                   //
bool armPID = false;                     //tells the computer when to use PID to automatically position the arm. see void armRotation()
bool shallowAngle = false;
//end miscelaneous booleans


//PID VARIABLES
//variables used by void doPID()
float error = 0;
float integral = 0;
float derivative = 0;
float output = 0;
float prevError = 0;
//end PID variables


//End Variable Definitions


//Create the I/O systems of the robot
//This tells the computer what objects are attached to it

class DriveControl: public IterativeRobot
{ 
	Jaguar *drive_right;
	Jaguar *drive_left;
	Jaguar *arm;
	Jaguar *shooterfront;
	Jaguar *shooterback; 
	
	
	DigitalInput *lsX;//Limit Switch eXample
	
	AnalogChannel *armPosition;
	
	Timer *TimeCounter;
	
	//Encoder *eExample;
	Encoder *leftEncoder;
	//Encoder *encoder_two;
	
	DriverStation *m_ds;
	Dashboard *m_dsh;
	UINT32 m_priorPacketNumber;
	UINT8 m_dsPacketReceivedInCurrentSecond;
	DriverStationLCD *console;
	DriverStationEnhancedIO *dsEIO;
	
	Joystick *rightStick;//set joysticks
	Joystick *leftStick;
	Joystick *gamePad;
	
	Solenoid *hGear; //we use double solenoids so we need two programming objects for one solenoid
	Solenoid *lGear; //hGear and lGear are actually the same solenoid
	Solenoid *launch; //as are launch and retract
	Solenoid *retract;
	
	Compressor* aircomp;
	UINT32 m_autoPeriodicLoops;//set others
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;
	
//End I/O creation
	
//Start telling computer where to look for I/O stuffs you just created

public:
	
	DriveControl(void)
	{
		UINT32 DRightPort = 2;
		UINT32 DLeftPort = 1;
		UINT32 JarmPort = 3;
		UINT32 JShootPort = 4;
		UINT32 ShootPortBack = 5;
		
		UINT32 comprelaychannel = 1;
		UINT32 pressureswitchchannel = 6;
		
		drive_right = new Jaguar(1, DRightPort);
		drive_left = new Jaguar(1, DLeftPort);
		arm = new Jaguar(1, JarmPort);
		shooterfront = new Jaguar(1, JShootPort); 
		shooterback = new Jaguar(1, ShootPortBack);
		
		//eExample = new Encoder(1, 11, 1, 11);
		leftEncoder = new Encoder (1, 1, true);
		//encoder_two = new Encoder (3,4);
		
		armPosition = new AnalogChannel(1);
		
		m_ds = DriverStation::GetInstance();//Define Driverstaion & Dashboard
		m_priorPacketNumber = 0;
		m_dsPacketReceivedInCurrentSecond = 0;
		dsEIO = &m_ds->GetEnhancedIO();
		console = DriverStationLCD::GetInstance();
		console->UpdateLCD(); //End Define Driverstaion & Dashboard
	

		leftStick = new Joystick(1); //define joysticks
		rightStick = new Joystick(2);
		gamePad = new Joystick(3);
		
		hGear = new Solenoid(1, 3); //shifter solenoid. arguments - 9472 module #; port
		lGear = new Solenoid(1, 4); //shifter solenoid
		launch = new Solenoid(1, 1); //launcher solenoid
		retract = new Solenoid(1, 2); //launcher solenoid
		
		aircomp = new Compressor(1, pressureswitchchannel, 1, comprelaychannel);
		
	}
		
	void control_compressor(void)
	{
		if (!aircomp->GetPressureSwitchValue())
		{
			aircomp->Start();
		}
		if (aircomp->GetPressureSwitchValue())
		{
			aircomp->Stop();
		}
	}
	
	//End telling computer where to look for I/O stuffs
	
	
	void armRotation(void)                                                  //controls the position/angle of the arm
	{
		gamepad_axis3 = gamePad -> GetRawAxis(3);                           //gets updates on the state of the gamepad's analog axis #3
		override = gamePad -> GetX();                                       //ditto
		
		victorArm = 0;                                                      //sets the victorArm variable to zero to stop the victor if no input is detected later on
		
		if(override > 0.95){                                                //if the arm power override is pressed
			
			if (gamepad_axis3 > 0.1 || gamepad_axis3 < -0.1){               //checks if the analog control axis is out of the buffer zone
				victorArm = gamepad_axis3;                                  //sets the victorArm variable directly to the joystick value
				armPID = false;                                             //cancels automatic positioning
			}
		}
		
		if(override < 0.95){                                                //if the arm power override is not pressed
			
			if (gamepad_axis3 > 0.1 || gamepad_axis3 < -0.1){               //checks if the analog control axis is out of the buffer zone
				victorArm = gamepad_axis3 * 0.5;                            //sets the victorArm variable to the joystick value but scaled back by 50% for finer control
				valueChecked = false;
				resetPID();
				armPID = false;                                             //cancels automatic positioning
			}
			
			if (!(gamepad_axis3 > 0.1 || gamepad_axis3 < -0.1) && valueChecked == false ){
				armTarget = armPosition -> GetValue();
				valueChecked = true;
				armPID = true;
			}
		}
		
		if(gamePad -> GetRawButton(9) && adjustAngleDown == false){         //HERE TO (look down) is fairly standard button toggle code
			adjustAngleDown = true;                                         //keep in mind this code is cycling thousands of times per second
		}                                                                   //rather than perform the function whenever the button is pressed (which results in it being performed a few thousand times)
		                                                                    //we first tell the computer that the button is being pressed with a third party variable
		if(!gamePad -> GetRawButton(9) && adjustAngleDown == true){         //then have the function execute when the third party variable is true but not when the actual button is being pressed
			angleCorrection = angleCorrection - 5;                          //then we make sure to reset that third party variable. bam! function executes only once
			adjustAngleDown = false;                                        //HERE
		}
		
		if(gamePad -> GetRawButton(10) && adjustAngleUp == false){
			adjustAngleUp = true;
		}
				
		if(!gamePad -> GetRawButton(10) && adjustAngleUp == true){
			angleCorrection = angleCorrection + 5;
			adjustAngleUp = false;
		}
		
		console->Printf(DriverStationLCD::kUser_Line4, 1, "Correction: %d", angleCorrection);  //prints the value of our angle correction to the driver station
		
		if(gamePad -> GetRawButton(1)){                                     //shoot to the left
			resetPID();                                                     //resets the PID loop
			armPID = true;                                                  //enables automatic positioning
			armTarget = armShootLeft - angleCorrection;                     //sets angle to default plus angle correction 
		}                                                                   //it is minus in this case to remain consistent that positive angle corrections will put arm closer to horizontal
		
		if(gamePad -> GetRawButton(3)){                                     //shoot to the right
			resetPID();
			armPID = true;
			armTarget = armShootRight + angleCorrection;
		}
		
		if(gamePad -> GetRawButton(4)){                                     //arm vertical
			resetPID();
			armPID = true;
			armTarget = armVertical;
		}
		
		if(gamePad -> GetRawButton(trussRight)){                                     //arm vertical
			resetPID();
			armPID = true;
			armTarget = armTrussRight;
		}
		
		if(gamePad -> GetRawButton(trussLeft)){                                     //arm vertical
			resetPID();
			armPID = true;
			armTarget = armTrussLeft;
		}
		
		if(armPID == true){                                                 //checks to see if automatic positioning is enabled
			doPID(1, 0 , 0.15, 0.01, armPosition->GetValue(), armTarget);   //performs PID output. PID coefficients = 1, 0, 0.15 respectively; time to differentiate = 1 ms; reads position from potentiometer; target angle
			output = output / 100;                                          //scales output back by 100 since the potentiometer says a few hundred counts equals less than 100 degrees in RL
			
			if(output > 1){ output = 1;}                                    //these two lines set the output to max should it exceed the max
			if(output < -1){ output = -1;}
			
			arm -> Set(output);                                             //sets the arm to the PID controlled power
		}
		
		if(armPID == false){                                                //checks to see if automatic positioning is disabled
			arm -> Set(victorArm);                                         //sets the arm to joystick controlled power
		}
	}
	
	void tankDrive(void){                                                   //standard tank drive control. each joystick controls a side of the robot's drive train
		joyLeft_y = leftStick -> GetY();                                    //gets updates on the state of the left joystick's Y position
		joyRight_y = rightStick-> GetY();                                   //ditto
					
		victorRight_speed = 0;                                              //sets the victorRight_speed variable to zero to stop the victor if no input is detected later on
		victorLeft_speed = 0;                                               //ditto

		if (joyLeft_y > joybuffer || joyLeft_y < -joybuffer){               //checks if the left joystick is out of the buffer zone
			victorLeft_speed = joyLeft_y;				                    //sets the left drive victor speed to the position of the left joystick
		}
	
		if (joyRight_y > joybuffer || joyRight_y < -joybuffer){             //ditto (see above and use your head)
			victorRight_speed = joyRight_y;
		}
		
		drive_right -> Set(-victorRight_speed);                             //sets the speeds of each side of the drive train                             
		drive_left -> Set(victorLeft_speed);
	}
	
	void RCdrive(void){                                                     //non-standard drive train control. this is similar to arcade drive. splits it into two sticks - throttle and steering
		joyRight_y = rightStick -> GetY();                                  //updating joystick states. right stick is throttle & direction
		joyLeft_x = leftStick-> GetX();                                     //left stick is the steering
							 
		victorRight_speed = 0;                                              //stops the motors if no input is detected later on
		victorLeft_speed = 0; 
		
		if(joyRight_y > 0.1 || joyRight_y < -0.1){                          //checks to see if the throttle and direction joystick is out of the buffer zone
			
			if(joyLeft_x > 0.1){                                            //if the left joystick is pushed to the right
				victorLeft_speed = joyRight_y;                              //left speed is unimpeded - set directly to the throttle&direction indicated by the right stick
				victorRight_speed = joyRight_y * (1 - joyLeft_x);           //right speed is scaled back proportional to the deflection of the left joystick
			}                                                               //long story short this checks to see if we want a right turn and then makes robot do a sweeping right turn
			
			if(joyLeft_x < -0.1){                                           //if we want left turn - ditto. see previous comments
				victorLeft_speed = joyRight_y * (1 + joyLeft_x);
				victorRight_speed = joyRight_y;
			}
			
			if(!(joyLeft_x > 0.1 || joyLeft_x < -0.1)){                     //if we want no turn
				victorLeft_speed = joyRight_y;                              //just go straight. set both directly to throttle and direction
				victorRight_speed = joyRight_y;
			}
		}
		
		if(!(joyRight_y > 0.1 || joyRight_y < -0.1) && (joyLeft_x > 0.1 || joyLeft_x < -0.1)){  //if throttle is at zero but we are still trying to steer
			
			if(joyLeft_x > 0.1){                                            //check to see which way
				victorLeft_speed = -joyLeft_x;                              //pivot in place
				victorRight_speed = joyLeft_x;
			}
			
			if(joyLeft_x < -0.1){
				victorLeft_speed = -joyLeft_x;
				victorRight_speed = joyLeft_x;
			}
		}
		
		drive_right -> Set(-victorRight_speed); 
		drive_left -> Set(victorLeft_speed);
	}
	
	void shootSpeedChange(void) {                                          //adjusts the speed of the wheels on the shooter. launching only; picking up is a set speed
		
		if(gamePad->GetRawButton(plusFive) && plusFive_pressed == false){    //checks to see if the INCREASE BY 10 button is pressed and if it is not already being pressed
			shooterspeed = shooterspeed + .05;                              //increases shooter speed by 10 percent
			plusFive_pressed = true;                                        //indicates the button is being pressed so that it will not add 10 indefinitely
		}
		/*
		if(gamePad->GetRawButton(plusone) && plusone_pressed == false){    //checks to see if the INCREASE BY 1 button is pressed
			shooterspeed = shooterspeed + .01;                             //yeah you know the story
			plusone_pressed = true;
		}
		*/
		if(gamePad->GetRawButton(minusFive) && minusFive_pressed == false){  //checks to see if the DECREASE BY 10 button is pressed
			shooterspeed = shooterspeed - .05;
			minusFive_pressed = true;
		}
		/*
		if(gamePad->GetRawButton(minusone) && minusone_pressed == false){  //checks to see if the DECREASE BY 1 button is pressed
			shooterspeed = shooterspeed - .01;
			minusone_pressed = true;
		}
		*/
		if(shooterspeed>1){            //caps the shooter speed at 100% because it can't go higher anyway
			shooterspeed = 1;
		}
		
		if(shooterspeed<0){            //caps the shooter speed at 0% because it can go lower but this would screw up motor direction in void shooterControl()
			shooterspeed = 0;
		}
		
		if(!gamePad -> GetRawButton(plusFive)){                             //if INCREASE BY 10 button is no longer being pressed
			plusFive_pressed = false;                                       //allow the add 10% function to run again
		}
		/*
		if(!gamePad -> GetRawButton(plusone)){                             //if INCREASE BY 1 button is no longer being pressed
			plusone_pressed = false;                                       //allow the add 1% function to run again
		}
		*/
		if(!gamePad -> GetRawButton(minusFive)){                            //ditto
			minusFive_pressed = false;
		}
		/*
		if(!gamePad -> GetRawButton(minusone)){                            //ditto
			minusone_pressed = false;
		}
		*/
		console->Printf(DriverStationLCD::kUser_Line1, 1, "Shooter Speed: %4.2f", shooterspeed);	
	}
	
	void shooterControl(void){
			victorShooterFront = 0;
			victorShooterBack = 0;
			
			shooterMode = gamePad -> GetY();
			
			if(shooterMode < -0.95){
				victorShooterFront = shooterspeed;
				victorShooterBack = shooterspeed;
				shootAllowed = true;
			}
			
			if(shooterMode > 0.95){
				victorShooterFront = -0.5;
				victorShooterBack = -0.5;
				shootAllowed = false;
			}
			
			shooterfront -> Set(victorShooterFront);
			shooterback -> Set(victorShooterBack);
			
			if(shootAllowed == true){
			
				if(gamePad -> GetRawButton(ShootSole)){
					launch->Set(true);    //launches the ball. changes launcher side to active
					retract->Set(false);  //changes the retract side to inactive
					initial_time = GetTime();
				}
				
				current_time = GetTime();
				
				if((current_time - initial_time) > 0.7){
					launch->Set(false); //ditto. see above comments
					retract->Set(true);
				}
			}
			
			if(shootAllowed == false){
					
				if(gamePad -> GetRawButton(ShootSole)){
					console->Printf(DriverStationLCD::kUser_Line5, 1, "Im sorry Dave");
					initial_time = GetTime();
				}
						
				current_time = GetTime();
						
				if((current_time - initial_time) > 0.7){
					console->Printf(DriverStationLCD::kUser_Line5, 1, "----------------");
				}
			}
		}
	
	void changeGear(void){
		
		if(leftStick -> GetRawButton(shifter) || rightStick -> GetRawButton(shifter)){
			shifterPressed = true;
		}
		
		if(!(leftStick -> GetRawButton(shifter) || rightStick -> GetRawButton(shifter)) && shifterPressed == true){
			shifterPressed = false;
			highGear = !highGear;
		}
		
		if(highGear == true){
			hGear->Set(true);    //changes gear. switches high gear side of the solenoid to active
			lGear->Set(false);   //changes the low gear side to inactive
		}
		
		if(highGear == false){
			hGear->Set(false);   //ditto. see above comments
			lGear->Set(true);
		}
	}
	
	void changeDrive(void){
		if(leftStick -> GetRawButton(driveScheme) || rightStick -> GetRawButton(driveScheme)){
			schemePressed = true;
		}
				
		if(!(leftStick -> GetRawButton(driveScheme) || rightStick -> GetRawButton(driveScheme)) && schemePressed == true){
			schemePressed = false;
			tankMode = !tankMode;
			console->Clear();
		}
		
		if(tankMode == false){ console->Printf(DriverStationLCD::kUser_Line2, 1, "RC Drive");}
		if(tankMode == true){ console->Printf(DriverStationLCD::kUser_Line2, 1, "Tank Drive");}
	}
	
	void encoderTest(void){
		leftCount = leftEncoder -> GetRaw();
		console->Printf(DriverStationLCD::kUser_Line6, 1, "Left Encoder: %d", leftCount);
	}
	
	void armAngle(void){
		console->Printf(DriverStationLCD::kUser_Line3, 1, "Arm Angle: %d", armPosition -> GetValue());
	}
	
	void EncoderInit(void){
		
		leftEncoder -> SetMinRate(0);
		
		//encoder_two -> Start();
	}
	
	void talonCalibrate(void){
		gamepad_axis3 = gamePad -> GetRawAxis(3);
		
		arm -> Set(gamepad_axis3);
	}
	
	
	void doPID(float P, float I, float D, float dt, int position, int setPoint){ //arm to left is less than 470; to the right greater than 470
		Wait(dt);
		error = position - setPoint;
		integral = integral + (error * dt);
		derivative = (error - prevError) / dt;
		output = (P * error) + (I * integral) + (D * derivative);
		prevError = error;
		
	}
	
	
	void resetPID(void){
		error = 0;
		integral = 0;
		derivative = 0;
		prevError = 0;
	}
	
	
	void distanceCalculator(int inches){
		rawCounts = ((inches / wheel_circ) * encodingRatio) * 460;
	}
	
	
	void RobotInit(void)
	{
		//Ininitial Initialization
		//When Robot First Turns On
		//putting actions in this void is the leading cause of robots going SkyNet
	}

	void DisableInit(void)
	{
		drive_right -> Set(0); //Set 
		drive_left -> Set(0);
		shooterfront -> Set(0);
		shooterback -> Set(0);
		aircomp->Stop();
		console->Clear();
		leftEncoder->Stop();
		//encoder_two->Stop();
		//What happens when You disable robot
		//HAVE EVERYTHING STOP HERE!
	}

	void AutonomousInit(void)
	{
		//What happens when you Start Autonomous
		resetPID();
		current_time = 0;
		initial_time = 0;
		launch->Set(false);
		retract->Set(true);
		lGear->Set(true);
		hGear->Set(false);
		autoStage = 0;
		distanceCalculator(135);
		
		leftEncoder -> Start();
		leftEncoder -> Reset();
		
		shooterfront -> Set(0.55);
		shooterback -> Set(0.55);
	}
	
	void TeleopInit(void)
	{
		aircomp->Start();
		
		launch->Set(false);
		retract->Set(true);
		
		hGear->Set(false);
		lGear->Set(true);
		
		resetPID();
		highGear = false;
		
		leftEncoder -> Stop();
		leftEncoder -> Reset();
		//sole[1]->Set(DoubleSolenoid::kReverse);
		//What happens when you Start Teleop
	}
	
	void DisabledPeriodic(void)
	{	
		aircomp->Stop();
		drive_right -> Set(0); 
		drive_left -> Set(0); 
		arm -> Set(0); 
		shooterfront -> Set(0);
		leftEncoder->Stop();
		shootSpeedChange();
	
		console -> UpdateLCD();
		//encoder_two->Stop();
		//What happens while Robot is Disabled
		/*HAVE EVERYTHING STOP HERE!!!
		 * IMPORTANT
		 * IMPORTANT
		 * IMPORTANT
		 */
	}
	
	void AutonomousPeriodic(void)
	{
		leftCount = leftEncoder -> GetRaw();
		
		current_time = GetTime();
		
		console->Printf(DriverStationLCD::kUser_Line3, 1, "Travelled: %d", leftCount);
		console->Printf(DriverStationLCD::kUser_Line5, 1, "Target: %5.0f", rawCounts);
		console -> UpdateLCD();
				
		if(autoStage == 0 && leftCount < rawCounts){
			drive_right -> Set(1);
			drive_left -> Set(-1);
			
			doPID(0.01, 0, 0.0015, 0.01, armPosition -> GetValue(), 546);
			
			if(output > 1){ output = 1;}
			if(output < -1){ output = -1;}
			
			arm -> Set(output);
		}
				
		if(autoStage == 0 && leftCount > rawCounts){
			drive_right -> Set(0);
			drive_left -> Set(0);
			arm -> Set(0);
			
			autoStage = 1;
						
			distanceCalculator(40);
						
			leftEncoder -> Reset();
		
			Wait(0.1);
			
			leftCount = 0;
		}
				
		if(autoStage == 1 && leftCount < rawCounts){
			drive_left -> Set(-1);
			
			doPID(0.01, 0, 0.0015, 0.01, armPosition -> GetValue(), 546);
						
			if(output > 1){ output = 1;}
			if(output < -1){ output = -1;}
						
			arm -> Set(output);
		}
				
		if(autoStage == 1 && leftCount > rawCounts){
			drive_left -> Set(0);
			arm -> Set(0);
					
			Wait(0.1);
			
			autoStage = 2;
			
			initial_time = GetTime();
		}
		
		
		if(autoStage == 2){
			doPID(0.03, 0, 0.0015, 0.01, armPosition -> GetValue(), 546);
									
			if(output > 1){ output = 1;}
			if(output < -1){ output = -1;}
									
			arm -> Set(output);
			
			if((current_time - initial_time) > 1){
				autoStage = 3;
				
				initial_time = GetTime();
			}
		}
		
		if(autoStage == 3){
			current_time = GetTime();
			
			doPID(0.01, 0, 0.0015, 0.01, armPosition -> GetValue(), 546);
												
			if(output > 1){ output = 1;}
			if(output < -1){ output = -1;}
												
			arm -> Set(output);
			
			launch -> Set(true);
			retract -> Set(false);
			
			if((current_time - initial_time) > 1){
				launch -> Set(false);
				retract -> Set(true);
				
				autoStage = 4;
				
				resetPID();
			}
		}
		
		if(autoStage == 4){
			drive_right ->Set(0);
			drive_left -> Set(0);
			shooterfront -> Set(0);
			shooterback -> Set(0);
			
			doPID(0.01, 0, 0.0015, 0.01, armPosition -> GetValue(), 460);
															
			if(output > 1){ output = 1;}
			if(output < -1){ output = -1;}
															
			arm -> Set(output);
			
			launch -> Set(false);
			retract -> Set(true);
		}
		//While Autonomous is Running
	};
	

	void TeleopPeriodic(void)
	{
		//While Teleop is Running
		
		if(tankMode == true){
			tankDrive();
		}
		
		if(tankMode == false){
			RCdrive();
		}
		
		armRotation();
		//talonCalibrate();
		shootSpeedChange();
		shooterControl();
		changeGear();
		changeDrive();
		armAngle();
		
		//encoderTest();
		
		console->UpdateLCD();
		//printf("%d", leftEncoder -> Get(), encoder_two -> Get());
	}
	
	
//End I/0 Naming	

};

START_ROBOT_CLASS(DriveControl);
	
	
