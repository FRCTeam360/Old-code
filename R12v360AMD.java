/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Watchdog;
//import edu.wpi.first.wpilibj.Timer;
import java.util.Timer;
import java.util.TimerTask;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class R12v360AMD extends IterativeRobot {
    //Joysticks

    private Joystick joy1 = new Joystick(1);
    private Joystick joy3 = new Joystick(3);
    //Driver Station
    private DriverStation ds = DriverStation.getInstance();
    private DriverStationLCD lcd = DriverStationLCD.getInstance();
    //Digital Inputs
    private DigitalInput encRightA = new DigitalInput(4, 2);//encRightA
    private DigitalInput encRightB = new DigitalInput(4, 4);//encRightB
    private DigitalInput encLeftA = new DigitalInput(4, 3);//encLeftA
    private DigitalInput encLeftB = new DigitalInput(4, 5);//encLeftB
    private DigitalInput encLiftA = new DigitalInput(4, 10);//encLiftA
    private DigitalInput encLiftB = new DigitalInput(4, 11);//encLiftB
    private DigitalInput liftBottom = new DigitalInput(4, 6);//liftBottom
    private DigitalInput liftTop1 = new DigitalInput(4, 7);//liftTop
    private DigitalInput liftTop2 = new DigitalInput(4, 12);//liftTop
    private DigitalInput armBottom = new DigitalInput(4, 8);//armBottom
    private DigitalInput armTop = new DigitalInput(4, 13);//armTop , yes 14  :/
    //Encoders
    private Encoder encLeft = new Encoder(encLeftA, encLeftB); //left drive encoder

    {
        //encLeft = new Encoder(encLeftA, encLeftB);
        encLeft.setReverseDirection(true);
    }
    private Encoder encRight = new Encoder(encRightA, encRightB); //right drive encoder

    {
        // encRight = new Encoder(encRightA, encRightB);
    }
    private Encoder encLift = new Encoder(encLiftA, encLiftB); //right drive encoder

    {
        // encLift = new Encoder(encLiftA, encLiftB);
    }
    //Compressor
    private Compressor compressor = new Compressor(4, 1, 4, 1);
    //Solenoids
    private DoubleSolenoid claw = new DoubleSolenoid(8, 1, 2);
    private DoubleSolenoid shifter = new DoubleSolenoid(8, 7, 8);
    //Servos
    private Servo launchCatch = new Servo(4, 9);
    private Servo whiskerCatch = new Servo(4, 10);
    //Jaguars
    private Jaguar rightFront = new Jaguar(4, 2); //right front
    private Jaguar leftFront = new Jaguar(4, 3); //left front
    private Jaguar rightBack = new Jaguar(4, 4); //right back
    private Jaguar leftBack = new Jaguar(4, 5); //left back
    private Jaguar liftRight = new Jaguar(4, 6); //lift right
    private Jaguar liftLeft = new Jaguar(4, 7); //lift left
    private Jaguar arm = new Jaguar(4, 8); //arm
    //RobotDrives for stuff
    private RobotDrive driveBase = new RobotDrive(leftFront, leftBack, rightFront, rightBack);
    private RobotDrive driveLift = new RobotDrive(liftLeft, liftRight);
    //Timers for the lift and arm
    Timer timeLimit = new Timer();
     edu.wpi.first.wpilibj.Timer deployTimer = new  edu.wpi.first.wpilibj.Timer();

    /*TimerTask deployTimerTask = new TimerTask() {
        public void run() {
           deployTime++;
        }
    };*/
    TimerTask updateTime = new TimerTask() {

        public void run() {
            liftTime++;
            armTime++;
        }
    };
    //Constant Stuff
    private final double JOY_BUFFER = 0.08;
    private final double DISTANCE_PER_CLICK = 1;
    private final int BOTTOM = 0;
    private final int TOP = 1;
    private final int CLICKS_PER_INCH = 178;
    //Variable Stuff
    private double out;
    private int liftTarget;
    private int armTarget;
    private boolean autoLift;
    private boolean autoArm;
    private int liftTime = 0;
    private int armTime = 0;
    private int deployTime = 0;
    double GainMultiplier = 0.0061; //.0002
    double Iadjustment = 0.000;// 0.008//.00222
    double motor_speed = 1.0;
    double control_stick;
    double left_motor;
    double right_motor;
    int l_enc_count = 0;
    int r_enc_count = 0;
    long lefttotal = 0;
    long righttotal = 0;
    double Dgain = 0.011;
    double Pgain = 0.45;
    double Igain = 0.024;//.012
    double error;
    double last_error = 0;
    double Padjustment;
    double Dadjustment;
    double PID_adjust;
    boolean deployPrep;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        //clear the drivestation lcd
        lcd.println(DriverStationLCD.Line.kMain6, 1, "                                             ");
        lcd.println(DriverStationLCD.Line.kUser2, 1, "                                             ");
        lcd.println(DriverStationLCD.Line.kUser3, 1, "                                             ");
        lcd.println(DriverStationLCD.Line.kUser4, 1, "                                             ");
        lcd.println(DriverStationLCD.Line.kUser5, 1, "                                             ");
        lcd.println(DriverStationLCD.Line.kUser6, 1, "                                             ");

        //Invert all of the motors for reasons that elude me
        //Oh, it's probably because the RobotDrive automatically inverts one of the drives
        driveBase.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        driveBase.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
        driveBase.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
        driveBase.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);

        //disable watchdog
        Watchdog.getInstance().setEnabled(false);
        //set the distance per click on the encoders
        encLeft.setDistancePerPulse(DISTANCE_PER_CLICK);
        encRight.setDistancePerPulse(DISTANCE_PER_CLICK);
        //Set default grabbing
        claw.set(DoubleSolenoid.Value.kForward);//grab
        //Set default high gear
        shifter.set(DoubleSolenoid.Value.kForward); //high gear
        //set launchCatch to closed
        launchCatch.set(0.0);
        whiskerCatch.set(0.0);
        //Start the timer
        timeLimit.scheduleAtFixedRate(updateTime, 0, 1000);
    }

    public void disabledInit() {
        //set percent to 0

        driveBase.stopMotor();
        driveLift.stopMotor();
        arm.set(0);
        //set launchCatch to closed
        launchCatch.set(0.0);
        //stop the compressor
        compressor.stop();
        //stop the encoders
        encLeft.stop();
        encRight.stop();
        encLift.stop();
    }

    public void disabledPeriodic() {
        //set launchCatch to closed
        launchCatch.set(0.0);
        whiskerCatch.set(0.0);
    }

    public void autonomousInit() {
        //zero variables
        out = 0;
        liftTarget = BOTTOM;
        armTarget = TOP;
        autoArm = false;
        autoLift = false;
        deployPrep = false;
        //set percent to 0
        driveBase.stopMotor();
        driveLift.stopMotor();
        arm.set(0);
        //start the compressor
        compressor.start();
        //start the encoders
        encLeft.reset();
        encRight.reset();
        encLift.reset();
        encLeft.start();
        encRight.start();
        encLift.start();
        //Set default grabbing
        claw.set(DoubleSolenoid.Value.kForward);//grab
        //set launchCatch to closed
        launchCatch.set(0.0);
        whiskerCatch.set(0.0);

    }

    public void autonomousPeriodic() {
    }

    private void autoDisplay() {
        lcd.println(DriverStationLCD.Line.kUser2, 1, "                                             ");
        lcd.println(DriverStationLCD.Line.kUser3, 1, "                                             ");
        lcd.println(DriverStationLCD.Line.kUser4, 1, "                                             ");
        lcd.println(DriverStationLCD.Line.kUser5, 1, "                                             ");
        //lcd.println(DriverStationLCD.Line.kUser6, 1, "                                             ");
        lcd.updateLCD();
        lcd.println(DriverStationLCD.Line.kUser2, 1, "Left Encoder: " + Get_Left_Total());
        lcd.println(DriverStationLCD.Line.kUser3, 1, "Right Encoder: " + Get_Right_Total());
        lcd.println(DriverStationLCD.Line.kUser4, 1, "Lift Encoder: " + encLift.get());
        lcd.updateLCD();

    }

    public void autonomousContinuous() {
         //start the encoders
        encLeft.reset();
        encRight.reset();
        encLift.reset();
        encLeft.start();
        encRight.start();
        encLift.start();
        double liftHeight = 3000 * 1.23+150+20;//1.19
        //reset time limits for actions
        liftTime = 0;
        //continue lifting the lift
        while (encLift.get() < liftHeight && liftTime < 8) {
            autoDisplay();
            lcd.println(DriverStationLCD.Line.kMain6, 1, "                                             ");
            //lcd.println(DriverStationLCD.Line.kUser6, 1, "                                             ");
            lcd.updateLCD();
            lcd.println(DriverStationLCD.Line.kMain6, 1, "I'm liftin");
            lcd.updateLCD();
            driveLift.tankDrive(1.0, 1.0);
        }
        //stop lift
        driveLift.stopMotor();
        //reset time limit for drive
        armTime = 0;
        //drive forward while lifting the lift
        while (Get_Right_Total() < 6000 - 500 && armTime < 6) {//-200
            autoDisplay();
            lcd.println(DriverStationLCD.Line.kMain6, 1, "                                             ");
            //lcd.println(DriverStationLCD.Line.kUser6, 1, "                                             ");
            lcd.updateLCD();
            lcd.println(DriverStationLCD.Line.kMain6, 1, "I'm rollin");
            lcd.updateLCD();
            //driveBase.tankDrive(1.0, 1.0);
            doPID(0.80);//.85//.65
            try {
                //sleep the thread 50 milliseconds
                Thread.sleep(50);
            } catch (InterruptedException ex) {
            }
        }
        //stop driving
        driveBase.stopMotor();
        //wait a sec
        try {
            Thread.sleep(500);
        } catch (InterruptedException ex) {
        }

        //release tube
        claw.set(DoubleSolenoid.Value.kReverse);//release
        //wait a sec
        try {
            Thread.sleep(1000);
        } catch (InterruptedException ex) {
        }
        //drive backwards
        doPID(-0.85);
        //wait a sec
        try {
            Thread.sleep(1000);
        } catch (InterruptedException ex) {
        }
        //stop driving
        driveBase.stopMotor();
        //reset lift time again for another use
        liftTime = 0;
        //bring the lift down
        while (!liftBottom.get() && liftTime < 8) {
            autoDisplay();
            driveLift.tankDrive(-1.0, -1.0);
        }
        //stop the lift
        driveLift.stopMotor();
        //and wait until autonomous is over
        while (isAutonomous()) {
            autoDisplay();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException ex) {
            }
        }

    }

    private boolean teleoped=false;
    
    public void teleopInit() {
        //zero variables
        out = 0;
        liftTarget = BOTTOM;
        armTarget = TOP;
        autoArm = false;
        autoLift = false;
        deployPrep = false;
        //set percent to 0
        driveBase.stopMotor();
        driveLift.stopMotor();
        arm.set(0);
        //start the compressor
        compressor.start();
        //start the encoders
        encLeft.reset();
        encRight.reset();
        encLift.reset();
        encLeft.start();
        encRight.start();
        encLift.start();
        //set launchCatch to closed
        launchCatch.set(0.0);
        whiskerCatch.set(0.0);
        //start the deploy timer
        deployTime = 0;
        deployTimer.reset();
        deployTimer.start();
       /* if(teleoped){
            //deployTimer.cancel();
        }
        teleoped=true;*/
       /* try{
            deployTimer= new Timer();
            deployTimer.scheduleAtFixedRate(deployTimerTask, 1200, 1000);
        }catch(IllegalStateException e){
            
        }*/

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    }

    public void teleopContinuous() {

        lcd.println(DriverStationLCD.Line.kMain6, 1, "                                             ");
        lcd.println(DriverStationLCD.Line.kUser2, 1, "                                             ");
        lcd.println(DriverStationLCD.Line.kUser3, 1, "                                             ");
        lcd.println(DriverStationLCD.Line.kUser4, 1, "                                             ");
        lcd.println(DriverStationLCD.Line.kUser5, 1, "                                             ");
        //lcd.println(DriverStationLCD.Line.kUser6, 1, "                                             ");
        lcd.updateLCD();
        lcd.println(DriverStationLCD.Line.kMain6, 1, "L_B: " + liftBottom.get() + "\tL_T: " + (liftTop1.get() && liftTop2.get()) + "\tA_B: " + armBottom.get() + "\tA_T: " + armTop.get());
        lcd.println(DriverStationLCD.Line.kUser2, 1, "Left Encoder: " + Get_Left_Total());
        lcd.println(DriverStationLCD.Line.kUser3, 1, "Right Encoder: " + Get_Right_Total());
        lcd.println(DriverStationLCD.Line.kUser4, 1, "Lift Encoder: " + encLift.get());
        lcd.println(DriverStationLCD.Line.kUser5, 1, "GM: " + GainMultiplier);
        lcd.updateLCD();

        if (joy1.getRawButton(5)) {
            last_error = 0;
            Iadjustment = 0.00;//.08
            encRight.reset();
            encLeft.reset();
        }

        //control the drive
        driveControl();
        //control the lift
        liftControl();
        //control the arm
        armControl();
        //control the claw
        clawControl();
        //control minibot deploying apparatti
        minibotControl();

        try {
            //sleep the thread 50 milliseconds
            Thread.sleep(50);
        } catch (InterruptedException ex) {
        }
    }

    private void driveControl() {
        //store needed joystick values
        double j1_y = -joy1.getY();
        double j2_y = j1_y;
        double j1_x = -joy1.getX();
        boolean j1_btn3 = joy1.getRawButton(3);
        boolean j1_trig = joy1.getTrigger();

        //set a buffer area for the joysticks
        if (j1_y > -JOY_BUFFER && j1_y < JOY_BUFFER) {
            j1_y = 0;
        }

        //set the target speed of the left side to be the speed of the right side
        double target = encLeft.getRate();
        /// pidControl.setSetpoint(target);

        //turn PID off and on
       /* if (!pidControl.isEnable() && j1_btn3) {
        //if PID is off, and we press button #3, reset and turn on PID
        out = 0;
        pidControl.reset();
        pidControl.setPID(p, i, d);
        pidControl.enable();
        } else if (pidControl.isEnable() && !j1_btn3) {
        //if PID is on, and we are not pressing button #3, turn PID off
        pidControl.disable();
        }*/

        //PID calibration
        /*if (joy1.getRawButton(6)) {
        p += 0.0001;
        } else if (joy1.getRawButton(7)) {
        p -= 0.0001;
        }
        if (joy1.getRawButton(8)) {
        i -= 0.0001;
        } else if (joy1.getRawButton(9)) {
        i += 0.0001;
        }
        if (joy1.getRawButton(11)) {
        d += 0.0001;
        } else if (joy1.getRawButton(10)) {
        d -= 0.0001;
        }
        p = Math.max(p, 0);
        i = Math.max(i, 0);
        d = Math.max(d, 0);
         *
         */
        if (joy1.getRawButton(8)) {
            GainMultiplier -= 0.0001;
        } else if (joy1.getRawButton(9)) {
            GainMultiplier += 0.0001;
        }
        //constrain to -1 to 1
        j1_y = Math.min(j1_y, 1.0);
        j1_y = Math.max(j1_y, -1.0);
        j2_y = Math.min(j2_y, 1.0);
        j2_y = Math.max(j2_y, -1.0);

        //figure out how we are supposed to drive
        //and....DRIVE!
        if (j1_btn3) {
            //j2_y -= out;
            //out = (calcP() + calcI() + calcD()) / 256.0;
            //driveBase.tankDrive(j1_y, j2_y + out);
            System.out.println(GainMultiplier + "\t" + error + "\t" + Padjustment + "\t" + Iadjustment + "\t" + Dadjustment);

           doPID(-joy1.getY());
        } else if (joy1.getRawButton(2)) {
            driveBase.tankDrive(j1_y, j1_y);
        } else {
            driveBase.arcadeDrive(j1_y, j1_x);

        }

        //activate low gear only if trigger is pressed
        if(!deployPrep){
            if (j1_trig) {
                shifter.set(DoubleSolenoid.Value.kReverse); //low gear
            } else {
                shifter.set(DoubleSolenoid.Value.kForward); //high gear
            }
        }else{
           if (j1_trig) {
                shifter.set(DoubleSolenoid.Value.kReverse); //low gear
            } else {
                shifter.set(DoubleSolenoid.Value.kForward); //high gear
            }
        }

    }

    private int Get_Left_Total() {
        return encLeft.get();
    }

    private int Get_Right_Total() {
        return encRight.get();
    }

    private void doPID(double control_stick) {
        motor_speed = control_stick;

        if (control_stick > 0.1 && Get_Left_Total() < (196.0 / 200.0 * 6000) - 500) {
            left_motor = motor_speed;
            right_motor = motor_speed;

            /*************************************************
             ******** ERROR CALCULATION ********************
             *************************************************/
            lefttotal = Get_Left_Total();
            righttotal = Get_Right_Total();
            error = righttotal - lefttotal;


            /*************************************************
             ******** PROPORTIONAL ****************************
             *************************************************/
            Padjustment = error * Pgain * GainMultiplier;

            /*********************************************
             ******** INTEGRAL ****************************
             *********************************************/
            Iadjustment += error * Igain * GainMultiplier;

            /*************************************************
             ******** DERIVATIVE ******************************
             *************************************************/
            Dadjustment = (error - last_error) * Dgain * GainMultiplier;

            last_error = error;

            /*************************************************
             ******** PID AJUSTMENT ******************************
             *************************************************/
            PID_adjust = Padjustment + Iadjustment + Dadjustment;

            left_motor = left_motor + PID_adjust;
            right_motor = right_motor;// - PID_adjust;


            /**  print:   lefttotal,   righttotal,   PID_adjust,  left_motor,  right_motor  **/
        } else if (control_stick < -0.1) {
            left_motor = motor_speed;
            right_motor = motor_speed;
        } else {
            left_motor = 0;
            right_motor = 0;
        }
        driveBase.tankDrive(left_motor, right_motor);
        /********************/
        /**   End of Program  **/
        /*******************/
    }

    private void liftControl() {
        //boolean liftHigh = false;//joy3.getRawButton(5);
        boolean liftLow = joy3.getRawButton(5);
        boolean override = joy3.getRawButton(9);
        double j3_y = -joy3.getRawAxis(2);

        if (liftBottom.get()) {
            encLift.reset();
        }

        if (liftLow && !autoLift) {
            liftTime = 0;
            liftTarget = BOTTOM;
            autoLift = true;
        }
        /*if (j3_y > JOY_BUFFER && (!(liftTop1.get() && liftTop2.get()) || override)) {
        driveLift.tankDrive(j3_y, j3_y);
        } else if (j3_y < -JOY_BUFFER && (!liftBottom.get() || override)) {
        driveLift.tankDrive(j3_y, j3_y);
        } else {
        driveLift.tankDrive(0.0, 0.0);
        }*/
        if (j3_y > JOY_BUFFER * 2 || j3_y < -JOY_BUFFER * 2) {
            autoLift = false;
            if (j3_y > JOY_BUFFER * 2 && (!(liftTop1.get() && liftTop2.get()) || override)) {
                driveLift.tankDrive(j3_y, j3_y);
            } else if (j3_y < -JOY_BUFFER * 2 && (!liftBottom.get() || override)) {
                driveLift.tankDrive(j3_y, j3_y);
            } else {
                driveLift.tankDrive(0.0, 0.0);
            }
        } else if (autoLift) {
            if (liftTarget == BOTTOM && (liftBottom.get() || override)) {
                driveLift.tankDrive(0.0, 0.0);
                autoLift = false;
                return;
            }
            if (j3_y > JOY_BUFFER * 2 || j3_y < -JOY_BUFFER * 2) {
                driveLift.tankDrive(0.0, 0.0);
                autoLift = false;
                return;
            }
            if (liftTime > 5) {
                driveLift.tankDrive(0.0, 0.0);
                autoLift = false;
                return;
            }
            if (liftTarget == BOTTOM) {
                driveLift.tankDrive(-1.0, -1.0);
            }
        } else {
            driveLift.tankDrive(0.0, 0.0);
        }
    }

    private void armControl() {
        boolean armHigh = joy3.getRawButton(6);
        //boolean armLow = false;//joy3.getRawButton(8);
        //boolean armManual = joy3.getRawButton(6);
        boolean override = joy3.getRawButton(9);
        double j3_y = -joy3.getRawAxis(4);

        if (armHigh && !autoLift) {
            armTime = 0;
        }
        if (armHigh) {
            armTarget = TOP;
            autoArm = true;
        }
        if (j3_y > JOY_BUFFER * 2 || j3_y < -JOY_BUFFER * 2) {
            autoArm = false;
            if (j3_y > JOY_BUFFER * 2 && (!armTop.get() || override)) {
                arm.set(j3_y);
            } else if (j3_y < -JOY_BUFFER * 2 && (!armBottom.get() || override)) {
                arm.set(j3_y);
            } else {
                arm.set(0.0);
            }
        } else if (autoArm) {
            if (armTarget == TOP && (armTop.get() || override)) {
                arm.set(0.0);
                autoArm = false;
                return;
            }

            if (j3_y > JOY_BUFFER * 2 || j3_y < -JOY_BUFFER * 2) {
                arm.set(0.0);
                autoArm = false;
                return;
            }
            if (armTime > 3) {
                arm.set(0.0);
                autoArm = false;
                return;
            }
            if (armTarget == TOP) {
                arm.set(1.0);
            }
        } else {
            arm.set(0.0);
        }
    }

    private void clawControl() {
        boolean grab = joy3.getRawButton(7);
        boolean release = joy3.getRawButton(8);
        //Claw control
        if (grab) {
            claw.set(DoubleSolenoid.Value.kForward);//grab
        } else if (release) {
            claw.set(DoubleSolenoid.Value.kReverse);//release
        }
    }

    private void minibotControl() {
        boolean whisk = joy1.getRawButton(10);
        boolean launch = joy1.getRawButton(7);
        boolean overrideLaunch = joy1.getRawButton(6);

        if (whisk) {
            //set launchCatch to open
            whiskerCatch.set(1.0);
            deployPrep = true;
        } else {
            whiskerCatch.set(0.0);
        }
        //System.out.println(deployTimer.get());
        if ((launch && (deployTimer.get()>=110.35 || overrideLaunch)) && deployPrep) {
            //set launchCatch to open
            launchCatch.set(1.0);
        } else {
            launchCatch.set(0.0);
        }


    }
}
