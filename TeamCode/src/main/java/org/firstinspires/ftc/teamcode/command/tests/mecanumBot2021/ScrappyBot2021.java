package org.firstinspires.ftc.teamcode.command.tests.mecanumBot2021;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.hardware.intakes.Intake;
import org.firstinspires.ftc.teamcode.hardware.linear_motion.LinearSlide_PIDControlled;
import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner.Drive_Mecanum_RoadRunner_Scrappy;
import org.firstinspires.ftc.teamcode.hardware.servo_managers.Servo_MultiSetState;
import org.firstinspires.ftc.teamcode.hardware.servo_managers.Servo_Spinner;
import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.Drive_Mecanum;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.PIDController;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.coefficient_wrappers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.utility.Math.AdvMath;
import org.firstinspires.ftc.teamcode.utility.Math.AngleMath;
import org.firstinspires.ftc.teamcode.utility.Math.MultislopeLinearInterpolator;
import org.firstinspires.ftc.teamcode.utility.Odometry.TrackingWheelLocalizer2021_Scrappy;
import org.firstinspires.ftc.teamcode.utility.VoltageCompensator.DcMotorExStable;
import org.firstinspires.ftc.teamcode.utility.VoltageCompensator.VoltageCompensator;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.NamedState;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.TimestampedValue;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.Vector3;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.Range2d;

import java.util.ArrayList;


/*
    Welcome to the template Provider class!
    To use it just make a copy of this class, and change all appearances of "20XX" with the starting year of the current season
    (for example, for the 2020-2021 season, the class would be named Provider2020 and all code within would have to reflect that change).

    Happy coding!
 */



// The main robot data class - called provider because it provides hardware classes with the robot data that they need to function

@Config

public class ScrappyBot2021 {
    public static String robotName = "Scrappy";


    public static double minDriveMotorMovePower = 0.055;

    public static PIDCoefficients translateCeoff = new PIDCoefficients(0.058, 0, 0);
    public static PIDCoefficients headingCoeff = new PIDCoefficients(2, 0, 0);
    public static double mecanumCounterRotPerStrafe = 0.05;


    public static PIDCoefficients bucketSlideCoeff = new PIDCoefficients(0.4, 0, 2);
    public static Range2d bucketIActiveRange = new Range2d(-0.5, 0.5);
    public static Range2d bucketIWindupBounds = new Range2d(-1, 1);

    public static double bucketIntakeRot = 0.06;
    public static double bucketPrepRot = 0.4;
    public static double bucketDumpRot = 1.0;
    public static double dumpMilliseconds = 800;

    public static double bucketSlideDownPos = 0.0;
    public static double bucketSlideLvl1Pos = 7.5;
    public static double bucketSlideLvl2Pos = 12.5;
    public static double bucketSlideLvl3Pos = 17.5;
    public static double armToIntakePosThreshold = 2.0;
    public static double bucketSlideMaxSpeed = 1.0;
    public static double bucketSlideForcedTravelTime = 40;
    public static double odoDownPos = 0.0;
    public static double odoUpPos = 0.5;

    public static PIDCoefficients duckSlideCoeff = new PIDCoefficients(0.38, 0, 2);
    public static int duckSpinDuration = 2500;
    public static int duckSpinStopDuration = 700;
    public static Range2d duckSlideExtremePositions = new Range2d(0, 43.55);
    public static Vector2d duckSlidePositionOnRobot = new Vector2d( 6.75, -2.875 );

    public static double endpointDuckAddition = 0.5;


    public static double intakeRunSpeed = 0.7;
    public static double intakeEjectSpeed = -0.25;


    public static TimestampedValue duckSlideDistancePoint0 = new TimestampedValue(-3, 1);
    public static TimestampedValue duckSlideDistancePoint1 = new TimestampedValue(20, 1);
    public static TimestampedValue duckSlideDistancePoint2 = new TimestampedValue(25, 1.1);
    public static TimestampedValue duckSlideDistancePoint3 = new TimestampedValue(35, 1.05);
    public static TimestampedValue duckSlideDistancePoint4 = new TimestampedValue(50, 1);


    public static double capperUpPos = 0.34;
    public static double capperDownPos = 0.039;
    public static double capperPlacePos = 0.28;
    public static double capperStartPos = 0.9;




    // Drive motors
    private DcMotorEx driveFL = null;
    private DcMotorEx driveFR = null;
    private DcMotorEx driveBL = null;
    private DcMotorEx driveBR = null;

    // Manipulator motors (motors for other things, not driving) - names are example names, they can be set for whatever application you have
    private DcMotor motorLift = null;
    private DcMotor motorIntake = null;
    private DcMotor motorDuck = null;


    // Servo Variables - names are example names, they can be set for whatever application you have
    private Servo duckSpinServo = null;
    public Servo bucketArmServo = null;
    public Servo capArmServo = null;
    private Servo odoServo = null;



    // Sensor Variables
    // Distance Sensor Variables - names are just examples, feel free to change to whatever you like. Just note that "flight" represents the fact that they are Time of Flight sensors (think radar, but with lasers)
    //public Rev2mDistanceSensor flightFront0;
    //public Rev2mDistanceSensor flightLeft1;
    //public Rev2mDistanceSensor flightRight2;
    //public Rev2mDistanceSensor flightBack3;

    public BNO055IMU imu; // the IMU class instance

    // The all important hardware map (basically a log of what devices are plugged into what ports. Setup on the FTC Robot Controller app)
    private HardwareMap mainMap;

    // All hardware/process class instances
    public Drive_Mecanum drive;
    public LinearSlide_PIDControlled duckSlide;
    public LinearSlide_PIDControlled bucketSlide;
    public Servo_Spinner duckSpinner;
    public Servo_MultiSetState bucketArm;
    public Intake intake;
    public Localizer odometry;


    public PIDController translatePID;
    public PIDController headingPID;

    public Drive_Mecanum_RoadRunner_Scrappy driveRoadRunner;


    // Default constructor
    public ScrappyBot2021(HardwareMap hMap, boolean isAuto){
        initMap(hMap, isAuto); // pull information from the hardware map - MUST BE DONE BEFORE

        initIMU(); // setup the IMU and calibrate the current position to be 0

        if( !VoltageCompensator.isSetup() && isAuto )
            VoltageCompensator.init(hMap);

        // init all hardware/processing object here
        drive = new Drive_Mecanum(driveFL, driveFR, driveBL, driveBR);
        intake = new Intake(motorIntake);
        duckSpinner = new Servo_Spinner(duckSpinServo, 0.5, 1.0, 0.0, duckSpinStopDuration, duckSpinDuration);
        bucketArm = new Servo_MultiSetState(bucketArmServo, new NamedState("Intake", bucketIntakeRot), new NamedState("Prep", bucketPrepRot), new NamedState("Dump", bucketDumpRot));
        duckSlide = new LinearSlide_PIDControlled(motorDuck, new PIDController(duckSlideCoeff));
        bucketSlide = new LinearSlide_PIDControlled(motorLift, new PIDController(bucketSlideCoeff, bucketIActiveRange, bucketIWindupBounds));
        odometry = new TrackingWheelLocalizer2021_Scrappy(hMap);
        driveRoadRunner = new Drive_Mecanum_RoadRunner_Scrappy(hMap, odometry);

        driveRoadRunner.setLocalizer(odometry);

        drive.setCounterRotPerStrafe(mecanumCounterRotPerStrafe);
        bucketSlide.setForcedTravelTime(bucketSlideForcedTravelTime);

        translatePID = new PIDController(translateCeoff);
        headingPID = new PIDController(headingCoeff);

        ArrayList<TimestampedValue> duckDistMults = new ArrayList<>();
        duckDistMults.add( duckSlideDistancePoint0 );
        duckDistMults.add( duckSlideDistancePoint1 );
        duckDistMults.add( duckSlideDistancePoint2 );
        duckDistMults.add( duckSlideDistancePoint3 );
        duckDistMults.add( duckSlideDistancePoint4 );
        duckSlide.setDistanceMaintainInterp( new MultislopeLinearInterpolator( duckDistMults ) );

        odoServo.setPosition(odoDownPos);
    }
    public ScrappyBot2021(HardwareMap hMap, String robotName){
        this(hMap, false);

        this.robotName = robotName;
    }
    public ScrappyBot2021(HardwareMap hMap){
        this(hMap, false);
    }


    // Initialization functions

    private void initMap(HardwareMap hMap, boolean isAuto){    // setup the hardware map dependant classes (usually by grabbing their components out of the hardware map)
        mainMap = hMap;

        /* NOTE - hardware map requests for hardware that is not in the hardware map (because they are not being used this season for example),
                  should either be removed or commented out to prevent the computer looking for hardware that isn't there.

                  That said, it is ok to have hardware map requests for hardware that isn't physically plugged in to the hubs at the moment,
                  as long as it is present in the hardcware map (again, configured in the FTC Robot Controller app).
                  Just be aware that any attempt to get readings from sensors or motors that aren't plugged in will not be reliable,
                  and motors that aren't plugged in will not move (Don't worry, we've all done that at least once ;)
         */


        // Grabbing motors from hardware map


        if(isAuto){
            driveFL = new DcMotorExStable(mainMap.get(DcMotorEx.class, "driveFL"));
            driveFR = new DcMotorExStable(mainMap.get(DcMotorEx.class, "driveFR"));
            driveBL = new DcMotorExStable(mainMap.get(DcMotorEx.class, "driveBL"));
            driveBR = new DcMotorExStable(mainMap.get(DcMotorEx.class, "driveBR"));

            ((DcMotorExStable)driveFL).setMinMovePower( minDriveMotorMovePower );
            ((DcMotorExStable)driveFR).setMinMovePower( minDriveMotorMovePower );
            ((DcMotorExStable)driveBL).setMinMovePower( minDriveMotorMovePower );
            ((DcMotorExStable)driveBR).setMinMovePower( minDriveMotorMovePower );
        }
        else {
            driveFL = mainMap.get(DcMotorEx.class, "driveFL");
            driveFR = mainMap.get(DcMotorEx.class, "driveFR");
            driveBL = mainMap.get(DcMotorEx.class, "driveBL");
            driveBR = mainMap.get(DcMotorEx.class, "driveBR");
        }

        motorLift = mainMap.get(DcMotor.class, "motorLift");
        motorIntake = mainMap.get(DcMotor.class, "motorIntake");
        motorDuck = mainMap.get(DcMotor.class, "motorDuck");



        // Set motors to run with encoders (uncomment if you are, comment out if you are not)
        //driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       /* motorIntakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorIntakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */

        // Reverse motor direction as needed
        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBL.setDirection(DcMotor.Direction.REVERSE);
        motorIntake.setDirection(DcMotor.Direction.REVERSE);

        // and set motors to break mode (when given 0 power, they maintain position
        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDuck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Grabbing servos from hardware map (uncomment if you are using them, comment out if you are not)
        duckSpinServo = mainMap.get(Servo.class, "duckSpinServo");
        bucketArmServo = mainMap.get(Servo.class, "bucketArmServo");
        capArmServo = mainMap.get(Servo.class, "capArmServo");
        odoServo = mainMap.get(Servo.class, "odoServo");


        bucketArmServo.setDirection(Servo.Direction.REVERSE);
        /*intakeDropL = mainMap.get(Servo.class, "intakeDropL");
        intakeDropR = mainMap.get(Servo.class, "intakeDropR");
        pullerDropL = mainMap.get(Servo.class, "pullerDropL");
        pullerDropR = mainMap.get(Servo.class, "pullerDropR");
        */

        // Grabbing sensors from hardware map (uncomment if you are using them, comment out if you are not)
        /*
        touchSensor0 = mainMap.get(DigitalChannel.class, "touchLift0");
        touchSensor1 = mainMap.get(DigitalChannel.class, "touchArm1");
        touchSensor2 = mainMap.get(DigitalChannel.class, "touchBlock2");
        touchSensor3 = mainMap.get(DigitalChannel.class, "touchLiftUp3");
        touchSensor4 = mainMap.get(DigitalChannel.class, "touchLeft4");
        touchSensor5 = mainMap.get(DigitalChannel.class, "touchRight5");
        touchSensor6 = mainMap.get(DigitalChannel.class, "touchBack6");
        touchSensor7 = mainMap.get(DigitalChannel.class, "touchClamp7");
        */

        // Time of flight sensor setup (uncomment if you are using them, comment out if you are not)
        /*
        flightFront0 = (Rev2mDistanceSensor)mainMap.get(DistanceSensor.class, "flightFront0");
        flightLeft1  = (Rev2mDistanceSensor)mainMap.get(DistanceSensor.class, "flightLeft1");
        flightRight2 = (Rev2mDistanceSensor)mainMap.get(DistanceSensor.class, "flightRight2");
        flightBack3  = (Rev2mDistanceSensor)mainMap.get(DistanceSensor.class, "flightBack3");
        */

        imu = mainMap.get(BNO055IMU.class, "imu");   // get Inertial Measurement Unit from the hardware map
    }

    private void initIMU(){ // create a new IMU class instance and set our IMU to that
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "DroopyIMU.json"; // see the calibration sample opmode
        parameters.loggingEnabled      =  true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
    }


    // External utility functions (ones that will be used outside this class)

    public void resetIMU(){ // an alternative name for init_imu, which is in this case public. The purpose of this is to more clearly convey application (you don't need to init the imu as the user, but you may want to reset it)
        initIMU();
    }

    public double getHeading(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle; // get the current heading of the robot in degrees
    }
    public double getHeading(AngleUnit angleUnit){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit).firstAngle; // get the current heading of the robot in degrees
    }
    public Pose2d getPosition(){
        return odometry.getPoseEstimate();
    }
    public String getName(){
        return robotName;
    }



    // hardware subroutine methods
    void stopRobot(){
        drive.driveRobotRelative(0, 0, 0);
        intake.spinDown();
        duckSpinner.stop();
        duckSlide.runToGivenPosition( duckSlide.getInchPosition() ); // tell PIDs to run to their current position, hopefully stoppign them
        bucketSlide.runToGivenPosition( bucketSlide.getInchPosition() );
        bucketArmServo.setPosition( bucketArmServo.getPosition() ); // same idea for the servo
    }

    void togglePeriodicDuckSpin(Servo_Spinner.SpinDirection direction){
        if( !duckSpinner.isPeroidicSpinning() || duckSpinner.getPeriodicSpinDirection() != direction)
            duckSpinner.startPeriodicSpin( direction );
        else
            duckSpinner.stopPeriodicSpin();
    }

    private boolean dumpRoutineRunning = false;
    public int dumpRoutineState = 0;
    private ElapsedTime dumpRoutineStateTimer;
    public boolean isDumpRoutineRunning(){return dumpRoutineRunning;}
    public void startDumpRoutine(){
        dumpRoutineRunning = true;
        dumpRoutineState = 0;
        dumpRoutineStateTimer = new ElapsedTime();
    }
    public void stopDumpRoutine(){
        dumpRoutineRunning = false;
    }
    public void toggleDumpRoutine(){
        if( !dumpRoutineRunning )
            startDumpRoutine();
        else
            stopDumpRoutine();
    }
    public void updateDumpRoutine(){
        if( dumpRoutineRunning ){
            switch (dumpRoutineState){
                case 0:
                    if( bucketSlide.getTargetPos() < armToIntakePosThreshold )
                        bucketSlide.setTargetPosition(bucketSlideLvl3Pos);
                    if( bucketSlide.getInchPosition() > armToIntakePosThreshold )
                        bucketArm.goToStateNamed("Prep");

                    if( bucketSlide.isAtTarget() ){
                        dumpRoutineState = 1;
                        dumpRoutineStateTimer.reset();
                    }
                    break;

                case 1:
                    bucketArm.goToStateNamed("Dump");
                    if( dumpRoutineStateTimer.milliseconds() >= dumpMilliseconds){
                        dumpRoutineState = 2;
                    }
                    break;

                case 2:
                    bucketArm.goToStateNamed("Intake");
                    bucketSlide.setTargetPosition(bucketSlideDownPos);
                    dumpRoutineRunning = false;
                    break;
            }
        }
    }



    private Vector2d endpointTargetPosition = duckSlidePositionOnRobot;
    private boolean endpointControlling = false;
    public boolean isEndpointControlling(){return endpointControlling;}
    public void startEndpointControl(){
        endpointControlling = true;
        endpointTargetPosition = getDuckSpinnerEndPos();
    }
    public void stopEndpointControl(){
        endpointControlling = false;
    }
    public void toggleEndpointControl(){
        if( !endpointControlling )
            startEndpointControl();
        else
            stopEndpointControl();
    }
    public Vector2d getEndpointTargetPosition(){return endpointTargetPosition;}
    public Vector2d getEndpointHeadingTargetPoint( Vector2d robotXY ){
        if( duckSlidePositionOnRobot.getY() != 0 ){
            double distanceToEndpoint = endpointTargetPosition.distTo( robotXY );

            /*double angleFromEndpointToBot = endpointTargetPosition.angleBetween( robotXY );
            double distanceToAngleEndpoint = Math.sqrt(Math.pow(distanceToEndpoint, 2) - Math.pow(duckSlidePositionOnRobot.getY(), 2));
            double rotTargetAngleFromEndpoint = Math.atan( distanceToAngleEndpoint / -duckSlidePositionOnRobot.getY() ); // y offset on robot made negative as offsetting back to be in line with robot center

            return endpointTargetPosition.plus( Vector2d.polar( angleFromEndpointToBot + rotTargetAngleFromEndpoint, Math.abs(duckSlidePositionOnRobot.getY()) ) );
            */
            Vector2d rotHeadingOffset = Vector2d.polar( duckSlidePositionOnRobot.getY(), this.getPosition().getHeading() - Math.PI/2 );

            return endpointTargetPosition.plus( rotHeadingOffset );
        }
        else{
            return endpointTargetPosition;
        }
    }
    public double getEndpointTargetHeading(Vector2d robotXY){
        Vector2d rotationTargetPos = getEndpointHeadingTargetPoint( robotXY );

        return AngleMath.getAngleToPoint( robotXY, rotationTargetPos );
    }
    public Vector3 getEndpointDriveCommands(){
        Pose2d robotPos = getPosition();
        Vector2d robotXY = new Vector2d(robotPos.getX(), robotPos.getY());

        double xCommand = 0;
        double yCommand = 0;

        double distanceToEndpoint = endpointTargetPosition.distTo( robotXY );
        if( distanceToEndpoint < AngleMath.getVectorMagnitude( duckSlidePositionOnRobot )){ // if endpoint closer than the center of the robot is to the start of the duck slide, gotta move away
            Vector2d minimumDistancePoint = Vector2d.polar( endpointTargetPosition.angleBetween( robotXY ), distanceToEndpoint ).plus(endpointTargetPosition);

            xCommand = -translatePID.getOutput(robotPos.getX(), minimumDistancePoint.getX()); // PID to the minimum distance point
            yCommand = translatePID.getOutput(robotPos.getY(), minimumDistancePoint.getY());

            Vector2d XYCommand = new Vector2d(xCommand, yCommand);
            XYCommand = XYCommand.rotated( robotPos.getHeading() );
            xCommand = XYCommand.getX();
            yCommand = XYCommand.getY();

            xCommand = AdvMath.rangeClip(xCommand, new Range2d(-0.8, 0.8)); // clip them between -0.8 and 0.8 to ensure the driver can override it (the values should never be that high anyways though
            yCommand = AdvMath.rangeClip(yCommand, new Range2d(-0.8, 0.8));
        }

        double endpointTargetAngle = getEndpointTargetHeading( robotXY );
        double headingCommand = headingPID.getOutput(robotPos.getHeading(), AngleMath.findClosestCoterminalTarget( robotPos.getHeading(), endpointTargetAngle ));

        return new Vector3(xCommand, yCommand, headingCommand);
    }
    public double getEndpointSlideTargetLen(){
        return AdvMath.rangeClip(endpointTargetPosition.distTo( getDuckSlideStartPos() ) + endpointDuckAddition, duckSlideExtremePositions);
    }

    public void addToEndpointTargetPosition(Vector2d addVector){
        endpointTargetPosition = endpointTargetPosition.plus(addVector);
    }
    public void setEndpointTargetPosition(Vector2d newPos){
        endpointTargetPosition = newPos;
    }



    public Vector2d getDuckSlideStartPos(){
        Pose2d robotPos = getPosition();

        Vector2d robotXY = new Vector2d(robotPos.getX(), robotPos.getY());
        Vector2d rotatedSlideOffset = duckSlidePositionOnRobot.rotated(robotPos.getHeading());

        return robotXY.plus( rotatedSlideOffset );
    }
    public Vector2d getDuckSpinnerEndPos(){
        Pose2d robotPos = getPosition();

        Vector2d robotXY = new Vector2d(robotPos.getX(), robotPos.getY());
        Vector2d unrotatedEndOffset = duckSlidePositionOnRobot.plus( new Vector2d(duckSlide.getInchPosition(), 0) );
        Vector2d rotatedEndOffset = unrotatedEndOffset.rotated( robotPos.getHeading() );

        return robotXY.plus( rotatedEndOffset );
    }


    private int capperState = 10;
    public int getCapperState(){
        return capperState;
    }

    public void capperStart(){
        capperState = 0;
        capArmServo.setPosition(capperStartPos);

    }

    public void capperDown(){
        capperState = 1;
        capArmServo.setPosition( capperDownPos );
    }

    public void capperUp(){
        capperState = 2;
        capArmServo.setPosition( capperUpPos );
    }

    public void capperPlace(){
        capperState = 0;
        capArmServo.setPosition( capperPlacePos );
    }
    public void progressCapperCycle(){
        if( capperState==10 )
            capperStart();
        else if(capperState == 0){
            capperDown();
        }
        else if(capperState == 1) {
            capperUp();
        }
        else if (capperState == 2){
            capperPlace();
        }

    }


    private boolean odoIsRaised = false;
    public void toggleOdoRaised(){
        if(odoIsRaised){
            odoServo.setPosition(odoDownPos);
        }
        else {
            odoServo.setPosition(odoUpPos);
        }
    }
    public boolean isOdoRaised(){return odoIsRaised;}
}

