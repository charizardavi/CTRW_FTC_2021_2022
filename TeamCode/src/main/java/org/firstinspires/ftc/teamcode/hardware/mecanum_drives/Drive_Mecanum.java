package org.firstinspires.ftc.teamcode.hardware.mecanum_drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.PIDController;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.coefficient_wrappers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.utility.Math.AngleMath;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.Vector3;


public class Drive_Mecanum {
    ElapsedTime localRuntime;

    // Default speed modifier values
    private static double DEFAULT_TURN_MULTIPLIER = 1.0; // default values to use in the event no custom values are passed
    private static double DEFAULT_TRANSLATE_MULTIPLIER = 1.0;
    private static double DEFAULT_LATERAL_MULT = 1.2;
    private static PIDCoefficients DEFAULT_PID_COEFF = new PIDCoefficients(0, 0, 0);

    // Create and initialize speed modifier variables - using default values (then can be set via a constructor)
    private double turnMultiplier; // what percentage of maximum turning speed should be used as a base turning speed (50% = 0.5, etc) - a multiplier
    private double translateMultiplier; // what percentage of maximum translational speed should be used as a base translational speed (50% = 0.5, etc) - a multiplier


    // PID local variables
    private PIDController xPID, yPID, headingPID;

    private double counterRotPerStrafe = 0.0;
    private double lateralMult = DEFAULT_LATERAL_MULT;


    //Motor variables
    private DcMotor driveFL, driveFR, driveBL, driveBR; // motors that are being used for mecanum driving


    // Default constructor
    public Drive_Mecanum(DcMotor driveMotorFL, DcMotor driveMotorFR, DcMotor driveMotorBL, DcMotor driveMotorBR, PIDController xPID, PIDController yPID, PIDController headingPID){ // passing of individual motors in a constructor as an alternative to needing a robot class passed with the proper motor names
        localRuntime = new ElapsedTime();

        // setup motors from passed motors
        driveFL = driveMotorFL;
        driveFR = driveMotorFR;
        driveBL = driveMotorBL;
        driveBR = driveMotorBR;

        this.xPID = xPID;
        this.yPID = yPID;
        this.headingPID = headingPID;
    }
    public Drive_Mecanum(DcMotor driveMotorFL, DcMotor driveMotorFR, DcMotor driveMotorBL, DcMotor driveMotorBR){
        this(   driveMotorFL, driveMotorFR, driveMotorBL, driveMotorBR,
                new PIDController(DEFAULT_PID_COEFF),
                new PIDController(DEFAULT_PID_COEFF),
                new PIDController(DEFAULT_PID_COEFF)
        );
    }
    public Drive_Mecanum(DcMotor driveMotorFL, DcMotor driveMotorFR, DcMotor driveMotorBL, DcMotor driveMotorBR, PIDController translatePID, PIDController headingPID){
        this(   driveMotorFL, driveMotorFR, driveMotorBL, driveMotorBR,
                translatePID,
                translatePID.getCopy(),
                headingPID
        );
    }



    // Drive functions
    /**
     * Drives the robot relative to the field according to the given commands (powers) for each axis
     * @param x the x movement command, between -1 and 1
     * @param y the y movement command, between -1 and 1
     * @param r the r movement command, between -1 and 1
     * @param currentHeading the current heading of the robot in RADIANS, used for converting the field relative commands to robot relative commands so they can be applied
     */
    public void driveFieldRelative(double x, double y, double r, double currentHeading) { // this drives relative to field (+x is forward, +y is left, heading is in radians)
        // if using controller inputs, ensure you reverse the y on the stick input before passing into this method because down on the stick is positive and up is negative, and we need that to be the opposite way

        double heading = currentHeading * -1; // multiplied by -1 because that's how the code was written by me several years ago (and it works), but you could go through and alter the formulas to make it equivalent without requiring the negation :)
        heading = AngleMath.clipAngle(heading, AngleUnit.RADIANS);

        // Set up heading factor for relative to robot (convert the heading to radians, then get the sine and cosine of that radian heading)
        double sin = Math.sin(heading);
        double cos = Math.cos(heading);

        // do math to adjust to make the input drive vector relative to the robot (rather than relative to field)
        double robotRelativeX = (x * cos) - (y * sin);
        double robotRelativeY = (x * sin) + (y * cos);


        driveRobotRelative(robotRelativeX, robotRelativeY, r); // send the newly robot relative commands into the robot relative drive function
    }
    public void driveFieldRelative(Vector3 driveCommands, double currentHeading){
        driveFieldRelative(driveCommands.x, driveCommands.y, driveCommands.heading, currentHeading);
    }

    /**
     * Drives the robot (relative to itself) according to the given commands (powers) for each axis
     * @param x the x movement command, between -1 and 1
     * @param y the y movement command, between -1 and 1
     * @param r the r movement command, between -1 and 1
     */
    public void driveRobotRelative(double x, double y, double r) {
        y *= lateralMult;
        r -= counterRotPerStrafe * y;

        // apply the cartesian mecanum formula
        double veloFL = x - r - y;
        double veloFR = x + r + y;
        double veloBL = x - r + y;
        double veloBR = x + r - y;

        // Normalize the powers before we pass them into the motors (so that no power is outside of the range when passed in, preserving the intended ratio of the powers to each other)
        double largest_power = Math.max( Math.max( Math.abs(veloFL), Math.abs(veloFR)), Math.max(Math.abs(veloBL), Math.abs(veloBR)) ); // first find the largest of all the powers (get the max of the first two, max of the second two, then get the max of the two maxes)
        if(largest_power > 1.0){ // if the largest power value is greater than 1
            veloFL /= largest_power; // divide each power by the largest one
            veloFR /= largest_power; // resulting in the largest power being 1 (x/x = 1)
            veloBL /= largest_power; // and the rest scaled appropriately
            veloBR /= largest_power;
        }

        // set the motor powers based off of the math done previously - Make that robot go VROOOOM
        driveFL.setPower(veloFL);
        driveFR.setPower(veloFR);
        driveBL.setPower(veloBL);
        driveBR.setPower(veloBR);
    }
    public void driveRobotRelative(Vector3 driveCommands){
        driveRobotRelative(driveCommands.x, driveCommands.y, driveCommands.heading);
    }


    /**
     * Outputs a set of command values, used for reaching a specified position
     * @param currentPose the current position for the robot
     * @param targetPose the target position for the robot
     * @return the correct powers to reach the target position, according to the PIDs
     */
    public Vector3 calcPIDCommands(Vector3 currentPose, Vector3 targetPose){
        return new Vector3(
                xPID.getOutput(currentPose.x, targetPose.x),
                yPID.getOutput(currentPose.y, targetPose.y),
                //headingPID.getOutput(currentPose.heading, targetPose.heading)
                headingPID.getOutput(currentPose.heading, AngleMath.findClosestCoterminalTarget(currentPose.heading, targetPose.heading) )
        );
    }


    public void setCounterRotPerStrafe(double rotationPerStrafe){
        this.counterRotPerStrafe = rotationPerStrafe;
    }
    public void setLateralMultiplier(double lateralMultiplier){
        this.lateralMult = lateralMultiplier;
    }
}

