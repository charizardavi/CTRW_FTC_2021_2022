package org.firstinspires.ftc.teamcode.command.tests._drive_tests;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.legacy.Provider2020;
import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.Drive_Mecanum;
import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.legacy.Drive_Mecanum_Tele_2020;
import org.firstinspires.ftc.teamcode.utility.Odometry.StandardTrackingWheelLocalizer;


/*
    Welcome to the 2020-2021 TeleOp class!

    Robot control scheme:
        Main Drive:
        - Gamepad1 left stick = translation
        - Gamepad1 right stick (x axis only) = rotation
        - Gamepad1 right bumbper = boost button

        Mode toggling:
        - Gamepad1 dpad_up = toggle drive relative to field
        - Gamepad1 dpad_down = toggle drive using encoders
 */


@TeleOp(name = "MecanumDriveTest", group = "@@T")
@Config

public class MecanumDriveTest extends LinearOpMode{
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    String robotName = "Scotty";

    // Robot Speed variables
    public static double turnSpeed = 0.6; // Speed multiplier for turning (1 being 100% of power going in)
    public static double translateSpeed = 0.4; // Speed multiplier for translation (1 being 100% of power going in)
    public static double boostMult = 2.0; // Speed multiplier for BOOSTING (1 being 100% of power going in)
    public static double stopSpeed = 0.0; // the motor speed for stopping the robot
    public static double counterRotPerStrafe = 0.05;

    // Constants
    static final double DEAD_ZONE_RADIUS = 0.05; // the minimum value that can be passed into the drive function

    // Robot Classes
    private Provider2020 robot; // Main robot data class (ALWAYS CREATE AN INSTANCE OF THIS CLASS FIRST - HARDWARE MAP SETUP IS DONE WITHIN)
    private ElapsedTime runtime; // internal clock
    Drive_Mecanum mecanum_drive; // the main mecanum drive class
    StandardTrackingWheelLocalizer localizer; // the odometry based localizer - uses dead wheels to determine (x, y, r) position on the field

    // Flags
    private boolean driveFieldRelative = true; // default
    private boolean firstToggleDriveRelative = true; // used to ensure proper toggling behavior (see usage under logic section)
    private boolean firstToggleRunEncoders = true; // used to ensure proper toggling behavior (see usage under logic section)

    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        robot = new Provider2020(hardwareMap, true);
        runtime = new ElapsedTime();
        mecanum_drive = new Drive_Mecanum(robot.driveFL, robot.driveFR, robot.driveBL, robot.driveBR); // pass in the drive motors and the speed variables to setup properly
        localizer = new StandardTrackingWheelLocalizer(hardwareMap);


        robot.setEncoderActive(false); // start the game without running encoders
        mecanum_drive.setCounterRotPerStrafe(counterRotPerStrafe);

        telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();

        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate


        //robot.driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // The main run loop - write the main robot run code here
        while (opModeIsActive()) {
            if(localizer != null){ // if the localier exists
                localizer.update(); // update our current position
            }

            // Variables
            boolean isBoosting = gamepad1.right_bumper;  // If true, the robot will go at the boost speed, otherwise it will go at the base speed (just impacts translation)
            double xTranslatePower = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y); // set the robot translation/rotation speed variables based off of controller input (set later in hardware manipluation section)
            double yTranslatePower = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x); // specifically the y stick is negated because up is negative on the stick, but we want up to move the robot forward
            double rotatePower = gamepad1.right_stick_x;




            // Logic (figuring out what the robot should do)

            if(gamepad1.dpad_up && firstToggleDriveRelative){ // toggle driving realtive to field if dpad up is pressed
                driveFieldRelative = !driveFieldRelative; // toggle the value

                firstToggleDriveRelative = false; // set the variable false so that it cannot toggle again
            }
            else if (!gamepad1.dpad_up){ // wait to set the flag back to true until the button is released
                firstToggleDriveRelative = true; // until the button is released
            }

            if(gamepad1.dpad_down && firstToggleRunEncoders){ // toggle driving using encoders on the press of dpad down
                robot.driveUsingEncoders = !robot.driveUsingEncoders; // toggle the value
                robot.setEncoderActive(robot.driveUsingEncoders); //update the encoder mode

                firstToggleRunEncoders = false; // set the variable false so that it cannot toggle again
            }
            else if (!gamepad1.dpad_down){ // wait to set the flag back to true until the button is released
                firstToggleRunEncoders = true; // until the button is released
            }

            //setup a dead zone for the controllers
            if(Math.abs(xTranslatePower) <= DEAD_ZONE_RADIUS){ // if the value is less than the maximum deadzone value, set to zero (to stop the motor)
                xTranslatePower = stopSpeed;
            }
            if(Math.abs(yTranslatePower) <= DEAD_ZONE_RADIUS){ // if the value is less than the maximum deadzone value, set to zero (to stop the motor)
                yTranslatePower = stopSpeed;
            }
            if(Math.abs(rotatePower) <= DEAD_ZONE_RADIUS){ // if the value is less than the maximum deadzone value, set to zero (to stop the motor)
               rotatePower = stopSpeed;
            }

            if( isBoosting ){
                xTranslatePower *= boostMult;
                yTranslatePower *= boostMult;
            }


            // Hardware instruction (telling the hardware what to do)
            if(driveFieldRelative){
                mecanum_drive.driveFieldRelative(xTranslatePower, yTranslatePower, rotatePower, robot.getHeading()); // call the drive field relative method
            }
            else {
                mecanum_drive.driveRobotRelative(xTranslatePower, yTranslatePower, rotatePower); // call the drive robot relative method
            }


            // Telemetry
            if(driveFieldRelative){ // add telemetry relating to robot drive mode
                telemetry.addLine("Driving field relative");
            }
            else{
                telemetry.addLine("Driving robot relative");
            }

            if(robot.driveUsingEncoders){
                telemetry.addLine("Driving using encoders");

                telemetry.addData("Drive FL Encoder: ", robot.driveFL.getCurrentPosition()); // add telemetry data for motor encoders
                telemetry.addData("Drive FR Encoder: ", robot.driveFR.getCurrentPosition());
                telemetry.addData("Drive BL Encoder: ", robot.driveBL.getCurrentPosition());
                telemetry.addData("Drive BR Encoder: ", robot.driveBR.getCurrentPosition());
            }
            else{
                telemetry.addLine("Driving without encoders");
            }

            telemetry.addData("Boosting: ", isBoosting);

            if(localizer != null){ // if we have a localizer that exists, get the position estimate from it
                telemetry.addData("Field Position", localizer.getPoseEstimate());
                //telemetry.addData("Current Velocity", localizer.getPoseVelocity());
            }

            telemetry.addData("Left Encoder: ", localizer.getWheelPositions().get(0));
            telemetry.addData("Right Encoder: ", localizer.getWheelPositions().get(1));

            //  telemetry.addData("Encoder reading", robot.driveFL.getCurrentPosition());

            telemetry.update();
        }
    }


    /* PUT ALL FUNCTIONS HERE */

}
