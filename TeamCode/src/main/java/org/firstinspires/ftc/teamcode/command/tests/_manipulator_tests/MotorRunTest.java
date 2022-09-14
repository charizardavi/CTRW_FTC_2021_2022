package org.firstinspires.ftc.teamcode.command.tests._manipulator_tests;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
    Welcome to the 2020-2021 TeleOp class!

    Just kidding! This is test class. You can't trust any of the comments here, it is made in a hurry to do a job, usually lots of copy paste
 */


@TeleOp(name = "Single Motor Run Test", group = "@@T")
@Config

//hi emma - Suineg
public class MotorRunTest extends LinearOpMode{
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    String robotName = "TestBot";
    public static String MOTOR_NAME = "motorIntake";

    // Robot Speed variables
    private double stopSpeed = 0.0; // the motor speed for stopping the robot
    private double motorSpeed = 0.70;
    private double changeAmount = 0.05;

    // Constants
    static final double DEAD_ZONE_RADIUS = 0.05; // the minimum value that can be passed into the drive function

    // Robot Classes
    private ElapsedTime runtime; // internal clock

    // Flags
    private boolean reversed = false; // default
    private boolean firstIncreaseSpeed = true; // used to ensure proper toggling behavior (see usage under logic section)
    private boolean firstDecreaseSpeed = true; // used to ensure proper toggling behavior (see usage under logic section)
    private boolean firstReverseToggle = true;



    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        runtime = new ElapsedTime();

        DcMotor mainMotor = hardwareMap.get(DcMotor.class, MOTOR_NAME);

        telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();


        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate



        // The main run loop - write the main robot run code here
        while (opModeIsActive()) {
            // Variables
            boolean isRunning = gamepad1.right_bumper;  // If true, the robot will go at the boost speed, otherwise it will go at the base speed (just impacts translation)


            // Logic (figuring out what the robot should do)

            if(gamepad1.dpad_up && firstIncreaseSpeed){ // toggle driving realtive to field if dpad up is pressed
                motorSpeed += changeAmount;

                motorSpeed = Math.min(motorSpeed, 1);

                firstIncreaseSpeed = false; // set the variable false so that it cannot toggle again
            }
            else if (!gamepad1.dpad_up){ // wait to set the flag back to true until the button is released
                firstIncreaseSpeed = true; // until the button is released
            }

            if(gamepad1.dpad_down && firstDecreaseSpeed){ // toggle driving using encoders on the press of dpad down
                motorSpeed -= changeAmount;

                motorSpeed = Math.max(motorSpeed, 0);

                firstDecreaseSpeed = false; // set the variable false so that it cannot toggle again
            }
            else if (!gamepad1.dpad_down){ // wait to set the flag back to true until the button is released
                firstDecreaseSpeed = true; // until the button is released
            }


            if(gamepad1.dpad_left && firstReverseToggle){ // toggle driving using encoders on the press of dpad down
                reversed = !reversed;

                firstReverseToggle = false; // set the variable false so that it cannot toggle again
            }
            else if (!gamepad1.dpad_left){ // wait to set the flag back to true until the button is released
                firstReverseToggle = true; // until the button is released
            }



            // Telemetry
            if(isRunning){ // add telemetry relating to robot drive mode
                telemetry.addLine("Running motors at " + motorSpeed * 100.0 + "% of max speed");

                if(reversed){
                    mainMotor.setPower(-motorSpeed);
                }
                else {
                    mainMotor.setPower(motorSpeed);
                }
            }
            else{
                telemetry.addLine("Motor not running, hold the Right Bumper (on gamepad1) to run the motor.");

                mainMotor.setPower(stopSpeed);
            }
            telemetry.addLine("Press D-Pad Up to increase speed by " + (changeAmount * 100) + "%");
            telemetry.addLine("Press D-Pad Down to decrease speed by " +( -changeAmount * 100) + "%");
            telemetry.addLine("Is Reversed: " + reversed + " (press D-Pad left to toggle direction)");



            telemetry.update();
        }
    }


    /* PUT ALL FUNCTIONS HERE */

}
