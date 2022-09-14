package org.firstinspires.ftc.teamcode.command.tests._manipulator_tests;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.linear_motion.LinearSlide_PIDControlled;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.PIDController;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.VeloLimiter;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.coefficient_wrappers.PIDCoefficients;


/*
    Welcome to the 2020-2021 TeleOp class!

    Just kidding! This is test class. You can't trust any of the comments here, it is made in a hurry to do a job, usually lots of copy paste
 */


@TeleOp(name = "Linear Slide Test", group = "@@T")
@Config

//hi emma - Suineg
public class LinearSlideTest extends LinearOpMode{
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    String robotName = "TestBot";
    public static String MOTOR_NAME = "motorDuck";

    public static PIDCoefficients PID_GAINS = new PIDCoefficients(0.4, 0.0, 6);
    public static double maxAccelPerSec = 0;
    public static double CONTROLLER_DEAD_ZONE_RADIUS = 0.05;
    public static double primaryTargetPos = 16.0;
    public static double secondaryTargetPos = 0;
    public static double forcedTravelTime = 0;
    public static double slideMinToMoveServo = 5.0;

    // Robot Classes
    private LinearSlide_PIDControlled slide;
    private ElapsedTime runtime; // internal clock

    // Flags
    private boolean targetIsPrimaryTarget = false;



    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        runtime = new ElapsedTime();

        DcMotor mainMotor = hardwareMap.get(DcMotor.class, MOTOR_NAME);
        slide = new LinearSlide_PIDControlled(mainMotor, new PIDController(PID_GAINS), new VeloLimiter(1.0, maxAccelPerSec, false));
        slide.setForcedTravelTime(forcedTravelTime);

        //Servo testServo = hardwareMap.get(Servo.class, "bucketArmServo");
        //testServo.setDirection(Servo.Direction.REVERSE);

        telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();


        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate



        // The main run loop - write the main robot run code here
        while (opModeIsActive()) {
            // Variables
            boolean isRunning = gamepad1.right_bumper || gamepad1.left_bumper;  // If true, the robot will go at the boost speed, otherwise it will go at the base speed (just impacts translation)
            double motorSpeed = -gamepad1.left_stick_y;
            boolean runningPID = false;

            if (Math.abs(motorSpeed) < CONTROLLER_DEAD_ZONE_RADIUS){
                runningPID = true;
            }
            if(gamepad1.dpad_up){
                targetIsPrimaryTarget = true;
                slide.setTargetPosition(primaryTargetPos);
            }
            else if(gamepad1.dpad_down){
                targetIsPrimaryTarget = false;
                slide.setTargetPosition(secondaryTargetPos);
            }

            if(targetIsPrimaryTarget && slide.getInchPosition() >= slideMinToMoveServo){
               // testServo.setPosition(MultiSetstateServoTest.setState3);
            }
            else{
               // testServo.setPosition(MultiSetstateServoTest.setState1);
            }

            if(slide.getErrorToTarget() >= 0){
                //slide.getVeloLimiter().setMaxSpeed(0.8);
            }

            if( targetIsPrimaryTarget ){
                telemetry.addLine("Current target is the primary target ("+ primaryTargetPos + ")");
                telemetry.addLine("Press D-Pad Down to set the target to the secondary target (" + secondaryTargetPos + ")");
            }
            else{
                telemetry.addLine("Current target is the secondary target ("+ secondaryTargetPos + ")");
                telemetry.addLine("Press D-Pad Up set the target to the primary target (" + primaryTargetPos + ")");
            }

            telemetry.addData("Slide position", slide.getInchPosition());
            telemetry.addData("Error to target", slide.getErrorToTarget());
            telemetry.addData("Motor encoder position", mainMotor.getCurrentPosition());

            if(isRunning){ // add telemetry relating to robot drive mode
                telemetry.addLine("Running motor at " + mainMotor.getPower() * 100.0 + "% of max speed");

                if(runningPID){
                    slide.runToInternalTarget();
                }
                else{
                    mainMotor.setPower(motorSpeed);
                }
            }
            else{
                telemetry.addLine("Safety enabled, hold a bumper (on gamepad1) to run the motor.");

                mainMotor.setPower(0.0);
            }

            telemetry.addData("Slide Max Speed", slide.getVeloLimiter().getMaxSpeed());


            telemetry.update();
        }
    }


    /* PUT ALL FUNCTIONS HERE */

}
