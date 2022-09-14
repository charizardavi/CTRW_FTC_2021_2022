package org.firstinspires.ftc.teamcode.command.tests._manipulator_tests;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.linear_motion.LinearSlide_PIDControlled;
import org.firstinspires.ftc.teamcode.hardware.servo_managers.Servo_MultiSetState;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.PIDController;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.VeloLimiter;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.coefficient_wrappers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.NamedState;


/*
    Welcome to the 2020-2021 TeleOp class!

    Just kidding! This is test class. You can't trust any of the comments here, it is made in a hurry to do a job, usually lots of copy paste
 */


@TeleOp(name = "Multi-SetState Servo Test", group = "@@T")
@Config

//hi emma - Suineg
public class MultiSetstateServoTest extends LinearOpMode{
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    String robotName = "TestBot";
    public static String SERVO_NAME = "bucketArmServo";

    public static double setState1 = 0.03;
    public static double setState2 = 0.5;
    public static double setState3 = 0.8;

    // Robot Classes
    private Servo_MultiSetState servoManager;
    private ElapsedTime runtime; // internal clock

    // Flags
    private String targetStateName = "1";



    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        runtime = new ElapsedTime();

        Servo servo = hardwareMap.get(Servo.class, SERVO_NAME);
        servo.setDirection(Servo.Direction.REVERSE);
        servoManager = new Servo_MultiSetState(servo, new NamedState("1", setState1), new NamedState("2", setState2), new NamedState("3", setState3));

        telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();


        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate



        // The main run loop - write the main robot run code here
        while (opModeIsActive()) {

            if(gamepad1.dpad_down || gamepad2.dpad_down)
                targetStateName = "1";
            else if(gamepad1.dpad_right || gamepad2.dpad_right)
                targetStateName = "2";
            else if(gamepad1.dpad_up || gamepad2.dpad_up)
                targetStateName = "3";


            servoManager.goToStateNamed(targetStateName);

            telemetry.addData("Active state", servoManager.getActiveState());
            telemetry.addLine("Use the D-PAD to set which target state the servo goes to:");
            telemetry.addLine(" - Down = State 1");
            telemetry.addLine(" - Right = State 2");
            telemetry.addLine(" - Up = State 3");


            telemetry.update();
        }
    }


    /* PUT ALL FUNCTIONS HERE */

}
