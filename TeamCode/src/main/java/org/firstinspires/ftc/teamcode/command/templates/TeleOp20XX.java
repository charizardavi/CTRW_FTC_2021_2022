package org.firstinspires.ftc.teamcode.command.templates;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.legacy.Drive_Mecanum_Tele_2020;


/*
    Welcome to the template TeleOp class!
    To use it just make a copy of this class, and change all appearances of "20XX" with the starting year of the current season
    (for example, for the 2020-2021 season, the class would be named TeleOp2020 and all code within would have to reflect that change).
    It is recommended that somewhere near the top you have a comment block that describes your robot control scheme for easy reference.
    Also, don't forget to remove or comment out the "@Disabled" line, so that it will show up in the list of opmodes on the driver station.


    Happy coding!
 */


@TeleOp(name = "TeleOp20XX", group = "@@@")
@Disabled

public class TeleOp20XX extends LinearOpMode{
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    String robotName = "INSERT_ROBOT_NAME_HERE";

    // Robot Speed variables
    double turnSpeed = 0.5; // Speed multiplier for turning (1 being 100% of power going in)
    double translateSpeed = 0.4; // Speed multiplier for transslation (1 being 100% of power going in)


    // Robot Classes
    private Robot20XX robot; // Main robot data class (ALWAYS CREATE AN INSTANCE OF THIS CLASS FIRST - HARDWARE MAP SETUP IS DONE WITHIN)
    private ElapsedTime runtime; // internal clock


    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        robot = new Robot20XX(hardwareMap);
        runtime = new ElapsedTime();


        telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();


        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate


        // The main run loop - write the main robot run code here
        while (opModeIsActive()) {
            // Variables



            // Hardware instruction (telling the hardware what to do)

            robot.drive.driveRobotRelative(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }


    /* PUT ALL FUNCTIONS HERE */

}
