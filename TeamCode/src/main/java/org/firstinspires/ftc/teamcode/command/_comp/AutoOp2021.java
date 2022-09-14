package org.firstinspires.ftc.teamcode.command._comp;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner.Drive_Mecanum_RoadRunner;


/*
    Welcome to the template AutoOp class!
    To use it just make a copy of this class, and change all appearances of "20XX" with the starting year of the current season
    (for example, for the 2020-2021 season, the class would be named TeleOp2020 and all code within would have to reflect that change).
    It is recommended that somewhere near the top you have a comment block that describes your robot control scheme for easy reference.
    Also, don't forget to remove or comment out the "@Disabled" line, so that it will show up in the list of opmodes on the driver station.


    Happy coding!
 */


@Autonomous(name = "AutoOp", group = "@@@")
@Disabled

public class AutoOp2021 extends LinearOpMode{
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    String robotName = "INSERT_ROBOT_NAME_HERE";

    // Robot Speed variables
    double turnSpeed = 0.5; // Speed multiplier for turning (1 being 100% of power going in)
    double translateSpeed = 0.4; // Speed multiplier for transslation (1 being 100% of power going in)


    // Robot Classes
    private Robot2021 robot; // Main robot data class (ALWAYS CREATE AN INSTANCE OF THIS CLASS FIRST - HARDWARE MAP SETUP IS DONE WITHIN)
    private ElapsedTime runtime; // internal clock


    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        robot = new Robot2021(hardwareMap);
        runtime = new ElapsedTime();
        Drive_Mecanum_RoadRunner autoDrive = new Drive_Mecanum_RoadRunner(hardwareMap);


        telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();


        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate










    }


    /* PUT ALL FUNCTIONS HERE */


}
