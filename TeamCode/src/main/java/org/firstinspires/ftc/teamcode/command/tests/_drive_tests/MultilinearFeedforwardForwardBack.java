package org.firstinspires.ftc.teamcode.command.tests._drive_tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command._comp.Robot2021;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.FeedforwardController;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.feedforward_profiles.FeedforwardProfile;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.feedforward_profiles.FeedforwardProfile.ProfileEndBehavior;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.feedforward_profiles.MultislopeLinearProfile;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.TimestampedValue;

import java.util.ArrayList;
import java.util.Arrays;


/*
    Welcome to the template AutoOp class!
    To use it just make a copy of this class, and change all appearances of "20XX" with the starting year of the current season
    (for example, for the 2020-2021 season, the class would be named TeleOp2020 and all code within would have to reflect that change).
    It is recommended that somewhere near the top you have a comment block that describes your robot control scheme for easy reference.
    Also, don't forget to remove or comment out the "@Disabled" line, so that it will show up in the list of opmodes on the driver station.


    Happy coding!
 */


@Autonomous(name = "Linear Feedforward Forward-Back", group = "@@T")

@Config
public class MultilinearFeedforwardForwardBack extends LinearOpMode{
    // TeleOp Variables


    // Robot Classes
    private Robot2021 robot; // Main robot data class (ALWAYS CREATE AN INSTANCE OF THIS CLASS FIRST - HARDWARE MAP SETUP IS DONE WITHIN)
    private ElapsedTime runtime; // internal clock
    private FeedforwardController ffController;

    private FtcDashboard dashboard; // declare the dashboard variable in the opmode's global scope


    public static double FEEDFORWARD_K = 1.0;
    public static ProfileEndBehavior END_BEHAVIOR = ProfileEndBehavior.RESTART;

    private double timeMult = 0.8;
    private TimestampedValue[] timestampedValues = { // the graph of power over time
            new TimestampedValue(0, 0),
            new TimestampedValue(1*timeMult, 0.8),
            new TimestampedValue(2*timeMult, 0),
            new TimestampedValue(3*timeMult, 0),
            new TimestampedValue(4*timeMult, -0.8),
            new TimestampedValue(6*timeMult, 0),
            new TimestampedValue(7*timeMult, 0)
    };


    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        robot = new Robot2021(hardwareMap);
        runtime = new ElapsedTime();


        FeedforwardProfile driveProfile = new MultislopeLinearProfile( // setup the drive profile
                new ArrayList( Arrays.asList(timestampedValues) ),  // converts the array to a list, which the array list constructor then turns into a regular list
                FEEDFORWARD_K,
                1,
                END_BEHAVIOR
        );

        ffController = new FeedforwardController(driveProfile);


        dashboard = FtcDashboard.getInstance(); // initialize the dashboard
        // done this way rather than "new FtcDashboard()" because it has to hook up with the wifi system of the dashboard
        // and there is only one instance of dashboard possible, as two could cause conflicting data to be sent/received
        // so the FtCDashboard class sets up one instance, and hands that instance out

        dashboard.setTelemetryTransmissionInterval( 25 ); // set the transmission interval to 25 milliseconds (default is 100ms)


        telemetry.addData(robot.getName() + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();



        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate

        ffController.resetAndStartProfile(); // start the feedforward profile


        while( opModeIsActive() ){
            double yPower = ffController.getPrimaryOutput(); // get the feedforward motor output

            robot.drive.driveRobotRelative(0, yPower, 0);


            // Driver station telemetry
            telemetry.addData("Seconds since start", runtime.seconds()); // add data to the driver station packet
            telemetry.addData("Milliseconds on profile", ffController.getProfileTime());
            telemetry.addData("Target power", yPower);

            telemetry.update(); // send the data to the driver station


            // Dashboard telemetry
            TelemetryPacket dashPacket = new TelemetryPacket(); // create a dashboard telemetry packet variable

            dashPacket.put( "Profile Primary Output", yPower ); // add a data to the packet of data
            dashPacket.put( "Profile Secondary Output", ffController.getSecondaryOutput() );
            dashPacket.put( "Profile Tertiary Output", ffController.getTertiaryOutput() );

            dashboard.sendTelemetryPacket( dashPacket ); // tell the robot end of the dashboard system to send the packet
        }

    }


    /* PUT ALL FUNCTIONS HERE */


}
