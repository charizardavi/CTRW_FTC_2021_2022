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

import org.firstinspires.ftc.teamcode.utility.ControlTheory.feedforward_profiles.TrapezoidalProfile;


/*
    Welcome to the template AutoOp class!
    To use it just make a copy of this class, and change all appearances of "20XX" with the starting year of the current season
    (for example, for the 2020-2021 season, the class would be named TeleOp2020 and all code within would have to reflect that change).
    It is recommended that somewhere near the top you have a comment block that describes your robot control scheme for easy reference.
    Also, don't forget to remove or comment out the "@Disabled" line, so that it will show up in the list of opmodes on the driver station.


    Happy coding!
 */


@Autonomous(name = "Smooth Feedforward Forward-Back", group = "@@T")

@Config
public class SmoothFeedforwardForwardBack extends LinearOpMode{
    // TeleOp Variables
    public static double PRIMARY_K = 1.0;
    public static double SECONDARY_K = 1.0;
    public static double TERTIARY_K = 1.0;


    // Robot Classes
    private Robot2021 robot; // Main robot data class (ALWAYS CREATE AN INSTANCE OF THIS CLASS FIRST - HARDWARE MAP SETUP IS DONE WITHIN)
    private ElapsedTime runtime; // internal clock
    private FeedforwardController ffController;

    private FtcDashboard dashboard;


    private FeedforwardProfile profileDriveForward = new TrapezoidalProfile(0.0, 0.8, 1.0, 0.5, PRIMARY_K, SECONDARY_K, TERTIARY_K, ProfileEndBehavior.BACKTRACK);
    private FeedforwardProfile profileDriveBackward = new TrapezoidalProfile(0.0, -0.8, 1.0, 0.5, PRIMARY_K, SECONDARY_K, TERTIARY_K, ProfileEndBehavior.BACKTRACK);


    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        robot = new Robot2021(hardwareMap);
        runtime = new ElapsedTime();
        ffController = new FeedforwardController(profileDriveForward);

        dashboard = FtcDashboard.getInstance(); // init the dashboard object
        dashboard.setTelemetryTransmissionInterval( 25 ); // set interval to 25 msec


        telemetry.addData(robot.getName() + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();


        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate

        ffController.resetAndStartProfile(); // start the feedforward profile


        while( opModeIsActive() ){
            if( ffController.getProfileTime() >= 2*ffController.getProfile().getProfileDuration() ){ // if enough time has passed on this profile to have gone all the way up in power and back down, swap profiles
                if( ffController.getProfile().equals( profileDriveForward ) ){ // if the forward profile, set to the backwards profile
                    ffController.setProfile( profileDriveBackward );
                }
                else { // otherwise set to the forward profile
                    ffController.setProfile( profileDriveForward );
                }
            }

            double yPower = ffController.getPrimaryOutput(); // get the feedforward motor output

            robot.drive.driveRobotRelative(0, yPower, 0);


            telemetry.addData("Seconds since start", runtime.seconds());
            telemetry.addData("Milliseconds on profile", ffController.getProfileTime());
            telemetry.addData("Target power", yPower);


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
