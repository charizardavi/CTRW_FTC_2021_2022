package org.firstinspires.ftc.teamcode.command.tests.susPension;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utility.OpenCV.FreightFrenzyPipeline;


/*
    Welcome to the template TeleOp class!
    To use it just make a copy of this class, and change all appearances of "20XX" with the starting year of the current season
    (for example, for the 2020-2021 season, the class would be named TeleOp2020 and all code within would have to reflect that change).
    It is recommended that somewhere near the top you have a comment block that describes your robot control scheme for easy reference.
    Also, don't forget to remove or comment out the "@Disabled" line, so that it will show up in the list of opmodes on the driver station.


    Happy coding!
 */


@TeleOp(name = "testOpMode", group = "@@@")
@Config
public class testOpMode extends LinearOpMode{
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    String robotName = "susPension";

    // Robot Speed variables
    double turnSpeed = 0.5; // Speed multiplier for turning (1 being 100% of power going in)
    double translateSpeed = 0.4; // Speed multiplier for translation (1 being 100% of power going in)

//    public static double runMsec = 3000;
    public static double bigPower = 0;
    public static double smallPower = 0.2;


    // Robot Classes
    private SusPensionBot robot;
    private FtcDashboard dashboard;
    public ElapsedTime runtime;

    private boolean isOutaking = false;

    FreightFrenzyPipeline.ElementPosition elementPosition = FreightFrenzyPipeline.ElementPosition.LEFT;

    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        robot = new SusPensionBot(hardwareMap);
        runtime = new ElapsedTime();

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(7);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot.vision.startWebcamStreaming();

        telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();






        waitForStart(); // Wait for the start button to be pressed before continuing further

        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate

        String vote = robot.vision.getOutput();
        if(vote.equals("LEFT")){
            elementPosition = FreightFrenzyPipeline.ElementPosition.LEFT;
            robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_LOWER;
        }
        else if(vote.equals("RIGHT")) {
            elementPosition = FreightFrenzyPipeline.ElementPosition.RIGHT;
            robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_UPPER_CENTER;
        }
        else{
            elementPosition = FreightFrenzyPipeline.ElementPosition.CENTER;
            robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_MIDDLE_CENTER;
        }


        telemetry.addData("vote",elementPosition);
        telemetry.update();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.x){
                robot.vision.stopWebcamStreaming();
            }
        }


    }
}

