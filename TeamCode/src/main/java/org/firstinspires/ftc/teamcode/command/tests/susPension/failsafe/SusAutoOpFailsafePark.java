package org.firstinspires.ftc.teamcode.command.tests.susPension.failsafe;

//about to retrieve freight pos: x = -5.3 and y = -.6
//retrieve freight: x= -2.65 and y=0
//outakepos: x= -14 and y=-3.3 and heading = .5 in radians, don't convert from deg to rad
//about to intake pos: x= .5 and y = 5 and heading = 90 deg
//intake pos: x= .5 and y=41 and heading = 90 deg
//potential park pos: x=-18.8 and y= 56.4 and heading = 0

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.tests.susPension.SusPensionBot;
import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner.DriveConstantsSus;
import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner.Drive_Mecanum_RoadRunner_Sus;
import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.OpenCV.FreightFrenzyPipeline;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.Vector2;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.Vector3;

import java.util.ArrayList;
import java.util.Collections;


@Autonomous(group = "@@F")
@Config
public class SusAutoOpFailsafePark extends LinearOpMode {

    public int state;

    public int cycleCount = 0;

    //public static Pose2d preRetreivePos = new Pose2d(-5.3+8, 0-62, Math.toRadians(-90));
    //public static Pose2d retreivePos = new Pose2d(-2.65+8, 0-62, Math.toRadians(-90));

    public static double outakeMaxVeloMod = -5;
    public static double outakeMaxAccelMod = -5;

    public static Pose2d startPosition = new Pose2d(15, -62, Math.toRadians(-90));
    public static Pose2d outakePosFirst = new Pose2d(6, -47.5, Math.toRadians(0));
    public static Pose2d outakePos = new Pose2d(4, -52, Math.toRadians(-80));
    public static Vector2d intakeEntryPos = new Vector2d(25, -63.8);
    public static Pose2d intakeExitPos = new Pose2d(22, -64.1, Math.toRadians(180));
    public static Vector2d intakePos = new Vector2d(68, -64.1);

   // public static Pose2d intakePreExitPos = new Pose2d(50, -67.1, 0);
    public static Pose2d driftCorrection = new Pose2d(0, -1, Math.toRadians(-3));

    private ElapsedTime runTimer;

    public static int totalCycle = 4; //-1 because of if statement logic

    public static double forwardPower = 0.4;

    public static double forwardTime = 2350;

    public String result;


    public boolean firstTime  = true;

    public ElapsedTime duckTimer;




    FreightFrenzyPipeline.ElementPosition elementPosition = FreightFrenzyPipeline.ElementPosition.LEFT;


    @Override
    public void runOpMode() throws InterruptedException {

        runTimer = new ElapsedTime();
        duckTimer = new ElapsedTime();


        SusPensionBot robot = new SusPensionBot(hardwareMap, true);




        waitForStart();

        robot.capServo.setPosition(0.0);




       // outakeTimer.reset();
        runTimer.reset();








        while (opModeIsActive() && !isStopRequested() && runTimer.milliseconds() < forwardTime) {

            robot.drive.driveRobotRelative(forwardPower, 0, 0);





            robot.updateDuckSpinner(false, false);
        }

        robot.drive.driveRobotRelative(new Vector3(0, 0, 0));
    }
}
