package org.firstinspires.ftc.teamcode.command.tests.z_system_specific_tests.ReplayRecorder;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.legacy.Provider2020;
import org.firstinspires.ftc.teamcode.utility.Odometry.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.ReplayRecorder.Drive_Mecanum_RepRec;

@Autonomous(name = "ReplayRecorder Back+Forth", group = "@@T")
public class RepRecBackAndForth extends LinearOpMode {
    public static Pose2d startPose = new Pose2d(0, 0, 0);
    public static Pose2d drivePose = new Pose2d(15, 0, 0);
    public static double DRIVE_REVERSE_TIME = 5000; // gives the robot 5 seconds to drive there


    // Robot Classes
    private Provider2020 robot; // Main robot data class (ALWAYS CREATE AN INSTANCE OF THIS CLASS FIRST - HARDWARE MAP SETUP IS DONE WITHIN)
    private Drive_Mecanum_RepRec mecanumDrive; // the main mecanum drive class
    private StandardTrackingWheelLocalizer localizer; // the odometry based localizer - uses dead wheels to determine (x, y, r) position on the field
    private ElapsedTime driveTime;

    private boolean runningForward = true;


    public void runOpMode(){
        robot = new Provider2020(hardwareMap);
        mecanumDrive = new Drive_Mecanum_RepRec(robot.driveFL, robot.driveFR, robot.driveBL, robot.driveBR); // pass in the drive motors and the speed variables to setup properly
        localizer = new StandardTrackingWheelLocalizer(hardwareMap);
        driveTime = new ElapsedTime();

        localizer.setPoseEstimate(startPose);


        waitForStart();
        driveTime.reset();


        while(opModeIsActive() && !isStopRequested()){
            localizer.update();

            if(driveTime.milliseconds() >= DRIVE_REVERSE_TIME){
                runningForward = !runningForward;

                driveTime.reset();
            }

            if(runningForward){
                mecanumDrive.driveToReplayPose(localizer.getPoseEstimate(), drivePose);
            }
            else {
                mecanumDrive.driveToReplayPose(localizer.getPoseEstimate(), startPose);
            }
        }
    }
}
