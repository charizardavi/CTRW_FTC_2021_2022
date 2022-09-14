package org.firstinspires.ftc.teamcode.command.tests._drive_tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.Drive_Mecanum;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.PIDController;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.VeloLimiter;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.coefficient_wrappers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.filters.LowPassFilter;
import org.firstinspires.ftc.teamcode.utility.FTCDashboard.DashboardUtil;
import org.firstinspires.ftc.teamcode.utility.Math.AngleMath;
import org.firstinspires.ftc.teamcode.utility.Odometry.TrackingWheelLocalizer2021_Scrappy;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.Range2d;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.Vector3;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "@@T")
@Config

public class DrivePIDTest extends LinearOpMode {
    // Test Points
    private static final Pose2d DOWN_POINT  = new Pose2d(0, 0, Math.toRadians(0));
    private static final Pose2d LEFT_POINT  = new Pose2d(0, 5, Math.toRadians(90));
    private static final Pose2d UP_POINT    = new Pose2d(10, 5, Math.toRadians(180));
    private static final Pose2d RIGHT_POINT = new Pose2d(15, 0, Math.toRadians(225));

    public static PIDCoefficients transPIDCoeff = new PIDCoefficients(0.058, 0, 0);
    public static Range2d transPIDIActiveRange = new Range2d(-0.7, 0.7);
    public static double transPIDPreviousStateGain = 0.2;
    public static double transPIDMinMotorMovePower = 0.025;

    public static PIDCoefficients headingPIDCoeff = new PIDCoefficients(0.5, 0, 0);
    public static Range2d headingPIDIActiveRange = new Range2d(-0.7, 0.7);
    public static double headingPIDPreviousStateGain = 0.2;
    public static double headingPIDMinMotorMovePower = 0.035;



    public static boolean LIMIT_ACCEL = false;
    public static double MAX_TRANS_ACCEL_PER_SEC = 0.01;
    public static double MAX_TRANS_SPEED_PER_SEC = 30;

    public static double MAX_HEADING_ACCEL_PER_SEC = 0.05;
    public static double MAX_HEADING_SPEED_PER_SEC = 90;

    public static double DRIVE_LATERAL_MULT = 1.2;



    private Pose2d targetPose = DOWN_POINT;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance(); // setup the dashboard
        dashboard.setTelemetryTransmissionInterval(7); // interval in milliseconds

        DcMotor driveFL = hardwareMap.get(DcMotor.class, "driveFL");
        DcMotor driveFR = hardwareMap.get(DcMotor.class, "driveFR");
        DcMotor driveBL = hardwareMap.get(DcMotor.class, "driveBL");
        DcMotor driveBR = hardwareMap.get(DcMotor.class, "driveBR");
        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBL.setDirection(DcMotor.Direction.REVERSE);

        PIDController transPID = new PIDController(transPIDCoeff, transPIDIActiveRange, new LowPassFilter(transPIDPreviousStateGain));
        transPID.setMinMotorMovePower( transPIDMinMotorMovePower );
        PIDController headingPID = new PIDController(headingPIDCoeff, headingPIDIActiveRange, new LowPassFilter(headingPIDPreviousStateGain));
        headingPID.setMinMotorMovePower( headingPIDMinMotorMovePower );

        Drive_Mecanum drive = new Drive_Mecanum(driveFL, driveFR, driveBL, driveBR,
                transPID,
                headingPID
        );
        drive.setCounterRotPerStrafe(0);
        drive.setLateralMultiplier(DRIVE_LATERAL_MULT);

        TrackingWheelLocalizer2021_Scrappy odo = new TrackingWheelLocalizer2021_Scrappy(hardwareMap);


        VeloLimiter xLimiter = new VeloLimiter(MAX_TRANS_SPEED_PER_SEC, MAX_TRANS_ACCEL_PER_SEC, false);
        VeloLimiter yLimiter = new VeloLimiter(MAX_TRANS_SPEED_PER_SEC, MAX_TRANS_ACCEL_PER_SEC, false);
        VeloLimiter headingLimiter = new VeloLimiter(MAX_HEADING_SPEED_PER_SEC, MAX_HEADING_ACCEL_PER_SEC, false);

        waitForStart();

        ElapsedTime clockTimer = new ElapsedTime();


        while (!isStopRequested()) {
            odo.update();
            Pose2d poseEstimate = odo.getPoseEstimate();


            if(gamepad1.dpad_down)
                targetPose = DOWN_POINT;
            else if(gamepad1.dpad_left)
                targetPose = LEFT_POINT;
            else if(gamepad1.dpad_right)
                targetPose = RIGHT_POINT;
            else if(gamepad1.dpad_up)
                targetPose = UP_POINT;


            Vector3 driveCommands = drive.calcPIDCommands(AngleMath.toVector3(poseEstimate), AngleMath.toVector3(targetPose));

            if(LIMIT_ACCEL){
                driveCommands.x = xLimiter.getLimitedVelo(driveCommands.x);
                driveCommands.y = yLimiter.getLimitedVelo(driveCommands.y);
                driveCommands.heading = headingLimiter.getLimitedVelo(driveCommands.heading);
            }

            drive.driveRobotRelative( driveCommands );


            double loopTime = clockTimer.milliseconds();
            clockTimer.reset();


            // FTC Dashboard telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("x", poseEstimate.getX());
            packet.put("y", poseEstimate.getY());
            packet.put("heading", Math.toDegrees(poseEstimate.getHeading()));
            packet.put("target position", targetPose);
            packet.put("loop time (msec)", loopTime);


            Canvas fieldOverlay = packet.fieldOverlay();
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.setStroke("#3F51B5"); // set the current draw color to blue

            DashboardUtil.drawRobot(fieldOverlay, poseEstimate);

            fieldOverlay.setStroke("#800080"); // set the current draw color to blue
            fieldOverlay.strokeCircle(targetPose.getX(), targetPose.getY(), 2);


            dashboard.sendTelemetryPacket(packet);
        }
    }
}
