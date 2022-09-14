package org.firstinspires.ftc.teamcode.command.tests.susPension.autos;

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
import org.firstinspires.ftc.teamcode.command.tests.susPension.failsafe.SusPensionBotFailsafe;
import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner.DriveConstantsSus;
import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner.Drive_Mecanum_RoadRunner_Sus;
import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.OpenCV.FreightFrenzyPipeline;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.Vector2;

import java.util.ArrayList;
import java.util.Collections;


@Autonomous(group = "@@@")
@Config
public class SusAutoOpRedCycle extends LinearOpMode {

    public int state;

    public int cycleCount = 0;

    //public static Pose2d preRetreivePos = new Pose2d(-5.3+8, 0-62, Math.toRadians(-90));
    //public static Pose2d retreivePos = new Pose2d(-2.65+8, 0-62, Math.toRadians(-90));

    public static double outakeMaxVeloMod = 0;
    public static double outakeMaxAccelMod = 0;

    public static Pose2d startPosition = new Pose2d(15, -62, Math.toRadians(-90));
    public static Pose2d outakePosFirst = new Pose2d(6, -47.5, Math.toRadians(0));
    public static Pose2d outakePos = new Pose2d(4, -55, Math.toRadians(-80));
    public static Vector2d intakeEntryPos = new Vector2d(25, -63.8);
    public static Pose2d intakeExitPos = new Pose2d(22, -64.1, Math.toRadians(180));
    public static Vector2d intakePos = new Vector2d(68, -64.1);

   // public static Pose2d intakePreExitPos = new Pose2d(50, -67.1, 0);
    public static Pose2d driftCorrection = new Pose2d(0, -1, Math.toRadians(-3));

    private ElapsedTime outakeTimer;

    public static int totalCycle = 2;



    public static int testCount = 20;

    public String result;


    public boolean firstTime  = true;

    public ElapsedTime duckTimer;



    enum State{
        FIRST_OUTAKE,
        INTAKE_CYCLE,
        EXIT_INTAKE,
        OUTAKE_CYCLE,
        IDLE
    }

    State currentState = State.IDLE;

    FreightFrenzyPipeline.ElementPosition elementPosition = FreightFrenzyPipeline.ElementPosition.LEFT;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.addData("status", "initializing");

        outakeTimer = new ElapsedTime();
        duckTimer = new ElapsedTime();

        telemetry.update();

        SusPensionBot robot = new SusPensionBot(hardwareMap, true);

        robot.driveRoadRunner.setPoseEstimate( startPosition );




        TrajectorySequence firstOutake = robot.driveRoadRunner.trajectorySequenceBuilder(startPosition)
                .back(7)
                .addDisplacementMarker(()->{
                    robot.unlockIntake();
                    robot.intake();
                })
                .strafeRight(8.9)

                .forward(5)//TUNE THIS
                .addDisplacementMarker(()->{robot.forceFreightDetect();})
                .lineToSplineHeading(outakePos)//, outakePos.getHeading()+ Math.toRadians(180))
                .addDisplacementMarker(()->{
                    robot.eject();
                })
                .build();

        Trajectory cycleIntake = new TrajectoryBuilder(firstOutake.end(), Math.toRadians(-90), Drive_Mecanum_RoadRunner_Sus.getVelocityConstraint(DriveConstantsSus.MAX_VEL + outakeMaxVeloMod, DriveConstantsSus.MAX_ANG_VEL, DriveConstantsSus.TRACK_WIDTH), Drive_Mecanum_RoadRunner_Sus.getAccelerationConstraint(DriveConstantsSus.MAX_ACCEL + outakeMaxAccelMod))
                .addDisplacementMarker(()->{
                    robot.intake();
                })
                .splineToSplineHeading(new Pose2d(intakeEntryPos.getX(), intakeEntryPos.getY(), 0), 0)
                .splineToSplineHeading(new Pose2d(intakePos.getX(), intakePos.getY(), 0), 0)
                .addDisplacementMarker(()->{
                    robot.forceFreightDetect();
                })
                .build();

        Trajectory cycleOutake = new TrajectoryBuilder(cycleIntake.end(), Math.toRadians(180), Drive_Mecanum_RoadRunner_Sus.getVelocityConstraint(DriveConstantsSus.MAX_VEL + outakeMaxVeloMod, DriveConstantsSus.MAX_ANG_VEL, DriveConstantsSus.TRACK_WIDTH), Drive_Mecanum_RoadRunner_Sus.getAccelerationConstraint(DriveConstantsSus.MAX_ACCEL + outakeMaxAccelMod))
                .addDisplacementMarker(()->{
                    firstTime = true;
                })
                .splineToSplineHeading( new Pose2d(intakeExitPos.getX(), intakeExitPos.getY(), 0), intakeExitPos.getHeading())
                .splineToSplineHeading(new Pose2d(outakePos.getX(), outakePos.getY(), outakePos.getHeading()), outakePos.getHeading())
                .addDisplacementMarker(()->{
                    robot.eject();
                })
                .build();

        robot.vision.startWebcamStreaming();

        telemetry.addData("status", "ready");
        telemetry.update();


        waitForStart();

        robot.capServo.setPosition(0.0);

        String vote;
        ArrayList<String> votes = new ArrayList<>();
        for(int i = 0; i < testCount; i++){
            vote = robot.vision.getOutput();
            votes.add(vote);
        }
        int leftOccur = Collections.frequency(votes, "LEFT");
        int rightOccur = Collections.frequency(votes, "RIGHT");
        int centerOccur = Collections.frequency(votes, "CENTER");

        if (leftOccur == Math.max(leftOccur, rightOccur)){
            if (leftOccur == Math.max(leftOccur, centerOccur)){
                result = "left";
                elementPosition = FreightFrenzyPipeline.ElementPosition.LEFT;
                robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_LOWER;
            }
            else if (centerOccur == Math.max(leftOccur, centerOccur)){
                result = "center";
                elementPosition = FreightFrenzyPipeline.ElementPosition.CENTER;
                robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_MIDDLE_CENTER;
            }
        }
        else if (rightOccur == Math.max(leftOccur, rightOccur)){
            if (rightOccur == Math.max(rightOccur, centerOccur)){
                result = "right";
                elementPosition = FreightFrenzyPipeline.ElementPosition.RIGHT;
                robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_UPPER_CENTER;
            }
            else if (centerOccur == Math.max(rightOccur, centerOccur)){
                result = "center";
                elementPosition = FreightFrenzyPipeline.ElementPosition.CENTER;
                robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_MIDDLE_CENTER;
            }
        }

//        if(vote.equals("LEFT")){
//            elementPosition = FreightFrenzyPipeline.ElementPosition.LEFT;
//            robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_NEUTRAL_RIGHT;
//        }
//        else if(vote.equals("RIGHT")) {
//            elementPosition = FreightFrenzyPipeline.ElementPosition.RIGHT;
//            robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_UPPER_RIGHT;
//        }
//        else{
//            elementPosition = FreightFrenzyPipeline.ElementPosition.CENTER;
//            robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_MIDDLE_RIGHT;
//        }

        robot.vision.stopWebcamStreaming();


        outakeTimer.reset();
        duckTimer.reset();

        currentState = State.FIRST_OUTAKE;

        robot.driveRoadRunner.followTrajectorySequenceAsync(firstOutake);





        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState){
                case FIRST_OUTAKE:
                    if (!robot.driveRoadRunner.isBusy()){
                        robot.eject();
                        if (firstTime){
                            outakeTimer.reset();
                            firstTime = false;
                        }
                        if( outakeTimer.milliseconds() >= robot.ejectTime ){
                            robot.ejectDone = true;
                            //robot.intakeMotor.setPower(0.0);
                            robot.stow();
                            //if (!robot.isThereFreight() && robot.outakeState == SusPensionBot.OutakeStateList.OUTAKE_UPPER_CENTER){
                            currentState = State.INTAKE_CYCLE;
                            robot.driveRoadRunner.followTrajectoryAsync(cycleIntake);
                            cycleCount++;
                            //outakeTimer.reset();
                        }
                        //}
                    }
                    break;
                case INTAKE_CYCLE:
                    if (!robot.driveRoadRunner.isBusy() && cycleCount < totalCycle){
                        //if (robot.isThereFreight()) {
                        currentState = State.OUTAKE_CYCLE;
                        //if (cycleCount < totalCycle) {
                        robot.driveRoadRunner.followTrajectoryAsync(cycleOutake);
                        robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_MIDDLE_RIGHT;
                        //}
                        //}
                    }
                    break;
               /* case EXIT_INTAKE:
                    if (!robot.driveRoadRunner.isBusy()){
                        //if (!robot.isThereFreight() && robot.outakeState == SusPensionBot.OutakeStateList.OUTAKE_UPPER_CENTER){
                        currentState = State.OUTAKE_CYCLE;
                        robot.driveRoadRunner.followTrajectoryAsync(cycleOutake);
                        cycleCount++;
                        //}
                    }
                    break;*/
                case OUTAKE_CYCLE:
                    if (!robot.driveRoadRunner.isBusy()){
                        robot.eject();
                        if (firstTime){
                            outakeTimer.reset();
                            firstTime = false;
                        }
                        if( outakeTimer.milliseconds() >= robot.ejectTime ){
                            robot.ejectDone = true;
                           // robot.intakeMotor.setPower(0.0);
                            robot.stow();
                            //if (!robot.isThereFreight() && robot.outakeState == SusPensionBot.OutakeStateList.OUTAKE_UPPER_CENTER){
                            currentState = State.INTAKE_CYCLE;
                            robot.driveRoadRunner.followTrajectoryAsync(cycleIntake);
                            cycleCount++;
                            outakeTimer.reset();
                            robot.odometry.setPoseEstimate(robot.odometry.getPoseEstimate().plus(driftCorrection));
                        }
                        //}
                    }
                    break;
            }

            robot.driveRoadRunner.update();

            robot.updateArm(telemetry, outakeTimer);

            telemetry.addData("time", outakeTimer.milliseconds());
            telemetry.addData("vision", result);

            telemetry.update();

            robot.updateDuckSpinner(false, false);
        }
    }
}
