package org.firstinspires.ftc.teamcode.command.tests.mecanumBot2021;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.command.tests.mecanumBot2021.ScrappyBot2021;


@Disabled
@Autonomous(group = "drive")
@Config
public class ScrappyAutoOpRedCycle extends LinearOpMode {

    public int state;

    public int cycleCount = 0;

    public static Pose2d startPosition = new Pose2d(15, -62, Math.toRadians(-90));
    public static Pose2d outakePos = new Pose2d(-6, -40, Math.toRadians(-70));
    public static Pose2d intakeEntryPos = new Pose2d(25, -63.8, Math.toRadians(15));
    public static Pose2d intakeExitPos = new Pose2d(25, -67.1, Math.toRadians(0));
    public static Pose2d driftCorrection = new Pose2d(0, -0.5, Math.toRadians(-3));


    public static int totalCycle = 4; //-1 because of if statement logic

    public static int forwardDist = 20;

    public static Pose2d intakePos = new Pose2d(52, -62.5, Math.toRadians(-40));



    enum State{
        FIRST_OUTAKE,
        INTAKE_CYCLE,
        OUTAKE_CYCLE,
        IDLE
    }

    State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addData("status", "initializing");

        telemetry.update();

        ScrappyBot2021 robot = new ScrappyBot2021(hardwareMap);

        robot.driveRoadRunner.setPoseEstimate( startPosition );

        //robot.bucketSlide.setForcedTravelTime(44);

        Trajectory firstOutake = robot.driveRoadRunner.trajectoryBuilder(startPosition, Math.toRadians(125))
                .splineToSplineHeading(outakePos, outakePos.getHeading()+ Math.toRadians(180))
                .addTemporalMarker(0, () -> {
                    robot.startDumpRoutine();
                })
                .build();
        Trajectory cycleIntake = robot.driveRoadRunner.trajectoryBuilder(firstOutake.end())
                .addTemporalMarker(0, () -> {
                    robot.intake.setRunning(true);
                })
                .splineToSplineHeading(intakeEntryPos, 0)
                .splineToSplineHeading(intakePos, Math.toRadians(-10))
                .build();
        Trajectory cycleOutake = robot.driveRoadRunner.trajectoryBuilder(cycleIntake.end(), Math.toRadians(180))
                .addTemporalMarker(.3, ()->{
                    robot.intake.setRunning(false);
                })
                .splineToSplineHeading(intakeExitPos, Math.toRadians(180))
                .addDisplacementMarker( () -> {
                    robot.startDumpRoutine();
                })
                .splineToSplineHeading(outakePos, outakePos.getHeading()+ Math.toRadians(174))
                .addDisplacementMarker(() -> {
                    robot.odometry.setPoseEstimate(robot.odometry.getPoseEstimate().plus(driftCorrection));
                })
                .build();

        telemetry.addData("status", "ready");
        telemetry.update();


        waitForStart();

        robot.bucketArmServo.setPosition(ScrappyBot2021.bucketIntakeRot);

        currentState = State.FIRST_OUTAKE;

        robot.driveRoadRunner.followTrajectoryAsync(firstOutake);

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState){
                case FIRST_OUTAKE:
                    if (!robot.driveRoadRunner.isBusy()){
                        if ( robot.isDumpRoutineRunning() ){
                            currentState = State.INTAKE_CYCLE;
                            robot.driveRoadRunner.followTrajectoryAsync(cycleIntake);
                            cycleCount++;
                        }
                    }
                    break;
                case INTAKE_CYCLE:
                    if (!robot.driveRoadRunner.isBusy()){
                        currentState = State.OUTAKE_CYCLE;
                        if (cycleCount<totalCycle){
                            robot.driveRoadRunner.followTrajectoryAsync(cycleOutake);
                        }
                    }
                    break;
                case OUTAKE_CYCLE:
                    if (!robot.driveRoadRunner.isBusy()){
                        if ( robot.isDumpRoutineRunning()){
                            currentState = State.INTAKE_CYCLE;
                            robot.driveRoadRunner.followTrajectoryAsync(cycleIntake);
                            cycleCount++;
                        }
                    }
                    break;
            }

            robot.driveRoadRunner.update();
            robot.updateDumpRoutine();
            robot.bucketSlide.runToInternalTarget(telemetry);
        }
    }
}
