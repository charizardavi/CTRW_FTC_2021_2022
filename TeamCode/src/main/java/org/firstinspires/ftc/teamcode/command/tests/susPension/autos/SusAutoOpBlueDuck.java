package org.firstinspires.ftc.teamcode.command.tests.susPension.autos;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.tests.susPension.failsafe.SusPensionBotFailsafe;
import org.firstinspires.ftc.teamcode.utility.OpenCV.FreightFrenzyPipeline;


@Autonomous(group = "drive")
@Config
public class SusAutoOpBlueDuck extends LinearOpMode {

    public int state;

    public static Pose2d startPosition = new Pose2d(-35, -62, Math.toRadians(-90));
    public static Pose2d outakePosFirst = new Pose2d(6, -49, Math.toRadians(0));
    //public static Pose2d driftCorrection = new Pose2d(0, -0.5, Math.toRadians(-3));


    public static int totalCycle = 4; //-1 because of if statement logic

    public static int forwardDist = 20;

    public static int testCount = 20;

    public static Pose2d intakePos = new Pose2d(52, -62.5, Math.toRadians(-40));

    public static Pose2d duckSpinPos = new Pose2d(-44, -58, Math.toRadians(180));

    public static Pose2d parkPos = new Pose2d(-60, -35, Math.toRadians(-90));

    enum State{
        FIRST_OUTAKE,
        DUCK_SPINNER,
        PARK,
        IDLE
    }

    State currentState = State.IDLE;

    FreightFrenzyPipeline.ElementPosition elementPosition = FreightFrenzyPipeline.ElementPosition.LEFT;

    public ElapsedTime autoDuckTimerRedSus;

    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addData("status", "initializing");

        telemetry.update();

        SusPensionBotFailsafe robot = new SusPensionBotFailsafe(hardwareMap);

        robot.driveRoadRunner.setPoseEstimate( startPosition );

        autoDuckTimerRedSus = new ElapsedTime();

        Trajectory firstOutake = robot.driveRoadRunner.trajectoryBuilder(startPosition, Math.toRadians(180))
                .lineToSplineHeading(outakePosFirst)
                .build();
        Trajectory duckSpinTraj = robot.driveRoadRunner.trajectoryBuilder(firstOutake.end(), 0)
                .lineToSplineHeading(duckSpinPos)
                .addDisplacementMarker( () -> {robot.startDuckSpinner(autoDuckTimerRedSus);})
                .build();
        Trajectory park = robot.driveRoadRunner.trajectoryBuilder(duckSpinTraj.end(), 0)
                .lineToSplineHeading(parkPos)
                .build();

        robot.vision.startWebcamStreaming();

        telemetry.addData("status", "ready");
        telemetry.update();


        waitForStart();

        autoDuckTimerRedSus.reset();

        String vote = robot.vision.getOutput();
        if(vote.equals("LEFT")){
            elementPosition = FreightFrenzyPipeline.ElementPosition.LEFT;
            robot.outakeStateInput = SusPensionBotFailsafe.outakeStateList.OUTAKE_LOWER;
        }
        else if(vote.equals("RIGHT")) {
            elementPosition = FreightFrenzyPipeline.ElementPosition.RIGHT;
            robot.outakeStateInput = SusPensionBotFailsafe.outakeStateList.OUTAKE_UPPER;
        }
        else{
            elementPosition = FreightFrenzyPipeline.ElementPosition.CENTER;
            robot.outakeStateInput = SusPensionBotFailsafe.outakeStateList.OUTAKE_MIDDLE;
        }

        robot.vision.stopWebcamStreaming();



        currentState = State.FIRST_OUTAKE;

        robot.driveRoadRunner.followTrajectoryAsync(firstOutake);





        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState){
                case FIRST_OUTAKE:
                    if (!robot.driveRoadRunner.isBusy()){
                        if (!robot.isThereFreight() && robot.outakeState == SusPensionBotFailsafe.outakeStateList.OUTAKE_UPPER){
                            currentState = State.DUCK_SPINNER;
                            robot.driveRoadRunner.followTrajectoryAsync(duckSpinTraj);
                        }
                    }
                    break;
                case DUCK_SPINNER:
                    if (!robot.driveRoadRunner.isBusy()){

                        if (robot.updateDuckSpinner(autoDuckTimerRedSus.milliseconds()) != 1.0){
                            telemetry.addData("duckspinner", "running");
                            telemetry.update();
                        }
                        else{
                            robot.intakeMotor.setPower(0.0);
                            currentState = State.PARK;
                            robot.driveRoadRunner.followTrajectoryAsync(park);
                        }

                    }
                    break;
                case PARK:
                    if (!robot.driveRoadRunner.isBusy()){
                        if (!robot.isThereFreight() && robot.outakeState == SusPensionBotFailsafe.outakeStateList.OUTAKE_UPPER){
                            robot.raiseOdo();
                            robot.driveBL.setPower(1);
                            robot.driveBL.setPower(1);
                            robot.driveBL.setPower(1);
                            robot.driveBL.setPower(1);
                            wait(3000);
                            robot.driveBL.setPower(0);
                            robot.driveBL.setPower(0);
                            robot.driveBL.setPower(0);
                            robot.driveBL.setPower(0);
                        }
                    }
                    break;
            }

            robot.driveRoadRunner.update();
            robot.updateArm(telemetry);

            telemetry.update();
        }
    }
}
