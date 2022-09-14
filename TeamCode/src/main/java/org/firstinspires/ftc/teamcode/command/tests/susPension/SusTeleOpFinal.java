package org.firstinspires.ftc.teamcode.command.tests.susPension;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.Math.AdvMath;
import org.firstinspires.ftc.teamcode.utility.OpenCV.FreightFrenzyPipeline;

import java.security.spec.EllipticCurve;


@TeleOp(name = "SusTeleOp-Final", group = "@@@")
@Config
public class SusTeleOpFinal extends LinearOpMode{

    String robotName = "Mater";

    double turnSpeed = 0.5;
    double translateSpeed = 1.0;

    private SusPensionBot robot;

    private ElapsedTime runtime;
    private ElapsedTime duckTimer;
    private ElapsedTime outakeTimer;

    private boolean startDuckSpin = false;
    private boolean canCycleCapping = true;


    public ElapsedTime capTimer;

    public static double triggerThreshold = .4;

    FreightFrenzyPipeline.ElementPosition elementPosition = FreightFrenzyPipeline.ElementPosition.LEFT;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("status", "initializing");
        telemetry.update();
        robot = new SusPensionBot(hardwareMap);
        runtime = new ElapsedTime();
        duckTimer = new ElapsedTime();
        outakeTimer = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.vision.startWebcamStreaming();



        robot.setSide("blue");

        telemetry.addData("status", "ready to go!");
        telemetry.update();


        waitForStart();

        robot.unlockIntake();
        robot.raiseOdo();
        robot.stow();
        robot.capServo.setPosition(0.0);

        runtime.reset();
        duckTimer.reset();
        outakeTimer.reset();


        String vote = robot.vision.getOutput();
//        if(vote.equals("LEFT")){
//            elementPosition = FreightFrenzyPipeline.ElementPosition.LEFT;
//            robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_LOWER;
//        }
//        else if(vote.equals("RIGHT")) {
//            elementPosition = FreightFrenzyPipeline.ElementPosition.RIGHT;
//            robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_UPPER_CENTER;
//        }
//        else{
//            elementPosition = FreightFrenzyPipeline.ElementPosition.CENTER;
//            robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_MIDDLE_CENTER;
//        }
        telemetry.addData("result", elementPosition);

        while (opModeIsActive()) {
            robot.drive.driveRobotRelative(AdvMath.squareAndKeepSign( -gamepad1.left_stick_y )*translateSpeed, AdvMath.squareAndKeepSign( -gamepad1.left_stick_x )*translateSpeed, AdvMath.squareAndKeepSign( -gamepad1.right_stick_x)*turnSpeed);

            if(gamepad2.dpad_right && gamepad2.y){
                robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_UPPER_RIGHT;
            }
            else if(gamepad2.dpad_left && gamepad2.y){
                robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_UPPER_LEFT;
            }
            else if(gamepad2.dpad_up && gamepad2.y){
                robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_UPPER_CENTER;
            }
            else if(gamepad2.dpad_up && gamepad2.x){
                robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_MIDDLE_CENTER;
            }
            else if(gamepad2.dpad_right && gamepad2.x){
                robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_MIDDLE_RIGHT;
            }
            else if(gamepad2.dpad_left && gamepad2.x){
                robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_MIDDLE_LEFT;
            }
            else if(gamepad2.dpad_right && gamepad2.a){
                robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_NEUTRAL_RIGHT;
            }
            else if(gamepad2.dpad_left && gamepad2.a){
                robot.outakeStateInput = SusPensionBot.OutakeStateList.OUTAKE_NEUTRAL_LEFT;
            }
            else if (gamepad2.dpad_down){
                robot.ejectDone = true;
                robot.intakeMotor.setPower(0.0);
                robot.stow();
            }

            if (gamepad2.right_bumper){
                robot.intake();
            }
            else if(gamepad2.left_bumper){
                robot.eject();
            }

            if (gamepad2.right_trigger>.5){
                startDuckSpin = true;
            }

            if (gamepad2.right_stick_button && canCycleCapping){
                robot.capCycle();
                canCycleCapping = false;
            }
            else if(!gamepad2.right_stick_button){
                canCycleCapping = true;
            }

            if( gamepad2.left_stick_button ){
                robot.ejectDone = false;
                robot.outakeState = SusPensionBot.OutakeStateList.STOW;
            }


            robot.updateDuckSpinner(gamepad2.right_trigger > triggerThreshold, gamepad2.left_trigger > triggerThreshold);

            robot.raiseOdo();

            robot.updateCap();

            robot.updateArm(telemetry, outakeTimer);

            telemetry.addData("red", robot.intakeDetect.red());
            telemetry.addData("green", robot.intakeDetect.green());
            telemetry.addData("blue", robot.intakeDetect.blue());
            telemetry.addData("arm-state", robot.outakeState);
            telemetry.addData("outake-state-input", robot.outakeStateInput);
            telemetry.addData("right_trigger", gamepad2.right_trigger);
            telemetry.addData("left_trigger", gamepad2.left_trigger);

            telemetry.update();
        }

    }
}
