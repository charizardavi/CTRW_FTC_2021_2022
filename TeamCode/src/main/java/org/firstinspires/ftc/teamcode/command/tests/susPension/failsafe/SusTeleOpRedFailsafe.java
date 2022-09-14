package org.firstinspires.ftc.teamcode.command.tests.susPension.failsafe;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.command.tests.susPension.SusPensionBot;
import org.firstinspires.ftc.teamcode.utility.Math.AdvMath;


@Disabled
@TeleOp(name = "SusTeleOp-Red-Failsafe", group = "@@F")
@Config
public class SusTeleOpRedFailsafe extends LinearOpMode{

    // Robot Name
    String robotName = "Mater";

    // Robot Speed variables
    double turnSpeed = 0.5;
    double translateSpeed = 1.0;


    // Robot Classes
    private SusPensionBotFailsafe robot;

    private ElapsedTime runtime;
    private ElapsedTime duckTimer;

    private boolean isOutaking = false;

    private boolean startDuckSpin = false;
    private boolean duckSpin = false;

    private boolean canToggleFreight = true;

    @Override
    public void runOpMode() {
        telemetry.addData("status", "initializing");
        telemetry.update();
        robot = new SusPensionBotFailsafe(hardwareMap);
        runtime = new ElapsedTime();
        duckTimer = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());




        robot.setSide("red");

        telemetry.addLine(robotName + "\'s setup complete!\nPress play to begin TeleOp");
        telemetry.update();


        waitForStart();

        robot.unlockIntake();
        robot.raiseOdo();
        robot.stow();

        runtime.reset();
        duckTimer.reset();




        while (opModeIsActive()) {
            robot.drive.driveRobotRelative(AdvMath.squareAndKeepSign( -gamepad1.left_stick_y )*translateSpeed, AdvMath.squareAndKeepSign( -gamepad1.left_stick_x )*translateSpeed, AdvMath.squareAndKeepSign( -gamepad1.right_stick_x)*turnSpeed);
//            telemetry.addData("distance", robot.intakeDetect.getDistance(DistanceUnit.CM));
//            telemetry.update();

            if( gamepad2.left_bumper && canToggleFreight ){
                robot.toggleIsFreight();
                canToggleFreight = false;
            }
            if( !gamepad2.left_bumper )
                canToggleFreight = true;


            if(gamepad2.dpad_up){
                robot.outakeStateInput = SusPensionBotFailsafe.outakeStateList.OUTAKE_UPPER;
            }
            else if(gamepad2.dpad_down){
                robot.outakeStateInput = SusPensionBotFailsafe.outakeStateList.OUTAKE_LOWER;
            }
            else if(gamepad2.dpad_right){
                robot.outakeStateInput = SusPensionBotFailsafe.outakeStateList.OUTAKE_NEUTRAL_RIGHT;
            }
            else if(gamepad2.dpad_left){
                robot.outakeStateInput = SusPensionBotFailsafe.outakeStateList.OUTAKE_NEUTRAL_LEFT;
            }

            if (gamepad2.x){
                robot.intake();
            }
            else if(gamepad2.y){
                robot.eject();
            }

            if (gamepad2.right_bumper){
                startDuckSpin = true;
            }

            if (startDuckSpin){
                startDuckSpin = false;
                duckTimer.reset();
                duckSpin = true;
            }
            if (duckSpin){
                if (robot.updateDuckSpinner(duckTimer.milliseconds()) == 1.0){
                    duckSpin = false;
                    robot.duckSpinMotor.setPower(0.0);
                }
            }

//            if (gamepad2.right_bumper){
//                robot.outakeIsGo = true;
//            }
            robot.raiseOdo();

            robot.updateArm(telemetry);


            telemetry.addData("Big Rot", robot.getBigRot());
            telemetry.addData("Small Rot", robot.getSmallRot());
            telemetry.addData("armR", robot.armR.getCurrentPosition());
            telemetry.addData("armL", robot.armL.getCurrentPosition());
            telemetry.addData("distance", robot.intakeDetect.getDistance(DistanceUnit.CM));

            telemetry.update();
        }

    }
}
