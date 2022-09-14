package org.firstinspires.ftc.teamcode.command.tests.mecanumBot2021;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.FTCDashboard.DashboardUtil;
import org.firstinspires.ftc.teamcode.utility.Math.AdvMath;
import org.firstinspires.ftc.teamcode.utility.Math.LinearInterpolator;
import org.firstinspires.ftc.teamcode.utility.ReplayRecorder.backend_classes.GamepadState;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.Vector2;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.Vector3;


/*
    Welcome to the template TeleOp class!
    To use it just make a copy of this class, and change all appearances of "20XX" with the starting year of the current season
    (for example, for the 2020-2021 season, the class would be named TeleOp2020 and all code within would have to reflect that change).
    It is recommended that somewhere near the top you have a comment block that describes your robot control scheme for easy reference.
    Also, don't forget to remove or comment out the "@Disabled" line, so that it will show up in the list of opmodes on the driver station.


    Happy coding!
 */


@Disabled
@TeleOp(name = "Scrappy TeleOp", group = "@@@")
@Config

public class TeleOpScrappyBot2021 extends LinearOpMode{
    // TeleOp Variables
    public String controllerConfig = "Controller Configurations:\n" +
            "Gamepad 1 -\n" +
            " - Left stick = Robot translation\n" +
            " - Right stick left/right = Robot rotation\n" +
            " - Left trigger = Slows robot movement the more it's pressed\n" +
            " - Right shoulder button = Toggle if the intake is running\n" +
            " - Left shoulder button (hold) = Reverse intake motor direction\n" +
            "\n" +
            "Gamepad 2 -\n" +
            " - Right bumper = Manual duck spin right\n" +
            " - Left bumper = Manual duck spin left\n" +
            " - Press down on right stick = toggle endpoint control\n" +
            " - DPad-Down = Bucket slide move to intake position\n" +
            " - DPad-Left = Bucket slide move to score lvl 1 position\n" +
            " - DPad-Right = Bucket slide move to score lvl 2 position\n" +
            " - DPad-Up = Bucket slide move to score lvl 3 position\n" +
            " - Y = Dump bucket then move slide to intake position\n" +
            " - Right Stick up/down = Move out/in the duck spinner linear slide\n" +
            " - X = Toggle if the capping arm is stowed\n" +
            " - Right trigger = Move the capping arm when un-stowed\n";

    // Robot Speed constants
    public static double LOWER_ROTATE_SPEED_MULT = 0.25; // Speed multipliers for turning (1 being 100% of power going in)
    public static double UPPER_ROTATE_SPEED_MULT = 0.68;

    public static double LOWER_TRANS_SPEED_MULT = 0.55; // Speed multipliers for translation (1 being 100% of power going in)
    public static double UPPER_TRANS_SPEED_MULT = 1.0;

    public static double CAPPER_UP_POSITION = 0.4;
    public static double CAPPER_DOWN_POSITION = 0.95;

    public static double DUCK_SLIDE_MANUAL_SPEED_MULT = 0.5;
    public static double DUCK_SLIDE_FEEDFORWARD_MULT = 0.9;
    public static double DUCK_SLIDE_ENDPOINT_SPEED_MULT = 0.1;

    private Pose2d vel;

    private double greatestVel = 0.0;

    public static double CAPPER_UP_TIME = 800;


    // Misc constants
    public static double TELEOP_DURATION_SECONDS = 9000000; // cut off controls after the duration is complete to prevent penalties
    public static double GAMEPAD_DEADZONE_RADIUS = 0.05;
    public static double TRIGGER_DEADZONE = 0.05;


    // Robot Classes
    private ScrappyBot2021 robot; // Main robot data class (ALWAYS CREATE AN INSTANCE OF THIS CLASS FIRST - HARDWARE MAP SETUP IS DONE WITHIN)
    private ElapsedTime runtime; // internal clock
    private FtcDashboard dashboard;
    private ElapsedTime capperUpTimer;

    // Value manipulation classes
    private LinearInterpolator transMultiplierInterp;
    private LinearInterpolator rotMultiplierInterp;
    private LinearInterpolator capUpTargetInterp;

    // flag variables
    private boolean canToggleIntake = true;
    private boolean canToggleEndpointControl = true;
    private boolean canToggleDumping = true;
    private boolean canToggleCapperStow = true;
    private boolean canToggleOdo = true;

    // tracking variables
    private double duckSlideTarget = 0;


    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        robot = new ScrappyBot2021(hardwareMap);
        runtime = new ElapsedTime();
        capperUpTimer = new ElapsedTime();
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(7);

        transMultiplierInterp = new LinearInterpolator(new Vector2(0, UPPER_TRANS_SPEED_MULT), new Vector2(1, LOWER_TRANS_SPEED_MULT));
        rotMultiplierInterp = new LinearInterpolator(new Vector2(0, UPPER_ROTATE_SPEED_MULT), new Vector2(1, LOWER_ROTATE_SPEED_MULT));
        capUpTargetInterp = new LinearInterpolator(new Vector2(0, ScrappyBot2021.capperDownPos), new Vector2(CAPPER_UP_TIME, ScrappyBot2021.capperUpPos));

        telemetry.addData(robot.getName() + "'s setup completed ", ")\n"); // Tell the user that robot setup has completed :)
        telemetry.addLine(controllerConfig);
        telemetry.update();


        waitForStart(); // Wait for the start button to be pressed before continuing further


        robot.progressCapperCycle();

        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate




        // The main run loop - write the main robot run code here
        while ( opModeIsActive() && runtime.seconds() <= TELEOP_DURATION_SECONDS ) {
            // take the gamepads and turn them into custom gamepad objects
            // (only here for future compatibility with ReplayRecorder, if not using ReplayRecorder just replace gp1 with gamepad1 in this code and remove this section)
            GamepadState gp1 = new GamepadState(gamepad1);
            GamepadState gp2 = new GamepadState(gamepad2);


            vel = robot.driveRoadRunner.getPoseVelocity();

            if (vel == null){
                telemetry.addData("velocity", "not available");
            }
            else {
                telemetry.addData("velocity", vel);
//                if (vel.getX() > greatestVel){
//                    vel = new Pose2d(greatestVel, vel.getY(), vel.getHeading());
//                    telemetry.addData("velocity", greatestVel);
//                }

            }



            // update odometry
            robot.odometry.update();


            // the speed modifiers
            double transMult = transMultiplierInterp.interpolate(gp1.left_trigger);
            double rotMult = rotMultiplierInterp.interpolate(gp1.left_trigger);

            // the robot movement commands
            double xTranslatePower = AdvMath.squareAndKeepSign( -gp1.left_stick_y ) * transMult;
            double yTranslatePower = AdvMath.squareAndKeepSign( -gp1.left_stick_x ) * transMult; // the y value is made negative, as up on the gamepad stick is negative and we want it to be positive
            double rotatePower = AdvMath.squareAndKeepSign( -gp1.right_stick_x  ) * rotMult;


            // intake movement control
            if( gp1.right_bumper && canToggleIntake ){ // if first frame of button being pressed
                robot.intake.setRunning( !robot.intake.isRunning() ); // toggle the running state
                canToggleIntake = false; // make it so it can't toggle again until the button is released
            }
            else if( !gp1.right_bumper )
                canToggleIntake = true;
            boolean intakeRunningBackwards = gp1.left_bumper;
            if( gp1.left_bumper ){
                robot.intake.setRunning(true);
            }


            // duck spinner control
            if( gp2.right_stick_button && canToggleEndpointControl){ // duck spinner periodic spinning toggles
                robot.toggleEndpointControl();

                canToggleEndpointControl = false;
            }

            else if( !gp2.right_stick_button ){
                canToggleEndpointControl = true;
            }
            // manual duck overrides
            if( gp2.right_bumper ){
                robot.duckSpinner.runForwards();
            }

            else if( gp2.left_bumper){
                robot.duckSpinner.runBackwards();
            }
            else
                robot.duckSpinner.stop();


            // dumping control
            if( gp2.y && canToggleDumping ){
                robot.toggleDumpRoutine();

                canToggleDumping = false;
            }
            else if( !gp2.y )
                canToggleDumping = true;

            robot.updateDumpRoutine();

            // capper control
            if( gp2.x && canToggleCapperStow ){
                robot.progressCapperCycle();
                canToggleCapperStow = false;

                if(robot.getCapperState() == 1)
                    capperUpTimer.reset();
            }
            else if( !gp2.x )
                canToggleCapperStow = true;

            if(robot.getCapperState() == 2){
                if( capperUpTimer.milliseconds() <= CAPPER_UP_TIME )
                    robot.capArmServo.setPosition( capUpTargetInterp.interpolate( capperUpTimer.milliseconds() ) );
                else
                    robot.capArmServo.setPosition( ScrappyBot2021.capperUpPos );
            }


            if( gp2.left_stick_button && canToggleOdo ){
                robot.toggleOdoRaised();
                canToggleOdo = false;
            }
            else if( !gp2.left_stick_button )
                canToggleOdo = true;



            // manual bucket slide control
            if(!robot.isDumpRoutineRunning()){
                if( gp2.dpad_down )
                    robot.bucketSlide.setTargetPosition( ScrappyBot2021.bucketSlideDownPos);
                else if( gp2.dpad_left )
                    robot.bucketSlide.setTargetPosition( ScrappyBot2021.bucketSlideLvl1Pos);
                else if( gp2.dpad_right )
                    robot.bucketSlide.setTargetPosition( ScrappyBot2021.bucketSlideLvl2Pos);
                else if( gp2.dpad_up )
                    robot.bucketSlide.setTargetPosition( ScrappyBot2021.bucketSlideLvl3Pos);
            }


            telemetry.addData("xtrans (before)", xTranslatePower);
            telemetry.addData("ytrans (before)", yTranslatePower);
            telemetry.addData("rot (before)", rotatePower);

            Vector3 movementCommands = new Vector3();
            double endpointHeading = -1;

            if(robot.isEndpointControlling()){
                duckSlideTarget = robot.getEndpointSlideTargetLen();

                movementCommands = robot.getEndpointDriveCommands();
                rotatePower += movementCommands.getHeading();


                endpointHeading = Math.toDegrees(robot.getEndpointTargetHeading( new Vector2d(robot.getPosition().getX(), robot.getPosition().getY()) ));


                double endpointXMovement = 0; // move the endpoint using human inputs if inputs are given
                if( Math.abs(gamepad2.right_stick_y) >= GAMEPAD_DEADZONE_RADIUS )
                    endpointXMovement = gamepad2.right_stick_y * DUCK_SLIDE_ENDPOINT_SPEED_MULT;
                double endpointYMovement = 0;
                if( Math.abs(gamepad2.right_stick_x) >= GAMEPAD_DEADZONE_RADIUS )
                    endpointYMovement = gamepad2.right_stick_x * DUCK_SLIDE_ENDPOINT_SPEED_MULT;


                robot.addToEndpointTargetPosition( new Vector2d(endpointXMovement, endpointYMovement) );

                robot.duckSlide.setBaseMotorPower( -xTranslatePower * DUCK_SLIDE_FEEDFORWARD_MULT ); // counter movement feedforward
            }
            else{ // allow for manual duck slide control
                duckSlideTarget += -gp2.right_stick_y * DUCK_SLIDE_MANUAL_SPEED_MULT;

                duckSlideTarget = AdvMath.rangeClip(duckSlideTarget, ScrappyBot2021.duckSlideExtremePositions);

                robot.duckSlide.setBaseMotorPower( 0 );
            }
            robot.duckSlide.setTargetPosition(duckSlideTarget); // no matter what, set duck slide target to the target position variable


            telemetry.addData("automated movement commands", movementCommands);
            telemetry.addData("xtrans (after)", xTranslatePower);
            telemetry.addData("ytrans (after)", yTranslatePower);
            telemetry.addData("rot (after)", rotatePower);
            telemetry.addData("endpoint heading target", endpointHeading);

            // passing powers into systems
            robot.drive.driveRobotRelative(xTranslatePower, yTranslatePower, rotatePower);


            if( intakeRunningBackwards ){
                robot.intake.setIntakeRunSpeed( ScrappyBot2021.intakeEjectSpeed );
            }
            else if(robot.bucketSlide.getTargetPos() < ScrappyBot2021.armToIntakePosThreshold && robot.bucketSlide.isAtTarget()){
                robot.intake.setIntakeRunSpeed( ScrappyBot2021.intakeRunSpeed );
            }
            else{
                robot.intake.setRunning(false);
            }


            if( !robot.isDumpRoutineRunning() ){
                if( robot.bucketSlide.getTargetPos() > ScrappyBot2021.armToIntakePosThreshold && robot.bucketSlide.getInchPosition() >= ScrappyBot2021.armToIntakePosThreshold )
                    robot.bucketArm.goToStateNamed("Prep");
                else
                    robot.bucketArm.goToStateNamed("Intake");
            }

            if(robot.bucketSlide.getErrorToTarget() >= 0)
                robot.bucketSlide.getVeloLimiter().setMaxSpeed( ScrappyBot2021.bucketSlideMaxSpeed ); // override the forced travel time max speed when going up


            robot.bucketSlide.runToInternalTarget(); // update slides
            robot.duckSlide.runToInternalTarget();


            Pose2d poseEstimate = robot.odometry.getPoseEstimate();
            Vector2d robotXY = new Vector2d(poseEstimate.getX(), poseEstimate.getY());
            Vector2d targetHeadingPoint = robot.getEndpointHeadingTargetPoint(robotXY);

            // robot telemetry
            telemetry.addLine("Robot position (GP1 sticks): " + robot.getPosition());
            telemetry.addLine("Translation boost factor (GP1 left trigger): " + transMult);
            telemetry.addLine("Rotation boost factor (GP1 right trigger): " + rotMult);
            telemetry.addLine("Intake running (GP1 right bumper): " + robot.intake.isRunning());

            telemetry.addLine("Duck spinner slide pos (GP2 right stick): " + robot.duckSlide.getInchPosition() + " in");
            telemetry.addLine("Bucket slide pos (GP2 D-PAD): " + robot.bucketSlide.getInchPosition() + " in");
            telemetry.addLine("Bucket slide target pos (GP2 D-PAD): " + robot.bucketSlide.getTargetPos() + " in");

            telemetry.addLine("Bucket dumping (GP2 Y button): " + robot.isDumpRoutineRunning());
            telemetry.addLine("Duck spinning automatically (GP2 bumpers): " + robot.duckSpinner.isPeroidicSpinning());
            if(robot.isEndpointControlling()){
                telemetry.addLine("Duck spinner endpoint position (GP2 right stick to move): " + robot.getEndpointTargetPosition());
                telemetry.addData("Target heading point", targetHeadingPoint);
            }
            telemetry.addLine("Duck servo power (GP2 triggers to override): " + robot.duckSpinner.getServo().getPosition());
            telemetry.addLine("Odo Raised? " + robot.isOdoRaised());
            telemetry.update();






            // FTC Dashboard telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("x", poseEstimate.getX());
            packet.put("y", poseEstimate.getY());
            packet.put("heading", Math.toDegrees(poseEstimate.getHeading()));

            packet.put("automated movement commands", movementCommands);
            packet.put("xtrans (after)", xTranslatePower);
            packet.put("ytrans (after)", yTranslatePower);
            packet.put("rot (after)", rotatePower);
            packet.put("endpoint target", robot.getEndpointTargetPosition());
            packet.put("endpoint heading target point", targetHeadingPoint);
            packet.put("endpoint heading target degrees", endpointHeading);


            Canvas fieldOverlay = packet.fieldOverlay();
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.setStroke("#ef3038"); // set the current draw color to red
            DashboardUtil.drawRectRobot(fieldOverlay, poseEstimate, 13, 17.5);

            if(robot.isEndpointControlling()){
                fieldOverlay.setStroke("#800080"); // set the current draw color to purple
                Vector2d duckEndpoint = robot.getEndpointTargetPosition();
                fieldOverlay.fillCircle( duckEndpoint.getX(), duckEndpoint.getY(), 1.5 );

                fieldOverlay.setStroke("#3F51B5");
                fieldOverlay.fillCircle( targetHeadingPoint.getX(), targetHeadingPoint.getY(), 1.5 );
            }


            dashboard.sendTelemetryPacket(packet);

        }

        // once loop done, stop everything
        robot.stopRobot();

        telemetry.addLine("TeleOp duration complete -> robot stopped");
        telemetry.update();
    }



    /* PUT ALL FUNCTIONS HERE */
    public void updateDashboard(){

    }
}
