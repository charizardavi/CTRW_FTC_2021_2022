package org.firstinspires.ftc.teamcode.command.legacy;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner.Drive_Mecanum_RoadRunner;
import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.legacy.Drive_Mecanum_Tele_2020;
import org.firstinspires.ftc.teamcode.utility.Odometry.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.hardware.intakes.Intake_Ring_Drop;
import org.firstinspires.ftc.teamcode.hardware.shooters.J_Shooter_Ring_ServoFed;
import org.firstinspires.ftc.teamcode.hardware.arms.Arm_Wobble_Grabber;
import org.firstinspires.ftc.teamcode.utility.DynamicStateMachine.DriveFollowerTask;

import java.util.ArrayList;


/*
    Welcome to the 2020-2021 TeleOp class!

    Robot control scheme:
        Main Drive:
        - Controller 1 Left Stick = translation
        - Controller 1 Right Stick (x axis only) = rotation
        - Controller 1 Right Bumper = boost button
        - Controller 1 D-Pad Up = toggle drive relative to field
        - Controller 1 D-Pad Left/Right = preform the powershot shooting subroutine

        Ring Shooter:
        - Controller 2 X button = start a firing sequence (spins up if not spun up, then shoots a ring. can be held to fire rapidly)
        - Controller 2 Right Bumper = toggle if the shooter motor is spun up
        - Constroller 2 Y Button = toggle powershot/highgoal mode

        Ring Intake:
        - Controller 2 Left Bumper = toggle if the ring intake is active
        - Controller 2 Left Trigger = toggle ring blocking arm

        Wobble Intake/Arm:
        - Controller 2 Right Stick (y axis) = up moves the intake wheels to outtake the wobble goal, down moves the intake wheels to intake the wobble goal
        - Controller 2 D-Pad Up = Move the wobble arm to the lifted position (for going over the wall)
        - Controller 2 D-Pad Down = Move the wobble arm to the grab position (for grabbing the wobble goal)
        - Controller 2 D-Pad Right = Move the wobble arm to the folded position (NOT recommended while holding the wobble goal)
 */

@Disabled

@TeleOp(name = "TeleOp2020 Robot Relative", group = "@@@")

//@Config
public class TeleOp2020_RobotRelative extends LinearOpMode{
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    String robotName = "Lil' ring flinga";

    // Robot Speed variables
    public static double turnSpeed = 0.50; // Speed multiplier for turning (1 being 100% of power going in) when not boosting
    public static double translateSpeed = 0.40; // Speed multiplier for translation (1 being 100% of power going in)
    public static double boostSpeed = 1.00; // Speed multiplier for BOOSTING (1 being 100% of power going in)
    public static double stopSpeed = 0.00; // the motor speed for stopping the robot

    // Constants
    static final double DEAD_ZONE_RADIUS = 0.005; // the minimum value that can be passed into the drive function
    static final int TELEMETRY_TRANSMISSION_INTERVAL = 25;
    static final int ENDGAME_START_TIME = 120000;

    // Robot Classes
    private Provider2020 robot; // Main robot data class (ALWAYS CREATE AN INSTANCE OF THIS CLASS FIRST - HARDWARE MAP SETUP IS DONE WITHIN)
    private Drive_Mecanum_Tele_2020 mecanum_drive; // the main mecanum drive class
    private StandardTrackingWheelLocalizer localizer; // the odometry based localizer - uses dead wheels to determine (x, y, r) position on the field
    private Intake_Ring_Drop intake; // the intake class instance
    private J_Shooter_Ring_ServoFed shooter; // the shooter class instance
    private Arm_Wobble_Grabber wobble; // the wobble intake/arm class instance
    private Drive_Mecanum_RoadRunner auto_drive;

    private FtcDashboard dashboard;
    private ElapsedTime runtime;

    // Flags
    private boolean firstToggleDriveRelative = true; // used to ensure proper toggling behavior (see usage under logic section)
    private boolean firstSpinUpToggle = true; // used to ensure proper toggling behavior (see usage under logic section)
    private boolean firstIntakeRunToggle = true; // used to ensure proper toggling behavior (see usage under logic section)
    private boolean firstAngleToggle = true;

    private boolean driveFieldRelative = false; // default to driving robot relative
    private boolean isSpinningUp = false;
    private boolean shooterAngledUp = true;
    private int wobbleArmPosition = 0; // 0 = folded pos, 1 = up pos, 2 = grab position
    private int wobbleIntakeDirection = 0; // 0 = stopped, 1 = intaking, -1 = outtaking
    private boolean intakeIsRunning = false; // holds if the intake should be running or not
    private boolean powershotDriving = false;
    private int lastPowershotIndex = 0;
    private boolean firstPowershotDrive = true;
    private boolean firstPowershotDriveToggle = true;
    private boolean firstGateMoveToggle = true;


    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        robot = new Provider2020(hardwareMap);
        runtime = new ElapsedTime();
        mecanum_drive = new Drive_Mecanum_Tele_2020(robot.driveFL, robot.driveFR, robot.driveBL, robot.driveBR, turnSpeed, translateSpeed, boostSpeed); // pass in the drive motors and the speed variables to setup properly
        localizer = new StandardTrackingWheelLocalizer(hardwareMap);
        intake = new Intake_Ring_Drop(robot.intakeMotor, robot.ringGateServo);
        shooter = new J_Shooter_Ring_ServoFed(robot.JShootFront, robot.JShootBack, robot.shooterFeederServo, robot.shooterIndexerServo, robot.shooterAnglerServo);
        wobble = new Arm_Wobble_Grabber(robot.wobbleArmMotor, robot.wobbleLeftWheelServo, robot.wobbleRightWheelServo, 1.0/5.0);

        auto_drive = new Drive_Mecanum_RoadRunner(hardwareMap); // setup the second automated drive class

        dashboard = FtcDashboard.getInstance(); // setup the dashboard
        dashboard.setTelemetryTransmissionInterval(TELEMETRY_TRANSMISSION_INTERVAL); // interval in milliseconds


        robot.setEncoderActive(false); // start the game without running encoders on drive encoders

        ArrayList<DriveFollowerTask> autoPowershotTasks = getAutoPowershotTasks(); // calculate the trajectories for the powershot driving


        telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();


        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate
        intake.raiseGate();


        // The main run loop - write the main robot run code here
        while (opModeIsActive()) {
            if(localizer != null){ // if the localizer exists
                localizer.update(); // update our current position
            }

            // Variables
            boolean isBoosting = !gamepad1.right_bumper;  // If true, the robot will go at the boost speed, otherwise it will go at the base speed (just impacts translation)
            double xTranslatePower = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y); // specifically the y stick is negated because up is negative on the stick, but we want up to move the robot forward
            double yTranslatePower = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x); // set the robot translation/rotation speed variables based off of controller input (set later in hardware manipluation section)
            double rotatePower = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);
            boolean instructFire = gamepad2.x; // if pressing the second gamepad x, instruct a fire event


            // Logic (figuring out what the robot should do)
            if(gamepad1.dpad_up && firstToggleDriveRelative){ // toggle driving relative to field if dpad up is pressed
                driveFieldRelative = !driveFieldRelative; // toggle the value

                if(driveFieldRelative){ // if toggling back to driving field relative
                    robot.reset_imu(); // reset the robot's imu
                }
                firstToggleDriveRelative = false; // set the variable false so that it cannot toggle again
            }
            else if (!gamepad1.dpad_up){ // wait to set the flag back to true until the button is released
                firstToggleDriveRelative = true; // until the button is released
            }

            if( gamepad2.right_bumper && firstSpinUpToggle ){ // code to toggle if the shooter is spinning up
                isSpinningUp = !isSpinningUp;

                firstSpinUpToggle = false;
            }
            else if (!gamepad2.right_bumper){
                firstSpinUpToggle = true;
            }
            if( gamepad2.y && firstAngleToggle ){ // code to toggle if the shooter is spinning up
                shooterAngledUp = !shooterAngledUp;

                firstAngleToggle = false;
            }
            else if (!gamepad2.y){
                firstAngleToggle = true;
            }

            if( gamepad2.left_bumper == true && firstIntakeRunToggle ){ // code to toggle if the intake is running
                intakeIsRunning = !intakeIsRunning;

                firstIntakeRunToggle = false;
            }
            else if (!gamepad2.left_bumper){
                firstIntakeRunToggle = true;
            }

            if(gamepad2.dpad_up){ // if pressing up
                wobbleArmPosition = 1; // tell it to go to up position
            }
            else if(gamepad2.dpad_down){ // if pressing down
                wobbleArmPosition = 2; // tell it to go to down position
            }
            else if(gamepad2.dpad_right){ // if pressing right
                wobbleArmPosition = 0; // tell it to go to the idle position
            }

            if(gamepad2.right_stick_y > DEAD_ZONE_RADIUS){ // if pulling down on the stick enough, intake
                wobbleIntakeDirection = -1;
            }
            else if(gamepad2.right_stick_y < -DEAD_ZONE_RADIUS){ // if pushing up on the stick enough, outtake
                wobbleIntakeDirection = 1;
            }
            else { // default state is 0
                wobbleIntakeDirection = 0;
            }

            if(gamepad2.left_trigger >= 0.5 && firstGateMoveToggle){
                String gatePos = intake.getGatePosition();// get what position the gate thinks it is in

                if(gatePos.equals("UP") || gatePos.equals("PREP")){ // if in one of the two up positions, we wanna toggle down
                    intake.lowerGate();
                    telemetry.addLine("Gate down");
                }
                else if(runtime.milliseconds() < ENDGAME_START_TIME){ // if not up, we are down. If we are down and not in endgame, we don't wanna go all the way up
                    intake.prepGate();
                    telemetry.addLine("Gate prep");
                }
                else {
                    intake.raiseGate();
                    telemetry.addLine("Gate up");
                }

                firstGateMoveToggle = false;
            }
            else if(gamepad2.left_trigger < 0.5){
                firstGateMoveToggle = true;
            }

            if(wobbleIntakeDirection != 0 || shooter.isFiring()){
                isBoosting = false; // if intaking/outtaking with the wobble or the shooter is spun up, slow down the robot to allow for finer control
            }


            //setup a dead zone for the controllers
            if(Math.abs(xTranslatePower) <= DEAD_ZONE_RADIUS){ // if the value is less than the maximum deadzone value, set to zero (to stop the motor)
                xTranslatePower = stopSpeed;
            }
            if(Math.abs(yTranslatePower) <= DEAD_ZONE_RADIUS){ // if the value is less than the maximum deadzone value, set to zero (to stop the motor)
                yTranslatePower = stopSpeed;
            }
            if(Math.abs(rotatePower) <= DEAD_ZONE_RADIUS){ // if the value is less than the maximum deadzone value, set to zero (to stop the motor)
                rotatePower = stopSpeed;
            }


            if( (gamepad1.dpad_right || gamepad1.dpad_left) && firstPowershotDriveToggle){ // code to toggle if the shooter is spinning up
                powershotDriving = !powershotDriving;

                if (powershotDriving) {
                    auto_drive.setPoseEstimate(new Pose2d(0, 0, 0));
                    auto_drive.setTasks(autoPowershotTasks);
                }

                firstPowershotDriveToggle = false;
            }
            else if (!gamepad1.dpad_right && !gamepad1.dpad_left){
                firstPowershotDriveToggle = true;
            }

            if(powershotDriving){
                shooterAngledUp = false; // optimize for powershots by setting shooter to angle down
                shooter.optimizeForPowershots();
                shooter.setTargetShooterSpeed(shooter.getTargetShooterShootingSpeed());
                shooter.spinUp();


                if(auto_drive.getTaskIndex() == 1 || auto_drive.getTaskIndex() == 3 || auto_drive.getTaskIndex() == 5){ // once the feeder goes to the retracting stage, a ring has been shot and we can start turning (redundant for the next 2 rings as the flag will stay flipped
                    shooter.instructFire(); // tell the shooter to start shooting

                    //if (shooter.getFiringState() > 1) {
                    //    lastPowershotIndex = auto_drive.getTaskIndex();
                    //}
                }


                shooter.updateFeeder();
                powershotDriving = !auto_drive.doTasksAsync();
            }
            else {
                if(gamepad1.left_bumper){ // if we want the robot to rotate to 0
                    rotatePower = mecanum_drive.calcTurnPIDPower(Math.toRadians(0), Math.toRadians(robot.getHeading())); // override the rotation with a PID output
                }

                powershotDriving = false; // if neither are pressed, reset the turning variable
                lastPowershotIndex = 0;
            }



            // Hardware instruction (telling the hardware what to do)
            if( !powershotDriving ) {
                if(driveFieldRelative) {
                    mecanum_drive.drive_field_relative(xTranslatePower, yTranslatePower, rotatePower, robot.getHeading(), isBoosting); // call the drive field relative method
                }
                else {
                    mecanum_drive.drive_robot_relative(xTranslatePower, yTranslatePower, rotatePower, isBoosting); // call the drive robot relative method
                }

                if (instructFire) {
                    shooter.instructFire(); // tell the shooter it should fire (only ever queues a single fire)
                } else {
                    shooter.resetShotCount();
                }

                if (shooter.isFiring()) { // if the shooter is firing, make sure the be updating the feeder
                    shooter.setTargetShooterSpeed(shooter.getTargetShooterShootingSpeed());
                    shooter.spinUp();
                    shooter.updateFeeder(); // update the shooter feeder position based off of where it is in the cycle
                    intake.spinDown();

                    if(shooter.getFiringState() > 0){ // lower the gate once the shooter has actually started moving rings out of the shooter, meaning we won't get penalized for controlling more than 3 rings because it will have shot at least 1 by the time it gets all the way down
                        intake.lowerGate();
                    }
                }
                else if (gamepad2.right_trigger > 0.5) { // then next in the priority list, if the shooter isn't firing check if the intake should be ejecting
                    shooter.indexerDown(); // move the indexer to the intaking position
                    shooter.setFlywheelMode(isSpinningUp); // set the shooter mode based on the toggle

                    intake.setIntakeRunSpeed(-intake.DEFAULT_INTAKE_RUN_SPEED);
                    intake.spinUp(); // and run the intake
                }
                else if (intakeIsRunning) { // if the intake is set to be running by the user and the shooter isn't firing
                    shooter.indexerDown(); // move the indexer to the intaking position

                    if (isSpinningUp) { // if the shooter should be spinning up
                        shooter.setTargetShooterSpeed(shooter.getTargetShooterShootingSpeed()); // set the shooter to its full speed
                    } else { // if not supposed to be spinnig up shooter, spin it up to a low speed to help with intaking
                        shooter.setTargetShooterSpeed(0.2);
                    }
                    shooter.spinUp();


                    intake.setIntakeRunSpeed(intake.DEFAULT_INTAKE_RUN_SPEED);
                    intake.spinUp(); // and run the intake
                }
                else { // otherwise set the shooter to the proper mode
                    shooter.setFlywheelMode(isSpinningUp); // make sure the shooting mode it set properly
                    shooter.indexerUp();

                    intake.spinDown(); // and ensure that the intake is spun down
                }

                if (gamepad2.b) {
                    shooter.optimizeForLonggoal();
                } else if (shooterAngledUp) {
                    shooter.optimizeForHighgoal();
                } else {
                    shooter.optimizeForPowershots();
                }
            }

            wobble.setIntakeDirection(wobbleIntakeDirection); // make sure it is intaking properly
            //wobbleClamp.setIntakeDirection(wobbleIntakeDirection);
            if(wobbleArmPosition == 1) { // set the wobble arm position
                wobble.goToUpPos();
                //wobbleClamp.goToUpPos();
            }
            else if(wobbleArmPosition == 2) {
                wobble.goToGrabPos();
                //wobbleClamp.goToGrabPos();
            }
            else{
                wobble.goToIdlePos();
                //wobbleClamp.goToIdlePos();
            }



            // Telemetry
            if(powershotDriving){
                telemetry.addLine("Automatically Powershot Driving");
            }
            else if(driveFieldRelative){ // add telemetry relating to robot drive mode
                telemetry.addLine("Driving field relative");
            }
            else{
                telemetry.addLine("Driving robot relative");
            }
            if(shooterAngledUp){
                telemetry.addLine("Shooter Optimized for: Highgoal");
            }
            else {
                telemetry.addLine("Shooter Optimized for: Powershots");
            }

            if(robot.driveUsingEncoders){
                telemetry.addLine("Driving using encoders");

                telemetry.addData("Drive FL Encoder: ", robot.driveFL.getCurrentPosition()); // add telemetry data for motor encoders
                telemetry.addData("Drive FR Encoder: ", robot.driveFR.getCurrentPosition());
                telemetry.addData("Drive BL Encoder: ", robot.driveBL.getCurrentPosition());
                telemetry.addData("Drive BR Encoder: ", robot.driveBR.getCurrentPosition());
            }
            else{
                telemetry.addLine("Driving without drive encoders");
            }

            

            telemetry.addData("Robot is Boosting?", isBoosting);

            if(localizer != null){ // if we have a localizer that exists, get the position estimate from it
                telemetry.addData("Field Position", localizer.getPoseEstimate());
                telemetry.addData("Wheel Positions", localizer.getWheelPositions());
            }

            telemetry.addData("Shooter is spun up?", shooter.isSpunUp());
            telemetry.addData("Shot Count: ", shooter.getShotCount());
            telemetry.addData("Firing state", shooter.getFiringState());
            telemetry.addData("Corrected Flywheel Velocity: ", shooter.encoderVeloToMotorSpeed(shooter.getFlywheelVelo()));
            telemetry.addData("Integral: ", shooter.getIntegral());
            telemetry.addData("Target Flywheel Velocity: ", shooter.getTargetShootingSpeed());
            telemetry.addData("Arm target position", wobble.getArmTargetPosition());
            telemetry.addData("Wheel arm position", wobble.getArmPosition());

            // telemetry.addData("Wheel arm encoder position", robot.wobbleArmMotor.getCurrentPosition());
            //telemetry.addData("Claw arm position", wobbleClamp.getArmPosition());
            //telemetry.addData("Claw arm encoder position", robot.wobbleArmMotor2.getCurrentPosition())


            telemetry.update();


            updateDashboard();
        }  // end of running while loop
    }


    /* PUT ALL FUNCTIONS HERE */
    public static double FIRST_POWERSHOT_BACK_DISTANCE = -21.0;
    public static double FIRST_POWERSHOT_RIGHT_DISTANCE = 14.5;
    public static double SECOND_POWERSHOT_RIGHT_DISTANCE = 9.5;
    public static double THIRD_POWERSHOT_RIGHT_DISTANCE = 6.7;
    public static double FORWARD_COMPENSATION_DISTANCE = 1.0; // how many inches forward the robot moves to compensate for a slight drift when strafing (for unknown reasons)
    private ArrayList<DriveFollowerTask> getAutoPowershotTasks(){
        ArrayList<DriveFollowerTask> driveTasks = new ArrayList<DriveFollowerTask>();


        driveTasks.add( new DriveFollowerTask( auto_drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineTo(new Vector2d(FIRST_POWERSHOT_BACK_DISTANCE, -FIRST_POWERSHOT_RIGHT_DISTANCE))
                .build()
        ));
        driveTasks.add( new DriveFollowerTask( (int)J_Shooter_Ring_ServoFed.FEEDER_EXTENSION_TIME) );

        driveTasks.add( new DriveFollowerTask( auto_drive.trajectoryBuilder(driveTasks.get(0).getTraj().end())
                .lineTo(new Vector2d(driveTasks.get(0).getTraj().end().getX() + FORWARD_COMPENSATION_DISTANCE,  driveTasks.get(0).getTraj().end().getY() - SECOND_POWERSHOT_RIGHT_DISTANCE))
                .build()
        ));
        driveTasks.add( new DriveFollowerTask( (int)J_Shooter_Ring_ServoFed.FEEDER_EXTENSION_TIME) );

        driveTasks.add( new DriveFollowerTask( auto_drive.trajectoryBuilder(driveTasks.get(2).getTraj().end())
                .lineTo(new Vector2d(driveTasks.get(2).getTraj().end().getX() + FORWARD_COMPENSATION_DISTANCE,  driveTasks.get(2).getTraj().end().getY() - THIRD_POWERSHOT_RIGHT_DISTANCE))
                .build()
        ));
        driveTasks.add( new DriveFollowerTask( (int)J_Shooter_Ring_ServoFed.FEEDER_EXTENSION_TIME) );

        return driveTasks;
    }

    public void updateDashboard(){
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("raw_shooter_velo", shooter.getFlywheelVelo()); // get the shooter velocity and add that
        packet.put("shooter_velo", J_Shooter_Ring_ServoFed.encoderVeloToMotorSpeed(shooter.getFlywheelVelo())); // get the shooter velocity and convert it to motor speed for readability


        if(dashboard != null){
            dashboard.sendTelemetryPacket(packet);
        }
    }

}
