package org.firstinspires.ftc.teamcode.command.tests.z_system_specific_tests.ReplayRecorder;


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

import org.firstinspires.ftc.teamcode.command.legacy.Provider2020;
import org.firstinspires.ftc.teamcode.utility.Odometry.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.ReplayRecorder.Drive_Mecanum_RepRec;
import org.firstinspires.ftc.teamcode.utility.ReplayRecorder.backend_classes.GamepadState;
import org.firstinspires.ftc.teamcode.utility.ReplayRecorder.backend_classes.ReplayManager;
import org.firstinspires.ftc.teamcode.utility.ReplayRecorder.backend_classes.RobotState;
import org.firstinspires.ftc.teamcode.utility.FTCDashboard.DashboardUtil;


/*
    Welcome to the 2020-2021 TeleOp class!

    Robot control scheme:
        Main Drive:
        - Controller 1 Left Stick = x and y translation
        - Controller 1 Right Stick (x axis only) = rotation
        - Controller 1 D-Pad Up = toggle drive relative to field


        Recording/Replaying:
        - Controllers 1&2 Y = Toggle recording
        - Controllers 1&2 B = Toggle replaying
 */

@Disabled
@TeleOp( group = "@@R")

@Config
public class ReplayRecorderOpMode extends LinearOpMode{
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    public static String robotName = "Lil' ring flinga";
    public static String REPLAY_FILE_NAME = "TestPath.bin";

    // Constants
    static final double DEAD_ZONE_RADIUS = 0.005; // the minimum value that can be passed into the drive function
    static final int TELEMETRY_TRANSMISSION_INTERVAL = 7;
    public static int RECORD_INTERVAL = 100; // how many milliseconds between recording waypoints, lower number = more waypoints but more computer resource use from waypoints
    public static Pose2d startPose = new Pose2d(0, 0, 0);

    // Robot Classes
    private Provider2020 robot; // Main robot data class (ALWAYS CREATE AN INSTANCE OF THIS CLASS FIRST - HARDWARE MAP SETUP IS DONE WITHIN)
    private Drive_Mecanum_RepRec mecanumDrive; // the main mecanum drive class
    private StandardTrackingWheelLocalizer localizer; // the odometry based localizer - uses dead wheels to determine (x, y, r) position on the field

    private ReplayManager replayManager;
    private FtcDashboard dashboard;
    private ElapsedTime runtime;
    private ElapsedTime timeSinceLastRecord;

    // Flags
    private boolean firstReplayToggle = true;
    private boolean firstRecordToggle = true;
    private boolean recordingPrimed = false;
    private boolean firstRelativeToFieldToggle = true;
    private boolean drivingFieldRelative = false;
    private boolean firstSpeedLimitingToggle = true;
    private boolean speedLimiting = true;

    // Tracking variables
    private RobotState currentTargetState = new RobotState();
    private GamepadState gp1; // gamepad state variables for replay-recorder
    private GamepadState gp2;


    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        robot = new Provider2020(hardwareMap);
        mecanumDrive = new Drive_Mecanum_RepRec(robot.driveFL, robot.driveFR, robot.driveBL, robot.driveBR); // pass in the drive motors and the speed variables to setup properly

        replayManager = new ReplayManager();
        localizer = new StandardTrackingWheelLocalizer(hardwareMap);
        runtime = new ElapsedTime();
        timeSinceLastRecord = new ElapsedTime();



        dashboard = FtcDashboard.getInstance(); // setup the dashboard
        dashboard.setTelemetryTransmissionInterval(TELEMETRY_TRANSMISSION_INTERVAL); // interval in milliseconds

        robot.setEncoderActive(false); // start the game without running encoders on drive encoders
        localizer.setPoseEstimate(startPose);


        telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();


        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate


        // The main run loop - write the main robot run code here
        while (opModeIsActive()) {
            if(localizer != null){ // if the localizer exists
                localizer.update(); // update our current position
            }

            // replay manager logic, comes first to ensure the rest of the program runs smoothly
            replayManager.updateModesFromGamepad(gamepad1, telemetry);



            if(replayManager.isReplaying()){
                currentTargetState = replayManager.getCurrentTargetState();
                gp1 = currentTargetState.getGamepad1State();
                gp2 = currentTargetState.getGamepad2State();
            }
            else { // if not replaying, set the gamepad states to the physical controller inputs
                gp1 = new GamepadState(gamepad1);
                gp2 = new GamepadState(gamepad2);
            }




            // Variables
            double xTranslatePower = -gp1.left_stick_y() * Math.abs(gp1.left_stick_y()); // specifically the y stick is negated because up is negative on the stick, but we want up to move the robot forward
            double yTranslatePower = gp1.left_stick_x() * Math.abs(gp1.left_stick_x()); // set the robot translation/rotation speed variables based off of controller input (set later in hardware manipluation section)
            double rotatePower = gp1.right_stick_x() * Math.abs(gp1.right_stick_x());


            // Main Logic
            if(gp1.right_bumper() && firstRelativeToFieldToggle && !replayManager.isRecording() && !replayManager.isReplaying()){ // toggle relative to field drive
                drivingFieldRelative = !drivingFieldRelative;
                localizer.setPoseEstimate(startPose);

                firstRelativeToFieldToggle = false;
            }
            else if( !gp1.right_bumper() ){
                firstRelativeToFieldToggle = true;
            }

            if(gp1.left_bumper() && firstSpeedLimitingToggle && !replayManager.isRecording() && !replayManager.isReplaying()){ // toggle drive speed limiting
                speedLimiting = !speedLimiting;

                firstSpeedLimitingToggle = false;
            }
            else if( !gp1.left_bumper() ){
                firstSpeedLimitingToggle = true;
            }


            //setup a dead zone for the controllers
            if(Math.abs(xTranslatePower) <= DEAD_ZONE_RADIUS){ // if the value is less than the maximum deadzone value, set to zero (to stop the motor)
                xTranslatePower = 0;
            }
            if(Math.abs(yTranslatePower) <= DEAD_ZONE_RADIUS){ // if the value is less than the maximum deadzone value, set to zero (to stop the motor)
                yTranslatePower = 0;
            }
            if(Math.abs(rotatePower) <= DEAD_ZONE_RADIUS){ // if the value is less than the maximum deadzone value, set to zero (to stop the motor)
                rotatePower = 0;
            }


            // Hardware instruction
            if(replayManager.isRecording()){
                //replayManager.recordRobotState(new RobotState(replayManager.getTimerMsec(), localizer.getPoseEstimate(), new GamepadState(gamepad1), new GamepadState(gamepad2))); // save the robot state

                replayManager.recordRobotState(new RobotState(replayManager.getTimerMsec(), localizer.getPoseEstimate())); // save the robot state
            }

            if(replayManager.isReplaying()){
                mecanumDrive.setReplayBaseMovement(xTranslatePower, yTranslatePower, rotatePower, localizer.getPoseEstimate().getHeading(), false, false);
                mecanumDrive.driveToReplayPose(localizer.getPoseEstimate(), currentTargetState.getPosition());
            }
            else if (drivingFieldRelative) { // if not replaying, allow the user to drive normally, either field relative or not
                mecanumDrive.driveFieldRelative(xTranslatePower, yTranslatePower, rotatePower, localizer.getPoseEstimate().getHeading(), speedLimiting);
            }
            else {
                mecanumDrive.driveRobotRelative(xTranslatePower, yTranslatePower, rotatePower, speedLimiting);
            }



            if(replayManager.isRecording()){
                //telemetry.addLine("Currently Recording a Path. Press Y again to stop recording.");
            }
            else {
                //telemetry.addLine("Not Recording a Path. Press the Y button to start recording.");
            }
            if(replayManager.isReplaying()){
                //telemetry.addLine("Currently Replaying a Path. Press X again to stop replaying.");
                telemetry.addData("Target Position:", currentTargetState.getPosition());
                //telemetry.addData("Gamepad 1 Recorded Left Stick", new Vector2d(currentTargetState.getGamepad1State().left_stick_x(), currentTargetState.getGamepad1State().left_stick_y()));
            }
            else {
                //telemetry.addLine("Not Replaying a Path. Press the X button to start replaying.");
            }
            telemetry.addData("Current Position:", localizer.getPoseEstimate());
            //telemetry.addData("Failed Load Count:", failedLoadCount);
            telemetry.addData("Driving field relative? (gp1 right bumper to toggle)", drivingFieldRelative);
            telemetry.addData("Driver Acceleration Limiting? (gp1 left bumper to toggle)", speedLimiting);
            telemetry.addLine("Gp1 Sticks:\nLeft: " + new Vector2d(gp1.left_stick_x(), gp1.left_stick_y()) + "\nRight: " + new Vector2d(gp1.right_stick_x(), gp1.right_stick_y()) );
            telemetry.addData("X Translate Power", xTranslatePower);
            telemetry.addData("Target state has gamepad states?", currentTargetState.hasGamepadStates());
            telemetry.addData("GP1 A button", gp1.a());
            telemetry.addData("GP1", gp1.toCSVLine());
            //telemetry.addData("Replay States:", replayManager.getReplayStates());
            telemetry.update();


            updateDashboard();
        }  // end of running while loop
    }


    /* PUT ALL FUNCTIONS HERE */
    public void updateDashboard(){
        TelemetryPacket packet = new TelemetryPacket();
        Pose2d currentPose = localizer.getPoseEstimate();


        packet.put("Recording", replayManager.isRecording()); // get the shooter velocity and add that
        packet.put("Replaying", replayManager.isReplaying()); // get the shooter velocity and add that

        packet.put("X", currentPose.getX()); // get the shooter velocity and add that
        packet.put("Target X", currentTargetState.getPosition().getX()); // get the shooter velocity and add that
        packet.put("Y", currentPose.getY()); // get the shooter velocity and add that
        packet.put("Target Y", currentTargetState.getPosition().getY()); // get the shooter velocity and add that
        packet.put("Heading", currentPose.getHeading()); // get the shooter velocity and add that
        packet.put("Target Heading", currentTargetState.getPosition().getHeading()); // get the shooter velocity and add that


        Canvas fieldOverlay = packet.fieldOverlay();
        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke("#3F51B5"); // set the current draw color to blue

        DashboardUtil.drawRobot(fieldOverlay, currentPose);

        if(replayManager.isRecording()){
            DashboardUtil.drawPoseHistory(fieldOverlay, replayManager.getRecordedPositionsHistory());
        }
        else if(replayManager.isReplaying()){
            fieldOverlay.setStroke("#632085"); // set the field draw color for this bit to dark purple
            DashboardUtil.drawPoseHistory(fieldOverlay, replayManager.getReplayPositions());
            DashboardUtil.drawRobot(fieldOverlay, currentTargetState.getPosition());
        }


        if(dashboard != null){ // only send to the dashboard if it is properly setup
            dashboard.sendTelemetryPacket(packet);
        }
    }

}
