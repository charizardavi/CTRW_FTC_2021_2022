package org.firstinspires.ftc.teamcode.command.legacy;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner.Drive_Mecanum_RoadRunner;
import org.firstinspires.ftc.teamcode.hardware.intakes.Intake_Ring_Drop;
import org.firstinspires.ftc.teamcode.hardware.shooters.J_Shooter_Ring_ServoFed;
import org.firstinspires.ftc.teamcode.utility.OpenCV.FarRingStackHeightPipeline;
import org.firstinspires.ftc.teamcode.utility.OpenCV.Vision_OpenCV_ExternalCam;
import org.firstinspires.ftc.teamcode.hardware.arms.Arm_Wobble_Grabber;
import org.firstinspires.ftc.teamcode.utility.DynamicStateMachine.AutoTask;
import org.firstinspires.ftc.teamcode.utility.DynamicStateMachine.AutoTaskManager;
import org.firstinspires.ftc.teamcode.utility.DynamicStateMachine.DriveFollowerTask;
import org.firstinspires.ftc.teamcode.utility.DynamicStateMachine.TargetDrivePosition;

import java.util.ArrayList;

// NOTE: Several sections of this code relating to the RR DriveConstant class have been commented out due to DriveConstant being no longer supported

/*
 * Welcome to the Autonomous OpMode! This opmode coordinates the robot's manipulators and drive without user input to complete challenges
 */
@Disabled
@Autonomous(name = "AutoOp2020_exp", group = "@@E") // the name and group for the opmode
//@Config // allow FTC dashboard to access public static member variables here

public class AutoOp2020_exp extends LinearOpMode {

    Provider2020 robot;
    Drive_Mecanum_RoadRunner drive;
    J_Shooter_Ring_ServoFed shooter;
    Intake_Ring_Drop intake;
    Arm_Wobble_Grabber wobble;
    Vision_OpenCV_ExternalCam vision;

    AutoTaskManager taskManager;
    ElapsedTime runtime;
    ElapsedTime currentTaskTime;


    public static String ROBOT_NAME = "Lil' ring flinga";

    private static Pose2d startPose = new Pose2d(-65.3, 7.39, Math.toRadians(0)); // the starting position of the robot relative to the middle of the field

    public static TargetDrivePosition wobbleGoalPosA  = new TargetDrivePosition(19.0, -20.0, Math.toRadians(0.0), Math.toRadians(-45)); // the positions that the robot needs to drive to
    public static TargetDrivePosition wobbleGoalPosB  = new TargetDrivePosition(15.5, -1.5, Math.toRadians(90.0));
    public static TargetDrivePosition wobbleGoalPosC  = new TargetDrivePosition(45.7, -24.2, Math.toRadians(70.0), Math.toRadians(-85));
    public static TargetDrivePosition wobblePickupPosA = new TargetDrivePosition(-39.9, -24, Math.toRadians(-45.0), Math.toRadians(-135));
    public static TargetDrivePosition wobblePickupPosB = new TargetDrivePosition(-39.4, -20.5, Math.toRadians(-45.0), Math.toRadians(-140));
    public static TargetDrivePosition wobblePickupPosC = new TargetDrivePosition(-40.9, -17.7, Math.toRadians(-38.0), Math.toRadians(-140));


    public static TargetDrivePosition lineShootPos = new TargetDrivePosition(-7.5, 0.3, Math.toRadians(-2.3));
    public static TargetDrivePosition powershot1Position = new TargetDrivePosition(-6, -9.75, Math.toRadians(15.2), Math.toRadians(-82));
    public static TargetDrivePosition powershot2Position = new TargetDrivePosition(-6.5, -0.4, Math.toRadians(15.2));
   //public static TargetDrivePosition powershot1Position = new TargetDrivePosition(-5.9, -13.4, Math.toRadians(15.2), Math.toRadians(-82));
   // public static TargetDrivePosition powershot2Position = new TargetDrivePosition(-5.8, -1.9, Math.toRadians(15.2));
    public static TargetDrivePosition powershot3Position = new TargetDrivePosition(-6.5, 7.2, Math.toRadians(15.2));
    public static TargetDrivePosition powerCollectStartPos = new TargetDrivePosition(53.1, 25, Math.toRadians(-69.0), Math.toRadians(-90));
    public static TargetDrivePosition powerCollectEndPos = new TargetDrivePosition(51.3, -28.2, Math.toRadians(-69.0), Math.toRadians(-90));


    public static TargetDrivePosition stackPickupPos = new TargetDrivePosition(-27, 8.4, Math.toRadians(-90.0));
    //public static TargetDrivePosition stackPickupPos = new TargetDrivePosition(-30, -6.2, Math.toRadians(285), Math.toRadians(-90));
    public static TargetDrivePosition ringPickupPos = new TargetDrivePosition(-40.5, -12.5, Math.toRadians(180.0));
    public static TargetDrivePosition parkPosA     = new TargetDrivePosition(2.0, -2.0, Math.toRadians(0.0));
    public static TargetDrivePosition parkPosB     = new TargetDrivePosition(2.0, -2.0, Math.toRadians(90.0));
    public static TargetDrivePosition parkPosC     = new TargetDrivePosition(0.0, -18.5, Math.toRadians(90.0));


    public static double ARM_OFFSET_DEGREES = -378; // an offset for the wobble arm
    public static double ARM_COLLECT_DROP_DISTANCE = 18; // how far the robot is from collecting a wobble before it deploys the arm early
    public static double ARM_DROP_DROP_DISTANCE = 3; // how far the robot is from the drop position before it deploys the arm early
    public static double RING_COLLECT_DISTANCE = 22.3;
    public static double RING_COLLECT_2RING_OFFSET = -11.5;
    public static int RING_SCAN_TIME = 100; // scan the ring for 100 msecs
    public static int POWERSHOT_SHOOT_TIME = (int)J_Shooter_Ring_ServoFed.INDEXER_MOVE_TIME - 130;
    public static int POWERSHOT_INTAKE_TIME = 1600; // start running the intake 1.6 seconds into driving to collect rings
    public static double MAX_RING_COLLECT_SPEED = 30.0;

    private String wobbleDropPosLabel = "C"; // the default position
    private TargetDrivePosition wobbleGoalPos = new TargetDrivePosition();
    private boolean scanningComplete = false;
    private boolean autonomousComplete = false;



    @Override
    public void runOpMode() throws InterruptedException {
        // Pre match start initialization
        runtime = new ElapsedTime();
        currentTaskTime = new ElapsedTime();

        robot = new Provider2020(hardwareMap); // setup the hardware classes with the hardware map info (device ports, device names, etc)
        drive = new Drive_Mecanum_RoadRunner(hardwareMap);
        intake = new Intake_Ring_Drop(robot.intakeMotor, robot.ringGateServo);
        shooter = new J_Shooter_Ring_ServoFed(robot.JShootFront, robot.JShootBack, robot.shooterFeederServo, robot.shooterIndexerServo, robot.shooterAnglerServo);
        wobble = new Arm_Wobble_Grabber(robot.wobbleArmMotor, robot.wobbleLeftWheelServo, robot.wobbleRightWheelServo);

        vision = new Vision_OpenCV_ExternalCam(hardwareMap, "Webcam 1", new FarRingStackHeightPipeline()); // setup the camera handling class, passing in the pipeline we want to use to process inputs



        drive.setPoseEstimate(startPose); // set the localizer's start position to our position on the field relative to the center


        wobble.setCurrentPosAs0();
        wobble.setArmOffset(ARM_OFFSET_DEGREES); // set the wobble's starting offset, such that the starting position behaves like 0 (as in Auto the encoder likes to start in a weird position)
        intake.raiseGate();
        wobble.stopIntake();

        // Do the ring scanning
        vision.startWebcamStreaming(); // start the streaming process

        currentTaskTime.reset();
        while(!isStopRequested() && currentTaskTime.milliseconds() < 1000);


        telemetry.addLine(ROBOT_NAME + "'s setup completed.");
     //   telemetry.addLine("Ring scan count: " + ringStackEstimates.size());
     //   telemetry.addLine("  Detected drop position: " + wobbleDropPosLabel);
     //   telemetry.addData("Chosen first task", taskManager.getCurrentTask().getTaskName());
        telemetry.update();



        waitForStart(); // wait until play is pressed on the robot controller to progress


        // Post start setup (usually for anything that requires moving any parts on the robot)
        currentTaskTime.reset();
        //while(); // wait for 500 milliseconds without stopping any background processes
        for(int i = 0; !isStopRequested() && currentTaskTime.milliseconds() < RING_SCAN_TIME; i++){ // scan for however long the ring_scan_time is
            updateSensors("Scan rings"); // call the update sensors method telling it that we are currently scanning rings

            telemetry.addLine("Scanning rings...");
            telemetry.addLine("Scan count: " + ringStackEstimates.size());
            int[] counts = getEstimateOccurances();
            telemetry.addLine("Ring Count Votes - 0: " + counts[0] + ", 1: " + counts[1] + ", 4: " + counts[2]);
            telemetry.update();
        }
        // count up the votes that the camera collected for what drop state it is
        wobbleDropPosLabel = getCalculatedDropPosition(); // get which position was voted for by the camera system the most
        if(wobbleDropPosLabel.equals("A"))
            wobbleGoalPos = wobbleGoalPosA;
        else if(wobbleDropPosLabel.equals("B"))
            wobbleGoalPos = wobbleGoalPosB;
        else
            wobbleGoalPos = wobbleGoalPosC;

        ArrayList<AutoTask> autoTasks = setupAutoTasks(); // add the list of task objects to the task list
        taskManager = new AutoTaskManager(autoTasks); // then set the drive to use those tasks when required
        setWobbleGoalPos(wobbleDropPosLabel); // update the task positions based on sensor data

        // set the first target position based on priorities and what is closest
        taskManager.updateCurrentTaskToClosest( drive.getPoseEstimate() ); // have the task manager set the current auto task to the next closest incomplete task

        //drive.setTasks(taskManager.generateCurrentTaskDriveTaskList( drive.getPoseEstimate(), drive.getDriveConstraints() )); // then set the drive to go to that task's position and do any of that auto task's drive tasks

        vision.webcam.closeCameraDevice(); // stop the camera to prevent slowing down the system
        scanningComplete = true;


        wobble.setArmPosition(ARM_OFFSET_DEGREES);

        currentTaskTime.reset();




        while (opModeIsActive() && !isStopRequested() && !autonomousComplete){ // while it is ok to keep running in a loop, do so
            String currentTaskName = taskManager.getCurrentTask().getTaskName();

            boolean autoTaskComplete = drive.doTasksAsync(); // use the built in drive state machine that decides if/when each task is over, and follows them in order appropriately


            updateManipulators(currentTaskName); // update any non-drive mechanical systems


            if( autoTaskComplete ){ // if the current task is complete
                taskManager.markCurrentTaskComplete(); // set the current task to complete

                // check the name of the current task against as list of prerequisite tasks
                if( currentTaskName.equals("Shoot Powershots") ){
                    taskManager.markTaskWithNameIncomplete("Shoot Power Rings"); // activate the dependant task
                    //taskManager.markTaskWithNameIncomplete("Collect Wobble"); // activate the dependant task
                }
                else if( currentTaskName.equals("Shoot Power Rings") ){  // if this task completed
                    if( !wobbleDropPosLabel.equals("A") ){ // if there are rings to collect (aka any target position other than the one that happens when no rings)
                        taskManager.markTaskWithNameIncomplete("Collect Rings"); // activate the dependant task
                    }
                    else { // on A we don't collect rings but we still need to actually collect the wobble
                        taskManager.markTaskWithNameIncomplete("Collect Wobble");
                    }
                }
                else if( currentTaskName.equals("Shoot Stack Rings") ){ // if this prerec task completed
                    if(wobbleDropPosLabel.equals("C")){
                        //taskManager.markTaskWithNameIncomplete("Collect 4th Ring"); // activate the dependant task
                    }
                }
                else if( currentTaskName.equals("Collect Rings") ){ // if this prerec task completed
                    if( wobbleDropPosLabel.equals("B")){
                        taskManager.markTaskWithNameIncomplete("Shoot Single Ring"); // activate the dependant task
                    }
                    else {
                        taskManager.markTaskWithNameIncomplete("Shoot Stack Rings"); // activate the dependant task
                    }

                    if(wobbleDropPosLabel.equals("B") || wobbleDropPosLabel.equals("C"))
                        taskManager.markTaskWithNameIncomplete("Place Wobble 2");
                }
                else if( currentTaskName.equals("Collect 4th Ring")){ // this one only ever happens in a 4 stack
                    taskManager.markTaskWithNameIncomplete("Shoot Single Ring");
                }
                else if( currentTaskName.equals("Place Wobble 1") ){ // if this prerec task completed
                    taskManager.markTaskWithNameIncomplete("Collect Wobble"); // activate the dependant task
                }
                else if( currentTaskName.equals("Collect Wobble") ){ // if this prerec task completed
                    taskManager.markTaskWithNameIncomplete("Place Wobble 2"); // activate the dependant task
                }
                else if( currentTaskName.equals("END")){
                    autonomousComplete = true; // breaks free of the main loop
                }

                Pose2d currentPos = drive.getPoseEstimate(); // get the current position of the robot
                taskManager.updateCurrentTaskToClosest( currentPos ); // have the task manager set the current auto task to the next closest incomplete task
                //drive.setTasks(taskManager.generateCurrentTaskDriveTaskList( currentPos, drive.getDriveConstraints() )); // then set the drive to go to that task's position and do any of that auto task's drive tasks

                shooter.resetShotCount();
                currentTaskTime.reset(); // reset the current task time after each completion of an AutoTask
            }


            // Print telemetry output to the phones for better understanding of what the robot is doing
            telemetry.addData("Current task name", taskManager.getCurrentTask().getTaskName() );
            telemetry.addData("Drive task index", drive.getTaskIndex()); // print out what the current drive task index is

            if(drive.currentTaskHasTrajectory()) { // if current task has a trajectory to follow
                telemetry.addLine("Currently following a trajectory. Elapsed time: " + changePrecision( mSecToSec(drive.getTaskElapsedTime()), 2) + " seconds"); // print the projected remaining time on that trajectory
                telemetry.addData("Target position", drive.getCurrentTask().getTraj().end()); // print the end position of the trajectory
            }
            else {
                telemetry.addLine("Currently waiting. Elapsed time: " + changePrecision(mSecToSec(drive.getCurrentTask().getNum() - drive.getRemainingWaitMSecs()), 2) + " seconds");
                telemetry.addLine("  Time remaining: " + changePrecision(mSecToSec(drive.getRemainingWaitMSecs()), 2) + " seconds");
            }
            telemetry.addData("Current position", drive.getLocalizer().getPoseEstimate()); // print the estimated current position of the robot

            telemetry.addLine("Shooter velocity: " + shooter.getFlywheelVelo());
            telemetry.addLine("Target Arm position: " + wobble.getArmTargetPosition() + " degrees");
            telemetry.addLine("Arm position: " + wobble.getArmPosition() + " degrees");
            telemetry.addLine("Wobble intake power: " + wobble.getIntakeDirection());
            telemetry.addLine("Ring intake power: " + intake.getCurrentIntakePower());

            if(scanningComplete) {
                telemetry.addLine("Stack scanning complete. Scan count: " + ringStackEstimates.size());
                telemetry.addLine("  Detected drop position: " + wobbleDropPosLabel);
                int[] counts = getEstimateOccurances();
                telemetry.addLine("Ring Count Votes - 0: " + counts[0] + ", 1: " + counts[1] + ", 4: " + counts[2]);
            }
            else {
                int[] counts = getEstimateOccurances();
                telemetry.addLine("Ring Count Votes - 0: " + counts[0] + ", 1: " + counts[1] + ", 4: " + counts[2]);
            }

            telemetry.update(); // push all of the telemetry to the phone

        }
    }


    void updateSensors(String currentTaskName){ // updates any processing related to sensor data, in this game that mostly means camera data

        if( currentTaskName.equals("Scan rings") ){ // if current task is this task and we have completed driving to location
            String visionOutput = vision.getOutput(); // get the first character of the output string, it holds the single digit number of rings output
            char ringCount = visionOutput.charAt(0);

            if(ringCount == '0'){ // if no rings, add a vote for A position
                ringStackEstimates.add("A");
            }
            else if(ringCount == '1'){ // if one ring, add a vote for B position
                ringStackEstimates.add("B");
            }
            else { // if some other amount of rings (usually meaning 4), add a vote for C position
                ringStackEstimates.add("C");
            }
        }
    }
    void updateManipulators(String currentTaskName) { // updates any processing and processes related to non-drive mechanical systems on the robot
        if ( (currentTaskName.equals("Place Wobble 1") || currentTaskName.equals("Place Wobble 2") )) { // if current task is this task (or the other one)
            shooter.spinDown();

            if(currentTaskName.equals("Place Wobble 1")){
                intake.spinUp();
                shooter.indexerDown();
            }
            else {
                intake.spinDown();
                shooter.indexerUp();
                intake.lowerGate();
            }

            if(drive.getTaskIndex() == 1 || (Math.abs(drive.getPoseEstimate().getHeading() - wobbleGoalPos.getHeading()) <= 6 && Math.abs(drive.getPoseEstimate().getX() - wobbleGoalPos.getX()) <= 8)){ // if at location and on the first subtask
                wobble.goToGrabPos();
                wobble.intakeSpinIn();
            }
            else {
                wobble.goToUpPos();

                if(wobbleDropPosLabel.equals("A")){
                    wobble.intakeSpinOut();
                }
                else {
                    wobble.stopIntake();
                }
            }
        }
        else if ( currentTaskName.equals("Shoot Powershots")){
            shooter.optimizeForPowershots();
            shooter.setTargetShooterSpeed(shooter.getTargetShootingSpeed());
            if(drive.getTaskIndex() < 6){
                shooter.spinUp();
                shooter.indexerUp();
            }
            else { // if after the last shooting task, set things up to be good to run the intake
                shooter.spinDown();
                shooter.indexerDown();
            }

            if(drive.firstTaskCompleted()){ // once at location, do the things
                if(drive.getTaskIndex() == 1 || drive.getTaskIndex() == 3 || drive.getTaskIndex() == 5){ // if a shooting task
                    shooter.instructFire();
                }
                else if(drive.getTaskIndex() >= 6){ // if driving to pick up rings

                    if(wobbleDropPosLabel.equals("C") && Math.abs(drive.getPoseEstimate().getHeading() - wobbleGoalPos.getHeading()) <= 0.9  && Math.abs(drive.getPoseEstimate().getX() - wobbleGoalPos.getX()) <= ARM_DROP_DROP_DISTANCE && Math.abs(drive.getPoseEstimate().getY() - wobbleGoalPos.getY()) <= ARM_DROP_DROP_DISTANCE){
                        wobble.goToGrabPos();
                        wobble.intakeSpinIn();
                        intake.setIntakeRunSpeed(1);
                    }
                    else if(Math.abs(drive.getPoseEstimate().getX() - wobbleGoalPos.getX()) <= 7 && Math.abs(drive.getPoseEstimate().getY() - wobbleGoalPos.getY()) <= 7){
                        wobble.goToUpPos();
                        //intake.setIntakeRunSpeed(-1);
                    }
                    else{
                        wobble.setArmPosition(ARM_OFFSET_DEGREES);
                        intake.setIntakeRunSpeed(1);
                    }
                    intake.spinUp();
                }
            }
        }
        else if ( currentTaskName.equals("Shoot Power Rings") || currentTaskName.equals("Shoot Stack Rings") || currentTaskName.equals("Shoot Single Ring")) { // if current task is this task (or the other one)
            wobble.goToUpPos();
            shooter.optimizeForHighgoal(); // set the shooter to optimize for shooting from across the field
            shooter.setTargetShooterSpeed(shooter.getTargetShootingSpeed());

            shooter.spinUp(); // spin up the shooter on the way to

            if(drive.firstTaskCompleted() || Math.abs(drive.getPoseEstimate().getX() - lineShootPos.getX()) < 6){ // if at location, start shooting
                if(Math.abs(drive.getPoseEstimate().getX() - lineShootPos.getX()) < 3){
                    shooter.instructFire();
                }
                intake.spinDown();
                shooter.indexerUp();

               /* if( shooter.getFiringState() > 0 || currentTaskName.equals("Shoot Power Rings") || ){
                    intake.spinDown();
                    shooter.indexerUp();
                }
                else {
                    intake.spinUp();
                    shooter.indexerDown();
                }*/
            }
            else { // if not at location
                intake.spinUp();
                shooter.indexerDown();
            }

            if(currentTaskName.equals("Shoot Stack Rings") || currentTaskName.equals("Shoot Single Ring")){
                wobble.intakeSpinOut();
            }
            else {
                wobble.stopIntake();
            }
        }
        else if( currentTaskName.equals("Collect Rings")  ){ // if current task is this task
            intake.setIntakeRunSpeed(1);
            intake.spinUp(); // spin up the intake for ring collection
            shooter.indexerDown();
            shooter.spinDown();
            wobble.intakeSpinOut();
            wobble.goToGrabPos();

        }
        else if( currentTaskName.equals("Collect Wobble") ){ // if current task is this task
           // shooter.spinDown();
            shooter.indexerDown();
            intake.spinUp();

            if( drive.getTaskIndex() == 1 || taskManager.getDistanceFromTaskLocation(drive.getPoseEstimate()) <= ARM_COLLECT_DROP_DISTANCE ) { // if close to location or on the first subtask
                wobble.goToGrabPos();
                wobble.intakeSpinOut();
            }
            else if (drive.firstTaskCompleted()){ // if at location and on the second subtask
                wobble.setArmPosition(ARM_OFFSET_DEGREES);
                wobble.stopIntake();
            }

        }
        else if (currentTaskName.equals("END")){
            shooter.spinDown();
            intake.spinDown();
            wobble.stopIntake();
            shooter.indexerUp();
            wobble.setArmPosition(ARM_OFFSET_DEGREES);
        }
        else { // if no tasks selected above, default states are as follows
            shooter.spinDown();
            intake.spinDown();
            wobble.intakeSpinIn();
            shooter.indexerUp();
            wobble.setArmPosition(ARM_OFFSET_DEGREES);
        }

        wobble.goToRecentPos();
        shooter.updateFeeder(); // always update the shooter feeder, so that it can start the feeding process as soon as possible
    }


    ArrayList<AutoTask> setupAutoTasks(){
        ArrayList<AutoTask> autoTasks = new ArrayList<AutoTask>(); // create an array list to hold the tasks that will be output by the method
        ArrayList<DriveFollowerTask> atLocationTasks = new ArrayList<DriveFollowerTask>(); // create an array list to hold the driveTasks for each location (reset between adding tasks unless their at location drive task list is the same)


        TargetDrivePosition wobbleGoalPosition = wobbleGoalPosA;
        TargetDrivePosition wobblePickupPosition = wobblePickupPosA;
        if(wobbleDropPosLabel.equals("B")){
            wobbleGoalPosition = wobbleGoalPosB;
            wobblePickupPosition = wobblePickupPosB;
        }
        else if(wobbleDropPosLabel.equals("C")){
            wobbleGoalPosition = wobbleGoalPosC;
            wobblePickupPosition = wobblePickupPosC;
        }

        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(420)); // once at this location, wait msecs
        atLocationTasks.add(new DriveFollowerTask(10)); // then wait msecs
        autoTasks.add(new AutoTask("Place Wobble 1", 3, wobbleGoalPosA, new ArrayList<DriveFollowerTask>()).setCompleted(wobbleDropPosLabel.equals("C")));
        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(1)); // once at this location, wait msecs
        atLocationTasks.add(new DriveFollowerTask(1)); // then wait msecs
        /*atLocationTasks.add(new DriveFollowerTask(drive.trajectoryBuilder(parkPosC.getPose2d())
                .lineTo(new Vector2d(parkPosC.getX(), parkPosC.getY()+5), new DriveConstraints(0.1, 0.1, 0.0, Math.toRadians(180.0), Math.toRadians(180.0), 0.0))
                .build()
        ));*/
        autoTasks.add(new AutoTask("Place Wobble 2", 3, wobbleGoalPosA, atLocationTasks).setCompleted(true)); // set completed so that the pathing algorithm won't consider it until it is set true (which will happen once its prerequisite task becomes completed)

        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(POWERSHOT_SHOOT_TIME)); // wait to shoot
        atLocationTasks.add(new DriveFollowerTask(drive.trajectoryBuilder(powershot1Position.getPose2d()) // drive to next place to shoot
                .lineToLinearHeading(powershot2Position.getPose2d())
                .build()
        ));
        atLocationTasks.add(new DriveFollowerTask(POWERSHOT_SHOOT_TIME)); // do the same as above
        atLocationTasks.add(new DriveFollowerTask(drive.trajectoryBuilder(powershot2Position.getPose2d())
                .lineToLinearHeading(powershot3Position.getPose2d())
                .build()
        ));
        atLocationTasks.add(new DriveFollowerTask(POWERSHOT_SHOOT_TIME)); // do the same as above
        if(!wobbleDropPosLabel.equals("C")){
            atLocationTasks.add(new DriveFollowerTask(drive.trajectoryBuilder(powershot3Position.getPose2d()) // then pickup the rings shot
                    .splineToSplineHeading(powerCollectStartPos.getPose2d(), powerCollectStartPos.getSplineHeading()) // and in the same movement go to the wobble drop position
                    .splineToSplineHeading(powerCollectEndPos.getPose2d(), powerCollectEndPos.getSplineHeading()) // and in the same movement go to the wobble drop position
                    .splineToSplineHeading(wobbleGoalPosition.getPose2d(), wobbleGoalPosition.getSplineHeading()) // and in the same movement go to the wobble drop position
                    .build()
            ));
        }
        else {
            atLocationTasks.add(new DriveFollowerTask(drive.trajectoryBuilder(powershot3Position.getPose2d()) // then pickup the rings shot
                    .splineToSplineHeading(powerCollectStartPos.getPose2d(), powerCollectStartPos.getSplineHeading()) // and in the same movement go to the wobble drop position
                    .splineToSplineHeading(powerCollectEndPos.getPose2d(), powerCollectEndPos.getSplineHeading()) // and in the same movement go to the wobble drop position
                    .build()
            ));
            atLocationTasks.add(new DriveFollowerTask(drive.trajectoryBuilder(powerCollectEndPos.getPose2d()) // then pickup the rings shot
                    .lineToSplineHeading(wobbleGoalPosition.getPose2d())
                    .build()
            ));
        }

        autoTasks.add(new AutoTask("Shoot Powershots", 1, powershot1Position, atLocationTasks));

        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(120));
        autoTasks.add(new AutoTask("Shoot Single Ring", 3, new TargetDrivePosition(lineShootPos.getX(), lineShootPos.getY(), lineShootPos.getHeading()), atLocationTasks).setCompleted(true)); // set completed so that the pathing algorithm won't consider it until it is set true (which will happen once its prerequisite task becomes completed)
        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(80)); // only for the second one, give it more time just to be safe
        atLocationTasks.add(new DriveFollowerTask(420)); // once at this location, wait msecs
        autoTasks.add(new AutoTask("Shoot Power Rings", 3, new TargetDrivePosition(lineShootPos.getX(), lineShootPos.getY(), lineShootPos.getHeading()), atLocationTasks).setCompleted(true)); // set completed so that the pathing algorithm won't consider it until it is set true (which will happen once its prerequisite task becomes completed)
        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(220)); // only for the second one, give it more time just to be safe
        atLocationTasks.add(new DriveFollowerTask(420)); // once at this location, wait msecs
        autoTasks.add(new AutoTask("Shoot Stack Rings", 3, new TargetDrivePosition(lineShootPos.getX(), lineShootPos.getY(), lineShootPos.getHeading()), atLocationTasks).setCompleted(true)); // set completed so that the pathing algorithm won't consider it until it is set true (which will happen once its prerequisite task becomes completed)


        atLocationTasks = new ArrayList<DriveFollowerTask>();
        double collectDistanceOffset = (wobbleDropPosLabel.equals("C")) ? 0 : RING_COLLECT_2RING_OFFSET;
        double maxCollectSpeed = (wobbleDropPosLabel.equals("C")) ? MAX_RING_COLLECT_SPEED : 50;
        /*atLocationTasks.add(new DriveFollowerTask(drive.trajectoryBuilder(stackPickupPos.getPose2d())
            .forward(RING_COLLECT_DISTANCE + collectDistanceOffset, new DriveConstraints(
                    maxCollectSpeed, 60.0, 0.0,
                    Math.toRadians(180.0), Math.toRadians(180.0), 0.0
            )) // go forward to pickup rings
            .splineToSplineHeading(wobblePickupPosition.getPose2d(), wobblePickupPosition.getSplineHeading(), new DriveConstraints(
                    maxCollectSpeed, 60.0, 0.0,
                    Math.toRadians(180.0), Math.toRadians(180.0), 0.0
            ))
            .build()
        ));*/
        atLocationTasks.add(new DriveFollowerTask(320)); // wait msecs once at location
        autoTasks.add(new AutoTask("Collect Rings", 3, stackPickupPos, atLocationTasks).setCompleted(true)); // set completed so it won't try to collect rings before shooting rings for the first time
        autoTasks.add(new AutoTask("Collect 4th Ring", 3, ringPickupPos, atLocationTasks).setCompleted(true)); // set completed so it won't try to collect rings before shooting rings for the first time

        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(240)); // wait msecs once at location
        if(wobbleDropPosLabel.equals("C"))
            atLocationTasks.add(new DriveFollowerTask(80)); // then wait msecs
        autoTasks.add(new AutoTask("Collect Wobble", 3, wobblePickupPosC, atLocationTasks).setCompleted(true)); // set completed so it won't try to collect rings before shooting rings for the first time

        atLocationTasks = new ArrayList<DriveFollowerTask>();
        autoTasks.add(new AutoTask("END", 7, parkPosA, atLocationTasks));

        return autoTasks;
    }

    private ArrayList<String> ringStackEstimates = new ArrayList<String>(); // holds all outputs given by the vision system during the capturing period
    public String getCalculatedDropPosition(){ // sees which position gets the most "votes" by the system and return it

        int[] counts = getEstimateOccurances();

        if(counts[2] > counts[0] && counts[2] > counts[1]){ // if there are the most votes for c
            return "C";
        }
        else if(counts[1] > counts[0] && counts[1] > counts[2]){ // if there are the most votes for b
            return "B";
        }
        else { // if nothing else, must be A
            return "A";
        }
    }
    private int[] getEstimateOccurances(){
        int[] occuranceCounts = new int[3];

        for (String estimate : ringStackEstimates){
            if (estimate.equals("A")){
                occuranceCounts[0]++;
            }
            else  if (estimate.equals("B")){
                occuranceCounts[1]++;
            }
            else { // if not A or B, then C must have just gotten a vote
                occuranceCounts[2]++;
            }
        }

        return occuranceCounts;
    }
    public void setWobbleGoalPos(String positionLabel){ // based on which position label is passed in, change the target position for dropping the wobble goal
        TargetDrivePosition parkPosition;

        if(positionLabel.equals("A")){
            wobbleGoalPos = wobbleGoalPosA;
            parkPosition = parkPosA;

            ArrayList<DriveFollowerTask> waitTasks = new ArrayList<DriveFollowerTask>();
            waitTasks.add(new DriveFollowerTask(700));

            taskManager.setTaskWithNameLocationTasks("Place Wobble 1", waitTasks);
            taskManager.setTaskWithNameLocationTasks("Place Wobble 2", waitTasks);

            taskManager.setTaskWithNameLocation("Place Wobble 1", new TargetDrivePosition(wobbleGoalPos.getX(), wobbleGoalPos.getY(), wobbleGoalPos.getHeading())); // once the position has been found, set the tasks to their new positions
            taskManager.setTaskWithNameLocation("Place Wobble 2", new TargetDrivePosition(wobbleGoalPos.getX() - 9.6, wobbleGoalPos.getY(), wobbleGoalPos.getHeading(), wobbleGoalPos.getSplineHeading())); // once the position has been found, set the tasks to their new positions
            taskManager.setTaskWithNameLocation("Collect Wobble", wobblePickupPosA);

            taskManager.setTaskWithNameLocationTasks("Place Wobble 2", taskManager.getTaskWithName("Place Wobble 1").getTasksAtLocation());
        }
        else if (positionLabel.equals("B")){
            wobbleGoalPos = wobbleGoalPosB;
            parkPosition = parkPosB;

            taskManager.setTaskWithNameLocation("Place Wobble 1", new TargetDrivePosition(wobbleGoalPos.getX(), wobbleGoalPos.getY(), wobbleGoalPos.getHeading())); // once the position has been found, set the tasks to their new positions
            taskManager.setTaskWithNameLocation("Place Wobble 2", new TargetDrivePosition(wobbleGoalPos.getX(), wobbleGoalPos.getY()-7, wobbleGoalPos.getHeading())); // note: it is ok that if it is "A" the setting is redundant, the resources required to set are low and in FTC readability is favored over efficiency
            taskManager.setTaskWithNameLocation("Collect Wobble", new TargetDrivePosition(wobblePickupPosB.getX(), wobblePickupPosB.getY(), wobblePickupPosB.getHeading()));

            taskManager.setTaskWithNameLocationTasks("Place Wobble 2", taskManager.getTaskWithName("Place Wobble 1").getTasksAtLocation());
        }
        else {
            wobbleGoalPos = wobbleGoalPosC;
            parkPosition = parkPosC;

            taskManager.setTaskWithNameLocation("Place Wobble 1", new TargetDrivePosition(wobbleGoalPos.getX(), wobbleGoalPos.getY(), wobbleGoalPos.getHeading())); // once the position has been found, set the tasks to their new positions
            taskManager.setTaskWithNameLocation("Place Wobble 2", new TargetDrivePosition(wobbleGoalPos.getX()-5.4, wobbleGoalPos.getY()+1, wobbleGoalPos.getHeading() )); // note: it is ok that if it is "A" the setting is redundant, the resources required to set are low and in FTC readability is favored over efficiency
            taskManager.setTaskWithNameLocation("Collect Wobble", wobblePickupPosC); // note: it is ok that if it is "A" the setting is redundant, the resources required to set are low and in FTC readability is favored over efficiency
        }


        taskManager.setTaskWithNameLocation("END", parkPosition); // once the position has been found, set the tasks to their new position
    }

    public static double mSecToSec(double mSec) { return (mSec/1000); }
    public static double changePrecision(double input, int precision){
        return (double)((int)(input * Math.pow(10, precision))) / Math.pow(10, precision);
    }
}
