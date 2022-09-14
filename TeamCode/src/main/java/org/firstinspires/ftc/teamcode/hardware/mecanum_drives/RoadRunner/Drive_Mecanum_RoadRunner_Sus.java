package org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner;


import static org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner.DriveConstants.MAX_VEL;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.utility.DynamicStateMachine.DriveFollowerTask;
import org.firstinspires.ftc.teamcode.utility.FTCDashboard.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
*/
@Config
public class Drive_Mecanum_RoadRunner_Sus extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(23, 0, 2);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8.5, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    public static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    public static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx driveFL, driveBL, driveBR, driveFR;
    private List<DcMotorEx> motors;

    private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    private ElapsedTime localRuntime;

    public Drive_Mecanum_RoadRunner_Sus(HardwareMap hardwareMap, Localizer odo) {
        super(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, DriveConstants.TRACK_WIDTH, DriveConstants.TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        driveFL = hardwareMap.get(DcMotorEx.class, "driveFL");
        driveBL = hardwareMap.get(DcMotorEx.class, "driveBL");
        driveBR = hardwareMap.get(DcMotorEx.class, "driveBR");
        driveFR = hardwareMap.get(DcMotorEx.class, "driveFR");

        driveFL.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBL.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(driveFL, driveBL, driveBR, driveFR);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (DriveConstants.RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (DriveConstants.RUN_USING_ENCODER && DriveConstants.MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        // TODO: if desired, use setLocalizer() to change the localization method
        setLocalizer(odo);

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);

        localRuntime = new ElapsedTime();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public boolean isFollowing(){ return trajectorySequenceRunner.isBusy(); }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(DriveConstants.encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(DriveConstants.encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        driveFL.setPower(v);
        driveBL.setPower(v1);
        driveBR.setPower(v2);
        driveFR.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return 0.0;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // TODO: This must be changed to match your configuration
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // The positive x axis points toward the USB port(s)
        //
        // Adjust the axis rotation rate as necessary
        // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
        // flat on a surface

        return 0.0;
    }

    // State machine controlled asynchronous follow
    private ArrayList<DriveFollowerTask> tasks; // an arraylist that holds the list of tasks for
    private int taskIndex = 0; // a number that keeps track of where in the task list we are

    private boolean firstTaskRun = true; // first run flag for doing tasks, ensure proper behavior

    private double waitEndTime; // the time that the program designates as the time to go ahead and move to the next task (in milliseconds)
    private double currentTaskStartTime;

    public void setTasks(ArrayList<DriveFollowerTask> newTasks){
        tasks = newTasks; // set the tasks like promised
        taskIndex = 0; // reset the task index to ensure that everything goes well with the new job
    }
    public ArrayList<DriveFollowerTask> getTasks(){ return tasks; } // gets the whole list of tasks
    public DriveFollowerTask getTaskAt(int index){
        if(index < tasks.size())
            return tasks.get(index);
        else
            return new DriveFollowerTask();
    } // gets a specified task from the list
    public DriveFollowerTask getCurrentTask(){
        if (taskIndex < tasks.size() ) // ensure that we don't get an out of bounds error by checking that the current index is less than the size of the list
            return tasks.get(taskIndex);
        else
            return new DriveFollowerTask();
    } // gets the current task (not the variable, but what it is according to the index)
    public int getTaskIndex(){ return taskIndex; } // get what the current task is
    public boolean firstTaskCompleted(){ return (taskIndex >= 1); } // returns true if the first task in the task list has been passed

    public boolean doTasksAsync(){ // the main state machine function that runs through each task - when all tasks complete it returns true
        boolean allComplete = false;

        if( taskIndex < tasks.size() ){ // if still within the bounds of the task list
            boolean taskComplete = false; // indicates if the current task is complete yet (default is false)

            DriveFollowerTask currentTask = getTaskAt(taskIndex); // get the current task and set the currentTask variable to it

            if(currentTask.getTraj() != null){ // if there is a trajector to follow, follow dat trajectory
                if(firstTaskRun){ // only on the first run of the task
                    followTrajectoryAsync( currentTask.getTraj() ); // set the follower to follow the current task trajectory
                    currentTaskStartTime = localRuntime.milliseconds();
                }
                update(); // actually do the following of the trajectory, also update any important information, including if we are done following or not

                taskComplete = !isFollowing(); // if isFollowing returns true that means that we are still going and therefore taskComplete will be false, and visa versa
            }
            else if (currentTask.getNum() > 0){ // if there is no trajectory to follow but there is a time, wait
                if(firstTaskRun){
                    waitEndTime = localRuntime.milliseconds() + currentTask.getNum(); // set the target time to the current time plus the input number of milliseconds
                    currentTaskStartTime = localRuntime.milliseconds();
                }

                taskComplete = (localRuntime.milliseconds() >= waitEndTime); // if runtime is greater than or equal to the set waitEndTime, taskComplete is set to true, otherwise it is set to false
            }
            else {
                taskComplete = true; // else if there are neither, we just go to the next one and pretend this one didn't happen
            }

            if(firstTaskRun){ // if it is the end of the first loop run for the new task
                firstTaskRun = false; // set the flag to no longer show it is the first run of this task
            }
            if(taskComplete){ // if the current task is complete
                taskIndex++; // go to the next task

                firstTaskRun = true;
            }
        } // end of task doing if
        else { // if we are complete, as the task index has exceeded the number of tasks we have
            allComplete = true;
        }

        return allComplete; // return
    }

    public void endCurrentWait(){ // if the drive is currently performing a wait task, it will stop waiting
        waitEndTime = localRuntime.milliseconds(); // set the target end time to the current time, meaning that when checked the wait time will be satisfied
    }
    public boolean currentTaskHasTrajectory(){
        return (getCurrentTask().getTraj() != null);
    }
    public double getRemainingWaitMSecs(){
        return waitEndTime - localRuntime.milliseconds();
    }
    public double getTaskElapsedTime(){
        return localRuntime.milliseconds() - currentTaskStartTime;
    }


    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}
