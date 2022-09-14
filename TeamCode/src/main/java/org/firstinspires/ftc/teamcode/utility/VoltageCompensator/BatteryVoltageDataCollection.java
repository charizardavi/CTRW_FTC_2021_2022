package org.firstinspires.ftc.teamcode.utility.VoltageCompensator;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.ControlTheory.filters.LowPassFilter;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.Encoder;

import java.util.ArrayList;


/*
    Welcome to the 2020-2021 TeleOp class!

    Just kidding! This is test class. You can't trust any of the comments here, it is made in a hurry to do a job, usually lots of copy paste
 */

@Disabled
@TeleOp(group = "@@D")
@Config


public class BatteryVoltageDataCollection extends LinearOpMode{
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    String robotName = "Data Collector";
    public static String MOTOR_NAME = "motorIntake";
    public static String MOTOR_ENCODER_NAME = "motorDuck";

    // Robot Speed variables
    private double stopSpeed = 0.0; // the motor speed for stopping the robot
    private double motorSpeed = 1.0;
    private double changeAmount = 0.05;
    public static double FULL_BATTERY_VELO = 20;
    public static double WRITE_INTERVAL_MSEC = 25000; // every 25 sec

    // Constants


    // Robot Classes
    private ElapsedTime writeTimer; // internal clock
    ElapsedTime loopClock;
    LowPassFilter veloFilter;
    LowPassFilter voltageFilter;
    VoltageSensor batteryVoltageSensor;

    // Flags
    private boolean firstIncreaseSpeed = true; // used to ensure proper toggling behavior (see usage under logic section)
    private boolean firstDecreaseSpeed = true; // used to ensure proper toggling behavior (see usage under logic section)
    private boolean firstRunningToggle = true;
    private boolean runningOpmode = true;

    boolean isRunning = false;
    double lastEncoderPosition = 0;
    public static double veloFilterLastVeloWeight = 0.999;
    public static double voltageFilterLastVoltageWeight = 0.9999;



    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        writeTimer = new ElapsedTime();
        loopClock = new ElapsedTime();

        DcMotor mainMotor = hardwareMap.get(DcMotor.class, MOTOR_NAME);
        DcMotorEx encoderMotor = hardwareMap.get(DcMotorEx.class, MOTOR_ENCODER_NAME);
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Encoder motorEncoder = new Encoder( encoderMotor );

        veloFilter = new LowPassFilter(veloFilterLastVeloWeight);
        voltageFilter = new LowPassFilter(voltageFilterLastVoltageWeight);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        ArrayList<Double[]> dataPoints = new ArrayList<>();

        telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();



        waitForStart(); // Wait for the start button to be pressed before continuing further



        writeTimer.reset(); // reset the clock once start has been pressed so runtime is accurate
        loopClock.reset();

        // The main run loop - write the main robot run code here
        while (opModeIsActive() && runningOpmode) {

            // Logic (figuring out what the robot should do)

            if(gamepad1.dpad_up && firstIncreaseSpeed){ // toggle driving realtive to field if dpad up is pressed
                motorSpeed += changeAmount;

                motorSpeed = Math.min(motorSpeed, 1);

                firstIncreaseSpeed = false; // set the variable false so that it cannot toggle again
            }
            else if (!gamepad1.dpad_up){ // wait to set the flag back to true until the button is released
                firstIncreaseSpeed = true; // until the button is released
            }

            if(gamepad1.dpad_down && firstDecreaseSpeed){ // toggle driving using encoders on the press of dpad down
                motorSpeed -= changeAmount;

                motorSpeed = Math.max(motorSpeed, 0);

                firstDecreaseSpeed = false; // set the variable false so that it cannot toggle again
            }
            else if (!gamepad1.dpad_down){ // wait to set the flag back to true until the button is released
                firstDecreaseSpeed = true; // until the button is released
            }


            if(gamepad1.right_bumper && firstRunningToggle){ // toggle driving using encoders on the press of dpad down
                isRunning = !isRunning;

                if(!isRunning){
                    runningOpmode = false;
                }

                firstRunningToggle = false; // set the variable false so that it cannot toggle again
            }
            else if (!gamepad1.right_bumper){ // wait to set the flag back to true until the button is released
                firstRunningToggle = true; // until the button is released
            }


            double encoderPosition = motorEncoder.getCurrentPosition(); // measure position so we can calulate velocity
            double timeSinceLastLoop = loopClock.milliseconds();

            double rawVelo = (encoderPosition - lastEncoderPosition) / timeSinceLastLoop;
            double velo = veloFilter.filter( rawVelo );
            double percentageOfMaxVelo = velo / FULL_BATTERY_VELO;

            lastEncoderPosition = encoderPosition; // reset everything for the next loop
            loopClock.reset();


            double rawBatteryVoltage = batteryVoltageSensor.getVoltage();
            double batteryVoltage = voltageFilter.filter( rawBatteryVoltage );



            if( isRunning && writeTimer.milliseconds() >= WRITE_INTERVAL_MSEC ){
                Double[] dataPoint = new Double[2];

                dataPoint[0] = batteryVoltage;
                dataPoint[1] = velo;

                dataPoints.add(dataPoint);

                writeTimer.reset();
            }



            // Telemetry
            if(isRunning){ // add telemetry relating to robot drive mode
                telemetry.addLine("Running motors at " + motorSpeed * 100.0 + "% of max speed.");

                mainMotor.setPower(motorSpeed);
            }
            else{
                telemetry.addLine("Motor not running, tap the Right Bumper (on gamepad1) to run the motor at " + motorSpeed * 100.0 + "% of max speed.");

                mainMotor.setPower(stopSpeed);
            }
            telemetry.addLine("Press D-Pad Up to increase speed by " + (changeAmount * 100) + "%");
            telemetry.addLine("Press D-Pad Down to decrease speed by " +( -changeAmount * 100) + "%");

            telemetry.addData("raw velo", rawVelo);
            telemetry.addData("smoothed velo", velo);
            telemetry.addData("percentage of max velo", percentageOfMaxVelo * 100);
            telemetry.addData("battery voltage", batteryVoltage);
            telemetry.addData("time until write", WRITE_INTERVAL_MSEC - writeTimer.milliseconds());

            if(batteryVoltage <= 11.713844175222){
                runningOpmode = false;
            }

            if(!runningOpmode){
                telemetry.addLine("");
                for(Double[] dataPoint : dataPoints){
                    telemetry.addLine("" + dataPoint[0] + "," + dataPoint[1]);
                }
            }

            telemetry.update();
        }
    }


    /* PUT ALL FUNCTIONS HERE */
   /* public File openFile(String fileName){

        File root = new File(Environment.getExternalStorageDirectory(), STORAGE_DIRECTORY);
        if(!root.exists()){
            root.mkdirs();
        }

        File file = new File(root, fileName);

        try{
            file.createNewFile(); // if unable to create a new file at location, could load the file (the create new file method returns true if could create a new file)
        } catch (IOException e){ e.printStackTrace(); }

        return file;
    }*/





}
