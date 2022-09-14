package org.firstinspires.ftc.teamcode.command.tests.susPension.failsafe;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.Drive_Mecanum;
import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner.Drive_Mecanum_RoadRunner_Sus;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.PIDController;
import org.firstinspires.ftc.teamcode.utility.Math.LinearInterpolator;
import org.firstinspires.ftc.teamcode.utility.Odometry.TrackingWheelLocalizer2021_Sus;
import org.firstinspires.ftc.teamcode.utility.OpenCV.Vision_OpenCV_ExternalCam;


//TO DO: MAKE INTAKEMOTORISBUSY MY OWN VARIABLE (make it true when setting vels)
//TO DO PART 2: MAKE STOW POSITION FOR CROSSING BARRIER
@Config
public class SusPensionBotFailsafe {

    public enum outakeStateList{
        INTAKE,
        STOW,
        OUTAKE_UPPER,
        OUTAKE_MIDDLE,
        OUTAKE_LOWER,
        OUTAKE_NEUTRAL_LEFT,
        OUTAKE_NEUTRAL_RIGHT,
        DUCK_SPIN
    }

    public outakeStateList outakeStateInput = outakeStateList.OUTAKE_UPPER;

    public outakeStateList outakeState = outakeStateList.INTAKE;


    public static double duckRunTime = 2500;
    public static double duckStartPower = 0;
    public static double duckEndPower = 0.72;


    String robotName = "susPension";


    public DcMotor driveFL = null; //port 0 on control hub
    public DcMotor driveFR = null; //port 1 on control hub
    public DcMotor driveBL = null; //port 2 on control hub
    public DcMotor driveBR = null; //port 3 on control hub

    public DcMotorEx armR; //port 0 on expansion hub
    public DcMotorEx armL; //port 2 on expansion hub

    public DcMotorEx intakeMotor; //port 1 on expansion hub

    public DcMotorEx duckSpinMotor;

    public PIDController pidS;
    public PIDController pidB;
    public PIDFController duckSpinnerFF;

    public DistanceSensor intakeDetect;


    public double smallRotTarget = 0;
    public double bigRotTarget = 0;

    public double intakePower = 1;
    public double outakePower = -.27;

    public double bigRotOutput;
    public double smallRotOutput;

    public double armLPower = 0;
    public double armRPower = 0;

    public  boolean intakeMotorIsBusy = false;

    public ServoImplEx odoR;
    public ServoImplEx odoL;
    public ServoImplEx odoM;
    public Servo intakeLockServo;

    public boolean outakeBusy = false;

    public double sideFlip = -1.0;

    public static org.firstinspires.ftc.teamcode.utility.ControlTheory.coefficient_wrappers.PIDCoefficients CoeffS = new org.firstinspires.ftc.teamcode.utility.ControlTheory.coefficient_wrappers.PIDCoefficients(-.1,0,0);
    public static org.firstinspires.ftc.teamcode.utility.ControlTheory.coefficient_wrappers.PIDCoefficients CoeffB = new org.firstinspires.ftc.teamcode.utility.ControlTheory.coefficient_wrappers.PIDCoefficients(.1,0,0);

    public ElapsedTime duckTimer;

    public PIDCoefficients duckSpinnerCoeff;
    public HardwareMap mainMap;
    public Localizer odometry;
    public Drive_Mecanum drive;
    public Drive_Mecanum_RoadRunner_Sus driveRoadRunner;
    public MotionProfile duckProfile;
    public LinearInterpolator duckLinear;
    public Vision_OpenCV_ExternalCam vision;



    public static double armMaxSpeed = 0.5;

    public static double outakeUpperBigRot = 155;
    public static double outakeMidBigRot = 190;
    public static double outakeNeutralBigRot = 200;
    public static double outakeLowerBigRot = 210;
    public static double intakeStowBigRot = 15;
    public static double intakeBigRot = 0;

    public static double smallRotAtTargetMargin = 10;


    public static double outakeUpperSmallRot = 0;
    public static double outakeNeutralLeftSmallRot = -83;
    public static double outakeNeutralRightSmallRot = 83;
    public static double outakeMidSmallRot = -45;

    public static double armAtTargetMargin = 5;


    public static double odoLRaisePos = 0.72;
    public static double odoRRaisePos = 0.25;
    public static double odoMRaisePos = .3;

    public static double intakeServoUnlockPos = 0.75;


    public static double armMotorTicsPerRev = 604.8;
    public static double armGearRatio = 1.0/6.0;



    public SusPensionBotFailsafe(HardwareMap hMap){
        initMap(hMap);

        drive = new Drive_Mecanum(driveFL, driveFR, driveBL, driveBR);

        armDegreesPerTic = (1/armMotorTicsPerRev) * armGearRatio * 360;

    }
    public SusPensionBotFailsafe(HardwareMap hMap, String robotName){
        this(hMap);

        this.robotName = robotName;
    }



    private void initMap(HardwareMap hMap) {
        duckLinear = new LinearInterpolator(0, duckStartPower, duckRunTime, duckEndPower);
        duckProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, 0),
                new MotionState(1000, 0, 0),
                1000,
                2,
                100
        );


        mainMap = hMap;

        duckSpinnerCoeff = new PIDCoefficients(0,0,0);

        duckTimer = new ElapsedTime();

        odoR = mainMap.get(ServoImplEx.class, "odoR");
        odoL = mainMap.get(ServoImplEx.class, "odoL");
        odoM = mainMap.get(ServoImplEx.class, "odoM");
        intakeLockServo = mainMap.get(ServoImplEx.class, "intakeLockServo");

        intakeDetect = mainMap.get(DistanceSensor.class, "intakeDetect");

        driveFL = mainMap.get(DcMotor.class, "driveFL");
        driveFR = mainMap.get(DcMotor.class, "driveFR");
        driveBL = mainMap.get(DcMotor.class, "driveBL");
        driveBR = mainMap.get(DcMotor.class, "driveBR");

        armL = mainMap.get(DcMotorEx.class, "armL");
        armR = mainMap.get(DcMotorEx.class, "armR");
        intakeMotor = mainMap.get(DcMotorEx.class, "intakeMotor");
        duckSpinMotor = mainMap.get(DcMotorEx.class, "duckSpinner");

        odometry = new TrackingWheelLocalizer2021_Sus(hMap);
        driveRoadRunner = new Drive_Mecanum_RoadRunner_Sus(hMap, odometry);

        driveFR.setDirection(DcMotor.Direction.REVERSE);
        driveBR.setDirection(DcMotor.Direction.REVERSE);

        pidB = new PIDController(CoeffB);
        pidS = new PIDController(CoeffS);

        duckSpinnerFF = new PIDFController(duckSpinnerCoeff);

        //intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoR.setPwmRange(new PwmControl.PwmRange(500, 2500));


        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckSpinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odoR.setPosition(2400);

       // vision = new Vision_OpenCV_ExternalCam(hMap, "Webcam 1", new FreightFrenzyPipeline());
    }

    String getName(){
        return robotName;
    }

    private double armDegreesPerTic;
    public double getBigRot(){
        return -((armL.getCurrentPosition()-armR.getCurrentPosition())/2.0)*armDegreesPerTic;
    }
    public double getSmallRot(){
        return ((armL.getCurrentPosition()+armR.getCurrentPosition())/2.0)*armDegreesPerTic;
    }

    public void setArmPowers(double bigRotPower, double smallRotPower){
        armLPower = -smallRotPower - bigRotPower;
        armRPower = -smallRotPower + bigRotPower;

        armL.setPower(Range.clip(armLPower, -armMaxSpeed, armMaxSpeed));
        armR.setPower(Range.clip(armRPower, -armMaxSpeed, armMaxSpeed));
    }
    public void updateArm(Telemetry telem){
        if(outakeState != outakeStateList.INTAKE && outakeState != outakeStateList.STOW){

        }

        if(outakeState != outakeStateList.INTAKE && outakeState != outakeStateList.STOW && !isThereFreight()){
            stow();
        }
        if(outakeState == outakeStateList.STOW && bigRotTarget != intakeStowBigRot)
            stow();

        outakeByInput();

        if (outakeBusy){
            if (Math.abs(getBigRot() - bigRotTarget) < armAtTargetMargin && Math.abs(getSmallRot() - smallRotTarget) < armAtTargetMargin){
                outakeBusy = false;
            }
        }

        if( outakeState != outakeStateList.INTAKE && outakeBusy ){
            intakeMotor.setPower(0);
        }


        bigRotOutput = pidB.getOutput(getBigRot(), bigRotTarget);
        smallRotOutput = pidS.getOutput(getSmallRot(), smallRotTarget);
        telem.addData("bigRotPID", pidB.getOutput(getBigRot(), bigRotTarget));
        telem.addData("smallRotPID", pidS.getOutput(getSmallRot(), smallRotTarget));


        setArmPowers(bigRotOutput, smallRotOutput);
    }

    public void outakeByInput(){
        if(isThereFreight()){
            if (outakeStateInput == outakeStateList.OUTAKE_UPPER){
                outakeUpper();
            }
            else if(outakeStateInput == outakeStateList.OUTAKE_MIDDLE){
                outakeMiddle();
            }
            else if(outakeStateInput == outakeStateList.OUTAKE_NEUTRAL_LEFT){
                outakeNeutralLeft();
            }
            else if(outakeStateInput == outakeStateList.OUTAKE_NEUTRAL_RIGHT){
                outakeNeutralRight();
            }
            else if(outakeStateInput == outakeStateList.OUTAKE_LOWER){
                outakeLower();
            }
        }
    }


    public void moveArm(double bigRot, double smallRot){
        smallRotTarget = sideFlip * smallRot;

        if((outakeState != outakeStateList.INTAKE && outakeState != outakeStateList.STOW) || (Math.abs(smallRotTarget-getSmallRot()) < smallRotAtTargetMargin))
            bigRotTarget = bigRot;
    }

    private boolean isFreight = false;
    public void toggleIsFreight(){
        isFreight = !isFreight;
    }
    public boolean isThereFreight(){
        /*if (intakeDetect.getDistance(DistanceUnit.CM) <7){
            return true;
        }
        else{
            return false;
        }*/
        return isFreight;
    }

    public void startDuckSpinner(ElapsedTime timer){
        timer.reset();
    }

    public double updateDuckSpinner(double time){
        duckSpinMotor.setPower(-sideFlip * duckLinear.interpolate(time));
        return duckLinear.interpolate(time);
    }

    public void outakeUpper(){
        outakeState = outakeState.OUTAKE_UPPER;
        outakeBusy = true;
        moveArm(outakeUpperBigRot, outakeUpperSmallRot);
    }
    public void outakeMiddle(){
        outakeState = outakeState.OUTAKE_MIDDLE;
        outakeBusy = true;
        moveArm(outakeMidBigRot, outakeMidSmallRot);
    }
    public void outakeNeutralLeft(){
        outakeState = outakeState.OUTAKE_NEUTRAL_LEFT;
        outakeBusy = true;
        moveArm(outakeNeutralBigRot, outakeNeutralLeftSmallRot);

    }
    public void outakeNeutralRight(){
        outakeState = outakeState.OUTAKE_NEUTRAL_RIGHT;
        outakeBusy = true;
        moveArm(outakeNeutralBigRot, outakeNeutralRightSmallRot);

    }
    public void outakeLower(){
        outakeState = outakeState.OUTAKE_LOWER;
        outakeBusy = true;
        moveArm(outakeLowerBigRot, 0);

    }
    public void stow(){
        outakeState = outakeState.STOW;
        outakeBusy = true;
        moveArm(intakeStowBigRot, 0);

    }

    public void intake(){
        outakeState = outakeStateList.INTAKE;
        outakeBusy = true;
        moveArm(intakeBigRot,0);
        intakeMotor.setPower(intakePower);

    }

    public void eject(){
        intakeMotor.setPower(outakePower);
    }

    public void setSide(String side){
        if (side == "blue"){
            sideFlip = -1;
        }
        else{
            sideFlip = 1;
        }
    }

    public void raiseOdo() {
        odoR.setPosition(odoRRaisePos);
        odoL.setPosition(odoLRaisePos);
        odoM.setPosition(odoMRaisePos);
    }

    public void unlockIntake(){
        intakeLockServo.setPosition(intakeServoUnlockPos);
    }
}
