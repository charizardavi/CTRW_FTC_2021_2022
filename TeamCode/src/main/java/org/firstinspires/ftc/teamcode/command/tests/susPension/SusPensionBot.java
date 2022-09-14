package org.firstinspires.ftc.teamcode.command.tests.susPension;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.Drive_Mecanum;
import org.firstinspires.ftc.teamcode.hardware.mecanum_drives.RoadRunner.Drive_Mecanum_RoadRunner_Sus;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.FeedforwardController;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.PIDController;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.feedforward_profiles.TrapezoidalProfile;
import org.firstinspires.ftc.teamcode.utility.Math.LinearInterpolator;
import org.firstinspires.ftc.teamcode.utility.Odometry.TrackingWheelLocalizer2021_Sus;
import org.firstinspires.ftc.teamcode.utility.OpenCV.FreightFrenzyPipeline;
import org.firstinspires.ftc.teamcode.utility.OpenCV.Vision_OpenCV_ExternalCam;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.TimestampedValue;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.Vector2;


@Config
public class SusPensionBot {

    public enum OutakeStateList {
        INTAKE,
        STOW,
        OUTAKE_STOW,
        OUTAKE_UPPER_RIGHT,
        OUTAKE_UPPER_LEFT,
        OUTAKE_UPPER_CENTER,
        OUTAKE_MIDDLE_RIGHT,
        OUTAKE_MIDDLE_LEFT,
        OUTAKE_MIDDLE_CENTER,
        OUTAKE_NEUTRAL_LEFT,
        OUTAKE_NEUTRAL_RIGHT,
        OUTAKE_LOWER
    }

    public enum CapStateList {
        CAP_PLACE,
        CAP_COLLECT,
        CAP_UP,
        IDLE
    }

    public OutakeStateList outakeStateInput = OutakeStateList.OUTAKE_UPPER_CENTER;

    public OutakeStateList outakeState = OutakeStateList.INTAKE;

    public CapStateList capState = CapStateList.IDLE;

    String robotName = "susPension";

    public DcMotor driveFL = null; //port 0 on control hub
    public DcMotor driveFR = null; //port 1 on control hub
    public DcMotor driveBL = null; //port 2 on control hub
    public DcMotor driveBR = null; //port 3 on control hub

    public DcMotorEx armR; //port 0 on expansion hub
    public DcMotorEx armL; //port 2 on expansion hub

    public double intakeSlow = .2;

    public DcMotorEx intakeMotor; //port 1 on expansion hub

    public DcMotorEx duckSpinMotor;

    public PIDController pidS;
    public PIDController pidB;
    public com.acmerobotics.roadrunner.control.PIDFController duckSpinnerFF;

    public ColorSensor intakeDetect;
    //public TouchSensor armZeroDetect;

    public boolean intakeMotorIsBusy = false;

    public boolean ejectDone = false;

    public boolean outakeIsDone = true;

    public ServoImplEx odoR;
    public ServoImplEx odoL;
    public ServoImplEx odoM;
    public Servo intakeLockServo;

    public Servo capServo;

    public double ejectTime = 500;

    public static double capUpPos = 0.5;
    public static double capPlacePos = 0.54;
    public static double capCollectPos = 0.86;

    public boolean outakeBusy = false;

    public double sideFlip = -1.0;

    public static org.firstinspires.ftc.teamcode.utility.ControlTheory.coefficient_wrappers.PIDCoefficients CoeffS = new org.firstinspires.ftc.teamcode.utility.ControlTheory.coefficient_wrappers.PIDCoefficients(-.06,0,0);
    public static org.firstinspires.ftc.teamcode.utility.ControlTheory.coefficient_wrappers.PIDCoefficients CoeffB = new org.firstinspires.ftc.teamcode.utility.ControlTheory.coefficient_wrappers.PIDCoefficients(.06,0,0);

    public ElapsedTime duckTimer;
    public ElapsedTime capTimer;

    public PIDCoefficients duckSpinnerCoeff;
    public HardwareMap mainMap;
    public Localizer odometry;
    public Drive_Mecanum drive;
    public Drive_Mecanum_RoadRunner_Sus driveRoadRunner;
    public MotionProfile duckProfile;
    public LinearInterpolator duckLinear;
    public Vision_OpenCV_ExternalCam vision;

    public LinearInterpolator capCollectUpLinear = new LinearInterpolator(new Vector2(0, capCollectPos), new Vector2(2000, capUpPos));
    public LinearInterpolator capUpPlaceLinear = new LinearInterpolator(new Vector2(0, capUpPos), new Vector2(300, capPlacePos));
    public LinearInterpolator capPlaceCollectLinear = new LinearInterpolator(new Vector2(0, capPlacePos), new Vector2(1700, capCollectPos));


    public static double armMaxPower = 1;

    public static Vector2 outakeUpperCenterArmPos = new Vector2(170,0);
    public static Vector2 outakeUpperRightArmPos = new Vector2(162,45);
    public static Vector2 outakeUpperLeftArmPos = new Vector2(162,-45);
    public static Vector2 outakeMidCenterArmPos = new Vector2(190, 0);
    public static Vector2 outakeMidRightArmPos = new Vector2(190, 45);
    public static Vector2 outakeMidLeftArmPos = new Vector2(190, -45);
    public static Vector2 outakeNeutralRightArmPos = new Vector2(225, 78);
    public static Vector2 outakeNeutralLeftArmPos = new Vector2(225, -78);
    public static Vector2 outakeLowerArmPos = new Vector2(210, 0);
    public static Vector2 intakeStowArmPos = new Vector2(15,0);
    public static Vector2 intakeArmPos = new Vector2(0,0);
    public static Vector2 outakeStowPos = new Vector2(90,0);



    public static double smallRotAtTargetMargin = 25;

    public static double armAtTargetMargin = 5;


    public static double odoLRaisePos = .7;
    public static double odoRRaisePos = .3;
    public static double odoMRaisePos = .3;

    public static TimestampedValue duckSpinStartSet = new TimestampedValue(0, 0.1);
    public static TimestampedValue duckSpinEndSet = new TimestampedValue(2000, 0.4);

    public static double intakeServoUnlockPos = 0.75;

    public static double outakeTime = 100;

    public static double armMotorTicsPerRev = 604.8;
    public static double armGearRatio = 1.0/6.0;

    public double smallRotTarget = 0;
    public double bigRotTarget = 0;
    public double lastSmallRotTarget = 0;
    public double lastBigRotTarget = 0;

    public double intakePower = 1;
    public double outakePower = -.25;

    public static double bigPosK = 1;
    public static double bigVeloK = .3;
    public static double bigAccelK = 0;
    public static double teleOpBigMaxDegPerSec = 250;
    public static double teleOpBigMaxAccelPerSec = 150;
    public static double autoBigMaxDegPerSec = 140;
    public static double autoBigMaxAccelPerSec = 75;

    public double bigMaxDegPerSec;
    public double bigMaxAccelPerSec;

    public static double smallPosK = 1;
    public static double smallVeloK = 0;
    public static double smallAccelK = 0;
    public static double smallMaxDegPerSec = 200;
    public static double smallMaxAccelPerSec = 150;

    public static double ballEjectPower = -.25;
    public static double yellowEjectPower = -.265;


    public double bigRotOutput;
    public double smallRotOutput;

    public double armLPower;
    public double armRPower;

    public FeedforwardController bigRotFFController;
    public FeedforwardController smallRotFFController;


    private CapStateList lastCapState = CapStateList.CAP_PLACE;




    public SusPensionBot(HardwareMap hMap){
        this(hMap, false);
    }
    public SusPensionBot(HardwareMap hMap, boolean auto){
        if (auto){
            bigMaxDegPerSec = autoBigMaxDegPerSec;
            bigMaxAccelPerSec = autoBigMaxAccelPerSec;
        }
        else{
            bigMaxDegPerSec = teleOpBigMaxDegPerSec;
            bigMaxAccelPerSec = teleOpBigMaxAccelPerSec;
        }
        initMap(hMap);

        drive = new Drive_Mecanum(driveFL, driveFR, driveBL, driveBR);

        armDegreesPerTic = (1/armMotorTicsPerRev) * armGearRatio * 360;
    }
    public SusPensionBot(HardwareMap hMap, String robotName){
        this(hMap);

        this.robotName = robotName;
    }



    private void initMap(HardwareMap hMap) {
        duckLinear = new LinearInterpolator(duckSpinStartSet, duckSpinEndSet);
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
        capTimer = new ElapsedTime();

        odoR = mainMap.get(ServoImplEx.class, "odoR");
        odoL = mainMap.get(ServoImplEx.class, "odoL");
        odoM = mainMap.get(ServoImplEx.class, "odoM");
        intakeLockServo = mainMap.get(ServoImplEx.class, "intakeLockServo");

        intakeDetect = mainMap.get(ColorSensor.class, "intakeDetect");
        //armZeroDetect = mainMap.get(TouchSensor.class, "armZeroDetect");

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

        capServo = mainMap.get(Servo.class, "capServo");

        odoR.setPosition(2400);

       vision = new Vision_OpenCV_ExternalCam(hMap, "Webcam 1", new FreightFrenzyPipeline());

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckSpinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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


        double maxVal = Math.max(Math.abs(armLPower), Math.abs(armRPower));

        if(maxVal > 1){
            armLPower = armLPower / maxVal;
            armRPower = armRPower / maxVal;
        }

        armL.setPower(Range.clip(armLPower, -armMaxPower, armMaxPower));
        armR.setPower(Range.clip(armRPower, -armMaxPower, armMaxPower));
    }
    public void updateArm(Telemetry telem, ElapsedTime outakeTimer){
//        if(outakeState != outakeStateList.INTAKE && outakeState != outakeStateList.STOW && !isThereFreight()){
//            stow();
//        }
//        if(outakeState == outakeStateList.STOW && bigRotTarget != intakeStowArmPos.x)
//            stow();
//
//        outakeByInput();
//
//        if (outakeBusy){
//            if (Math.abs(getBigRot() - bigRotTarget) < armAtTargetMargin && Math.abs(getSmallRot() - smallRotTarget) < armAtTargetMargin){
//                outakeBusy = false;
//            }
//        }
//
//        if( outakeState != outakeStateList.INTAKE && outakeBusy ){
//            intakeMotor.setPower(0);
//        }

        if (isThereFreight() && outakeState == OutakeStateList.INTAKE){
//            if (outakeIsDone){
//                outakeTimer.reset();
//                outakeIsDone = false;
//            }
//            else {
//                if (outakeTimer.milliseconds() >= outakeTime){
//                    outakeIsDone = true;
//                }
//            }
            outakeState = outakeStateInput;
            intakeMotor.setPower(intakeSlow);
        }

        if (outakeState != OutakeStateList.OUTAKE_STOW && outakeState != OutakeStateList.STOW && outakeState != OutakeStateList.INTAKE && !isThereFreight()){
            intakeMotor.setPower(0.0);
            stow();
        }
        if (outakeState == OutakeStateList.STOW){
            stow();
        }
        if (outakeState == OutakeStateList.INTAKE){
            intake();
        }


        outakeByInput();



        if( bigRotTarget != lastBigRotTarget ){
            TrapezoidalProfile bigProfile = new TrapezoidalProfile( getBigRot(), bigRotTarget, bigMaxDegPerSec, bigMaxAccelPerSec, bigPosK, bigVeloK, bigAccelK );

            bigRotFFController = new FeedforwardController( bigProfile );
            bigRotFFController.resetAndStartProfile();

            lastBigRotTarget = bigRotTarget;
        }
        if( smallRotTarget != lastSmallRotTarget ){
            TrapezoidalProfile smallProfile = new TrapezoidalProfile( getSmallRot(), smallRotTarget, smallMaxDegPerSec, smallMaxAccelPerSec, smallPosK, smallVeloK, smallAccelK );

            smallRotFFController = new FeedforwardController( smallProfile );
            smallRotFFController.resetAndStartProfile();

            lastSmallRotTarget = smallRotTarget;
        }




        if( bigRotFFController == null ){
            bigRotOutput = pidB.getOutput(getBigRot(), bigRotTarget);
        }
        else{ // use motion profile
            bigRotOutput = pidB.getOutput(getBigRot(), bigRotFFController.getPrimaryOutput());
            bigRotOutput += bigRotFFController.getSecondaryOutput();
            bigRotOutput += bigRotFFController.getTertiaryOutput();

            telem.addData("bigRotProfileDuration", bigRotFFController.getProfile().getProfileDuration());
            telem.addData("bigRotFFPos", bigRotFFController.getPrimaryOutput());
            telem.addData("bigRotFFPower", bigRotFFController.getSecondaryOutput());
            telem.addData("bigProfileTime", bigRotFFController.getProfileTime());
            telem.addData("bigProfileStart", ((TrapezoidalProfile)bigRotFFController.getProfile()).getStartPos());
            telem.addData("bigProfileEnd", ((TrapezoidalProfile)bigRotFFController.getProfile()).getEndPos());
        }

        if( smallRotFFController == null ){
            smallRotOutput = pidS.getOutput(getSmallRot(), smallRotTarget);
        }
        else{ // use motion profile
            smallRotOutput = pidS.getOutput(getSmallRot(), smallRotFFController.getPrimaryOutput());
            smallRotOutput += smallRotFFController.getSecondaryOutput();
            smallRotOutput += smallRotFFController.getTertiaryOutput();

            telem.addData("smallRotFFPos", smallRotFFController.getSecondaryOutput());
            telem.addData("smallRotFFPower", smallRotFFController.getSecondaryOutput());
            telem.addData("smallProfileTime", smallRotFFController.getProfileTime());
        }

        telem.addData("bigRotPID", pidB.getOutput(getBigRot(), bigRotTarget));
        telem.addData("smallRotPID", pidS.getOutput(getSmallRot(), smallRotTarget));
        telem.addData("bigrottarget", bigRotTarget);
        telem.addData("smallrottarget", smallRotTarget);
        telem.addData("bigPos", getBigRot());
        telem.addData("smallPos", getSmallRot());



        setArmPowers(bigRotOutput, smallRotOutput);


        if (outakeBusy){
            if (Math.abs(getBigRot() - bigRotTarget) < armAtTargetMargin && Math.abs(getSmallRot() - smallRotTarget) < armAtTargetMargin){
                outakeBusy = false;
            }
        }

//        if (outakeState == outakeStateList.STOW && !outakeBusy && !armZeroDetect.isPressed()){
//            armR.setPower(-.05);
//            armL.setPower(-.05);
//            while (!armZeroDetect.isPressed()){
//                telem.addData("status", "moving arm to zero pos");
//                telem.update();
//            }
//            armR.setPower(0);
//            armL.setPower(0);
//        }

    }

    public void outakeByInput(){
        if(isThereFreight() /*outakeState == outakeStateList.OUTAKE_STOW*/ && !outakeBusy ){
            if (outakeStateInput == OutakeStateList.OUTAKE_UPPER_CENTER){
                outakeUpperCenter();
            }
            else if(outakeStateInput == OutakeStateList.OUTAKE_UPPER_RIGHT){
                outakeUpperRight();
            }
            else if(outakeStateInput == OutakeStateList.OUTAKE_UPPER_LEFT){
                outakeUpperLeft();
            }
            else if(outakeStateInput == OutakeStateList.OUTAKE_MIDDLE_CENTER){
                outakeMiddleCenter();
            }
            else if(outakeStateInput == OutakeStateList.OUTAKE_MIDDLE_RIGHT){
                outakeMiddleRight();
            }
            else if(outakeStateInput == OutakeStateList.OUTAKE_MIDDLE_LEFT){
                outakeMiddleLeft();
            }
            else if(outakeStateInput == OutakeStateList.OUTAKE_NEUTRAL_LEFT){
                outakeNeutralLeft();
            }
            else if(outakeStateInput == OutakeStateList.OUTAKE_NEUTRAL_RIGHT){
                outakeNeutralRight();
            }
            else if(outakeStateInput == OutakeStateList.OUTAKE_LOWER){
                outakeLower();
            }
        }
        else if(outakeStateInput == OutakeStateList.STOW){
            stow();
        }
    }


    public void moveArm(double bigRot, double smallRot){
        smallRotTarget = sideFlip * smallRot;

        //if((outakeState != OutakeStateList.INTAKE && outakeState != OutakeStateList.STOW) || (Math.abs(smallRotTarget-getSmallRot()) < smallRotAtTargetMargin))
        bigRotTarget = bigRot;
    }

    public void forceFreightDetect(){
        ejectDone = false;
        outakeState = OutakeStateList.STOW;
    }
    public boolean isThereFreight(){
        if (intakeDetect.red() > 2000 && intakeDetect.green() > 2000 && intakeDetect.blue()<1200){//yellow
            ejectDone = false;
            outakePower = yellowEjectPower;
            return true;
        }
        if (intakeDetect.red() > 8000 && intakeDetect.green() > 8000 && intakeDetect.blue()<5000){//other yellow
            ejectDone = false;
            outakePower = yellowEjectPower;
            return true;

        }
        else if (intakeDetect.red() >9000 && intakeDetect.green() > 9000 && intakeDetect.blue()>9000){//white
            ejectDone = false;
            outakePower = ballEjectPower;
            return true;
        }
        else if (!ejectDone && outakeState != OutakeStateList.INTAKE){
            return true;
        }
        else{
            return false;
        }
    }


    public void updateDuckSpinner(boolean spinLeft, boolean spinRight){
        if (spinLeft){
            double spinPower = duckLinear.interpolate(duckTimer.milliseconds());
            if(Math.abs(spinPower) > Math.abs(duckSpinEndSet.value))
                spinPower = 0;
            duckSpinMotor.setPower(spinPower);
        }
        else if (spinRight){
            double spinPower = -duckLinear.interpolate(duckTimer.milliseconds());
            if(Math.abs(spinPower) > Math.abs(duckSpinEndSet.value))
                spinPower = 0;
            duckSpinMotor.setPower(spinPower);

        }
        else{
            duckTimer.reset();
            duckSpinMotor.setPower(0.0);
        }
    }

    public void outakeUpperCenter(){
        outakeState = OutakeStateList.OUTAKE_UPPER_CENTER;
        outakeBusy = true;
        moveArm(outakeUpperCenterArmPos.x, outakeUpperCenterArmPos.y);
    }
    public void outakeUpperRight(){
        outakeState = OutakeStateList.OUTAKE_UPPER_RIGHT;
        outakeBusy = true;
        moveArm(outakeUpperRightArmPos.x, outakeUpperRightArmPos.y);
    }
    public void outakeUpperLeft(){
        outakeState = OutakeStateList.OUTAKE_UPPER_LEFT;
        outakeBusy = true;
        moveArm(outakeUpperLeftArmPos.x, outakeUpperLeftArmPos.y);
    }
    public void outakeMiddleCenter(){
        outakeState = OutakeStateList.OUTAKE_MIDDLE_CENTER;
        outakeBusy = true;
        moveArm(outakeMidCenterArmPos.x, outakeMidCenterArmPos.y);
    }
    public void outakeMiddleRight(){
        outakeState = OutakeStateList.OUTAKE_MIDDLE_RIGHT;
        outakeBusy = true;
        moveArm(outakeMidRightArmPos.x, outakeMidRightArmPos.y);
    }
    public void outakeMiddleLeft(){
        outakeState = OutakeStateList.OUTAKE_MIDDLE_LEFT;
        outakeBusy = true;
        moveArm(outakeMidLeftArmPos.x, outakeMidLeftArmPos.y);
    }
    public void outakeNeutralLeft(){
        outakeState = OutakeStateList.OUTAKE_NEUTRAL_LEFT;
        outakeBusy = true;
        moveArm(outakeNeutralLeftArmPos.x, outakeNeutralLeftArmPos.y);

    }
    public void outakeNeutralRight(){
        outakeState = OutakeStateList.OUTAKE_NEUTRAL_RIGHT;
        outakeBusy = true;
        moveArm(outakeNeutralRightArmPos.x, outakeNeutralRightArmPos.y);

    }
    public void outakeLower(){
        outakeState = OutakeStateList.OUTAKE_LOWER;
        outakeBusy = true;
        moveArm(outakeLowerArmPos.x, outakeLowerArmPos.y);

    }
    public void stow(){
        outakeState = OutakeStateList.STOW;
        outakeBusy = true;
        moveArm(intakeStowArmPos.x, intakeStowArmPos.y);
    }

    public void outakeStow(){
        outakeState = OutakeStateList.OUTAKE_STOW;
        outakeBusy = true;
        moveArm(outakeStowPos.x, outakeStowPos.y);
    }

    public void intake(){
        outakeState = OutakeStateList.INTAKE;
        outakeBusy = true;
        moveArm(intakeArmPos.x,intakeArmPos.y);
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



    public void capCycle(){
        if(lastCapState == CapStateList.CAP_PLACE){
            capState = CapStateList.CAP_COLLECT;
        }
        else if(lastCapState == CapStateList.CAP_COLLECT){
            capState = CapStateList.CAP_UP;
        }
        else if(lastCapState == CapStateList.CAP_PLACE){
            capState = CapStateList.CAP_PLACE;
        }
    }

    public boolean capIsRunning = false;

    public void updateCap(){
        if (capState == CapStateList.CAP_COLLECT && !capIsRunning){
            capTimer.reset();
            capIsRunning = true;
            capServo.setPosition(capPlaceCollectLinear.interpolate(capTimer.milliseconds()));
        }
        else if (capState == CapStateList.CAP_COLLECT && capIsRunning){
            capServo.setPosition(capPlaceCollectLinear.interpolate(capTimer.milliseconds()));
            if (capPlaceCollectLinear.interpolate(capTimer.milliseconds()) <=capCollectPos){
                capIsRunning = false;
                lastCapState = CapStateList.CAP_COLLECT;
                capState = CapStateList.IDLE;
            }
        }
        if (capState == CapStateList.CAP_UP && !capIsRunning){
            capTimer.reset();
            capIsRunning = true;
            capServo.setPosition(capCollectUpLinear.interpolate(capTimer.milliseconds()));
        }
        else if (capState == CapStateList.CAP_UP && capIsRunning){
            capServo.setPosition(capCollectUpLinear.interpolate(capTimer.milliseconds()));
            if (capCollectUpLinear.interpolate(capTimer.milliseconds()) >=capUpPos){
                capIsRunning = false;
                lastCapState = CapStateList.CAP_UP;
                capState = CapStateList.IDLE;
            }
        }
        if (capState == CapStateList.CAP_PLACE && !capIsRunning){
            capTimer.reset();
            capIsRunning = true;
            capServo.setPosition(capUpPlaceLinear.interpolate(capTimer.milliseconds()));
        }
        else if (capState == CapStateList.CAP_UP && capIsRunning){
            capServo.setPosition(capUpPlaceLinear.interpolate(capTimer.milliseconds()));
            if (capUpPlaceLinear.interpolate(capTimer.milliseconds()) <=capPlacePos){
                capIsRunning = false;
                lastCapState = CapStateList.CAP_UP;
                capState = CapStateList.IDLE;
            }
        }
    }
}
