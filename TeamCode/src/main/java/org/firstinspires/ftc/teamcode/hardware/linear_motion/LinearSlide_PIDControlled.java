package org.firstinspires.ftc.teamcode.hardware.linear_motion;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.ar.pl.DebugLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.PIDController;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.VeloLimiter;
import org.firstinspires.ftc.teamcode.utility.Math.MultislopeLinearInterpolator;
import org.firstinspires.ftc.teamcode.utility.VoltageCompensator.VoltageCompensator;


public class LinearSlide_PIDControlled {
    private DcMotor slideMotor;
    private PIDController PID;
    private VeloLimiter limiter;

    private MultislopeLinearInterpolator distanceMaintainInterp;


    public static double DEFAULT_INCH_TARGET_MARGIN = 0.25;
    public static double DEFAULT_INCHES_PER_MOTOR_TIC = 0.0132;
    private double inchesPerMotorTic = DEFAULT_INCHES_PER_MOTOR_TIC;
    private double forcedTravelTime = 0;

    private double baseMotorPower = 0;

    private double targetPos = 0;
    private double lastTargetPos = 0;


    public LinearSlide_PIDControlled(DcMotor slideMotor, PIDController PID){
        this.slideMotor = slideMotor;
        if(slideMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        this.PID = PID;

        this.limiter = new VeloLimiter(1, 0, false);
    }
    public LinearSlide_PIDControlled(DcMotor slideMotor, PIDController PID, VeloLimiter limiter){
        this.slideMotor = slideMotor;
        if(slideMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        this.PID = PID;
        this.limiter = limiter;

        limiter.setLimitDeceleration( false );
    }

    public void runToInternalTarget(){
        runToGivenPosition(targetPos);
    }
    public void runToInternalTarget(Telemetry debugLog){
        runToGivenPosition(targetPos, debugLog);
    }
    public void runToGivenPosition(double targetPosition){
        double currentPos = getInchPosition();
        if(targetPosition != lastTargetPos)
            updateMaxVeloToMeetForcedTime(targetPosition);

        double outputPower = PID.getOutput(currentPos, targetPosition);

        //outputPower = VoltageCompensator.compensatePower( limiter.getLimitedVelo( outputPower ) + baseMotorPower );
        outputPower = limiter.getLimitedVelo( outputPower ) + baseMotorPower;


        slideMotor.setPower(outputPower);

        lastTargetPos = targetPosition;
    }
    public void runToGivenPosition(double targetPosition, Telemetry debugLog){
        double currentPos = getInchPosition();
        debugLog.addData("getinches", "finished");
        debugLog.update();

        if(targetPosition != lastTargetPos)
            updateMaxVeloToMeetForcedTime(targetPosition);
        debugLog.addData("updateMaxVelo", "finished");
        debugLog.update();

        double outputPower = PID.getOutput(currentPos, targetPosition);
        debugLog.addData("getOutputPID", "finished");
        debugLog.update();

        outputPower = limiter.getLimitedVelo( outputPower ) + baseMotorPower;
        //outputPower = VoltageCompensator.compensatePower( limiter.getLimitedVelo( outputPower ) + baseMotorPower );
        //debugLog.addData("getOutputPower", "finished");
        //debugLog.update();

        slideMotor.setPower(outputPower);
        debugLog.addData("slideMotorSetPower", "finished");
        debugLog.update();

        lastTargetPos = targetPosition;
        debugLog.addData("getLastTargetPos", "finished");
        debugLog.update();
    }

    public void setTargetPosition(double targetPosition){
        this.targetPos = targetPosition;

        updateMaxVeloToMeetForcedTime(targetPosition);
    }

    public double getInchPosition(){
        double position = slideMotor.getCurrentPosition() * inchesPerMotorTic;

        if( distanceMaintainInterp != null )
            position *= distanceMaintainInterp.interpolate( position );

        return position;
    }
    public double getErrorToTarget(){
        return targetPos - getInchPosition();
    }
    public double getTargetPos(){return targetPos;}


    public boolean isAtTarget(){
        return Math.abs(getInchPosition() - targetPos) <= DEFAULT_INCH_TARGET_MARGIN;
    }
    public VeloLimiter getVeloLimiter(){return limiter;}

    public void setVeloLimiter(VeloLimiter veloLimiter){
        this.limiter = veloLimiter;
    }
    public void setForcedTravelTime(double travelTime){
        this.forcedTravelTime = travelTime;
    }

    private void updateMaxVeloToMeetForcedTime(double targetPosition){
        if( forcedTravelTime != 0 ){
            double distanceToTarget = Math.abs(targetPosition - getInchPosition());

            limiter.setMaxSpeed( distanceToTarget/forcedTravelTime );
        }

    }

    public void setBaseMotorPower(double baseMotorPower){
        this.baseMotorPower = baseMotorPower;
    }


    public void setDistanceMaintainInterp(MultislopeLinearInterpolator distanceMaintainInterp){
        this.distanceMaintainInterp = distanceMaintainInterp;
    }
}
