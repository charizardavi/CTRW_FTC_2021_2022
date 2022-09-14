package org.firstinspires.ftc.teamcode.utility.ControlTheory;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.ControlTheory.coefficient_wrappers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.filters.LowPassFilter;
import org.firstinspires.ftc.teamcode.utility.Math.AdvMath;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.Range2d;


@Config
public class PIDController {
    public static final Range2d DEFAULT_I_ACTIVE_RANGE = new Range2d(-Double.MAX_VALUE, Double.MAX_VALUE); // the default range that a PID is active
    public static final Range2d DEFAULT_I_WINDUP_BOUNDS = new Range2d(-Double.MAX_VALUE, Double.MAX_VALUE); // the default limits to the integral values
    public static double DEFAULT_MIN_MOTOR_MOVE_POWER = 0.0;


    ElapsedTime localRuntime;

    public PIDCoefficients coefficients;
    public Range2d iActiveErrorRange;
    public Range2d iWindupBounds;
    public LowPassFilter derivativeFilter;

    private double lastError = 0.0;
    private double lastRuntime = 0.0;
    private double integral = 0.0;
    private double lastPosition = 0.0;
    private double lastTarget = 0.0;
    private double minMotorMovePower = DEFAULT_MIN_MOTOR_MOVE_POWER;


    public PIDController(PIDCoefficients coefficients, Range2d iActiveErrorRange, Range2d iWindupBounds, LowPassFilter derivativeSmoothingFilter){
        this.coefficients = coefficients;
        this.iActiveErrorRange = iActiveErrorRange;
        this.iWindupBounds = iWindupBounds;
        this.derivativeFilter = derivativeSmoothingFilter;

        localRuntime = new ElapsedTime();
    }
    public PIDController(PIDCoefficients coefficients, Range2d iActiveErrorRange, Range2d iWindupBounds){
        this(coefficients, iActiveErrorRange, iWindupBounds, null);
    }
    public PIDController(PIDCoefficients coefficients, Range2d iActiveErrorRange){
        this(coefficients, iActiveErrorRange, DEFAULT_I_WINDUP_BOUNDS, null);
    }
    public PIDController(PIDCoefficients coefficients, Range2d iActiveErrorRange, LowPassFilter derivativeSmoothingFilter){
        this(coefficients, iActiveErrorRange, DEFAULT_I_WINDUP_BOUNDS, derivativeSmoothingFilter);
    }
    public PIDController(PIDCoefficients coefficients, LowPassFilter derivativeSmoothingFilter){
        this(coefficients, DEFAULT_I_ACTIVE_RANGE, DEFAULT_I_WINDUP_BOUNDS, derivativeSmoothingFilter);
    }
    public PIDController(PIDCoefficients coefficients){
        this(coefficients, DEFAULT_I_ACTIVE_RANGE, DEFAULT_I_WINDUP_BOUNDS, null);
    }



    public double getOutput(double current, double target){ // ACCEPTS RADIANS
        double error = target - current; // the error is the difference between where we want to be and where we are right now
        double timeDifference = localRuntime.milliseconds() - lastRuntime; // timeDifference is the time since the last runtime


        // multiplied by the timeDifference to prevent wild variation in how much it is increase if cycle time increases/decreases for some reason
        if( AdvMath.isNumInRange(error, iActiveErrorRange) ){ // (only accumulate integral while inside the active error range)
            integral += error * timeDifference; // the integral is the sum of all error over time, and is used to push past unexpected resistance (as if the arm stays in a single position away from the set position for too long, it builds up over time and pushes past the resistance)
        }
        else
            integral = 0;

        if( integral > iWindupBounds.getMax() ){ // if the integral is greater than the upper windup bound, cap it
            integral = iWindupBounds.getMax();
        }
        else if( integral < iWindupBounds.getMin() ){ // if the integral is less than the lower windup bound, set it to the lower bound
            integral = iWindupBounds.getMin();
        }


        double dError = ((error - lastError) / timeDifference); // the rate of change of the current error, this component creates a smooth approach to the set point

        if(derivativeFilter != null){ // if our derivative low pass filter exists (we were provided one)
            dError = derivativeFilter.filter(dError);
        }

        double output = (coefficients.p * error) + (coefficients.i * integral) + (coefficients.d * dError); // multiply each term by its coefficient, then add together to get the final power


        lastError = error; // update the last error to be the current error
        lastRuntime = localRuntime.milliseconds(); // update the last runtime to be the current runtime
        lastPosition = current;
        lastTarget = target; //update the last target head to be the current target heading

        return AdvMath.minMotorMoveCompensate(minMotorMovePower, output);
    }


    public double getLastError(){
        return lastError;
    }
    public double getLastRuntime(){
        return lastRuntime;
    }
    public double getLastTarget(){
        return lastTarget;
    }
    public double getLastPosition(){
        return lastPosition;
    }
    public double getIntegral(){
        return integral;
    }

    public void setMinMotorMovePower(double power){
        this.minMotorMovePower = power;
    }

    public PIDController getCopy(){
        PIDController copy = new PIDController(this.coefficients);

        copy.iActiveErrorRange = this.iActiveErrorRange;
        copy.iWindupBounds = this.iWindupBounds;
        copy.derivativeFilter = this.derivativeFilter;

        return copy;
    }
}
