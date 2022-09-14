package org.firstinspires.ftc.teamcode.utility.VoltageCompensator;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.ControlTheory.filters.LowPassFilter;
import org.firstinspires.ftc.teamcode.utility.Math.MultislopeLinearInterpolator;

public abstract class VoltageCompensator {


    public static final double DEFAULT_VOLTAGE = 13.89894018; // inputting this into the voltage function would give a multiplier of 1, used when the voltage compensator isn't setup with a voltage sensor
    private static final double KEEP_CALCULATED_MULT_VOLTAGE_MARGIN = 0.2;
    private static final double FILTER_PREVIOUS_VOLTAGE_WEIGHT = 0.9999; // heavily weight the previous voltage, creates smooth voltage over time
    private static final double FILTERS_PER_MSEC = 0.1; // one filter pass every 10 msec

    private static VoltageSensor voltageSensor; // the robot voltage sensor
    private static MultislopeLinearInterpolator multiplierInterp;
    private static LowPassFilter voltageSmoothingFilter;
    private static ElapsedTime filterTimer;


    private static double lastCalcedVoltage = DEFAULT_VOLTAGE;
    private static double lastCalcedMult = 1;
    private static double lastVoltageFiltered;

    public static double tempVar = -1;


    public static void init(HardwareMap hardwareMap){
       // if( !isSetup() ){ // if not already setup, setup
        if(voltageSensor == null) {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        }

        if(voltageSmoothingFilter==null) {
            voltageSmoothingFilter = new LowPassFilter(FILTER_PREVIOUS_VOLTAGE_WEIGHT);
        }

        if (lastVoltageFiltered == 0) {
            lastVoltageFiltered = voltageSmoothingFilter.filter(voltageSensor.getVoltage()); // start the voltage smoothing sensor off with a good value
        }

        if (multiplierInterp == null) {
            multiplierInterp = new MultislopeLinearInterpolator(VoltageCompensationMultData.getInterpCompatableData());
        }

        if (filterTimer == null){
            filterTimer = new ElapsedTime();
        }
    }


    public static double getRobotVoltage(){
        if( isSetup() ){
            int filterPassCount = (int)( filterTimer.milliseconds() * FILTERS_PER_MSEC );
            tempVar = filterPassCount;
            if( filterPassCount > 0 ){
                for(int i = 0; i < filterPassCount; i++) // to keep the filter relatively consistent over time, filter a number of times proportional to the time since the last filtering
                    lastVoltageFiltered = voltageSmoothingFilter.filter( voltageSensor.getVoltage() );

                filterTimer.reset(); // then reset the clock that counts the number of filtering
            }

            return lastVoltageFiltered;
        }
        else
            return -1; // clearly show that the compensator isn't setup if someone tries to use it
    }


    public static double getMotorEfficacyForVoltage(double voltage){
        if( isSetup() ){
            double compensationMult = getMotorEfficacyForVoltage(voltage);

            if(compensationMult != 0) // if can divide by multiplier, the multiplier is the inverse of the motor efficacy
                return 1 / compensationMult;
            else
                return Double.MAX_VALUE; // if can't divide, give largest number possible, as div 0 approaches infinity
        }
        else
            return 0;
    }
    public static double getRobotMotorEfficacy(){
        if( isSetup() )
            return getMotorEfficacyForVoltage( getRobotVoltage() );
        else
            return 0;
    }

    public static double getCompensationMultForVoltage(double voltage){
        if( isSetup() ){
            if( !isWithinKeepCalcMargin( getRobotVoltage() ) ){ // if we need to recalulate the multiplier voltage
                lastCalcedVoltage = getRobotVoltage();

                lastCalcedMult = multiplierInterp.interpolate( voltage );
            }

            return lastCalcedMult;
        }
        else
            return 0;
    }
    public static double getRobotCompensationMult(){
        if( isSetup() )
            return multiplierInterp.interpolate( getRobotVoltage() );
        else
            return 0;
    }
    public static double compensatePower(double wantedRealPower){
        if( isSetup() )
            return wantedRealPower * getRobotCompensationMult();
        else
            return 0;
    }


    public static boolean isSetup(){
        return voltageSensor != null; // if voltage sensor is null, the voltage compensator isn't setup to give readings
    }


    private static boolean isWithinKeepCalcMargin(double newVoltage){
        return Math.abs(lastCalcedVoltage - newVoltage) <= KEEP_CALCULATED_MULT_VOLTAGE_MARGIN;
    }

}
