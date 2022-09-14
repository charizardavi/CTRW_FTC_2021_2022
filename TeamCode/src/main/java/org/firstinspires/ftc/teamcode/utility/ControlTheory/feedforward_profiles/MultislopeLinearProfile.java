package org.firstinspires.ftc.teamcode.utility.ControlTheory.feedforward_profiles;

import org.firstinspires.ftc.teamcode.utility.Math.LinearInterpolator;
import org.firstinspires.ftc.teamcode.utility.Math.MultislopeLinearInterpolator;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.TimestampedValue;

import java.util.ArrayList;


public class MultislopeLinearProfile extends FeedforwardProfile {
    MultislopeLinearInterpolator multislopeInterp; // the thing that interpolates between the setpoints
    double K1, K2; // the multiplier for the setpoint (ex. number you multiply setpoint velocity by to create the motor power required to reach that setpoint)


    public MultislopeLinearProfile(ArrayList<TimestampedValue> timestampedSetpoints, double primaryK, double secondaryK, ProfileEndBehavior endBehavior){
        super(timestampedSetpoints.get( timestampedSetpoints.size() ).timestamp, endBehavior); // call the FeedforwardProfile constructor to setup duration and end behavior variables (using the last timestamp in the setpoints as the duration)
        this.multislopeInterp = new MultislopeLinearInterpolator( timestampedSetpoints ); // setup the multislope linear interpolator

        this.K1 = primaryK;
        this.K2 = secondaryK;
    }
    public MultislopeLinearProfile(ArrayList<TimestampedValue> timestampedSetpoints, double primaryK, double secondaryK){
        this(timestampedSetpoints, primaryK, secondaryK, ProfileEndBehavior.MAINTAIN); // have maintain be the default end behavior
    }
    public MultislopeLinearProfile(ArrayList<TimestampedValue> timestampedSetpoints, ProfileEndBehavior endBehavior){
        this(timestampedSetpoints, 1.0, 1.0, endBehavior);  // if no value passed, use a gain of 1 to not modify values
    }
    public MultislopeLinearProfile(ArrayList<TimestampedValue> timestampedSetpoints){
        this(timestampedSetpoints, 1.0, 1.0, ProfileEndBehavior.MAINTAIN);  // if no value passed, use a gain of 1 to not modify values
    }


    @Override
    public double getPrimaryProfileTarget(double timestep){ // think position
        return K1 * multislopeInterp.interpolate( timestep ); // then use that to figure out what our current point should be between the two points
    }

    @Override
    public double getSecondaryProfileTarget(double timestep){ // think velocity
        return K2 * multislopeInterp.getSlopeAtTimestep(timestep);
    }
}
