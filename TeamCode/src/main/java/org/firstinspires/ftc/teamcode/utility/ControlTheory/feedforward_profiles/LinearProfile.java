package org.firstinspires.ftc.teamcode.utility.ControlTheory.feedforward_profiles;

import org.firstinspires.ftc.teamcode.utility.Math.AdvMath;
import org.firstinspires.ftc.teamcode.utility.Math.LinearInterpolator;

import java.util.ArrayList;

public class LinearProfile extends FeedforwardProfile {
    LinearInterpolator pointInterp; // the thing that interpolates between the setpoints
    double K1, K2; // the multiplier for the setpoint (ex. number you multiply setpoint velocity by to create the motor power required to reach that setpoint)


    public LinearProfile(double startSetpoint, double endSetpoint, double transversalTime, double K1, double K2, ProfileEndBehavior endBehavior){
        super(transversalTime, endBehavior); // call the FeedforwardProfile constructor to setup transversal time variables
        this.pointInterp = new LinearInterpolator(0, startSetpoint, transversalTime, endSetpoint); // setup the basic linear interpolator

        this.K1 = K1;
        this.K2 = K2;
    }
    public LinearProfile(double startPoint, double endPoint, double transversialTime, double K1, double K2){
        this(startPoint, endPoint, transversialTime, K1, K2, ProfileEndBehavior.MAINTAIN);
    }
    public LinearProfile(double startPoint, double endPoint, double transversialTime, ProfileEndBehavior endBehavior){
        this(startPoint, endPoint, transversialTime, 1.0, 1.0, endBehavior);  // if no value passed, use a gain of 1 to not modify values
    }
    public LinearProfile(double startPoint, double endPoint, double transversialTime){
        this(startPoint, endPoint, transversialTime, 1.0, 1.0, ProfileEndBehavior.MAINTAIN);  // if no value passed, use a gain of 1 to not modify values
    }


    @Override
    public double getPrimaryProfileTarget(double timestep){
        return K1 * pointInterp.interpolate( timestep ); // then use that to figure out what our current point should be between the two points
    }
    @Override
    public double getSecondaryProfileTarget(double timestep){
        return K2 * pointInterp.getSlope();
    }
}
