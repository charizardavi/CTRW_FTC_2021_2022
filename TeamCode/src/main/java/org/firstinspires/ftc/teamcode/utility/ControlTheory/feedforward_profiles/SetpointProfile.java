package org.firstinspires.ftc.teamcode.utility.ControlTheory.feedforward_profiles;

public class SetpointProfile extends FeedforwardProfile {
    double setpoint; // the constant target value (ex. motor velocity)
    double K; // the multiplier for the setpoint (ex. number you multiply setpoint velocity by to create the motor power required to reach that setpoint)


    public SetpointProfile(double setpoint){
        super(1000, ProfileEndBehavior.MAINTAIN);
        this.setpoint = setpoint;
        this.K = 1; // if no value passed, use a gain of 1 to not modify values
    }
    public SetpointProfile(double setpoint, double K){
        super(1000, ProfileEndBehavior.MAINTAIN);
        this.setpoint = setpoint;
        this.K = K;
    }
    public SetpointProfile(double setpoint, double K, double profileDuration){
        super(profileDuration, ProfileEndBehavior.MAINTAIN);
        this.setpoint = setpoint;
        this.K = K;
    }

    @Override
    public double getPrimaryProfileTarget(double timestep){
        return K * setpoint;
    }
}
