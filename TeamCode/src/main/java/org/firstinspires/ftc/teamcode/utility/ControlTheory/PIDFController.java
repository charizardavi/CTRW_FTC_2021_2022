package org.firstinspires.ftc.teamcode.utility.ControlTheory;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.ControlTheory.feedforward_profiles.FeedforwardProfile;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.feedforward_profiles.SetpointProfile;


public class PIDFController {

    ElapsedTime localRuntime;

    PIDController pid;
    FeedforwardController feedforward;

    boolean needsProfileRestart; // flag that keeps track of when

    public PIDFController(PIDController pid, FeedforwardProfile feedforwardProfile){
        this.pid = pid;
        this.feedforward = new FeedforwardController(feedforwardProfile);

        localRuntime = new ElapsedTime();
        needsProfileRestart = true; // need to reset and start profile at the beginning
    }
    public PIDFController(PIDController pid){
        this(pid, new SetpointProfile(0.0));
    }


    public void resetFeedforwardProfile(){
        needsProfileRestart = true; // setting this to true ensures the profile timer will be reset the next time getOutput is called
    }


    public double getOutput(double currentState){
        if(needsProfileRestart){ // if requested, restart/start the feedforward profile timer
            feedforward.resetAndStartProfile();
        }

        double fState = feedforward.getPrimaryOutput(); // get the current state of the feedforward profile (ex. the current predicted needed motor power)
        return fState + pid.getOutput(fState, currentState); // add that predicted value to a PID value attempting to maintain that value (ex. predicted motor power + PID corrections to reach that power)
    }


    public PIDController getPIDController(){return pid;}
    public FeedforwardController getFeedforwardController(){return feedforward;}
}
