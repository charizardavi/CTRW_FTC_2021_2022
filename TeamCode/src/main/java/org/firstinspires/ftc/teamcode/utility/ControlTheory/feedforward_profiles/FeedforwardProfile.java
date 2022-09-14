package org.firstinspires.ftc.teamcode.utility.ControlTheory.feedforward_profiles;

public abstract class FeedforwardProfile {
    public enum ProfileEndBehavior{
        MAINTAIN, // will keep outputting whatever values the last point on the profile outputs (the default)
        RESTART,  // will jump back to the beginning of the profile once complete
        BACKTRACK // will play the profile backwards once complete, then once complete with playing backwards will restart
    }

    double profileDuration; // local variables that all subclasses will use
    ProfileEndBehavior endBehavior;



    protected FeedforwardProfile(double profileDuration, ProfileEndBehavior endBehavior){ // constructor (called using super() by child classes)
        this.profileDuration = profileDuration;
        this.endBehavior = endBehavior;
    }


    public final double getPrimaryTarget(double timestep){ // the method called to get a profile's primary (think position) target, the method it calls (getProfileTarget) will change depending on subclass, but still always handles end behaviour before passing in
        return getPrimaryProfileTarget( handleEndBehavior(timestep) );
    }
    public final double getSecondaryTarget(double timestep) { // the method called to get a profile's primary (think velocity) target
        return getSecondaryProfileTarget( handleEndBehavior(timestep) );
    }
    public final double getTertiaryTarget(double timestep){ // the method called to get a profile's primary (think acceleration) target
        return getTertiaryProfileTarget( handleEndBehavior(timestep) );
    }
    protected double getPrimaryProfileTarget(double timestep){ // to be overriden by subclasses
        return 0.0;
    }
    protected double getSecondaryProfileTarget(double timestep){ // to be overriden by subclasses
        return 0.0;
    }
    protected double getTertiaryProfileTarget(double timestep){ // to be overriden by subclasses
        return 0.0;
    }


    private double handleEndBehavior(double timestep){ // manipulates the input timestep to simulate the proper type of end behavior
        if( endBehavior == ProfileEndBehavior.BACKTRACK){
            while(timestep > profileDuration * 2){ // remove from the timestep until it is within twice the profile duration, as one duration is the way there and a second duration the way back
                timestep -= profileDuration * 2;
            }

            if(timestep > profileDuration){ // if still greater than profile duration, means the time should put us on the way back, so backtrack (as less than the profile duration after correction would be the regular way, aka frowards on the profile)
                double distanceBacktracking = timestep - profileDuration; // subtract the profile duration from the timestep to see how far into the backtracking we are
                timestep = profileDuration - distanceBacktracking; // then backtrack in the path by that amount
            }
        }
        else if( endBehavior == ProfileEndBehavior.RESTART ){ // if on the restart
            while( timestep > profileDuration ){ // remove from the timestep until it is within the profile duration, will create restart like behavior
                timestep -= profileDuration;
            }
        }
        else if( timestep > profileDuration ){ // if no others, must be maintain, so check if we are beyond the end and need to use the end value
            timestep = profileDuration; // then use the end value by telling the profile it is at the end
        }

        return timestep;
    }


    public double getProfileDuration(){return profileDuration;}
}
