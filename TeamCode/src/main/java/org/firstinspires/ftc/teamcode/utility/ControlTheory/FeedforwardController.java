package org.firstinspires.ftc.teamcode.utility.ControlTheory;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.ControlTheory.feedforward_profiles.FeedforwardProfile;
import org.firstinspires.ftc.teamcode.utility.ControlTheory.feedforward_profiles.SetpointProfile;


@Config
public class FeedforwardController {

    ElapsedTime profileTime;
    FeedforwardProfile profile;

    double pauseOffset = 0; // an offset value that stores the time paused, used to act like the timer (which is reset when unpaused) is outputing time starting from the time the profile was paused
    boolean paused = false; // a flag that tracks of the profile is paused

    public FeedforwardController(){
        profile = new SetpointProfile(0);

        profileTime = new ElapsedTime();
    }
    public FeedforwardController(FeedforwardProfile profile){
        this.profile = profile;

        profileTime = new ElapsedTime();
    }


    public void resetAndStartProfile(){ // starts the profile and resets any pause offset
        profileTime.reset();
        pauseOffset = 0;
    }

    public void pauseProfile(){ // pauses profile at the current time
        if(!paused) {
            pauseOffset += profileTime.milliseconds();
            paused = true;
        }
    }
    public void unpauseProfile(){ // gets the train rolling again
        if(paused){
            profileTime.reset();
            paused = false;
        }
    }
    public boolean isProfilePaused(){ return paused; }


    public double getPrimaryOutput(){ // think position
        if( paused ) // if paused get the target of the profile at the paused time timestep
            return profile.getPrimaryTarget( pauseOffset );
        else    // if not paused, return the normal output based on the timestep (plus any required pause offset)
            return profile.getPrimaryTarget( pauseOffset + profileTime.milliseconds() );
    }
    public double getSecondaryOutput(){ // think velocity
        if( paused ) // if paused get the target of the profile at the paused time timestep
            return profile.getSecondaryTarget( pauseOffset );
        else    // if not paused, return the normal output based on the timestep (plus any required pause offset)
            return profile.getSecondaryTarget( pauseOffset + profileTime.milliseconds() );
    }
    public double getTertiaryOutput(){ // think accel
        if( paused ) // if paused get the target of the profile at the paused time timestep
            return profile.getTertiaryTarget( pauseOffset );
        else    // if not paused, return the normal output based on the timestep (plus any required pause offset)
            return profile.getTertiaryTarget( pauseOffset + profileTime.milliseconds() );
    }

    // getter methods
    public boolean isForwardComplete(){return profileTime.milliseconds() > profile.getProfileDuration();}
    public double getProfileTime(){return profileTime.milliseconds();}
    public FeedforwardProfile getProfile(){return profile;}

    public boolean setProfile(FeedforwardProfile profile){
        if(profile != null){
            this.profile = profile; // set the profile
            resetAndStartProfile(); // reset the profile to ensure

            return true;
        }
        else
            return false;
    }
}
