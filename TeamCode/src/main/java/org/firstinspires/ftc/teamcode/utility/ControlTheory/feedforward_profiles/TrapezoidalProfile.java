package org.firstinspires.ftc.teamcode.utility.ControlTheory.feedforward_profiles;


public class TrapezoidalProfile extends FeedforwardProfile { // creates smooth motion via a trapezoidal velocity profile
    // anytime a comment refers to position, velocity, or acceleration, that is assuming the profile output is positional, if it is say a velocity output shift everything back in your mind, the things that do velocity will really be doing acceleration, the things doing acceleration will be doing jerk, etc

    double acceleratingSquareCoefficient, stableLinearCoefficient, deceleratingSquareCoefficient;
    double K1, K2, K3; // the multiplier for the setpoint (ex. number you multiply setpoint velocity by to create the motor power required to reach that setpoint)


    boolean stabalizesAtMaxSpeed = false;
    double maxSpeed;

    double startPos;
    double posAfterAcceleration;
    double posAfterSteadySpeed;
    double posAfterDeceleration;

    double endOfAccelerationTime;
    double endOfConstantSpeedTime;
    double endOfDecelerationTime;

    double stableLinearVelo;



    public TrapezoidalProfile(double startPoint, double endPoint, double maxSpeed, double maxAccelPerSec, double primaryK, double secondaryK, double tertiaryK, ProfileEndBehavior endBehavior){
        super( 0, endBehavior ); // call the FeedforwardProfile constructor to setup transversal time variables

        this.startPos = startPoint;
        this.maxSpeed = maxSpeed;
        this.posAfterDeceleration = endPoint;
        createVeloProfile(startPoint, endPoint, maxAccelPerSec, maxSpeed); // make the acceleration profile, which can be used to calculate velocity for the output of this profile
        this.profileDuration = endOfDecelerationTime;

        this.K1 = primaryK;
        this.K2 = secondaryK;
        this.K3 = tertiaryK;
    }
    public TrapezoidalProfile(double startPoint, double endPoint, double maxSpeed, double maxAccelPerSec, double primaryK, double secondaryK, ProfileEndBehavior endBehavior){
        this(startPoint, endPoint, maxSpeed, maxAccelPerSec, primaryK, secondaryK, 1.0, endBehavior);
    }
    public TrapezoidalProfile(double startPoint, double endPoint, double maxSpeed, double maxAccelPerSec, double primaryK, ProfileEndBehavior endBehavior){
        this(startPoint, endPoint, maxSpeed, maxAccelPerSec, primaryK, 1.0, 1.0, endBehavior);
    }
    public TrapezoidalProfile(double startPoint, double endPoint, double maxSpeed, double maxAccelPerSec, double primaryK, double secondaryK, double tertiaryK){
        this(startPoint, endPoint, maxSpeed, maxAccelPerSec, primaryK, secondaryK, tertiaryK, ProfileEndBehavior.MAINTAIN);
    }
    public TrapezoidalProfile(double startPoint, double endPoint, double maxSpeed, double maxAccelPerSec, double primaryK, double secondaryK){
        this(startPoint, endPoint, maxSpeed, maxAccelPerSec, primaryK, secondaryK, 1.0, ProfileEndBehavior.MAINTAIN);
    }
    public TrapezoidalProfile(double startPoint, double endPoint, double maxSpeed, double maxAccelPerSec, double primaryK){
        this(startPoint, endPoint, maxSpeed, maxAccelPerSec, primaryK, 1.0, 1.0, ProfileEndBehavior.MAINTAIN);
    }





    @Override
    public double getPrimaryProfileTarget(double timestep){ // think position
        double rawPosition = 0;
        double timeSinceSectionStart = timestep;


        if (timestep <= 0) // if at or before start, return start
            rawPosition = startPos;

        else if (timestep < endOfAccelerationTime) // if at accelerating part of profile, return appropriate integral (position)
            rawPosition = acceleratingSquareCoefficient * Math.pow(timeSinceSectionStart, 2) + startPos; // n*x^2

        else if (stabalizesAtMaxSpeed && timestep < endOfConstantSpeedTime) { // if at stable part of profile, return appropriate integral (position)
            timeSinceSectionStart -= endOfAccelerationTime;
            rawPosition = (stableLinearVelo * timeSinceSectionStart) + posAfterAcceleration + startPos; // n*x + posAfterAcceleration
        }

        else if(timestep < endOfDecelerationTime){ // if decelerating part
            double lastEndPos;

            if(stabalizesAtMaxSpeed){ // if stabalizes at a max speed, use the end of that one
                timeSinceSectionStart -= endOfConstantSpeedTime;
                lastEndPos = posAfterSteadySpeed;
            }
            else { // if doesn't stabalize at max speed
                timeSinceSectionStart -= endOfAccelerationTime;
                lastEndPos = posAfterAcceleration;
            }

            rawPosition = (deceleratingSquareCoefficient * Math.pow(timeSinceSectionStart, 2)) + (stableLinearVelo * timeSinceSectionStart) + lastEndPos + startPos;
        }

        else // if at or beyond end, just return end
            rawPosition = posAfterDeceleration;


        return rawPosition;
    }


    @Override
    public double getSecondaryProfileTarget(double timestep){ // think velocity
        double rawVelo;

        if (timestep <= 0) // if at or before start, return 0, as stopped at start
            rawVelo = 0;

        else if (timestep < endOfAccelerationTime) // if at accelerating part of profile, return appropriate integral (position)
            rawVelo = (acceleratingSquareCoefficient * 2) * timestep; // the derivative of the first range's function

        else if (stabalizesAtMaxSpeed && timestep < endOfConstantSpeedTime) // if at stable part of profile, return appropriate integral (position)
            rawVelo = stableLinearVelo; // the max speed but multiplied by 1 or -1 in accordance with the direction of the velocity

        else if(timestep < endOfDecelerationTime){ // if decelerating part
            double timeSinceSectionStart = timestep;

            if(stabalizesAtMaxSpeed){
                timeSinceSectionStart -= endOfConstantSpeedTime;
            }
            else {
                timeSinceSectionStart -= endOfAccelerationTime;
            }

            rawVelo = (deceleratingSquareCoefficient * 2) * timeSinceSectionStart + stableLinearVelo; // the derivative of the last range's function
        }

        else // if at or beyond end, just return 0, as stopped
            rawVelo = 0;


        return rawVelo;
    }


    @Override
    public double getTertiaryProfileTarget(double timestep){ // think accel
        if (timestep <= 0) // if at or before start, return 0, as stopped at start
            return 0;

        else if (timestep < endOfAccelerationTime) // if at accelerating part of profile, return appropriate integral (position)
            return acceleratingSquareCoefficient / 2; // the second derivative of the first range's function

        else if (stabalizesAtMaxSpeed && timestep < endOfConstantSpeedTime)  // if at stable part of profile, return appropriate integral (position)
            return 0; // no acceleration when no change in velocity

        else if(timestep <= endOfDecelerationTime) // if decelerating part
            return deceleratingSquareCoefficient/2; // the second derivative of the last range's function

        else // if beyond end, just return 0, as stopped
            return 0;
    }




    private void createVeloProfile(double startPoint, double endPoint, double maxAccelPerSec, double maxSpeedPerSec){
        double travelError = endPoint - startPoint;
        double goingUp = Math.abs(travelError) / travelError; // if travel error is positive, going up will be 1, if negative, going up will be -1, to ensure that everything is going in the right direction
        double travelDistance = Math.abs(travelError);


        // we know the time of the start point is 0, and we know the max acceleration per second is the slope of the acceleration graph, so until flattening:
        // speed = maxAccelPerMsec*time
        // and we can put maxSpeedPerSec in for speed and solve for time to find when acceleration reaches max speed
        double maxAccelPerMsec = maxAccelPerSec / 1000000; // (dividing maxAccelPerSec by 1000 to scale it to be accel per millisecond)
        double maxSpeedPerMsec = maxSpeedPerSec / 1000;
        endOfAccelerationTime = maxSpeedPerMsec / maxAccelPerMsec;

        // the function of position given a time is simply the integral of the speed function, so:
        // position = (maxAccelPerMsec/2) * time^2
        // which we can plug in maxSpeedReachTime to find how far it has traveled by the time it has fully accelerated, and compare that to half the endpoint to see if it has time to fully accelerate before position gets over half way to the target
        double fullyAcceleratedDistance = (maxAccelPerMsec/2) * Math.pow(endOfAccelerationTime, 2);

        if( travelDistance/2 > fullyAcceleratedDistance){ // if travelDistance/2 is greater than the fully accelerated position, there is a flat component, so find how long it needs to be
            stabalizesAtMaxSpeed = true;

            // the accelerating portion of the profile
            acceleratingSquareCoefficient = goingUp * (maxAccelPerMsec/2); // taking the integral equation from before and storing it
            // no linear component to this integral equation as speed graph had no offset
            posAfterAcceleration = goingUp * fullyAcceleratedDistance; // store where we are at the end of this movement

            // the constant speed portion of the profile
            // finding the integral of this portion:
            // speed = maxSpeedPerSec  (the speed function)
            // position = maxSpeedPerSec * time  (the integral of the speed function), but be sure to
            double distanceLeftToHalfway = (travelDistance/2) - fullyAcceleratedDistance; // see how much distance is left to the half way point
            double flatSpeedTravelTime = 2 * (distanceLeftToHalfway/maxSpeedPerMsec); // the amount of time we will spend at flat speed is the amount of time from the fully accelerated point to the midpoint times two
            stableLinearVelo = goingUp * maxSpeedPerMsec; // this one only has a linear component (no acceleration), speed converted from in/sec to in/msec
            endOfConstantSpeedTime = endOfAccelerationTime + flatSpeedTravelTime; // and this part of the graph goes until time to slow down again
            posAfterSteadySpeed = goingUp * (2 * distanceLeftToHalfway) + posAfterAcceleration; // store where we are at the end of this movement

            // now find the integral of the deceleration portion
            // speed = -maxAccelPerMsec*time + maxSpeedPerSec   (the speed function)
            // position = -(maxAccelPerMsec/2) * time^2 + maxSpeedPerSec * time   (the integral of the speed function)
            deceleratingSquareCoefficient = goingUp * -(maxAccelPerMsec/2); // taking the integral equation from before and storing it
            endOfDecelerationTime = endOfConstantSpeedTime + endOfAccelerationTime; // and this part of the graph goes until the end
        }
        else{
            stabalizesAtMaxSpeed = false;

            // set the end of acceleration time to half the required travel time
            fullyAcceleratedDistance = travelDistance / 2;
            // solve the integral equation fullyAcceleratedDistance = (maxAccelPerMsec * endOfAccelerationTime^2) / 2
            // to be in terms of the time
            // so, step by step:
            // (maxAccelPerMsec * endOfAccelerationTime^2) / 2 = fullyAcceleratedDistance
            // maxAccelPerMsec * endOfAccelerationTime^2 = 2*fullyAcceleratedDistance
            // endOfAccelerationTime^2 = (2*endOfAccelerationTime) / maxAccelPerMsec
            // endOfAccelerationTime = sqrt( (2*fullyAcceleratedDistance) / maxAccelPerMsec )
            endOfAccelerationTime = Math.sqrt(  (2*fullyAcceleratedDistance) / maxAccelPerMsec  );

            // now we need to find the tangent line slope at the end of the acceleration section (this lets us draw a line that the deceleration portion
            // do this by inputting the endOfAcceleration time into the function that gives velocity based on how much time has passed, calculated before:
            // speed = maxAccelPerMsec * time
            stableLinearVelo = goingUp * maxAccelPerMsec * endOfAccelerationTime; // (remember, goingUp is either 1 or -1 depending on direction of travel)

            // the accelerating portion of the profile
            acceleratingSquareCoefficient = goingUp * (maxAccelPerMsec/2); // taking the integral equation from before and storing it
            // no linear component to this integral equation as speed graph had no offset
            posAfterAcceleration = goingUp * fullyAcceleratedDistance; // store where we are at the end of this movement

            // now find the integral of the deceleration portion
            // speed = -maxAccelPerMsec*time + maxSpeedPerSec   (the speed function)
            // position = -(maxAccelPerMsec/2) * time^2 + maxSpeedPerSec * time   (the integral of the speed function)
            deceleratingSquareCoefficient = goingUp * -(maxAccelPerMsec/2); // taking the integral equation from before and storing it
            endOfDecelerationTime = 2 * endOfAccelerationTime; // and this part of the graph goes until the end
        }

        this.profileDuration = endOfDecelerationTime; // update the profile duration to be the end
    }


    public double getStartPos(){
        return startPos;
    }
    public double getEndPos(){
        return posAfterDeceleration;
    }

}

