package org.firstinspires.ftc.teamcode.utility.ControlTheory;

import com.qualcomm.robotcore.util.ElapsedTime;

public class VeloLimiter {

    ElapsedTime localRuntime;

    double maxSpeed = -1;
    double maxAccelPerSec = -1;
    boolean limitDeceleration = true;
    double baseVelo = 0;

    double lastInputVelo = 0;
    double lastOutputVelo = 0;
    double lastRunTime = 0;

    public VeloLimiter(double maxSpeed, double maxAccelPerSec, boolean limitDeceleration){
        this.maxAccelPerSec = maxAccelPerSec;
        this.maxSpeed = maxSpeed;
        this.limitDeceleration = limitDeceleration;

        localRuntime = new ElapsedTime();
    }
    public VeloLimiter(double maxSpeed){
        this(maxSpeed, -1, false);
    }
    public VeloLimiter(double maxAccelPerSec, boolean limitDeceleration){
        this(-1, maxAccelPerSec, limitDeceleration);
    }

    public VeloLimiter setBaseVelo(double baseVelo){ // will cause the robot to think this is what the actual last velo was, causing it to start acceleration limiting and such from this velo
        this.baseVelo = baseVelo;
        lastOutputVelo = baseVelo;

        return this; // returns this object so this method can be used like "VeloLimiter limiter = new VeloLimiter(1, 0.5, true).setBaseVelo(0.5)"
    }
    public void reset() {
        lastOutputVelo = baseVelo;
        localRuntime.reset();
    }

    public double getLimitedVelo(double inputVelo){
        double outputVelo = inputVelo;

        if( maxAccelPerSec > 0 ) { // if an acceleration limit given and allowed to limit it, limit acceleration
            double timeFactor = (localRuntime.milliseconds() - lastRunTime) / 1000; // gets how long it has been since the last run, in seconds so we can use it the "acceleration/second" math
            double allowedAccel = maxAccelPerSec * timeFactor * maxSpeed; // how much the new speeds are allowed to be above/below the old speed

            // limit acceleration/deceleration
            if (inputVelo > lastOutputVelo + allowedAccel && (limitDeceleration || isIncreasingSpeed(lastOutputVelo + allowedAccel))) { // if x is accelerating too fast
                outputVelo = lastOutputVelo + allowedAccel; // set x to the fastest speed it is allowed to accelerate
            }
            else if (inputVelo < lastOutputVelo - allowedAccel && (limitDeceleration || isIncreasingSpeed(lastOutputVelo - allowedAccel))) { // or if decelerating too fast (and allowed to limit deceleration)
                outputVelo = lastOutputVelo - allowedAccel; // set x to the fastest speed it is allowed to accelerate
            }
        }

        if (maxSpeed >= 0 && Math.abs(outputVelo) > maxSpeed){ // if a speed excedes speed limit
            outputVelo = maxSpeed * (Math.abs(outputVelo) / outputVelo); // set output to the speed limit (multiplied by 1 if x is positive, negative 1 if x is negative)
        }

        lastInputVelo = inputVelo;
        lastOutputVelo = outputVelo;
        lastRunTime = localRuntime.milliseconds();
        return outputVelo;
    }

    private boolean isIncreasingSpeed(double newOutputVelo){ // returns true if the absolute value of our velocity is increasing
        return Math.abs(newOutputVelo) > Math.abs(lastOutputVelo);
    }

    public double getMaxSpeed(){return maxSpeed;}
    public double getMaxAccelPerSec(){return maxAccelPerSec;}
    public boolean isLimitingDeceleration(){return limitDeceleration;}

    public void setLimitDeceleration(boolean limitDeceleration){
        this.limitDeceleration = limitDeceleration;
    }
    public void setMaxSpeed(double speed){this.maxSpeed = speed;}
    public void setMaxAccelPerSec(double accelPerSec){this.maxAccelPerSec = accelPerSec;}
}
