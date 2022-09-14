package org.firstinspires.ftc.teamcode.hardware.servo_managers;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Servo_Spinner {
    Servo servo;
    double stopSpeed;
    double forwardSpeed;
    double backwardSpeed;
    int stopDuration;
    int spinDuration;
    int periodicSpinPeriod;

    public enum SpinDirection{
        STOPPED,
        FORWARD,
        BACKWARD
    }
    SpinDirection servoSpinDirection = SpinDirection.STOPPED;

    SpinDirection periodicSpinDirection = SpinDirection.STOPPED;
    ElapsedTime periodicSpinTimer;


    public Servo_Spinner(Servo servo){
        this.servo = servo;
        this.stopSpeed = 0.5; // set the speeds to their default values
        this.forwardSpeed = 1.0;
        this.backwardSpeed = 0.0;
        this.spinDuration = 1000;
        this.stopDuration = 1000;
        this.periodicSpinPeriod = spinDuration + stopDuration;

        this.periodicSpinTimer = new ElapsedTime();
    }
    public Servo_Spinner(Servo servo, double stopSpeed, double forwardSpeed, double backwardSpeed){
        this.servo = servo;
        this.stopSpeed = stopSpeed;
        this.forwardSpeed = forwardSpeed;
        this.backwardSpeed = backwardSpeed;
        this.spinDuration = 1000;
        this.stopDuration = 1000;
        this.periodicSpinPeriod = spinDuration + stopDuration;

        this.periodicSpinTimer = new ElapsedTime();
    }
    public Servo_Spinner(Servo servo, double stopSpeed, double forwardSpeed, double backwardSpeed, int stopDuration, int spinDuration){
        this.servo = servo;
        this.stopSpeed = stopSpeed;
        this.forwardSpeed = forwardSpeed;
        this.backwardSpeed = backwardSpeed;
        this.stopDuration = stopDuration;
        this.spinDuration = spinDuration;
        this.periodicSpinPeriod = spinDuration + stopDuration;

        this.periodicSpinTimer = new ElapsedTime();
    }


    public void setServoSpeed(double speed){
        servo.setPosition(speed);

        if( speed > stopSpeed ){
            if(forwardSpeed > stopSpeed)
                this.servoSpinDirection = SpinDirection.FORWARD;
            else
                this.servoSpinDirection = SpinDirection.BACKWARD;
        }
        else if( speed < stopSpeed ){
            if(backwardSpeed < stopSpeed)
                this.servoSpinDirection = SpinDirection.BACKWARD;
            else
                this.servoSpinDirection = SpinDirection.FORWARD;
        }
        else{
            this.servoSpinDirection = SpinDirection.STOPPED;
        }
    }
    public void runDirection(SpinDirection direction){
        if( direction == SpinDirection.FORWARD )
            runForwards();
        else if( direction == SpinDirection.BACKWARD )
            runBackwards();
        else
            stop();


    }
    public void runForwards(){
        servo.setPosition(forwardSpeed);
        servoSpinDirection = SpinDirection.FORWARD;
    }
    public void runBackwards(){
        servo.setPosition(backwardSpeed);
        servoSpinDirection = SpinDirection.BACKWARD;
    }
    public void stop(){
        servo.setPosition(stopSpeed);
        this.servoSpinDirection = SpinDirection.STOPPED;
    }
    public SpinDirection getServoSpinDirection(){
        return servoSpinDirection;
    }


    // periodic spin methods
    public void startPeriodicSpin(SpinDirection spinDirection){
        this.periodicSpinTimer.reset();
        this.periodicSpinDirection = spinDirection;
    }
    public void startPeriodicSpin(SpinDirection spinDirection, int stopDuration, int spinDuration){
        this.stopDuration = stopDuration;
        this.spinDuration = spinDuration;
        this.periodicSpinPeriod = spinDuration + stopDuration;
        this.startPeriodicSpin(spinDirection);
    }
    public void stopPeriodicSpin(){
        this.periodicSpinDirection = SpinDirection.STOPPED;
    }
    public SpinDirection getPeriodicSpinDirection(){
        return periodicSpinDirection;
    }
    public boolean isPeroidicSpinning(){ // if not stopped, is spinning periodically
        return periodicSpinDirection != SpinDirection.STOPPED;
    }
    public void updatePeriodicSpin(){
        if( isPeroidicSpinning() ){
            int timeIntoThisPeriod = (int)(periodicSpinTimer.milliseconds()) % periodicSpinPeriod;

            if( timeIntoThisPeriod <= spinDuration )
                runDirection( periodicSpinDirection );
            else
                stop();
        }
    }

    public Servo getServo(){
        return servo;
    }
}
