package org.firstinspires.ftc.teamcode.hardware.continuous_servos;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoSpinner {
    Servo servo;
    double stopSpeed;
    double forwardSpeed;
    double backwardSpeed;


    public ServoSpinner(Servo servo){
        this.servo = servo;
        this.stopSpeed = 0.5; // set the speeds to their default values
        this.forwardSpeed = 1.0;
        this.backwardSpeed = 0.0;
    }
    public ServoSpinner(Servo servo, double stopSpeed, double forwardSpeed, double backwardSpeed){
        this.servo = servo;
        this.stopSpeed = stopSpeed;
        this.forwardSpeed = forwardSpeed;
        this.backwardSpeed = backwardSpeed;
    }


    public void setServoSpeed(double speed){
        servo.setPosition(speed);
    }
    public void runForwards(){
        servo.setPosition(1.0);
    }
    public void runBackwards(){
        servo.setPosition(0.0);
    }
    public void stop(){
        servo.setPosition(0.5);
    }

}
