package org.firstinspires.ftc.teamcode.hardware.intakes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class Intake {
    private DcMotor intakeMotor;


    public static final double DEFAULT_INTAKE_RUN_SPEED = 1.0;

    private double intakeRunSpeed = DEFAULT_INTAKE_RUN_SPEED;
    private double currentIntakePower = 0.0;
    private boolean intakeIsRunning = false;


    public Intake(DcMotor intakeMotor){
        this.intakeMotor = intakeMotor;
    }


    public void spinUp(){ // sets the motor to run at the spinning speed (will continue to run at that speed until set otherwise
        intakeMotor.setPower( intakeRunSpeed );
        currentIntakePower = intakeRunSpeed;
        intakeIsRunning = true;
    }
    public void spinDown(){
        intakeMotor.setPower(0.0);
        currentIntakePower = 0.0;
        intakeIsRunning = false;
    }
    public void setRunning( boolean isRunning ){
        if (isRunning == true) {
            spinUp();
        }
        else {
            spinDown();
        }
    }

    public void setIntakeRunSpeed(double newSpeed){
        intakeRunSpeed = newSpeed;

        if( isRunning() )
            spinUp(); // if running, call the spin up method to update the running speed to the new speed
    }
    public double getCurrentIntakePower(){return currentIntakePower;}

    public boolean isRunning(){
        return intakeIsRunning;
    }
}
