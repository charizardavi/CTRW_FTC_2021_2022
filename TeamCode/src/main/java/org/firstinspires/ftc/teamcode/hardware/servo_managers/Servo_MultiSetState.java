package org.firstinspires.ftc.teamcode.hardware.servo_managers;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.Wrappers_General.NamedState;

import java.util.ArrayList;


public class Servo_MultiSetState {
    Servo servo;
    ArrayList<NamedState> states;
    NamedState activeState;

    public Servo_MultiSetState(Servo servo){
        this.servo = servo;

        this.states = new ArrayList<>();
        this.states.add(new NamedState("State1", 0.0)); // set the states to their default values


        this.activeState = states.get(0);
    }
    public Servo_MultiSetState(Servo servo, NamedState setState){
        this.servo = servo;

        this.states = new ArrayList<>();

        states.add(setState);

        this.activeState = states.get(0);
    }
    public Servo_MultiSetState(Servo servo, NamedState setState1, NamedState setState2){
        this.servo = servo;

        this.states = new ArrayList<>();

        states.add(setState1);
        states.add(setState2);

        this.activeState = states.get(0);
    }
    public Servo_MultiSetState(Servo servo, NamedState setState1, NamedState setState2, NamedState setState3){
        this.servo = servo;

        this.states = new ArrayList<>();

        states.add(setState1);
        states.add(setState2);
        states.add(setState3);

        this.activeState = states.get(0);
    }
    public Servo_MultiSetState(Servo servo, ArrayList<NamedState> setStates){
        this.servo = servo;

        if (setStates != null && setStates.size() > 0){
            this.states = setStates;
        }
        else{
            this.states = new ArrayList<>();
            this.states.add(new NamedState("State1", 0.0)); // set the states to their default values
        }

        this.activeState = states.get(0);
    }


    public void setServoPos(double pos){
        servo.setPosition(pos);
    }

    public void goToGivenState(NamedState givenState){
        activeState = givenState;
        servo.setPosition( givenState.getValue() );
    }

    public void goToStateNamed(String stateName){
        stateName = stateName.toUpperCase();
        for(NamedState state: states){
            if (state.getName().equals( stateName )){
                goToGivenState( state );
                break;
            }
        }
    }
    public void goToStateAtIndex(int index){
        if (index < states.size() && index >= 0){
            goToGivenState( states.get(index) );
        }
    }

    public NamedState getActiveState(){return activeState;}
}
