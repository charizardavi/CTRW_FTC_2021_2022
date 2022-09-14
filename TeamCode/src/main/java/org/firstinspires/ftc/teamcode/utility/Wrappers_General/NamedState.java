package org.firstinspires.ftc.teamcode.utility.Wrappers_General;

import com.acmerobotics.dashboard.config.Config;


public class NamedState { // a class that neatly holds a minimum and maximum for a 2d range of values
    String stateName;
    double stateValue;


    public NamedState(){
        this.stateName = "NONE";
        this.stateValue = 0;
    }
    public NamedState(String stateName, double stateValue){
        this.stateName = stateName.toUpperCase();
        this.stateValue = stateValue;
    }

    public void setName(String newName){
        this.stateName = newName;
    }
    public void setValue(double newValue){
        this.stateValue = newValue;
    }

    public String getName(){return this.stateName;}
    public double getValue(){return this.stateValue;}

    public boolean nameEquals(String checkName){
        return checkName.toUpperCase().equals( stateName );
    }

    public String toString(){
        return "State('" + stateName + "': " + stateValue + ")";
    }
}
