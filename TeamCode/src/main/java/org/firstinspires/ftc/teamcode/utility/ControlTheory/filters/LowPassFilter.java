package org.firstinspires.ftc.teamcode.utility.ControlTheory.filters;


import org.firstinspires.ftc.teamcode.utility.Math.AdvMath;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.Range2d;


// smooths out jerky inputs to be more regular at the expense of updating immediately (the higher the current state weight the more it prioritizes the current state over smoothing the data)
public class LowPassFilter {
    double previousStateWeight = 0.4; // how much the previous output is weighted in comparison to the current sensor input (0.5 = 50%, 0.9 = 90%, etc, only between 0 and 1)
    double sensorInputWeight = 0.6; // how much the current sensor input is weighted compared to the previous state

    boolean firstStateSet = false;
    double previousState = 0.0; // the most recent sensor output

    public LowPassFilter(double previousStateWeight){
        setPreviousStateWeight( previousStateWeight );
    }


    public double filter(double sensorInput){ // mathematically weight the previous state and the current state
        if(!firstStateSet){
            previousState = sensorInput;
            firstStateSet = true;
        }

        double output = (previousStateWeight * previousState) + (sensorInputWeight * sensorInput);
        previousState = output;
        return output;
    }

    public void setPreviousStateWeight(double previousStateWeight){
        if ( !AdvMath.isNumInRange( previousStateWeight, new Range2d(0, 1) ) ){ // if the new weight is not between 0 and 1 the weight isn't valid, assume previous sensor input has 0 weight
            previousStateWeight = 0;
        }

        this.previousStateWeight = previousStateWeight; // set the previous state weight
        this.sensorInputWeight = 1 - previousStateWeight; // and set the current sensor input weight to 1 - the previous state weight
    }

    public double getPreviousState(){return previousState;}
    public double getPreviousStateWeight(){return previousStateWeight;}
    public double getSensorInputWeight(){return sensorInputWeight;}
}
