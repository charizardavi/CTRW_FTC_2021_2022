package org.firstinspires.ftc.teamcode.utility.Math;


import org.firstinspires.ftc.teamcode.utility.Wrappers_General.TimestampedValue;
import java.util.ArrayList;


public class MultislopeLinearInterpolator {
    ArrayList<TimestampedValue> timestampedSetpoints;
    ArrayList<Double> slopes;
    ArrayList<Double> offsets;
    int lineCount;

    public MultislopeLinearInterpolator(ArrayList<TimestampedValue> timestampedSetpoints){ // constructor, creates lines to interpolate from using points given

        this.timestampedSetpoints = timestampedSetpoints; // setup the array lists
        this.slopes = new ArrayList<>();
        this.offsets = new ArrayList<>();

        this.lineCount = timestampedSetpoints.size() - 1;
        for(int i = 0; i < this.lineCount; i++){
            TimestampedValue firstValue = timestampedSetpoints.get(i);
            TimestampedValue secondValue = timestampedSetpoints.get(i+1);

            double slope = AdvMath.findLineSlope(firstValue, secondValue);
            slopes.add( slope );

            offsets.add( AdvMath.findLineOffset(slope, firstValue) );
        }

    }


    public double interpolate(double timestep){
        int lineIndex = 0; // the index of the corresponding line slope and offset for this timestep
        while( timestampedSetpoints.get(lineIndex).timestamp > timestep && lineIndex < this.lineCount){ // while the first timestamp of the current line is ahead of the target timestep (and still within range of the list)
            lineIndex++; // move forward in the list
        }

        double slope = slopes.get( lineIndex ); // get the appropriate slope
        double offset = offsets.get( lineIndex ); // and offset
        return (slope*timestep) + offset; // then apply  y = mx + b   (with x = timestep)
    }


    public double getSlopeAtTimestep(double timestep){
        int slopeIndex = 0; // the index of the corresponding line slope and offset for this timestep
        while( timestampedSetpoints.get(slopeIndex).timestamp > timestep && slopeIndex < this.lineCount){ // while the first timestamp of the current line is ahead of the target timestep (and still within range of the list)
            slopeIndex++; // move forward in the list
        }
        
        return slopes.get(slopeIndex);
    }
    public double getSlope(int slopeIndex){
        return slopes.get(slopeIndex);
    }
}
