package org.firstinspires.ftc.teamcode.utility.Math;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.utility.Wrappers_General.Range2d;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.TimestampedValue;

/**
 * This Advanced Math class is by FTC Team 6078 Cut the Red Wire
 *
 * It contains static methods useful for several general purpose mathematical applications
 */
public abstract class AdvMath {
    /**
     * Takes the input number and ensures it is between the given minimum and maximum values (inclusive)
     * @param inputNum the unclipped number
     * @param minimum the minimum value for the range
     * @param maximum the maximum value for the range
     * @return the range clipped number
     */
    public static double rangeClip(double inputNum, double minimum, double maximum){
        double outputNum = inputNum;

        if( inputNum < minimum ){ // if the num is smaller than the allowed minimum, make it the minimum
            outputNum = minimum;
        }
        else if( inputNum > maximum ){ // if the num is smaller than the allowed maximum, make it the maximum
            outputNum = maximum;
        }

        return outputNum;
    }

    /**
     * Takes the input number and ensures it is between the given minimum and maximum values (inclusive)
     * @param inputNum the unclipped number
     * @param permittedRange the minimum and maximum values for the range
     * @return the range clipped number
     */
    public static double rangeClip(double inputNum, Range2d permittedRange){
        return rangeClip( inputNum, permittedRange.getMin(), permittedRange.getMax() ); // just use the same code as for regular range clip, but separating the range object into its two values
    }

    /**
     * Squares an input number, while also preserving if the input number is positive or negative
     * @param inputNum the number to be squared
     * @return the input number multiplied by its absolute value
     */
    public static double squareAndKeepSign(double inputNum){
        return Math.abs(inputNum) * inputNum; // squares the number while keeping it negative if the input was negative
    }

    /**
     * Puts a number to a power, then ensures the number is negative if the original number was negative (which normally wouldn't be the case if it were put to an even power)
     * @param base the number being put to the power
     * @param power the power the number is being put to
     * @return the number put to a power, then made negative again if necessary
     */
    public static double powerAndKeepSign(double base, int power){
        double outputNum = Math.pow(base, power);

        if(base < 0 && power % 2 == 0){ // if the input number was negative and the power was even, make the output number negative again
            outputNum *= -1;
        }

        return outputNum;
    }

    /**
     * @param inputNum the input number for comparison
     * @param permittedRange the input range for comparison to the number
     * @return returns true if the number is within the range (inclusive), false otherwise
     */
    public static boolean isNumInRange(double inputNum, Range2d permittedRange){
        return ( permittedRange.getMin() <= inputNum ) && ( inputNum <= permittedRange.getMax() ); // if greater than (or equal to) the min and less than (or equal to) the max, is within the range
    }

    public static double weightedAverage(double num1, double num2, double num1WeightPercent){
        num1 = num1 * num1WeightPercent;
        num2 = num2 * (1 - num1WeightPercent);
        return (num1 + num2); // the adding part of the average
    }


    public static double interpolateBetween(double firstNumber, double secondNumber, double percentageLocationBetweenNumbers){ // outputs a number depending on where in between the two numbers via linear interpolation
        return ((secondNumber - firstNumber) * percentageLocationBetweenNumbers) + firstNumber; // offsets the difference to where the lower number is 0, then scale the higher number by the percentage between, then add the lower number back in
    }

    public static double getFractionBetween(double firstTime, double secondTime, double currentTime){
        if( secondTime > firstTime ) // prevent divide by zero errors by only working if the second time is past the first time
            return (currentTime - firstTime)/(secondTime - firstTime); // shift everything such that the first timestamp is 0, then see what the current time is out of the second timestamp
            // for example: the first timestamp = 2, second = 6, current = 3.  3-2 =1, 6-2 =4, we are currently 1/4 of the way between 2 and 6
        else
            return 0.0; //
    }


    public static double findLineSlope(double x1, double y1, double x2, double y2){
        if( x2 - x1 != 0 ){
            return (y2 - y1) / (x2 - x1); // rise over run baybeee
        }
        else{ // as a failsafe, if two of the same timestamps just add 0
            return 0;
        }
    }
    public static double findLineSlope(TimestampedValue val1, TimestampedValue val2){
        return findLineSlope(val1.timestamp, val1.value, val2.timestamp, val2.value);
    }
    public static double findLineSlope(Vector2d point1, Vector2d point2){
        return findLineSlope(point1.getX(), point1.getY(), point2.getX(), point2.getY());
    }

    public static double findLineOffset(double slope, double x1, double y1){
        double projectedStartValue = x1 * slope; // see what value no offset gives
        return y1 - projectedStartValue; // then compare that to the actual value, and the offset equals the difference
    }
    public static double findLineOffset(double slope, TimestampedValue refferencePoint){
        double projectedStartValue = refferencePoint.timestamp * slope; // see what value no offset gives
        return refferencePoint.value - projectedStartValue; // then compare that to the actual value, and the offset equals the difference
    }

    public static double minMotorMoveCompensate(double minimumMovePower, double rawPower){
        if(rawPower >= 0)
            return rawPower + minimumMovePower;
        else
            return rawPower - minimumMovePower;
    }
}
