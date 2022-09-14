package org.firstinspires.ftc.teamcode.utility.Math;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.Vector2;
import org.firstinspires.ftc.teamcode.utility.Wrappers_General.Vector3;


public abstract class AngleMath {
    public static double clipAngle(double angle){ // makes a radian input value to be between 0 and 2PI
        return clipAngle(angle, AngleUnit.RADIANS);
    }
    public static double clipAngle(double angle, AngleUnit angleUnit){ // makes a radian input value to be between 0 and 2PI
        double circleValue = 2*Math.PI;
        if( angleUnit.equals(AngleUnit.DEGREES) ) // change modes if not using radians
            circleValue = 360;

        while( angle < 0 ) // for as long as the output is less than 0, add 360 (aka  2PI radians)
            angle += circleValue;

        while( angle >= circleValue ) // and do the opposite to get it below 360
            angle -= circleValue;

        return angle;
    }


    /**
     * Thinking of an (x, y) coordinate like sides of a right triangle, find the angle of the hypotenuse from the x axis
     * @param vector the input (x, y) pair to find the theta (angle) of
     * @param angleUnit the unit the angle will be output in
     * @return The angle of the vector from the right half of the x axis
     */
    public static double getVectorAngle(Vector2d vector, AngleUnit angleUnit){ // will output a (specified angle unit) angle based on the angle of the vector from 0
        double angle = 0;
        double x = vector.getX();
        double y = vector.getY();

        if( Math.abs(x) < 0.00001 ){ // if x is essentially 0, don't risk a div 0 error (plus then angle must be either 90 or -90), remember 90 = PI/2 radians
            angle = ( Math.PI/2 ) * ( Math.abs(y)/y ); // multiply 90 degrees by either 1 or -1, if y is positive or negative respectively
        }
        else if( x < 0 ){ // if x is negative, the angle is in quadrant 2 or 3 (and arctan only outputs angles in quadrant 1 or 4)
            angle = Math.atan( y / x ) + Math.PI; // so to correct for this, add 180 (aka PI radians)
        }
        else { // if in quadrant 1 or 4, atan will work just fine as normal
            angle = Math.atan( y / x );
        }

        while(angle < 0)
            angle += Math.PI*2; // for as long as the output is less than 0, add 360 (aka  2PI radians)

        if(angleUnit.equals(AngleUnit.RADIANS)) // if they want radians, give them radians
            return angle;
        else // otherwise, convert to degrees and output that
            return Math.toDegrees(angle);
    }
    /**
     * Thinking of an (x, y) coordinate like sides of a right triangle, find the angle of the hypotenuse from the x axis
     * @param vector the input (x, y) pair to find the theta (angle) of
     * @return The angle of the vector from the right half of the x axis (in radians)
     */
    public static double getVectorAngle(Vector2d vector){ // will output a radian angle based on the angle of the vector from 0
        return getVectorAngle(vector, AngleUnit.RADIANS);
    }

    /**
     * Gets the magnitude of an (x, y) vector
     * @param vector input (x, y) vector
     * @return find the magnitude of (x, y), aka the length of the line from (0, 0) to (x, y), via the pythagorean theorem
     */
    public static double getVectorMagnitude(Vector2d vector){ // via pythagorean theorem, find the hypotenuse of the triangle formed by the sides x and y to find the length of the line from (0, 0) to (x, y)
        return Math.sqrt( Math.pow(vector.getX(), 2) + Math.pow(vector.getY(), 2) );
    }



    public static double findClosestCoterminalTarget(double currentAngle, double targetAngle, AngleUnit angleUnit){
        double circleValue = 2*Math.PI;
        if( angleUnit.equals(AngleUnit.DEGREES) ) // change modes if not using radians
            circleValue = 360;

        if(currentAngle + circleValue/2< targetAngle){ // if current heading is 180 (or more) degrees below the target position, subtract 360 from the current heading so we travel the most efficient route towards the target
            targetAngle -= circleValue;
        }
        else if(currentAngle - circleValue/2 > targetAngle){ // else if current heading is 180 (or more) degrees above the target position, add 360 to the target heading so we travel the most efficient route towards the target
            targetAngle += circleValue;
        }

        return targetAngle;
    }
    public static double findClosestCoterminalTarget(double currentAngle, double targetAngle){
        return findClosestCoterminalTarget(currentAngle, targetAngle, AngleUnit.RADIANS);
    }


    public static double getAngleToPoint(Vector2d startPoint, Vector2d endPoint){
        Vector2d displacement = endPoint.minus( startPoint );
        return displacement.angle();
    }



    public static Vector2 toVector2(Vector2d vector2d){
        return  new Vector2(vector2d.getX(), vector2d.getY());
    }
    public static Vector2d toVector2d(Vector2 vector2){
        return  new Vector2d(vector2.getX(), vector2.getY());
    }

    public static Vector3 toVector3(Pose2d pose2d){
        return  new Vector3(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }
    public static Pose2d toPose2d(Vector3 vector3){
        return  new Pose2d(vector3.getX(), vector3.getY(), vector3.getHeading());
    }

}
