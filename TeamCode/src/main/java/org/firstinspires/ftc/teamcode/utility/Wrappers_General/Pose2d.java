package org.firstinspires.ftc.teamcode.utility.Wrappers_General;


import com.acmerobotics.dashboard.config.Config;


@Config
public class Pose2d { // a class that neatly holds a minimum and maximum for a 2d range of values
    public double x;
    public double y;
    public double heading;


    public Pose2d(){
        this.x = 0;
        this.y = 0;
        this.heading = 0;
    }
    public Pose2d(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }


    public void setX(double x){this.x = x;}
    public void setY(double y){this.y = y;}
    public void setHeading(double heading){this.heading = heading;}


    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getHeading() {
        return heading;
    }


    public String toString(){
        return "(" + x + ", " + y + ", " + heading + ")";
    }
}
