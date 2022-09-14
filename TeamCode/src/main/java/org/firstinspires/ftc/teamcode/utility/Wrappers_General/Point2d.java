package org.firstinspires.ftc.teamcode.utility.Wrappers_General;


import com.acmerobotics.dashboard.config.Config;

@Config

public class Point2d { // a class that neatly holds a minimum and maximum for a 2d range of values
    public double x;
    public double y;


    public Point2d(){
        this.x = 0;
        this.y = 0;
    }
    public Point2d(double x, double y){
        this.x = x;
        this.y = y;
    }
    public Point2d(TimestampedValue value){
        this.x = value.timestamp;
        this.y = value.value;
    }
    public Point2d(Range2d range){
        this.x = range.min;
        this.y = range.max;
    }

    public void setX(double x){this.x = x;}
    public void setY(double y){this.y = y;}

    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }

    public String toString(){
        return "(" + x + ", " + y + ")";
    }

    public Point2d plus(Point2d other){
        return new Point2d(x + other.x, y + other.y);
    }
    public Point2d minus(Point2d other){
        return new Point2d(x - other.x, y - other.y);
    }
    public Point2d times(Point2d other){
        return new Point2d(x * other.x, y * other.y);
    }
    public Point2d dividedBy(Point2d other){
        return new Point2d(x / other.x, y / other.y);
    }
}
