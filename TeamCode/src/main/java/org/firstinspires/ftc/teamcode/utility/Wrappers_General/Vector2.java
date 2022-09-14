package org.firstinspires.ftc.teamcode.utility.Wrappers_General;


public class Vector2 { // a class that neatly holds a minimum and maximum for a 2d range of values
    public double x;
    public double y;


    public Vector2(){
        this.x = 0;
        this.y = 0;
    }
    public Vector2(double x, double y){
        this.x = x;
        this.y = y;
    }
    public Vector2(TimestampedValue value){
        this.x = value.timestamp;
        this.y = value.value;
    }
    public Vector2(Range2d range){
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

    public Vector2 plus(Vector2 other){
        return new Vector2(x + other.x, y + other.y);
    }
    public Vector2 minus(Vector2 other){
        return new Vector2(x - other.x, y - other.y);
    }
    public Vector2 times(Vector2 other){
        return new Vector2(x * other.x, y * other.y);
    }
    public Vector2 dividedBy(Vector2 other){
        return new Vector2(x / other.x, y / other.y);
    }
}
