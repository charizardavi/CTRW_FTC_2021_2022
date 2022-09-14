package org.firstinspires.ftc.teamcode.utility.ControlTheory.filters;


// Single Input Single Output Kalman Filter
public class SISOKalmanFilter { // smooths two inputs together and accounts for different sensors being less accorate than others
    double x = 0; // your initial state
    double Q = 0.1; // your model covariance (input by user or left at default)
    double R = 0.4; // your sensor covariance (input by user or left at default)
    double p = 1; // your initial covariance guess
    double K = 1; // your initial kalman gain guess

    double previousXInput = x;
    double previousCovaraiance = p;


    public SISOKalmanFilter(double modelCovariance, double sensorCovariance){
        this.Q = modelCovariance;
        this.R = sensorCovariance;
    }
    public SISOKalmanFilter(){}


    public double filter(double sensorInput1, double sensorInput2){ // mathematically weight the previous state and the current state
        x = previousXInput + sensorInput1;

        p = previousCovaraiance + Q;

        K = p/(p + R);

        x = x + K * (sensorInput2 - x);

        p = 1 - K * p;

        previousXInput = x;
        previousCovaraiance = p;

        return x;
    }


    public double getModelCovariance(){return Q;}
    public double getSensorCovariance(){return R;}
    public void setModelCovariance(double covariance){this.Q = covariance;}
    public void setSensorCovariance(double covariance){this.R = covariance;}

}
