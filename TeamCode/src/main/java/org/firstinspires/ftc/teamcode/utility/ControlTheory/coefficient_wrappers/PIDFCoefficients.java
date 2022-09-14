package org.firstinspires.ftc.teamcode.utility.ControlTheory.coefficient_wrappers;


import com.acmerobotics.dashboard.config.Config;


public class PIDFCoefficients {
    public double p;
    public double i;
    public double d;
    public double f;


    public PIDFCoefficients() {
        this.p = this.i = this.d = 0;
    }

    public PIDFCoefficients(PIDCoefficients pidCoefficients, double f){
        this.p = pidCoefficients.p;
        this.i = pidCoefficients.i;
        this.d = pidCoefficients.d;
        this.f = f;
    }

    public PIDFCoefficients(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    @Override public String toString() {
        return "PIDF-Coeff(" + p + ", " + i + ", " + d + ", " + f + ")";
    }
}