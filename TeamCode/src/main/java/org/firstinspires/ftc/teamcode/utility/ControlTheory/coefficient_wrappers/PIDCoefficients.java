package org.firstinspires.ftc.teamcode.utility.ControlTheory.coefficient_wrappers;

import com.acmerobotics.dashboard.config.Config;


public class PIDCoefficients {
    public double p;
    public double i;
    public double d;

    public PIDCoefficients() {
        this.p = this.i = this.d = 0;
    }

    public PIDCoefficients(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    @Override
    public String toString() {
        return "PID-Coeff(" + p + ", " + i + ", " + d + ")";
    }
}