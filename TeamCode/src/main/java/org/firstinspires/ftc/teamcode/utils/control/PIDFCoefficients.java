package org.firstinspires.ftc.teamcode.utils.control;

/**
 * Coefficients for PIDF control loops.
 * Team: TechMaker #23069.
 */
public class PIDFCoefficients {
    public double p;
    public double i;
    public double d;
    public double f;

    public PIDFCoefficients(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }
}
