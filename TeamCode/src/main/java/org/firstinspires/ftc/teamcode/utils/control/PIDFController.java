package org.firstinspires.ftc.teamcode.utils.control;

/**
 * General-purpose closed-loop PIDF Controller for TechMaker #23069.
 * Provides nanosecond-precision delta-time calculation, integral summation,
 * and derivative tracking.
 */
public class PIDFController {
    private PIDFCoefficients coefficients;
    private double error = 0.0;
    private double previousError = 0.0;
    private double totalError = 0.0;
    private long previousTime = 0;
    private boolean hasRun = false;

    public PIDFController(PIDFCoefficients coefficients) {
        this.coefficients = coefficients;
    }

    public void updateError(double error) {
        this.error = error;
    }

    public double run() {
        long currentTime = System.nanoTime();
        if (!hasRun) {
            previousTime = currentTime;
            previousError = error;
            hasRun = true;
            return (coefficients.p * error) + coefficients.f;
        }

        double dt = (currentTime - previousTime) / 1e9;
        if (dt <= 0.0) dt = 1e-4;

        totalError += error * dt;
        double derivative = (error - previousError) / dt;

        previousError = error;
        previousTime = currentTime;

        return (coefficients.p * error) + (coefficients.i * totalError) + (coefficients.d * derivative) + coefficients.f;
    }

    public void reset() {
        error = 0.0;
        previousError = 0.0;
        totalError = 0.0;
        hasRun = false;
        previousTime = 0;
    }

    public void setCoefficients(PIDFCoefficients coefficients) {
        this.coefficients = coefficients;
    }

    public PIDFCoefficients getCoefficients() {
        return coefficients;
    }
}
