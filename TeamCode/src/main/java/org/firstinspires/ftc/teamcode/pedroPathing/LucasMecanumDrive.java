package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.pedropathing.math.MathFunctions.findNormalizingScaling;

import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Advanced Mecanum drivetrain vector solver with anti-slip power balancing.
 * Dynamically normalizes pathing, heading, and corrective vectors to prevent
 * wheel slip, traction loss, and motor clipping during high-speed maneuvers.
 *
 * @author LucasDiegoHD - Team #23069
 */
public class LucasMecanumDrive extends Mecanum {

    public LucasMecanumDrive(HardwareMap hardwareMap, MecanumConstants mecanumConstants) {
        super(hardwareMap, mecanumConstants);
    }

    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // Clamp down input vector magnitudes to max power scaling
        if (correctivePower.getMagnitude() > maxPowerScaling) {
            correctivePower.setMagnitude(maxPowerScaling);
        }
        if (headingPower.getMagnitude() > maxPowerScaling) {
            headingPower.setMagnitude(maxPowerScaling);
        }
        if (pathingPower.getMagnitude() > maxPowerScaling) {
            pathingPower.setMagnitude(maxPowerScaling);
        }

        double[] wheelPowers = new double[4];
        Vector[] mecanumVectorsCopy = new Vector[4];
        Vector[] truePathingVectors = new Vector[2];

        if (pathingPower.getMagnitude() == maxPowerScaling) {
            truePathingVectors[0] = pathingPower.copy();
            truePathingVectors[1] = pathingPower.copy();
        } else {
            Vector leftSideVector = pathingPower.minus(headingPower);
            Vector rightSideVector = pathingPower.plus(headingPower);

            if (leftSideVector.getMagnitude() > maxPowerScaling || rightSideVector.getMagnitude() > maxPowerScaling) {
                // If combined corrective and heading exceeds 1.0, scale down heading power smoothly
                double headingScalingFactor = Math.min(
                        findNormalizingScaling(pathingPower, headingPower, maxPowerScaling),
                        findNormalizingScaling(pathingPower, headingPower.times(-1), maxPowerScaling)
                );
                truePathingVectors[0] = pathingPower.minus(headingPower.times(headingScalingFactor));
                truePathingVectors[1] = pathingPower.plus(headingPower.times(headingScalingFactor));
            } else {
                Vector leftSideVectorWithPathing = leftSideVector.plus(correctivePower);
                Vector rightSideVectorWithPathing = rightSideVector.plus(correctivePower);

                if (leftSideVectorWithPathing.getMagnitude() > maxPowerScaling || rightSideVectorWithPathing.getMagnitude() > maxPowerScaling) {
                    // Scale down corrective pathing vector smoothly
                    double pathingScalingFactor = Math.min(
                            findNormalizingScaling(leftSideVector, correctivePower, maxPowerScaling),
                            findNormalizingScaling(rightSideVector, correctivePower, maxPowerScaling)
                    );
                    truePathingVectors[0] = leftSideVector.plus(correctivePower.times(pathingScalingFactor));
                    truePathingVectors[1] = rightSideVector.plus(correctivePower.times(pathingScalingFactor));
                } else {
                    truePathingVectors[0] = leftSideVectorWithPathing.copy();
                    truePathingVectors[1] = rightSideVectorWithPathing.copy();
                }
            }
        }

        truePathingVectors[0] = truePathingVectors[0].times(2.0);
        truePathingVectors[1] = truePathingVectors[1].times(2.0);

        for (int i = 0; i < mecanumVectorsCopy.length; i++) {
            mecanumVectorsCopy[i] = vectors[i].copy();
            mecanumVectorsCopy[i].rotateVector(robotHeading);
        }

        wheelPowers[0] = (mecanumVectorsCopy[1].getXComponent() * truePathingVectors[0].getYComponent() - truePathingVectors[0].getXComponent() * mecanumVectorsCopy[1].getYComponent()) / (mecanumVectorsCopy[1].getXComponent() * mecanumVectorsCopy[0].getYComponent() - mecanumVectorsCopy[0].getXComponent() * mecanumVectorsCopy[1].getYComponent());
        wheelPowers[1] = (mecanumVectorsCopy[0].getXComponent() * truePathingVectors[0].getYComponent() - truePathingVectors[0].getXComponent() * mecanumVectorsCopy[0].getYComponent()) / (mecanumVectorsCopy[0].getXComponent() * mecanumVectorsCopy[1].getYComponent() - mecanumVectorsCopy[1].getXComponent() * mecanumVectorsCopy[0].getYComponent());
        wheelPowers[2] = (mecanumVectorsCopy[3].getXComponent() * truePathingVectors[1].getYComponent() - truePathingVectors[1].getXComponent() * mecanumVectorsCopy[3].getYComponent()) / (mecanumVectorsCopy[3].getXComponent() * mecanumVectorsCopy[2].getYComponent() - mecanumVectorsCopy[2].getXComponent() * mecanumVectorsCopy[3].getYComponent());
        wheelPowers[3] = (mecanumVectorsCopy[2].getXComponent() * truePathingVectors[1].getYComponent() - truePathingVectors[1].getXComponent() * mecanumVectorsCopy[2].getYComponent()) / (mecanumVectorsCopy[2].getXComponent() * mecanumVectorsCopy[3].getYComponent() - mecanumVectorsCopy[3].getXComponent() * mecanumVectorsCopy[2].getYComponent());

        double wheelPowerMax = Math.max(
                Math.max(Math.abs(wheelPowers[0]), Math.abs(wheelPowers[1])),
                Math.max(Math.abs(wheelPowers[2]), Math.abs(wheelPowers[3]))
        );

        if (wheelPowerMax > maxPowerScaling) {
            wheelPowers[0] = (wheelPowers[0] / wheelPowerMax) * maxPowerScaling;
            wheelPowers[1] = (wheelPowers[1] / wheelPowerMax) * maxPowerScaling;
            wheelPowers[2] = (wheelPowers[2] / wheelPowerMax) * maxPowerScaling;
            wheelPowers[3] = (wheelPowers[3] / wheelPowerMax) * maxPowerScaling;
        }

        return wheelPowers;
    }
}
