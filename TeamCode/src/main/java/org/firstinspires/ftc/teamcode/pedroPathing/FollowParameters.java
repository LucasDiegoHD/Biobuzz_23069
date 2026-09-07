package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

/**
 * Encapsulates path-following configuration with dynamic predictive braking adjustments.
 * Generates native Ivy commands for autonomous routines.
 *
 * @author LucasDiegoHD - Team #23069
 */
public class FollowParameters {

    private final PathChain pathChain;
    private final boolean holdEnd;
    private final double maxPower;
    private final double kP;
    private final double brakingStrength;

    public FollowParameters(PathChain pathChain, boolean holdEnd, double maxPower, double kP, double brakingStrength) {
        this.pathChain = pathChain;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
        this.kP = kP;
        this.brakingStrength = brakingStrength;
    }

    public FollowParameters(PathChain pathChain, boolean holdEnd, double maxPower, double kP) {
        this(pathChain, holdEnd, maxPower, kP, 1.0);
    }

    public FollowParameters(PathChain pathChain, boolean holdEnd) {
        this(pathChain, holdEnd, 1.0, Constants.DEFAULT_PROPORTIONAL, 1.0);
    }

    public FollowParameters(PathChain pathChain, double maxPower) {
        this(pathChain, true, maxPower, Constants.DEFAULT_PROPORTIONAL, 1.0);
    }

    public FollowParameters(double kP, PathChain pathChain) {
        this(pathChain, true, 1.0, kP, 1.0);
    }

    public FollowParameters(double kP, double brakingStrength, PathChain pathChain) {
        this(pathChain, true, 1.0, kP, brakingStrength);
    }

    public FollowParameters(PathChain pathChain) {
        this(pathChain, true, 1.0, Constants.DEFAULT_PROPORTIONAL, 1.0);
    }

    /**
     * Executes the path with the pre-configured braking strength on the given follower.
     *
     * @param follower The Pedro Follower instance.
     */
    public void follow(Follower follower) {
        follow(follower, brakingStrength);
    }

    /**
     * Dynamically modulates predictive braking coefficients and follows the path.
     *
     * @param follower        The Pedro Follower instance.
     * @param brakingStrength Multiplier for braking strength (1.0 = nominal, >1 = stronger, <1 = looser).
     */
    public void follow(Follower follower, double brakingStrength) {
        double scaledBraking = (brakingStrength == 0) ? 1.0 : (1.0 / brakingStrength);
        follower.vectorCalculator.predictiveBrakingController.setCoefficients(
                new PredictiveBrakingCoefficients(
                        kP,
                        Constants.K_LINEAR_BRAKE * scaledBraking,
                        Constants.K_QUADRATIC_BRAKE * scaledBraking
                )
        );
        follower.followPath(pathChain, maxPower, holdEnd);
    }

    /**
     * Returns an Ivy Command that executes this path with a default completion constraint of t &gt; 0.975.
     *
     * @param drivetrain The drivetrain subsystem.
     * @return An executable Ivy Command.
     */
    public Command followCommand(DrivetrainSubsystem drivetrain) {
        return followCommand(drivetrain, 0.975);
    }

    /**
     * Returns an Ivy Command that executes this path with a custom parametric progress threshold.
     *
     * @param drivetrain  The drivetrain subsystem.
     * @param tConstraint The parametric threshold (e.g. 0.95) to declare the command finished.
     * @return An executable Ivy Command.
     */
    public Command followCommand(DrivetrainSubsystem drivetrain, double tConstraint) {
        return Command.build()
                .setStart(() -> follow(drivetrain.getFollower()))
                .setDone(() -> drivetrain.getFollower().getCurrentTValue() > tConstraint);
    }

    /**
     * Returns an Ivy Command that completes when the robot slows below the given linear velocity.
     *
     * @param drivetrain           The drivetrain subsystem.
     * @param maxVelocityThreshold Linear velocity threshold in inches/second.
     * @return An executable Ivy Command.
     */
    public Command followVelocityCommand(DrivetrainSubsystem drivetrain, double maxVelocityThreshold) {
        return Command.build()
                .setStart(() -> follow(drivetrain.getFollower()))
                .setDone(() -> drivetrain.velocityCondition(maxVelocityThreshold));
    }

    public PathChain getPathChain() {
        return pathChain;
    }

    public boolean isHoldEnd() {
        return holdEnd;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public double getKP() {
        return kP;
    }

    public double getBrakingStrength() {
        return brakingStrength;
    }
}
