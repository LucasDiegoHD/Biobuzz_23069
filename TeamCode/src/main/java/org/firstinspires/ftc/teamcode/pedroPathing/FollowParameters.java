package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

/**
 * Encapsulates path-following configuration.
 * Generates native Ivy commands for autonomous routines.
 * Upgraded for Pedro Pathing 3.
 *
 * @author LucasDiegoHD - Team #23069
 */
public class FollowParameters {

    private final Path path;
    private final boolean holdEnd;
    private final double maxPower;

    public FollowParameters(Path path, boolean holdEnd, double maxPower) {
        this.path = path;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }

    public FollowParameters(Path path, boolean holdEnd) {
        this(path, holdEnd, 1.0);
    }

    public FollowParameters(Path path, double maxPower) {
        this(path, true, maxPower);
    }

    public FollowParameters(Path path) {
        this(path, true, 1.0);
    }

    public void follow(Follower follower) {
        follower.follow(path);
    }

    public Command followCommand(DrivetrainSubsystem drivetrain) {
        return followCommand(drivetrain, 0.975);
    }

    public Command followCommand(DrivetrainSubsystem drivetrain, double tConstraint) {
        return Command.build()
                .setStart(() -> follow(drivetrain.getFollower()))
                .setDone(() -> drivetrain.getFollower().parametricCompletion() > tConstraint);
    }

    public Command followVelocityCommand(DrivetrainSubsystem drivetrain, double maxVelocityThreshold) {
        return Command.build()
                .setStart(() -> follow(drivetrain.getFollower()))
                .setDone(() -> drivetrain.velocityCondition(maxVelocityThreshold));
    }

    public Path getPath() {
        return path;
    }

    public boolean isHoldEnd() {
        return holdEnd;
    }

    public double getMaxPower() {
        return maxPower;
    }
}
