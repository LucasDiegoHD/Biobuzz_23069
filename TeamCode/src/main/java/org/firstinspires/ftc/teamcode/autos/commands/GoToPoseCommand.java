package org.firstinspires.ftc.teamcode.autos.commands;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

import java.util.Arrays;
import java.util.List;

/**
 * Builds and follows a trajectory from the robot's current pose to one or more waypoints.
 * Upgraded for Pedro Pathing 3 Paths API.
 *
 * <p><b>Biobuzz — holdEnd de verdade:</b> na v3.0.0 real de vocês, {@code holdEnd} não é
 * parâmetro do {@code Follower.follow(path)} — é um {@code ConfigVar<Boolean>} GLOBAL do
 * Follower ({@code follower.holdEnd}), lido toda vez que um path termina. Por isso setamos
 * aqui, logo antes de cada {@code follow()}.
 *
 * <p><b>Constraints (setConstraints/withMaxPower):</b> descontinuado de propósito — era o jeito
 * de controlar velocidade de path no Pedro v2. No v3 o {@code Foresight} já faz frenagem
 * preditiva baseada nos coeficientes medidos (braking/deceleration reais), então não faz falta
 * um teto de velocidade manual pra precisão. Os métodos continuam existindo (não quebra nada
 * que já chama {@code .setConstraints(...)}), só não fazem mais nada. Pra reativar no futuro:
 * anexar {@code path.with(Constants.foresightConfig.maxPathSpeed.at(fração))} em
 * {@link #followPath()}, antes de {@code follow(path)} — é um {@code Modifier}, aplicado e
 * revertido sozinho pelo {@code PathTracker} só durante aquele path.
 *
 * @author LucasDiegoHD - Team #23069
 */
public class GoToPoseCommand {
    private final DrivetrainSubsystem drivetrain;
    private final List<Pose> waypoints;
    private final boolean holdEnd;

    // Heading Modes
    public enum HeadingMode { LINEAR, TANGENT, CONSTANT }
    private HeadingMode headingMode = HeadingMode.LINEAR;
    private double customConstantHeading = Double.NaN;

    private double exitTolerance = -1.0;
    private boolean useVelocityCondition = false;
    private double velocityExitTolerance = 4.0;

    public GoToPoseCommand(DrivetrainSubsystem drivetrain, Pose targetPose) {
        this(drivetrain, true, targetPose);
    }

    public GoToPoseCommand(DrivetrainSubsystem drivetrain, boolean holdEnd, Pose targetPose) {
        this(drivetrain, holdEnd, new Pose[]{targetPose});
    }

    public GoToPoseCommand(DrivetrainSubsystem drivetrain, boolean holdEnd, Pose... poses) {
        this.drivetrain = drivetrain;
        this.holdEnd = holdEnd;
        this.waypoints = Arrays.asList(poses);
    }

    /** Descontinuado (ver javadoc da classe) — mantido só por compatibilidade, não faz nada. */
    public GoToPoseCommand setConstraints(double maxPathSpeedFraction) {
        return this;
    }

    /** Descontinuado (ver javadoc da classe) — mantido só por compatibilidade, não faz nada. */
    public GoToPoseCommand withMaxPower(double maxPower) {
        return this;
    }

    public GoToPoseCommand withNoDeceleration() {
        return this;
    }

    public GoToPoseCommand withGlobalDeceleration() {
        return this;
    }

    public GoToPoseCommand withTangentHeading() {
        this.headingMode = HeadingMode.TANGENT;
        return this;
    }

    public GoToPoseCommand withConstantHeading() {
        this.headingMode = HeadingMode.CONSTANT;
        this.customConstantHeading = Double.NaN;
        return this;
    }

    public GoToPoseCommand withConstantHeading(double heading) {
        this.headingMode = HeadingMode.CONSTANT;
        this.customConstantHeading = heading;
        return this;
    }

    public GoToPoseCommand withExitTolerance(double inches) {
        this.exitTolerance = inches;
        return this;
    }

    /**
     * Enables early exit handoff by checking velocity condition.
     *
     * @param inches Arrival radius in inches.
     */
    public GoToPoseCommand withVelocityExit(double inches) {
        this.useVelocityCondition = true;
        this.velocityExitTolerance = inches;
        return this;
    }

    /** Builds and returns the schedulable Ivy command. */
    public Command toCommand() {
        return Command.build()
                .setStart(this::followPath)
                .setExecute(() -> drivetrain.getFollower().update())
                .setDone(this::isDone)
                .requiring(drivetrain);
    }

    private void followPath() {
        if (waypoints.isEmpty()) return;

        drivetrain.getFollower().holdEnd.set(holdEnd);

        Pose startPose = drivetrain.getFollower().pose();
        int size = waypoints.size();
        Path path;

        if (size == 1) {
            path = Paths.line(startPose, waypoints.get(0));
        } else {
            Pose[] allPoses = new Pose[size + 1];
            allPoses[0] = startPose;
            for (int i = 0; i < size; i++) {
                allPoses[i + 1] = waypoints.get(i);
            }
            path = Paths.curve(allPoses);
        }

        Pose lastPose = waypoints.get(size - 1);
        switch (headingMode) {
            case TANGENT:
                path = path.tangent();
                break;
            case CONSTANT:
                double targetAngle = Double.isNaN(customConstantHeading)
                        ? waypoints.get(0).heading()
                        : customConstantHeading;
                path = path.constant(targetAngle);
                break;
            case LINEAR:
            default:
                path = path.linear(startPose.heading(), lastPose.heading());
                break;
        }

        drivetrain.getFollower().follow(path);
    }

    private boolean isDone() {
        if (useVelocityCondition) {
            return drivetrain.velocityCondition(velocityExitTolerance);
        }

        if (exitTolerance > 0 && !waypoints.isEmpty()) {
            Pose currentPose = drivetrain.getFollower().pose();
            Pose targetPose = waypoints.get(waypoints.size() - 1);

            double distance = Math.hypot(
                    currentPose.x() - targetPose.x(),
                    currentPose.y() - targetPose.y()
            );

            if (distance <= exitTolerance) {
                return true;
            }
        }

        return !drivetrain.getFollower().isBusy();
    }
}
