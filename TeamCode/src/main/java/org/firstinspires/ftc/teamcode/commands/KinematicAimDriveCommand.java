package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.utils.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.utils.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

import java.util.function.Supplier;

/**
 * Condução com mira cinemática ativa para a meta selecionada.
 *
 * <p>O piloto controla a translação em campo enquanto o robô gira automaticamente em direção à meta
 * virtual — compensando velocidade própria, velocidade da peça, latência de disparo e velocidade
 * tangencial do atirador durante giros.
 *
 * <p>Substitui a condução contínua manual enquanto o gatilho direito estiver pressionado.
 *
 * <p><b>Biobuzz:</b> o alvo agora vem de um {@link Supplier}, lido a cada iteração, em vez de um
 * ponto fixo — porque a CÉLULA pontuável do HIVE muda de lado a cada TIP (ver
 * {@link org.firstinspires.ftc.teamcode.utils.HiveTargets}). Isso também faz o comando reagir na
 * hora se o motorista trocar o lado ativo (Left Bumper no teleop) no meio de uma mira. O comando
 * termina sozinho quando o Right Trigger é solto, devolvendo o controle pro
 * {@link TeleOpDriveCommand} contínuo (prioridade 0) automaticamente via scheduler do Ivy.
 */
public final class KinematicAimDriveCommand {

    private KinematicAimDriveCommand() {
    }

    private static final double ARTIFACT_VELOCITY_INCHES_PER_SEC = 280.0;
    private static final double SYSTEM_LATENCY_SECONDS = 0.05;
    private static final double SHOOTER_RADIUS_INCHES = 5.0;
    private static final double FEEDFORWARD_DEAD_ZONE = Math.toRadians(2.0);
    private static final double VEL_ALPHA = 0.3;
    private static final double TRIGGER_RELEASE_THRESHOLD = 0.2;

    /** Estado que persiste entre iterações. Um por comando construído. */
    private static final class State {
        boolean isAtTarget;
        double smoothedVelX;
        double smoothedVelY;
    }

    /**
     * @param targetSupplier fornece a pose de mundo do alvo atual a cada iteração (ex.:
     *                       {@code HiveTargets::getActiveCellPose}). Lido continuamente, então
     *                       reflete trocas de lado do HIVE em tempo real.
     */
    public static Command kinematicAimDrive(DrivetrainSubsystem drivetrain, Gamepad driver,
                                            Supplier<Pose> targetSupplier) {
        final Follower follower = drivetrain.getFollower();
        final State s = new State();
        final PIDFController turnController = new PIDFController(new PIDFCoefficients(
                ShooterConstants.ANGLE_KP,
                ShooterConstants.ANGLE_KI,
                ShooterConstants.ANGLE_KD,
                0));

        return Command.build()
                .setStart(() -> {
                    turnController.reset();

                    s.smoothedVelX = follower.velocity().vx;
                    s.smoothedVelY = follower.velocity().vy;
                    s.isAtTarget = false;
                })
                .setExecute(() -> {
                    Pose pose = follower.pose();
                    Velocity velocity = follower.velocity();
                    double heading = pose.heading();

                    Pose targetPose = targetSupplier.get();
                    double targetX = targetPose.x();
                    double targetY = targetPose.y();

                    double rawY = driver.left_stick_x;
                    double rawX = -driver.left_stick_y;

                    double targetYInput = rawY * Math.abs(rawY);
                    double targetXInput = rawX * Math.abs(rawX);

                    double xField = targetXInput * Math.cos(heading) - targetYInput * Math.sin(heading);
                    double yField = targetXInput * Math.sin(heading) + targetYInput * Math.cos(heading);

                    if (DataStorage.alliance == AllianceEnum.Blue) {
                        xField = -xField;
                        yField = -yField;
                    }

                    s.smoothedVelX = (VEL_ALPHA * velocity.vx) + ((1 - VEL_ALPHA) * s.smoothedVelX);
                    s.smoothedVelY = (VEL_ALPHA * velocity.vy) + ((1 - VEL_ALPHA) * s.smoothedVelY);

                    double velMagnitude = Math.hypot(s.smoothedVelX, s.smoothedVelY);
                    if (velMagnitude < 2.0) {
                        s.smoothedVelX = 0.0;
                        s.smoothedVelY = 0.0;
                    }

                    double robotX = pose.x();
                    double robotY = pose.y();

                    double diffX = targetX - robotX;
                    double diffY = targetY - robotY;
                    double distanceToTarget = Math.max(Math.hypot(diffX, diffY), 1.0);

                    double targetDirX = diffX / distanceToTarget;
                    double targetDirY = diffY / distanceToTarget;
                    double velTowardsGoal = (s.smoothedVelX * targetDirX) + (s.smoothedVelY * targetDirY);

                    double effectiveArtifactVelocity =
                            Math.max(ARTIFACT_VELOCITY_INCHES_PER_SEC + velTowardsGoal, 100.0);
                    double timeOfFlight = distanceToTarget / effectiveArtifactVelocity;
                    double totalPredictionTime = timeOfFlight + SYSTEM_LATENCY_SECONDS;

                    double angularVel = velocity.omega;
                    double tangentialVelMagnitude = angularVel * SHOOTER_RADIUS_INCHES;
                    double tangentialVelX = tangentialVelMagnitude * -targetDirY;
                    double tangentialVelY = tangentialVelMagnitude * targetDirX;

                    double virtualX = targetX - (s.smoothedVelX * totalPredictionTime)
                            - (tangentialVelX * totalPredictionTime);
                    double virtualY = targetY - (s.smoothedVelY * totalPredictionTime)
                            - (tangentialVelY * totalPredictionTime);

                    double lateralVel = (s.smoothedVelX * -targetDirY) + (s.smoothedVelY * targetDirX);
                    double omegaFeedforward = (lateralVel / distanceToTarget) * ShooterConstants.K_OMEGA;

                    double desiredAngle = Math.atan2(virtualY - robotY, virtualX - robotX);
                    double error = angleDifference(desiredAngle, heading);

                    // O alvo é erro zero, então o erro do controlador é -error.
                    turnController.updateError(-error);
                    double turnPower = turnController.run();

                    double innerTolerance = Math.toRadians(ShooterConstants.ANGLE_TOLERANCE);
                    double outerTolerance = innerTolerance + Math.toRadians(0.4);

                    if (s.isAtTarget) {
                        if (Math.abs(error) > outerTolerance) s.isAtTarget = false;
                    } else {
                        if (Math.abs(error) < innerTolerance) s.isAtTarget = true;
                    }

                    if (!s.isAtTarget && Math.abs(error) > FEEDFORWARD_DEAD_ZONE) {
                        turnPower += Math.copySign(ShooterConstants.ANGLE_KF, turnPower);
                    } else if (s.isAtTarget) {
                        turnPower = 0.0;
                    }

                    turnPower += omegaFeedforward;
                    turnPower = Math.max(-1.0, Math.min(1.0, turnPower));

                    drivetrain.drive(
                            xField,
                            -yField,
                            -turnPower,
                            true
                    );
                })
                .setDone(() -> driver.right_trigger < TRIGGER_RELEASE_THRESHOLD)
                .requiring(drivetrain)
                .setPriority(1);
    }

    private static double angleDifference(double target, double current) {
        double diff = target - current;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }
}
