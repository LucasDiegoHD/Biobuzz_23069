package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;
import androidx.annotation.NonNull;

import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/**
 * Relocalização por Limelight (MegaTag2).
 *
 * <p>Sem requirements de propósito: relocalizar não deve travar nenhum outro comando de visão ou
 * de drive.
 */
public final class UpdatePoseLimelightCommand {

    private UpdatePoseLimelightCommand() {
    }

    /**
     * Reposiciona o robô a partir da Limelight, com três casos distintos:
     * primeiro boot (aceita qualquer salto), operação normal (funde odometria e Limelight se o
     * salto for pequeno) e leitura suspeita (salto gigante, ignorada).
     */
    public static Command updatePoseLimelight(DrivetrainSubsystem drivetrain, VisionSubsystem vision,
                                              Pose fallbackPose) {
        return Commands.instant(() -> {
            Pose currentPose = drivetrain.getPose();

            vision.getRobotPoseMT2(currentPose.heading()).ifPresent(llPoseMT2 -> {

                double distInches = Math.hypot(
                        llPoseMT2.x() - currentPose.x(),
                        llPoseMT2.y() - currentPose.y()
                );
                double maxDeltaInches = VisionConstants.MAX_DELTA_METERS * VisionConstants.METERS_TO_INCHES;

                // CASO 1: O robô acabou de ligar (Está literalmente no 0,0)
                if (Math.abs(currentPose.x()) < 0.1 && Math.abs(currentPose.y()) < 0.1) {
                    drivetrain.setPose(
                            new Pose(llPoseMT2.x(), llPoseMT2.y(), fallbackPose.heading()));
                    Log.i("Vision", "Primeira inicialização via Limelight (Ignorando limite de pulo)");
                }
                // CASO 2: O robô já está andando. Só atualiza se o pulo for pequeno!
                else if (distInches < maxDeltaInches) {
                    drivetrain.setPose(getFusedPose(currentPose, llPoseMT2));
                    Log.d("Vision", "Pose atualizada via Fusão Limelight");
                }
                // CASO 3: A Limelight mentiu (Pulo gigante)
                else {
                    Log.w("Vision", "MT2 Ignorada: Pulo gigante evitado (" + distInches + " in)");
                }
            });
        });
    }

    /**
     * Redefine heading e pose imediatamente, sem passar pelo escalonador. Usado pelo botão START
     * do piloto, quando o robô é reposicionado à mão em quadra.
     */
    public static void forceHardReset(DrivetrainSubsystem drive, VisionSubsystem vis, double targetHeadingDegrees) {
        double targetHeadingRad = Math.toRadians(targetHeadingDegrees);
        Pose currentPose = drive.getPose();

        drive.setPose(new Pose(currentPose.x(), currentPose.y(), targetHeadingRad));

        vis.getRobotPoseMT2(targetHeadingRad).ifPresent(mt2Pose -> {
            drive.setPose(new Pose(
                    mt2Pose.x(),
                    mt2Pose.y(),
                    targetHeadingRad
            ));
            Log.i("Vision", "HARD RESET: Posição atualizada via Limelight");
        });
    }

    /** Funde X e Y por peso. O heading vem sempre do Pinpoint — a Limelight nunca o toca. */
    @NonNull
    private static Pose getFusedPose(Pose currentPose, Pose llPose) {
        double wOdo = VisionConstants.ODOMETRY_WEIGHT;
        double wLL = VisionConstants.LIMELIGHT_WEIGHT;
        double total = wOdo + wLL;

        double fusedX = (currentPose.x() * wOdo + llPose.x() * wLL) / total;
        double fusedY = (currentPose.y() * wOdo + llPose.y() * wLL) / total;

        return new Pose(fusedX, fusedY, currentPose.heading());
    }

    public static void resetLocalizationStatus() {
    }
}
