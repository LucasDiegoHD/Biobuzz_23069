package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autos.paths.BiobuzzAutoPaths;
import org.firstinspires.ftc.teamcode.commands.KinematicAimDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;
import org.firstinspires.ftc.teamcode.utils.HiveTargets;

import java.util.Locale;

/**
 * Dedicated TeleOp opmode focused purely on drivetrain control, sensor logging, and calibration.
 *
 * <p>Runs continuous high-performance field-centric drive through {@link TeleOpDriveCommand}
 * with closed-loop heading lock, cardinal snapping, and automated kick trajectory. Telemetry dispatch is delegated
 * solely to {@code super.loop()} to maximize loop execution frequency (Hz) for odometry tracking.
 *
 * <p><b>Biobuzz — mira no HIVE:</b>
 * <ul>
 *   <li><b>Right Trigger (segurar)</b>: ativa {@link KinematicAimDriveCommand}, mirando
 *       automaticamente na CÉLULA do HIVE atualmente marcada como voltada pra cima. Solta
 *       sozinho ao largar o gatilho (ver comentário na própria classe).</li>
 *   <li><b>Left Bumper (toque)</b>: alterna qual lado do HIVE (plateia / oposto) está marcado
 *       como voltado pra cima — use quando perceber ou causar um HIVE TIP.</li>
 * </ul>
 *
 * @author LucasDiegoHD - TechMaker (#23069)
 */
@TeleOp(name = "TeleOp - Drivetrain & Calibration", group = "Competition")
public class teleop extends RobotOpMode {

    private AllianceEnum alliance;
    private long lastLoopTime = 0;
    private boolean wasAiming = false;

    @Override
    public void start() {
        alliance = DataStorage.alliance;
        if (DataStorage.actualPose != null) {
            robot.applyTeleOpStartPose(alliance);
        } else {
            robot.drivetrain.getFollower().setPose(BiobuzzAutoPaths.getStartPose());
        }

        TeleOpDriveCommand.teleOpDrive(robot.drivetrain, gamepad1).schedule();
    }

    @Override
    public void loop() {
        long currentTime = System.currentTimeMillis();
        long loopDelta = (lastLoopTime > 0) ? (currentTime - lastLoopTime) : 10;
        lastLoopTime = currentTime;
        double loopHz = (loopDelta > 0) ? (1000.0 / loopDelta) : 0;

        // Press BACK to reset the complete pose to calibrated corner (heading 90°, X/Y = metade
        // do robô) — mesma convenção documentada em Constants.CORNER_RESET_POSE.
        if (gamepad1.backWasPressed()) {
            robot.drivetrain.getFollower().setPose(Constants.CORNER_RESET_POSE);
            robot.drivetrain.unlockHeading();
            robot.drivetrain.cancelKick();
            gamepad1.rumble(300);
        }

        // Guide / Logo button for bench-testing Robot-Centric mode toggle if ever needed
        if (gamepad1.guideWasPressed()) {
            robot.drivetrain.setFieldCentric(!robot.drivetrain.isFieldCentric());
            gamepad1.rumble(200);
        }

        // Left Bumper: alterna qual CÉLULA do HIVE está marcada como voltada pra cima (pontuável).
        // Use quando o HIVE tombar (TIP) — não temos sensor pra isso ainda, é manual por enquanto.
        if (gamepad1.leftBumperWasPressed()) {
            HiveTargets.toggleActiveCell();
            gamepad1.rumble(120);
        }

        // Right Trigger (segurar): mira cinemática ativa na célula ativa do HIVE.
        // Prioridade 1 sobre o TeleOpDriveCommand contínuo — suspende e retoma sozinho.
        boolean isAimingNow = gamepad1.right_trigger > 0.2;
        if (isAimingNow && !wasAiming) {
            KinematicAimDriveCommand
                    .kinematicAimDrive(robot.drivetrain, gamepad1, () -> HiveTargets.getActiveCellPose(alliance))
                    .schedule();
        }
        wasAiming = isAimingNow;

        Pose pose = robot.drivetrain.getFollower().pose();
        Velocity vel = robot.drivetrain.getFollower().velocity();
        double linearVel = (vel != null) ? vel.toVector2D().magnitude() : 0.0;
        double headingDeg = Math.toDegrees(pose.heading());

        String assistStatus;
        if (isAimingNow) {
            assistStatus = "MIRANDO NO HIVE (Right Trigger)";
        } else if (robot.drivetrain.isKicking()) {
            assistStatus = "KICK ATIVO (Indo para Zona de Chute)";
        } else if (robot.drivetrain.isHeadingLocked()) {
            assistStatus = String.format(Locale.US, "TRAVADO em %.0f° (Target: %.2f rad)",
                    Math.toDegrees(robot.drivetrain.getTargetHeading()), robot.drivetrain.getTargetHeading());
        } else {
            assistStatus = "LIVRE (Manual)";
        }
        telemetryM.addData("Assistência", assistStatus);

        telemetryM.addData("Active Alliance", alliance == AllianceEnum.Red ? "🔴 Red" : "🔵 Blue");
        telemetryM.addData("Célula HIVE ativa", DataStorage.activeCellIsAudienceSide ? "Lado da Plateia" : "Lado Oposto à Plateia");
        telemetryM.addData("Pose (X, Y)", String.format(Locale.US, "X: %.2f in | Y: %.2f in", pose.x(), pose.y()));
        telemetryM.addData("Heading", String.format(Locale.US, "%.1f deg (%.3f rad)", headingDeg, pose.heading()));
        telemetryM.addData("Linear Speed", String.format(Locale.US, "%.2f in/s", linearVel));
        telemetryM.addData("Battery", String.format(Locale.US, "%.2f V", robot.drivetrain.getVoltage()));
        telemetryM.addData("--- LOOP PERFORMANCE ---", "");
        telemetryM.addData("Loop Time", loopDelta + " ms");
        telemetryM.addData("Loop Frequency", String.format(Locale.US, "%.1f Hz", loopHz));

        super.loop();
    }
}
