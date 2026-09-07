package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

import java.util.Locale;

/**
 * Dedicated TeleOp opmode focused purely on drivetrain control, sensor logging, and calibration.
 *
 * <p>Runs continuous field-centric drive through {@link TeleOpDriveCommand} using vector-balanced
 * {@link org.firstinspires.ftc.teamcode.pedroPathing.LucasMecanumDrive}. Telemetry dispatch is delegated
 * solely to {@code super.loop()} to maximize loop execution frequency (Hz) for odometry tracking.
 *
 * @author LucasDiegoHD - Team #23069
 */
@TeleOp(name = "🎮 TeleOp - Drivetrain & Calibration", group = "Competition")
public class teleop extends RobotOpMode {

    private AllianceEnum alliance;
    private long lastLoopTime = 0;

    @Override
    public void start() {
        alliance = DataStorage.alliance;
        robot.applyTeleOpStartPose(alliance);

        TeleOpDriveCommand.teleOpDrive(robot.drivetrain, gamepad1).schedule();
    }

    @Override
    public void loop() {
        long currentTime = System.currentTimeMillis();
        long loopDelta = (lastLoopTime > 0) ? (currentTime - lastLoopTime) : 10;
        lastLoopTime = currentTime;
        double loopHz = (loopDelta > 0) ? (1000.0 / loopDelta) : 0;

        // Press START to re-zero heading against field perimeter without altering (X, Y)
        if (gamepad1.startWasPressed()) {
            Pose currentPose = robot.drivetrain.getFollower().getPose();
            robot.drivetrain.getFollower().setPose(new Pose(currentPose.getX(), currentPose.getY(), 0.0));
            gamepad1.rumble(150);
        }

        // Press BACK to reset the complete pose to (0, 0, 0) for distance measurement tests
        if (gamepad1.backWasPressed()) {
            robot.drivetrain.getFollower().setPose(new Pose(0, 0, 0));
            gamepad1.rumble(300);
        }

        Pose pose = robot.drivetrain.getFollower().getPose();
        Vector vel = robot.drivetrain.getFollower().getVelocity();
        double linearVel = (vel != null) ? vel.getMagnitude() : 0.0;
        double headingDeg = Math.toDegrees(pose.getHeading());

        telemetryM.addData("--- TECHMAKER DRIVETRAIN (#23069) ---", "");
        telemetryM.addData("Active Alliance", alliance == AllianceEnum.Red ? "🔴 Red" : "🔵 Blue");
        telemetryM.addData("Pose (X, Y)", String.format(Locale.US, "X: %.2f in | Y: %.2f in", pose.getX(), pose.getY()));
        telemetryM.addData("Heading", String.format(Locale.US, "%.1f deg (%.3f rad)", headingDeg, pose.getHeading()));
        telemetryM.addData("Linear Speed", String.format(Locale.US, "%.2f in/s", linearVel));
        telemetryM.addData("Battery", String.format(Locale.US, "%.2f V", robot.drivetrain.getVoltage()));
        telemetryM.addData("--- LOOP PERFORMANCE ---", "");
        telemetryM.addData("Loop Time", loopDelta + " ms");
        telemetryM.addData("Loop Frequency", String.format(Locale.US, "%.1f Hz", loopHz));

        super.loop();
    }
}
