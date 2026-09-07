package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.autos.paths.RearAutoPaths;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

import java.util.List;

/**
 * Robot container wiring the core chassis hardware and central control loop.
 *
 * <p>Streamlined exclusively to {@link DrivetrainSubsystem} for new season chassis
 * testing and calibration. Manages LynxModule manual bulk caching to ensure minimum latency.
 *
 * @author LucasDiegoHD - Team #23069
 */
public class Robot {

    public final DrivetrainSubsystem drivetrain;

    private final List<LynxModule> allHubs;
    private long lastLoopTime = 0;

    public Robot(HardwareMap hardwareMap, TelemetryManager telemetry) {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drivetrain = new DrivetrainSubsystem(hardwareMap, telemetry);
    }

    /**
     * Continuous robot control loop executed once per cycle by {@link RobotOpMode}.
     * Clears LynxModule bulk cache at the start of every cycle.
     */
    public void update() {
        clearBulkCache();
        drivetrain.update();
    }

    /** Clears the bulk cache of all LynxModules. Called at the start of {@link #update()}. */
    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public boolean isShooting() {
        return false;
    }

    /**
     * Configures the initial pose for TeleOp: loads persisted autonomous pose if valid,
     * otherwise falls back to the default alliance starting pose.
     */
    public void applyTeleOpStartPose(AllianceEnum alliance) {
        Pose savedPose = (DataStorage.actualPose != null) ? DataStorage.actualPose : PoseStorage.loadPose();

        if (savedPose != null && !Double.isNaN(savedPose.getX()) && !Double.isNaN(savedPose.getY())) {
            drivetrain.getFollower().setPose(savedPose);
        } else {
            drivetrain.getFollower().setPose(startPoseFor(alliance));
        }
    }

    /** Default starting pose for the specified alliance. */
    public static Pose startPoseFor(AllianceEnum alliance) {
        DataStorage.alliance = alliance;
        return RearAutoPaths.START_POSE.getPose();
    }

    /** Sets the starting pose prior to autonomous execution. */
    public void setAutoStartPose(Pose startPose) {
        drivetrain.getFollower().setStartingPose(startPose);
        drivetrain.getFollower().setPose(startPose);
    }

    public void printLoopTime() {
        long currentTime = System.currentTimeMillis();
        long loopDelta = currentTime - lastLoopTime;

        PanelsTelemetry.INSTANCE.getTelemetry().addData("⚡ Loop Time (ms)", loopDelta);
        lastLoopTime = currentTime;
    }
}
