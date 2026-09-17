package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.autos.paths.BiobuzzAutoPaths;
import org.firstinspires.ftc.teamcode.commands.AlignToAprilTagCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

import java.util.List;

/**
 * Robot container wiring the core chassis hardware, mechanisms, and central control loop.
 * Promoted Shooter, Indexer, Intake, Vision, and LED to active first-class subsystems.
 *
 * @author LucasDiegoHD - Team #23069
 */
public class Robot {

    public final DrivetrainSubsystem drivetrain;
    public ShooterSubsystem shooter;
    public IndexerSubsystem indexer;
    public IntakeSubsystem intake;
    public VisionSubsystem vision;
    public LEDSubsystem led;

    public boolean shooterAutoAdjust = true;

    private final List<LynxModule> allHubs;
    private long lastLoopTime = 0;

    public Robot(HardwareMap hardwareMap, TelemetryManager telemetry) {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drivetrain = new DrivetrainSubsystem(hardwareMap, telemetry);

        try {
            shooter = new ShooterSubsystem(hardwareMap, telemetry);
        } catch (Exception e) {
            shooter = null;
        }

        try {
            indexer = new IndexerSubsystem(hardwareMap, telemetry);
        } catch (Exception e) {
            indexer = null;
        }

        try {
            intake = new IntakeSubsystem(hardwareMap, telemetry);
        } catch (Exception e) {
            intake = null;
        }

        try {
            vision = new VisionSubsystem(hardwareMap, telemetry);
        } catch (Exception e) {
            vision = null;
        }

        try {
            if (indexer != null) {
                led = new LEDSubsystem(hardwareMap, indexer);
            } else {
                led = null;
            }
        } catch (Exception e) {
            led = null;
        }
    }

    /**
     * Continuous robot control loop executed once per cycle by {@link RobotOpMode}.
     * Clears LynxModule bulk cache at the start of every cycle.
     */
    public void update() {
        clearBulkCache();
        drivetrain.update();
        if (vision != null) vision.update();
        if (indexer != null) indexer.update();
        if (shooter != null) shooter.update();
        if (intake != null) intake.update();
        if (led != null) led.update();
    }

    /** Clears the bulk cache of all LynxModules. Called at the start of {@link #update()}. */
    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public boolean isShooting() {
        return AlignToAprilTagCommand.isAligning();
    }

    /**
     * Configures the initial pose for TeleOp: loads persisted autonomous pose if valid,
     * otherwise falls back to the default alliance starting pose.
     */
    public void applyTeleOpStartPose(AllianceEnum alliance) {
        Pose savedPose = (DataStorage.actualPose != null) ? DataStorage.actualPose : PoseStorage.loadPose();

        if (savedPose != null && !Double.isNaN(savedPose.x()) && !Double.isNaN(savedPose.y())) {
            drivetrain.getFollower().setPose(savedPose);
        } else {
            drivetrain.getFollower().setPose(startPoseFor(alliance));
        }
    }

    /** Default starting pose for the specified alliance. */
    public static Pose startPoseFor(AllianceEnum alliance) {
        DataStorage.alliance = alliance;
        return BiobuzzAutoPaths.getStartPose();
    }

    /** Sets the starting pose prior to autonomous execution. */
    public void setAutoStartPose(Pose startPose) {
        drivetrain.getFollower().setPose(startPose);
    }

    public void printLoopTime() {
        long currentTime = System.currentTimeMillis();
        long loopDelta = currentTime - lastLoopTime;

        PanelsTelemetry.INSTANCE.getTelemetry().addData("⚡ Loop Time (ms)", loopDelta);
        lastLoopTime = currentTime;
    }
}
