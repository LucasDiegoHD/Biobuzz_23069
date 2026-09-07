package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.ColoredBiobuzzPose;

/**
 * Universal Autonomous Waypoints Template for the Biobuzz season.
 *
 * <p>All positions are declared as {@link ColoredBiobuzzPose}. By default, coordinates
 * are automatically mirrored for Red and Blue alliances. If specific field elements
 * require empirical alliance offsets, use {@code new ColoredBiobuzzPose(bluePose, redPose)}.
 *
 * <p>Coordinate system: Field size is 144" x 144". (0,0) is at field corner.
 * Heading is in radians (use {@code Math.toRadians(degrees)}).
 *
 * @author LucasDiegoHD - Team #23069
 */
public final class BiobuzzAutoPaths {

    private BiobuzzAutoPaths() {}

    // =========================================================================
    // 1. STARTING POSE (Robot resting against perimeter wall before start)
    // =========================================================================
    public static final ColoredBiobuzzPose START_POSE = new ColoredBiobuzzPose(
            24.0, 134.0, Math.toRadians(90.0)
    );

    // =========================================================================
    // 2. PRELOAD SCORING (Initial preloaded element delivery)
    // =========================================================================
    public static final ColoredBiobuzzPose PRELOAD_SCORE = new ColoredBiobuzzPose(
            24.0, 100.0, Math.toRadians(90.0)
    );

    // =========================================================================
    // 3. CYCLE 1: INTAKE & SCORE
    // =========================================================================
    // Bezier control point to smooth out transit without sharp corners
    public static final ColoredBiobuzzPose CYCLE_1_INTAKE_CONTROL = new ColoredBiobuzzPose(
            36.0, 100.0
    );
    // Position where the robot intakes piece 1 from the floor
    public static final ColoredBiobuzzPose CYCLE_1_INTAKE = new ColoredBiobuzzPose(
            48.0, 80.0, Math.toRadians(180.0)
    );
    // Position where piece 1 is delivered
    public static final ColoredBiobuzzPose CYCLE_1_SCORE = new ColoredBiobuzzPose(
            24.0, 100.0, Math.toRadians(90.0)
    );

    // =========================================================================
    // 4. CYCLE 2: INTAKE & SCORE
    // =========================================================================
    public static final ColoredBiobuzzPose CYCLE_2_INTAKE = new ColoredBiobuzzPose(
            48.0, 70.0, Math.toRadians(180.0)
    );
    public static final ColoredBiobuzzPose CYCLE_2_SCORE = new ColoredBiobuzzPose(
            24.0, 100.0, Math.toRadians(90.0)
    );

    // =========================================================================
    // 5. PARK / END POSITION (Final parking zone for endgame points)
    // =========================================================================
    public static final ColoredBiobuzzPose PARK_POSE = new ColoredBiobuzzPose(
            60.0, 60.0, Math.toRadians(90.0)
    );

    /**
     * Resolves the active pose based on the selected alliance in DataStorage.
     */
    public static Pose getStartPose() {
        return START_POSE.getPose();
    }

    public static Pose getPreloadScore() {
        return PRELOAD_SCORE.getPose();
    }

    public static Pose getCycle1Intake() {
        return CYCLE_1_INTAKE.getPose();
    }

    public static Pose getCycle1Score() {
        return CYCLE_1_SCORE.getPose();
    }

    public static Pose getCycle2Intake() {
        return CYCLE_2_INTAKE.getPose();
    }

    public static Pose getCycle2Score() {
        return CYCLE_2_SCORE.getPose();
    }

    public static Pose getParkPose() {
        return PARK_POSE.getPose();
    }
}
