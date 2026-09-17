package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.ColoredBiobuzzPose;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;
import org.firstinspires.ftc.teamcode.utils.HiveTargets;

/**
 * Waypoints do autônomo pra temporada Biobuzz.
 *
 * <p>Poses declaradas como {@link ColoredBiobuzzPose} são espelhadas automaticamente em X pra
 * Red/Blue. Campo 144"x144", origem (0,0) no canto, heading em radianos.
 *
 * @author LucasDiegoHD - Team #23069
 */
public final class BiobuzzAutoPaths {

    private BiobuzzAutoPaths() {}
    public static double RED_START_X = 60.0;
    public static double RED_START_Y = 14.64;

    public static double BLUE_START_X = 84.0;
    public static double BLUE_START_Y = 129.35;

    public static double RED_PARK_X = 10.0;
    public static double RED_PARK_Y = 104.0;

    public static double BLUE_PARK_X = 134.0;
    public static double BLUE_PARK_Y = 40.0;

    // --- Ponto de disparo/entrega usado no preload e nos ciclos 1 e 2 ---
    public static final ColoredBiobuzzPose SCORE_POSE = new ColoredBiobuzzPose(
            24.0, 100.0, Math.toRadians(90.0)
    );

    // --- Ciclo 1: intake no chão + volta pro SCORE_POSE ---
    public static final ColoredBiobuzzPose CYCLE_1_INTAKE_CONTROL = new ColoredBiobuzzPose(
            36.0, 100.0
    );
    public static final ColoredBiobuzzPose CYCLE_1_INTAKE = new ColoredBiobuzzPose(
            48.0, 80.0, Math.toRadians(180.0)
    );

    // --- Ciclo 2: outro ponto de intake, 10in abaixo do ciclo 1 ---
    public static final ColoredBiobuzzPose CYCLE_2_INTAKE = CYCLE_1_INTAKE.down(10.0);

    public static Pose getStartPose() {
        return getStartPose(DataStorage.alliance);
    }

    public static Pose getStartPose(AllianceEnum alliance) {
        double startX = (alliance == AllianceEnum.Red) ? RED_START_X : BLUE_START_X;
        double startY = (alliance == AllianceEnum.Red) ? RED_START_Y : BLUE_START_Y;
        double heading = HiveTargets.headingTowardsInitialCell(startX, startY, alliance);
        return new Pose(startX, startY, heading);
    }

    public static Pose getParkPose() {
        return getParkPose(DataStorage.alliance);
    }

    public static Pose getParkPose(AllianceEnum alliance) {
        if (alliance == AllianceEnum.Red) {
            return new Pose(10.0, 104.0, Math.toRadians(90.0));
        } else {
            return new Pose(134.0, 104.0, Math.toRadians(270.0));
        }
    }

    public static Pose getPreloadScore() {
        return SCORE_POSE.getPose();
    }

    public static Pose getCycle1Intake() {
        return CYCLE_1_INTAKE.getPose();
    }

    public static Pose getCycle1Score() {
        return SCORE_POSE.getPose();
    }

    public static Pose getCycle2Intake() {
        return CYCLE_2_INTAKE.getPose();
    }

    public static Pose getCycle2Score() {
        return SCORE_POSE.getPose();
    }
}
