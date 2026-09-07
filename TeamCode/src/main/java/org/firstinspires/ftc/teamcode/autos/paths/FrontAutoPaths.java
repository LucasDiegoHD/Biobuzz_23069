package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.ColoredBiobuzzPose;

/**
 * Poses and trajectories for the Front autonomous routine.
 * Uses ColoredBiobuzzPose to maintain unified Blue and Red poses without code duplication.
 *
 * @author LucasDiegoHD - Team #23069
 */
public final class FrontAutoPaths {

    private FrontAutoPaths() {}

    public static final ColoredBiobuzzPose START_POSE = new ColoredBiobuzzPose(
            new Pose(34, 126, Math.toRadians(130)),
            new Pose(110, 126, Math.toRadians(40))
    );

    public static final ColoredBiobuzzPose GO_TO_SHOOT_1 = new ColoredBiobuzzPose(
            new Pose(59.134, 79, Math.toRadians(122)),
            new Pose(78.634, 83, Math.toRadians(48))
    );

    public static final ColoredBiobuzzPose GO_TO_LINE_1 = new ColoredBiobuzzPose(
            new Pose(58.134, 88, Math.toRadians(0)),
            new Pose(90.134, 88, Math.toRadians(180))
    );

    public static final ColoredBiobuzzPose CATCH_LINE_1 = new ColoredBiobuzzPose(
            new Pose(26, 88, Math.toRadians(0)),
            new Pose(116, 88, Math.toRadians(180))
    );

    public static final ColoredBiobuzzPose GO_TO_SHOOT_2 = new ColoredBiobuzzPose(
            new Pose(59.134, 79, Math.toRadians(120)),
            new Pose(78.634, 83, Math.toRadians(50))
    );

    public static final ColoredBiobuzzPose GO_TO_LINE_2 = new ColoredBiobuzzPose(
            new Pose(58.134, 65.169, Math.toRadians(0)),
            new Pose(90.134, 65.169, Math.toRadians(180))
    );

    public static final ColoredBiobuzzPose CATCH_LINE_2 = new ColoredBiobuzzPose(
            new Pose(20.776, 65.169, Math.toRadians(0)),
            new Pose(118.776, 65.169, Math.toRadians(180))
    );

    public static final ColoredBiobuzzPose END_POSE = new ColoredBiobuzzPose(
            new Pose(50, 60, Math.toRadians(90)),
            new Pose(84, 60, Math.toRadians(90))
    );

    public static final ColoredBiobuzzPose GO_TO_LINE_3 = new ColoredBiobuzzPose(
            new Pose(58.134, 68.669, Math.toRadians(0)),
            new Pose(120.134, 63.169, Math.toRadians(180))
    );

    public static final ColoredBiobuzzPose CATCH_LINE_3 = new ColoredBiobuzzPose(
            new Pose(14.774, 68.669, Math.toRadians(285)),
            new Pose(130, 63.169, Math.toRadians(255))
    );

    public static final ColoredBiobuzzPose GATE_POSE = new ColoredBiobuzzPose(
            new Pose(14.774, 69.169, Math.toRadians(0)),
            new Pose(130, 73.169, Math.toRadians(180))
    );

    /**
     * Resolves the pose for the currently active alliance.
     */
    public static Pose getPose(PosesNames name) {
        switch (name) {
            case StartPose:   return START_POSE.getPose();
            case GoToShoot1:  return GO_TO_SHOOT_1.getPose();
            case GoToLine1:   return GO_TO_LINE_1.getPose();
            case CatchLine1:  return CATCH_LINE_1.getPose();
            case GoToShoot2:  return GO_TO_SHOOT_2.getPose();
            case GoToLine2:   return GO_TO_LINE_2.getPose();
            case CatchLine2:  return CATCH_LINE_2.getPose();
            case EndPose:     return END_POSE.getPose();
            case GoToLine3:   return GO_TO_LINE_3.getPose();
            case CatchLine3:  return CATCH_LINE_3.getPose();
            case GatePose:    return GATE_POSE.getPose();
            default:          return START_POSE.getPose();
        }
    }
}
