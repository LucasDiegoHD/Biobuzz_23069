package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.ColoredBiobuzzPose;

/**
 * Poses and trajectories for the Rear autonomous routines (Normal & No Gate).
 * Uses ColoredBiobuzzPose to maintain unified Blue and Red poses without code duplication.
 *
 * @author LucasDiegoHD - Team #23069
 */
public final class RearAutoPaths {

    private RearAutoPaths() {}

    public static final ColoredBiobuzzPose START_POSE = new ColoredBiobuzzPose(
            new Pose(63.411, 13.399, Math.toRadians(90)),
            new Pose(83.35, 13.399, Math.toRadians(90))
    );

    public static final ColoredBiobuzzPose GO_TO_SHOOT_1 = new ColoredBiobuzzPose(
            new Pose(60.335, 18.547, Math.toRadians(111)),
            new Pose(81.35, 20.547, Math.toRadians(68))
    );

    public static final ColoredBiobuzzPose GO_TO_LINE_1 = new ColoredBiobuzzPose(
            new Pose(64.134, 38.169, Math.toRadians(0)),
            new Pose(74.134, 39.169, Math.toRadians(180))
    );

    public static final ColoredBiobuzzPose CATCH_LINE_1 = new ColoredBiobuzzPose(
            new Pose(24, 38.169, Math.toRadians(0)),
            new Pose(120.276, 39.169, Math.toRadians(180))
    );

    public static final ColoredBiobuzzPose GO_TO_SHOOT_2 = new ColoredBiobuzzPose(
            new Pose(60.335, 17.547, Math.toRadians(109)),
            new Pose(81.35, 20.547, Math.toRadians(68))
    );

    public static final ColoredBiobuzzPose GO_TO_LINE_2 = new ColoredBiobuzzPose(
            new Pose(41, 19.547, Math.toRadians(0)),
            new Pose(104.35, 19.547, Math.toRadians(180))
    );

    public static final ColoredBiobuzzPose CATCH_LINE_2 = new ColoredBiobuzzPose(
            new Pose(24, 13, Math.toRadians(0)),
            new Pose(130.776, 13, Math.toRadians(180))
    );

    public static final ColoredBiobuzzPose END_POSE = new ColoredBiobuzzPose(
            new Pose(60.335, 34, Math.toRadians(90)),
            new Pose(85.35, 34, Math.toRadians(90))
    );

    public static final ColoredBiobuzzPose GO_TO_LINE_3 = new ColoredBiobuzzPose(
            new Pose(64.134, 63.169, Math.toRadians(0)),
            new Pose(74.134, 63.169, Math.toRadians(180))
    );

    public static final ColoredBiobuzzPose CATCH_LINE_3 = new ColoredBiobuzzPose(
            new Pose(26, 63.169, Math.toRadians(0)),
            new Pose(120.276, 63.169, Math.toRadians(180))
    );

    public static final ColoredBiobuzzPose GATE_POSE = new ColoredBiobuzzPose(
            new Pose(18, 64.169, Math.toRadians(90)),
            new Pose(126.276, 67.169, Math.toRadians(90))
    );

    public static final ColoredBiobuzzPose GO_TO_LINE_4 = new ColoredBiobuzzPose(
            new Pose(20, 28, Math.toRadians(0)),
            new Pose(130.276, 28, Math.toRadians(180))
    );

    public static final ColoredBiobuzzPose CATCH_LINE_5 = new ColoredBiobuzzPose(
            new Pose(20, 14.5, Math.toRadians(0)),
            new Pose(114.276, 14.5, Math.toRadians(180))
    );

    public static final ColoredBiobuzzPose GOTO_LINE_6 = new ColoredBiobuzzPose(
            new Pose(60.335, 18.547, Math.toRadians(113)),
            new Pose(81.35, 25.547, Math.toRadians(68))
    );

    public static final ColoredBiobuzzPose CATCH_LINE_6 = new ColoredBiobuzzPose(
            new Pose(120.276, 18, Math.toRadians(90)),
            new Pose(122.276, 18, Math.toRadians(90))
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
            case GoToLine4:   return GO_TO_LINE_4.getPose();
            case CatchLine5:  return CATCH_LINE_5.getPose();
            case GotoLine6:   return GOTO_LINE_6.getPose();
            case CatchLine6:  return CATCH_LINE_6.getPose();
            default:          return START_POSE.getPose();
        }
    }
}
