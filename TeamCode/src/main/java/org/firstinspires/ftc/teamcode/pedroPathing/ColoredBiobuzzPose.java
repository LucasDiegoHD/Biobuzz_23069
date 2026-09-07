package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

import java.util.Arrays;

/**
 * Alliance-aware pose abstraction supporting lazy mirroring and relative offsets.
 *
 * @author LucasDiegoHD - Team #23069
 */
public class ColoredBiobuzzPose implements FuturePose {

    private final Pose pose;
    private final AllianceEnum color;
    private Pose blue;
    private Pose red;

    public ColoredBiobuzzPose(Pose blue, Pose red) {
        this.pose = blue;
        this.color = AllianceEnum.Blue;
        this.blue = blue;
        this.red = red;
    }

    public ColoredBiobuzzPose(double posX, double posY, double heading, AllianceEnum allianceColor) {
        this(new Pose(posX, posY, heading), allianceColor);
    }

    public ColoredBiobuzzPose(Pose pose, AllianceEnum color) {
        if (color == AllianceEnum.Error) {
            throw new IllegalArgumentException("Uncolored ColoredBiobuzzPose with Error alliance");
        }
        this.pose = pose;
        this.color = color;
        if (color == AllianceEnum.Red) {
            this.red = pose;
        } else if (color == AllianceEnum.Blue) {
            this.blue = pose;
        }
    }

    public ColoredBiobuzzPose(double x, double y, double heading) {
        this(new Pose(x, y, heading), AllianceEnum.Blue);
    }

    public ColoredBiobuzzPose(double x, double y) {
        this(new Pose(x, y), AllianceEnum.Blue);
    }

    public ColoredBiobuzzPose() {
        this(0, 0);
    }

    public Pose getPose(AllianceEnum desiredColor) {
        if (desiredColor == AllianceEnum.Red) {
            if (red == null) {
                red = pose.mirror();
            }
            return red;
        }

        if (blue == null) {
            blue = pose.mirror();
        }
        return blue;
    }

    @Override
    public Pose getPose() {
        return getPose(DataStorage.alliance);
    }

    public ColoredBiobuzzPose down(double inches) {
        return new ColoredBiobuzzPose(this.pose.plus(new Pose(0, -inches)), color);
    }

    public ColoredBiobuzzPose up(double inches) {
        return new ColoredBiobuzzPose(this.pose.plus(new Pose(0, inches)), color);
    }

    public ColoredBiobuzzPose towardsRedWall(double inches) {
        return new ColoredBiobuzzPose(this.pose.plus(new Pose(color == AllianceEnum.Blue ? inches : -inches, 0)), color);
    }

    public ColoredBiobuzzPose towardsBlueWall(double inches) {
        return new ColoredBiobuzzPose(this.pose.plus(new Pose(color == AllianceEnum.Red ? inches : -inches, 0)), color);
    }

    public static BezierCurve makeBezier(ColoredBiobuzzPose... poses) {
        Pose[] resolvedPoses = new Pose[poses.length];
        for (int i = 0; i < poses.length; i++) {
            resolvedPoses[i] = poses[i].getPose();
        }
        return new BezierCurve(resolvedPoses);
    }

    public static BezierCurve through(ColoredBiobuzzPose... poses) {
        Pose[] resolvedPoses = new Pose[poses.length];
        for (int i = 0; i < poses.length; i++) {
            resolvedPoses[i] = poses[i].getPose();
        }
        return BezierCurve.through(resolvedPoses);
    }

    public static BezierLine makeBezier(ColoredBiobuzzPose pose1, ColoredBiobuzzPose pose2) {
        return new BezierLine(pose1.getPose(), pose2.getPose());
    }

    public static BezierPoint makeBezier(ColoredBiobuzzPose pose) {
        return new BezierPoint(pose.getPose());
    }

    public static HeadingInterpolator mirror(HeadingInterpolator interpolation) {
        return t -> MathFunctions.normalizeAngle(Math.PI - interpolation.interpolate(t));
    }

    public AllianceEnum getColor() {
        return color;
    }

    public Pose getUnmodifiedPose() {
        return pose;
    }

    public double getHeading() {
        return getPose().getHeading();
    }

    public ColoredBiobuzzPose offsetOppositeColor(Pose offset) {
        if (blue == null || red == null) {
            getPose();
        }

        if (color == AllianceEnum.Red && blue != null) {
            blue = blue.plus(offset);
        }
        if (color == AllianceEnum.Blue && red != null) {
            red = red.plus(offset);
        }
        return this;
    }

    public static double getTangentHeading(ColoredBiobuzzPose pose1, ColoredBiobuzzPose pose2) {
        return Math.atan2(pose2.getPose().getY() - pose1.getPose().getY(),
                pose2.getPose().getX() - pose1.getPose().getX());
    }

    public static double getHeading(double heading) {
        if (DataStorage.alliance == AllianceEnum.Red) {
            return MathFunctions.normalizeAngle(Math.PI - heading);
        }
        return heading;
    }

    @Override
    public boolean initialized() {
        return true;
    }
}
