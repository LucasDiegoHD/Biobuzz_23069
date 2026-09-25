package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

/**
 * Alliance-aware pose abstraction supporting lazy mirroring and relative offsets.
 * Upgraded for Pedro Pathing 3.
 *
 * @author LucasDiegoHD - Team #23069
 */
public class ColoredBiobuzzPose {

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
        this(new Pose(x, y, 0), AllianceEnum.Blue);
    }

    public ColoredBiobuzzPose() {
        this(0, 0);
    }

    private static Pose mirrorPose(Pose p) {
        return new Pose(144.0 - p.x(), p.y(), p.heading());
    }

    public Pose getPose(AllianceEnum desiredColor) {
        if (desiredColor == AllianceEnum.Red) {
            if (red == null) {
                red = mirrorPose(pose);
            }
            return red;
        }

        if (blue == null) {
            blue = mirrorPose(pose);
        }
        return blue;
    }

    public Pose getPose() {
        return getPose(DataStorage.alliance);
    }

    public ColoredBiobuzzPose down(double inches) {
        return new ColoredBiobuzzPose(this.pose.plus(new Pose(0, -inches, 0)), color);
    }

    public ColoredBiobuzzPose up(double inches) {
        return new ColoredBiobuzzPose(this.pose.plus(new Pose(0, inches, 0)), color);
    }

    public ColoredBiobuzzPose towardsRedWall(double inches) {
        return new ColoredBiobuzzPose(this.pose.plus(new Pose(color == AllianceEnum.Blue ? inches : -inches, 0, 0)), color);
    }

    public ColoredBiobuzzPose towardsBlueWall(double inches) {
        return new ColoredBiobuzzPose(this.pose.plus(new Pose(color == AllianceEnum.Red ? inches : -inches, 0, 0)), color);
    }

    public static Path makePath(ColoredBiobuzzPose... poses) {
        Pose[] resolvedPoses = new Pose[poses.length];
        for (int i = 0; i < poses.length; i++) {
            resolvedPoses[i] = poses[i].getPose();
        }
        if (resolvedPoses.length == 2) {
            return Paths.line(resolvedPoses[0], resolvedPoses[1]);
        }
        return Paths.curve(resolvedPoses);
    }

    public AllianceEnum getColor() {
        return color;
    }

    public Pose getUnmodifiedPose() {
        return pose;
    }

    public double getHeading() {
        return getPose().heading();
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
        return Math.atan2(pose2.getPose().y() - pose1.getPose().y(),
                pose2.getPose().x() - pose1.getPose().x());
    }

    public static double getHeading(double heading) {
        if (DataStorage.alliance == AllianceEnum.Red) {
            return Angle.normalizeSigned(Math.PI - heading);
        }
        return heading;
    }

    public boolean initialized() {
        return true;
    }
}

