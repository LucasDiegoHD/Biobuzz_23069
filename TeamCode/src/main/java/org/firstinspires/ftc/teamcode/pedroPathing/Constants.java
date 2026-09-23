package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {
    public static double ROBOT_WIDTH_CM = 36.0;   // Se colocar chapa externa lateral, adicionar a espessura aqui
    public static double ROBOT_LENGTH_CM = 37.2;  // Se colocar chapa externa frontal/traseira, adicionar a espessura aqui
    public static double ROBOT_WIDTH_INCHES = ROBOT_WIDTH_CM / 2.54;   // ~14.173 in
    public static double ROBOT_LENGTH_INCHES = ROBOT_LENGTH_CM / 2.54; // ~14.646 in
    public static double HALF_WIDTH_INCHES = ROBOT_WIDTH_INCHES / 2.0;   // ~7.087 in (Offset X do centro)
    public static double HALF_LENGTH_INCHES = ROBOT_LENGTH_INCHES / 2.0; // ~7.323 in (Offset Y do centro)
    public static Pose CORNER_RESET_POSE = new Pose(HALF_WIDTH_INCHES, HALF_LENGTH_INCHES, Math.toRadians(90.0));

    public static class Drivetrain {
        public static String LEFT_FRONT_MOTOR = "leftFront";
        public static String RIGHT_FRONT_MOTOR = "rightFront";
        public static String LEFT_REAR_MOTOR = "leftRear";
        public static String RIGHT_REAR_MOTOR = "rightRear";
        public static String PINPOINT_LOCALIZER = "pinpoint";
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(Drivetrain.LEFT_FRONT_MOTOR);
        c.frontRightName.set(Drivetrain.RIGHT_FRONT_MOTOR);
        c.backLeftName.set(Drivetrain.LEFT_REAR_MOTOR);
        c.backRightName.set(Drivetrain.RIGHT_REAR_MOTOR);
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(5.777516402597502);
        c.yPodOffset.set(-0.7056641015480823);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static class DriveConstantsCompat {
        public String leftFrontMotorName = Drivetrain.LEFT_FRONT_MOTOR;
        public String rightFrontMotorName = Drivetrain.RIGHT_FRONT_MOTOR;
        public String leftRearMotorName = Drivetrain.LEFT_REAR_MOTOR;
        public String rightRearMotorName = Drivetrain.RIGHT_REAR_MOTOR;
    }
    public static final DriveConstantsCompat driveConstants = new DriveConstantsCompat();

    // Autonomous max-path-speed presets (fração 0.0-1.0, aplicados de verdade por
    // GoToPoseCommand.setConstraints via ForesightConfig.maxPathSpeed).
//    public static final double pathConstraints = 1.0;
//    public static final double autoShootConstraints = 0.6;   // mais devagar/preciso pra aproximar do HIVE
//    public static final double autoTransitConstraints = 1.0; // rápido, pros ciclos de intake

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.174811404783288);
                Controller secondaryTranslationalForward = Controller.proportional(0.0645881430206612);
                Controller primaryTranslationalLateral = Controller.proportional(0.27358304670789924);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1010816254849255);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01051550271902732));
                c.brake.set(Controller.proportionalFeedforward(0.008938177311173221));

                c.headingFeedback.set(Controller.proportional(2.691380444975961));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03246155555434772, 0.009573729501870054));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0612158081863579, 0.0785513848244777));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0019279464876478614, 0.0015225424965786835));

                c.maxAchievableForwardVelocity.set(92.09737008247103);
                c.maxAchievableStrafeVelocity.set(78.79714386695369);
                c.naturalForwardDeceleration.set(28.402614227196008);
                c.naturalStrafeDeceleration.set(47.8675802124677);
            }
    );

    public static com.pedropathing.drivetrain.Drivetrain drivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static Localizer localizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig);
    }

    public static Algorithm foresight() {
        return new Foresight(foresightConfig);
    }

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(localizer(hardwareMap), drivetrain(hardwareMap), foresight());
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return create(hardwareMap);
    }
}
