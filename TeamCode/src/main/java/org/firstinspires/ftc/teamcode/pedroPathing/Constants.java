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
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(Drivetrain.PINPOINT_LOCALIZER);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-5.70);
        c.yPodOffset.set(0.73);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
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

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        // Measured (calibrate_translational_feedback / calibrate_heading_feedback)
        c.forwardTranslational.set(Controller.piecewise(Controller.proportional(0.223475)).put(2.5, Controller.proportional(0.604847)));
        c.strafeTranslational.set(Controller.piecewise(Controller.proportional(0.095045)).put(2.5, Controller.proportional(0.257245)));
        c.headingFeedback.set(Controller.proportional(2.65204));
        c.headingStaticFF.set(Controller.zero);
        c.coast.set(Controller.proportionalFeedforward(0.0122618));
        c.brake.set(Controller.proportionalFeedforward(0.0104226));
        c.holdPointTranslationalScaling.set(1.0);
        c.holdPointHeadingScaling.set(1.0);
        c.maxBrakingPower.set(1.0);
        c.maxAccelerationConstraint.set(1000.0);
        c.maxVelocityConstraint.set(1000.0);
        c.maxDecelerationConstraint.set(ForesightConfig.Constraint.NONE);
        c.maxPathSpeed.set(1.0);
        c.maxDecelerationScale.set(1.0);
        c.brakeAggression.set(1.0);
        c.coastDownToVelocity.set(5.0);
        c.headingDeviationTolerance.set(0.05);
        c.translationalDeviationTolerance.set(0.5);
        c.brakeAtEnd.set(true);
        c.pathSkip.set(false);
        c.headingDriveRatio.set(1.0);

        // Measured (calibrate_foresight_braking)
        c.linearBrakeCoefficients.set(Matrix.diag(0.117823, 0.0547195));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.00125541, 0.00222427));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0627234, 0.006954));
        c.cosineScale.set(true);

        // Measured (calibrate_velocity / calibrate_zero_power)
        c.maxAchievableForwardVelocity.set(83.1065);
        c.maxAchievableStrafeVelocity.set(67.0362);
        c.naturalForwardDeceleration.set(25.3979);
        c.naturalStrafeDeceleration.set(47.3951);
        c.minCorrectionDistance.set(0.1);
        c.parametricTConstraint.set(0.995);
        c.headingConstraint.set(0.01);
        c.translationalConstraint.set(0.1);
        c.velocityConstraint.set(0.1);
        c.timeoutConstraint.set(0.1);
    });

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
