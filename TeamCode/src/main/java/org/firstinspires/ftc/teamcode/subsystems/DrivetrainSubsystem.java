package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autos.paths.BiobuzzAutoPaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.DataStorage;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.utils.control.PIDFController;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Arrays;
import java.util.List;

/**
 * The DrivetrainSubsystem is responsible for the robot's movement and path following.
 * Upgraded to Pedro Pathing 3 (v3.0.0) with Foresight algorithm and Ivy 1.1.1.
 * High-agility TeleOp with normalized denominator kinematics.
 *
 * Integrated TechMaker (#23069) pilot assists:
 * - Default Field-Centric drive.
 * - Closed-loop PIDF Heading Lock & Cardinal 90-degree snapping.
 * - High-speed Kick autonomous trajectory toward the scoring zone.
 * - Active Hold Pose resisting physical disturbances.
 * - Anti-Tipping vector protection.
 *
 * @author LucasDiegoHD - TechMaker (#23069)
 */
public class DrivetrainSubsystem {
    private static final Logger log = LoggerFactory.getLogger(DrivetrainSubsystem.class);
    private final Follower follower;
    private final TelemetryManager telemetry;
    private long ultimoTempoSalvo = 0;
    private final VoltageSensor voltageSensor;

    private final List<DcMotorEx> motors;
    public final DcMotorEx leftFront;
    public final DcMotorEx leftRear;
    public final DcMotorEx rightFront;
    public final DcMotorEx rightRear;

    private boolean prevVelZero;
    private boolean fieldCentric = true; // Field-Centric ativo por padrão!
    private double fieldHeadingOffset = 0.0; // Offset relativo do piloto para o campo
    private boolean isTeleOp = true;
    private boolean holdingPose = false;

    // Heading Lock (Closed-Loop PIDF)
    public static PIDFCoefficients HEADING_LOCK_PIDF = new PIDFCoefficients(1.3, 0.0, 0.05, 0.0);
    private final PIDFController headingPIDFController = new PIDFController(HEADING_LOCK_PIDF);
    private boolean headingLockEnabled = false;
    private double targetHeading = 0.0;

    // Kick Function (Autonomous Scoring Zone Burst)
    public static PIDFCoefficients KICK_TRANSLATIONAL_PIDF = new PIDFCoefficients(0.045, 0.0, 0.003, 0.0);
    private final PIDFController kickXController = new PIDFController(KICK_TRANSLATIONAL_PIDF);
    private final PIDFController kickYController = new PIDFController(KICK_TRANSLATIONAL_PIDF);
    private final ElapsedTime kickTimer = new ElapsedTime();
    private boolean kicking = false;
    private Pose kickTargetPose = null;
    public static double KICK_TIMEOUT_SECONDS = 0.9;
    public static double KICK_DISTANCE_TOLERANCE_INCHES = 10.0;

    // Anti-Tipping Vector Physics (Configurable, dormant by default for slide readiness)
    private boolean tipCorrectionEnabled = false;
    public static double TIP_CLAMP_POWER = 0.0;
    public static double TIP_MAX_LAMBDA = 0.95;
    public static double TIP_MAX_VELOCITY = 76.0; // in/s
    public static double TIP_ALPHA_THRESHOLD = 0.2;

    public static float MAGNITUDE_ZERO = 0.05f;
    public static float ANGLE_ZERO = (float) Math.tan(Math.toRadians(15));
    public static float MAX_POWER = 1.0f;

    /**
     * Constructs a new DrivetrainSubsystem.
     *
     * @param hardwareMap The hardware map to retrieve hardware devices from.
     * @param telemetry   The telemetry manager for logging.
     */
    public DrivetrainSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        follower = Constants.create(hardwareMap);
        this.telemetry = telemetry;
        Drawing.init();
        Drawing.drawRobot(follower.pose());
        Drawing.sendPacket();
        this.voltageSensor = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next()
                : null;

        leftFront = hardwareMap.get(DcMotorEx.class, Constants.Drivetrain.LEFT_FRONT_MOTOR);
        leftRear = hardwareMap.get(DcMotorEx.class, Constants.Drivetrain.LEFT_REAR_MOTOR);
        rightFront = hardwareMap.get(DcMotorEx.class, Constants.Drivetrain.RIGHT_FRONT_MOTOR);
        rightRear = hardwareMap.get(DcMotorEx.class, Constants.Drivetrain.RIGHT_REAR_MOTOR);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        follower.update();

    }

    /**
     * Gets the Follower instance used for path following.
     * @return The Follower instance.
     */
    public Follower getFollower() {
        return follower;
    }

    /**
     * Sets the power levels for each of the four motors in the system with zero-power caching
     * to avoid redundant hardware write cycles when stationary.
     *
     * @param lf Left front motor power (-1.0 to 1.0)
     * @param lr Left rear motor power (-1.0 to 1.0)
     * @param rf Right front motor power (-1.0 to 1.0)
     * @param rr Right rear motor power (-1.0 to 1.0)
     */
    public void setPowers(double lf, double lr, double rf, double rr) {
        if (lf == 0 && lr == 0 && rf == 0 && rr == 0) {
            if (prevVelZero) {
                return;
            }
            prevVelZero = true;
        } else {
            prevVelZero = false;
        }

        leftFront.setPower(lf);
        leftRear.setPower(lr);
        rightFront.setPower(rf);
        rightRear.setPower(rr);
    }

    /**
     * Smooths the gamepad angle to snap pure orthogonal movements
     * and avoid accidental thumb drift.
     */
    private Pose smoothGamepadAngle(double x, double y, double zeroAngle) {
        if (x == 0 || y == 0) {
            return new Pose(x, y, 0);
        } else if (Math.abs(y / x) < zeroAngle) {
            return new Pose(x, 0, 0);
        } else if (Math.abs(x / y) < zeroAngle) {
            return new Pose(0, y, 0);
        } else {
            return new Pose(x, y, 0);
        }
    }

    /**
     * Arcade drive method uniting high-agility mecanum kinematics with intelligent assists.
     */
    public void arcadeDrive(double x, double y, double rx, float DZ, float angleZero, float maxPower, double headingOffset) {
        // 0. ACTIVE POSE HOLD
        if (holdingPose) {
            if (Math.abs(y) > 0.08 || Math.abs(x) > 0.08 || Math.abs(rx) > 0.08) {
                stopHoldPose();
            } else {
                return;
            }
        }

        // 1. KICK TRAJECTORY (Closed-loop autonomous burst)
        if (kicking && kickTargetPose != null) {
            Pose currentPose = follower.pose();
            double distToTarget = Math.hypot(kickTargetPose.x() - currentPose.x(), kickTargetPose.y() - currentPose.y());

            if (kickTimer.seconds() > KICK_TIMEOUT_SECONDS || distToTarget < KICK_DISTANCE_TOLERANCE_INCHES) {
                cancelKick();
            } else if (Math.abs(x) > 0.08 || Math.abs(y) > 0.08 || Math.abs(rx) > 0.08) {
                cancelKick();
            } else {
                double xError = kickTargetPose.x() - currentPose.x();
                double yError = kickTargetPose.y() - currentPose.y();
                kickXController.updateError(xError);
                kickYController.updateError(yError);

                double kickFieldX = Utils.clamp(kickXController.run(), -maxPower, maxPower);
                double kickFieldY = Utils.clamp(kickYController.run(), -maxPower, maxPower);

                // Conversão corrigida para o referencial do robô
                double theta = currentPose.heading();
                y = kickFieldY * Math.cos(theta) + kickFieldX * Math.sin(theta);
                x = -kickFieldY * Math.sin(theta) + kickFieldX * Math.cos(theta);

                double headingError = Angle.error(theta, targetHeading);
                headingPIDFController.updateError(headingError);
                rx = Utils.clamp(headingPIDFController.run(), -maxPower, maxPower);

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
                double fl = (y + x + rx) / denominator;
                double bl = (y - x + rx) / denominator;
                double fr = (y - x - rx) / denominator;
                double br = (y + x - rx) / denominator;
                setPowers(fl, bl, fr, br);
                return;
            }
        }

        // 2. STANDARD TELEOP (Field-Centric)
        if (Math.abs(y) > DZ || Math.abs(x) > DZ || Math.abs(rx) > DZ || headingLockEnabled) {
            Pose movementVector = smoothGamepadAngle(x, y, angleZero);

            y = (Math.abs(y) > DZ) ? 0.5 * Math.tan(movementVector.y() * 1.12) : 0;
            x = (Math.abs(x) > DZ) ? 0.5 * Math.tan(movementVector.x() * 1.12) : 0;

            if (Math.abs(rx) > DZ) {
                if (headingLockEnabled) unlockHeading();
                rx = 0.5 * Math.tan(rx * 1.12);
            } else if (headingLockEnabled) {
                double currentHeading = follower.pose().heading();
                double headingError = Angle.error(currentHeading, targetHeading);

                if (Math.abs(headingError) < Math.toRadians(1.0)) {
                    rx = 0.0;
                } else {
                    headingPIDFController.updateError(headingError);
                    rx = Utils.clamp(headingPIDFController.run(), -maxPower, maxPower);
                }
            } else {
                rx = 0;
            }

            // Transformação Field-Centric Corrigida
            double robotHeading = follower.pose().heading();
            double angle = robotHeading - (fieldHeadingOffset + headingOffset);

            double rotY = y * Math.cos(angle) - x * Math.sin(angle);
            double rotX = y * Math.sin(angle) + x * Math.cos(angle);

            y = rotY;
            x = rotX;

            if (tipCorrectionEnabled) {
                double[] corrected = applyAntiTipCorrection(x, y);
                x = corrected[0];
                y = corrected[1];
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double frontLeftPow = (y + x + rx) / denominator;
            double backLeftPow = (y - x + rx) / denominator;
            double frontRightPow = (y - x - rx) / denominator;
            double backRightPow = (y + x - rx) / denominator;

            double highestPower = Math.max(Math.max(Math.abs(frontLeftPow), Math.abs(backLeftPow)),
                    Math.max(Math.abs(frontRightPow), Math.abs(backRightPow)));

            if (highestPower > maxPower) {
                frontLeftPow *= maxPower / highestPower;
                frontRightPow *= maxPower / highestPower;
                backRightPow *= maxPower / highestPower;
                backLeftPow *= maxPower / highestPower;
            }

            setPowers(frontLeftPow, backLeftPow, frontRightPow, backRightPow);
        } else if (!prevVelZero) {
            setPowers(0, 0, 0, 0);
        }
    }

    public void arcadeDrive(Gamepad gamepad, double headingOffset) {
        arcadeDrive(gamepad.left_stick_x, -gamepad.left_stick_y, -gamepad.right_stick_x,
                MAGNITUDE_ZERO, ANGLE_ZERO, MAX_POWER, headingOffset);
    }

    public void arcadeDrive(Gamepad gamepad) {
        arcadeDrive(gamepad, 0.0);
    }

    public void arcadeDrive(double x, double y, double rx) {
        arcadeDrive(x, y, rx, MAGNITUDE_ZERO, ANGLE_ZERO, MAX_POWER, 0.0);
    }

    /**
     * Triggers a high-speed closed-loop "Kick" towards a target pose.
     * The robot drives rapidly to the target pose while aligning its heading.
     * Can be cancelled at any time by driver joystick input or calling cancelKick().
     *
     * @param targetPose The field destination pose.
     */
    public void kick(Pose targetPose) {
        this.kickTargetPose = targetPose;
        this.kicking = true;
        this.kickTimer.reset();
        this.kickXController.reset();
        this.kickYController.reset();
        this.lockHeading(targetPose.heading());
    }

    /**
     * Triggers a Kick towards the primary scoring zone pose for the active alliance.
     */
    public void kick() {
        Pose scorePose = BiobuzzAutoPaths.getCycle1Score();
        kick(scorePose);
    }

    /**
     * Cancels the active Kick and returns to standard teleop control.
     */
    public void cancelKick() {
        if (!kicking) return;
        this.kicking = false;
        this.kickTargetPose = null;
        this.unlockHeading();
    }

    public boolean isKicking() {
        return kicking;
    }

    public Pose getKickTargetPose() {
        return kickTargetPose;
    }

    /**
     * Locks the robot's heading to a specific field angle in radians.
     * While locked, translation remains in full field-centric mode, and rotation is controlled
     * by the closed-loop PIDF controller.
     *
     * @param targetHeadingRad Desired heading in radians.
     */
    public void lockHeading(double targetHeadingRad) {
        double normalized = Angle.normalizeSigned(targetHeadingRad);
        if (!headingLockEnabled || Math.abs(Angle.smallestDifference(this.targetHeading, normalized)) > 1e-4) {
            this.targetHeading = normalized;
            this.headingLockEnabled = true;
            this.headingPIDFController.reset();
        }
    }

    /**
     * Unlocks the heading, returning full rotation control to the driver.
     */
    public void unlockHeading() {
        this.headingLockEnabled = false;
        this.headingPIDFController.reset();
    }

    public boolean isHeadingLocked() {
        return headingLockEnabled;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    /**
     * Snaps and locks to the nearest cardinal field angle (0, 90, 180, 270 degrees).
     * Ideal for quickly squaring against field perimeters, submersibles, or gates.
     */
    public void snapToNearestCardinal() {
        double currentHeading = follower.pose().heading();
        double[] alignAngles = {0.0, 0.5 * Math.PI, Math.PI, -0.5 * Math.PI};
        int bestIndex = 0;
        double smallestDiff = Double.POSITIVE_INFINITY;
        for (int i = 0; i < alignAngles.length; i++) {
            double diff = Math.abs(Angle.smallestDifference(currentHeading, alignAngles[i]));
            if (diff < smallestDiff) {
                bestIndex = i;
                smallestDiff = diff;
            }
        }
        lockHeading(alignAngles[bestIndex]);
    }

    /**
     * Snaps and locks to a specific target angle in radians.
     */
    public void snapToAngle(double targetHeadingRad) {
        lockHeading(targetHeadingRad);
    }

    /**
     * Resets the driver's field-centric heading reference to the current robot orientation
     * without altering or corrupting the underlying localizer coordinates.
     */
    public void resetHeading(double offsetRad) {
        this.fieldHeadingOffset = Angle.normalizeSigned(follower.pose().heading() - offsetRad);
        unlockHeading();
    }

    public void resetHeading() {
        resetHeading(0.0);
    }

    public void setFieldHeadingOffset(double offsetRad) {
        this.fieldHeadingOffset = Angle.normalizeSigned(offsetRad);
    }

    public double getFieldHeadingOffset() {
        return fieldHeadingOffset;
    }

    /**
     * Actively locks the robot pose using Pedro Pathing 3's intrinsic follower.hold() method.
     */
    public void holdCurrentPose() {
        follower.update();
        follower.hold(follower.pose());
        holdingPose = true;
    }

    /**
     * Toggles active pose hold.
     *
     * @return True if now holding, false if released.
     */
    public boolean toggleHoldPose() {
        if (holdingPose) {
            stopHoldPose();
            return false;
        } else {
            holdCurrentPose();
            return true;
        }
    }

    /**
     * Single-point hold pose command.
     */
    public Command holdPose() {
        return Groups.sequential(
                Commands.instant(this::holdCurrentPose),
                Commands.waitMs(350)
        );
    }

    /**
     * Releases active pose hold and returns chassis control to teleop.
     */
    public void stopHoldPose() {
        if (!holdingPose) return;
        holdingPose = false;
        follower.stop();
        follower.update();
    }

    public boolean isHoldingPose() {
        return holdingPose;
    }

    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public void setTeleOp(boolean teleOp) {
        this.isTeleOp = teleOp;
    }

    public boolean isTeleOp() {
        return isTeleOp;
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(behavior);
        }
    }

    public Pose getPose() {
        return follower.pose();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public Velocity getVelocity() {
        return follower.velocity();
    }

    public void drive(double forward, double strafe, double turn, boolean isFieldCentric) {
        if (isFieldCentric) {
            arcadeDrive(strafe, -forward, turn, MAGNITUDE_ZERO, ANGLE_ZERO, MAX_POWER, fieldHeadingOffset);
        } else {
            arcadeDrive(strafe, -forward, turn, MAGNITUDE_ZERO, ANGLE_ZERO, MAX_POWER, follower.pose().heading());
        }
    }

    public void driveRobotCentric(double strafe, double forward, double turn) {
        arcadeDrive(strafe, -forward, turn, MAGNITUDE_ZERO, ANGLE_ZERO, MAX_POWER, follower.pose().heading());
    }

    public boolean isRobotStopped() {
        double linearVelocity = (follower.velocity() != null) ? follower.velocity().toVector2D().magnitude() : 0.0;
        double angularVelocity = (follower.velocity() != null) ? Math.toDegrees(Math.abs(follower.velocity().omega)) : 0.0;
        return linearVelocity < 2.0 && angularVelocity < 3.0;
    }

    /**
     * Stops the robot.
     */
    public void stop() {
        follower.stop();
        setPowers(0, 0, 0, 0);
    }

    /**
     * Reseta o Pinpoint com o yaw da pose salva no DataStorage.
     * Deve ser chamado no início do TeleOp antes do primeiro update()
     * para garantir que o MegaTag2 receba o yaw correto desde o início.
     */
    public void restorePoseFromStorage() {
        if (DataStorage.actualPose != null) {
            follower.update();

            double autoX = DataStorage.actualPose.x();
            double autoY = DataStorage.actualPose.y();
            double hardwareYaw = follower.pose().heading();

            Pose safePose = new Pose(autoX, autoY, hardwareYaw);
            follower.setPose(safePose);
        }
    }

    public double getVoltage() {
        return (voltageSensor != null) ? voltageSensor.getVoltage() : 13.0;
    }

    /**
     * Called periodically to update the subsystem's state.
     * Durante drive manual (TeleOp, sem path/hold ativo), só atualiza o localizer (pose/heading)
     * sem deixar o Follower mexer no Drivetrain físico (senão o modo IDLE chama drivetrain.stop()
     * e briga com os setPowers() do arcadeDrive). Durante Hold Pose OU Autônomo (isTeleOp=false),
     * deixamos o Follower rodar completo — é ELE que precisa controlar os motores nesses casos
     * (path following e hold, respectivamente).
     *
     * <p>BUG corrigido (Biobuzz): antes só checava {@code holdingPose}, então durante o Autônomo
     * (onde não tem hold, mas tem path ativo via {@code follower.follow(path)}) o
     * {@code follower.update()} completo nunca rodava — só a pose era atualizada, os motores
     * nunca recebiam correção/frenagem, e o robô saía direto pra frente sem nunca desacelerar
     * até bater na parede. {@code isTeleOp} já existia pra essa distinção, só nunca era checado
     * aqui. Ver {@code Autos.start()}, que agora chama {@code drivetrain.setTeleOp(false)}.
     */
    public void update() {
        if (holdingPose || !isTeleOp) {
            follower.update();
        } else {
            follower.localizer.update();
        }

        DataStorage.actualPose = follower.pose();
        if (DataStorage.DEBUG_MODE) {
            telemetry.addData("Robot pose", follower.pose());
            telemetry.addData("DEBUG heading (dentro do update)", Math.toDegrees(follower.pose().heading()));
        }
        long tempoAtual = System.currentTimeMillis();
        if (tempoAtual - ultimoTempoSalvo > 1000) {
            PoseStorage.savePose(follower.pose());
            ultimoTempoSalvo = tempoAtual;
        }

        Drawing.drawRobot(follower.pose());
        Drawing.sendPacket();
    }

    /**
     * Follows a Path in Pedro Pathing 3.
     *
     * @param path The Path to follow.
     */
    public void followPath(Path path) {
        follower.follow(path);
    }

    /**
     * Checks if the follower is actively running a path.
     *
     * @return True if busy.
     */
    public boolean isBusy() {
        return follower.isBusy();
    }

    /**
     * Gets the current parametric progress (0.0 to 1.0) along the active Path.
     *
     * @return Current T value.
     */
    public double getCurrentTValue() {
        return follower.parametricCompletion();
    }

    /**
     * Checks whether parametric progress has passed a given threshold.
     *
     * @param t Threshold between 0.0 and 1.0.
     * @return True if current T value is greater than threshold.
     */
    public boolean tValueCondition(double t) {
        return follower.parametricCompletion() > t;
    }

    /**
     * Calculates the remaining distance to target along the active path.
     *
     * @return Remaining distance in inches.
     */
    public double distanceFromTarget() {
        return follower.distanceToEndpoint();
    }

    /**
     * Checks if the robot's tangential velocity along the path curve is below
     * the threshold.
     *
     * @return True if tangential velocity is within constraint.
     */
    public boolean velocityCondition() {
        return follower.tangentialVelocity() < 2.0;
    }

    /**
     * Intelligent path completion condition for autonomous sequences (Early Handoff).
     *
     * @param dist Distance threshold in inches.
     * @return True if within distance threshold and sufficiently decelerated, or if follower finished.
     */
    public boolean velocityCondition(double dist) {
        return !follower.isBusy() || (distanceFromTarget() < dist && velocityCondition());
    }

    /**
     * Overload checking distance and custom tangential velocity threshold.
     *
     * @param dist Distance threshold in inches.
     * @param maxVelocity Maximum tangential velocity in inches/sec.
     * @return True if within distance threshold and below maxVelocity, or if follower finished.
     */
    public boolean velocityCondition(double dist, double maxVelocity) {
        return !follower.isBusy() || (distanceFromTarget() < dist && follower.tangentialVelocity() < maxVelocity);
    }

    /**
     * Applies vector anti-tipping physics to limit aggressive decelerations that could pitch
     * or flip the robot when carrying high mechanisms (e.g. extended vertical slides).
     *
     * @param robotX Commanded lateral strafe power in robot frame.
     * @param robotY Commanded longitudinal forward power in robot frame.
     * @return Array containing [adjustedX, adjustedY].
     */
    public double[] applyAntiTipCorrection(double robotX, double robotY) {
        if (!tipCorrectionEnabled) {
            return new double[]{robotX, robotY};
        }

        Vector2D commanded = Vector2D.cartesian(robotX, robotY);
        double cmdMag = commanded.magnitude();
        if (cmdMag < 1e-4) {
            return new double[]{robotX, robotY};
        }

        if (follower.velocity() == null) {
            return new double[]{robotX, robotY};
        }

        // follower.velocity() returns field velocity
        Vector2D fieldVel = follower.velocity().toVector2D();
        double currentHeading = follower.pose().heading();

        // Rotate field velocity to robot coordinate frame
        Vector2D robotVel = Vector2D.cartesian(
                fieldVel.x() * Math.cos(-currentHeading) - fieldVel.y() * Math.sin(-currentHeading),
                fieldVel.x() * Math.sin(-currentHeading) + fieldVel.y() * Math.cos(-currentHeading)
        );

        double velMag = robotVel.magnitude();
        if (velMag < 5.0) { // Below 5 in/s, risk of tipping is negligible
            return new double[]{robotX, robotY};
        }

        // Alignment alpha = cos(theta) between commanded input and robot velocity
        double alpha = commanded.dot(robotVel) / (cmdMag * velMag + 1e-9);

        if (alpha < TIP_ALPHA_THRESHOLD) {
            // Robot is braking or reversing sharply against forward/backward velocity
            double forwardVel = Math.abs(robotVel.y());
            double normalizedSpeed = Math.min((forwardVel / TIP_MAX_VELOCITY) * 2.0, 1.0);
            double lambda = getLambda(normalizedSpeed, alpha);

            // Scale only the opposing forward/backward component, preserving lateral strafe authority
            robotY *= lambda;
        }

        return new double[]{robotX, robotY};
    }

    /**
     * Computes the quadratic damping lambda factor for anti-tipping.
     *
     * @param s Normalized velocity (0.0 to 1.0).
     * @param alpha Alignment cosine (-1.0 to 1.0).
     * @return Damping multiplier between 0.0 and 1.0.
     */
    public double getLambda(double s, double alpha) {
        double g = (1.0 - alpha) / 2.0;
        double lambda = 1.0 - (1.0 - TIP_CLAMP_POWER) * s * s * g * g;
        if (lambda > TIP_MAX_LAMBDA) return 1.0;
        return Math.max(lambda, 0.0);
    }

    public void setTipCorrectionEnabled(boolean enabled) {
        this.tipCorrectionEnabled = enabled;
    }

    public boolean isTipCorrectionEnabled() {
        return tipCorrectionEnabled;
    }
}

/**
 * The Drawing class handles the visualization of the robot on the Panels Dashboard.
 */
class Drawing {
    public static final double ROBOT_RADIUS = 8;
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style("#008000", "#3F51B5", 0.0);

    /**
     * Initializes the Panels Field with default FTC offsets.
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * Draws a representation of the robot at a specified Pose with a given style.
     *
     * @param pose  The Pose to draw the robot at.
     * @param style The style parameters for drawing.
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.x()) || Double.isNaN(pose.y()) || Double.isNaN(pose.heading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.x(), pose.y());
        panelsField.circle(ROBOT_RADIUS);

        Vector2D unit = Vector2D.polar(1.0, pose.heading());
        double x1 = pose.x() + unit.x() * (ROBOT_RADIUS / 2);
        double y1 = pose.y() + unit.y() * (ROBOT_RADIUS / 2);
        double x2 = pose.x() + unit.x() * ROBOT_RADIUS;
        double y2 = pose.y() + unit.y() * ROBOT_RADIUS;

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * Draws a representation of the robot at a specified Pose using the default style.
     *
     * @param pose The Pose to draw the robot at.
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * Sends the current drawing packet to the FTControl Panels dashboard.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}
