package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

/**
 * High-performance, agile field-centric drive command for the TechMaker (#23069) chassis.
 *
 * <p>Advanced integrated teleop drive architecture featuring:
 * <ul>
 *   <li>Default Field-Centric drive with alliance-aware orientation.</li>
 *   <li>Non-linear tangent response curves for millimeter precision at low speed.</li>
 *   <li>Mecanum denominator normalization allowing simultaneous translation and rotation.</li>
 *   <li><b>Kick (Right Bumper)</b>: Rapid closed-loop trajectory toward the scoring zone.</li>
 *   <li><b>Hold Pose (Button B)</b>: Active closed-loop position holding resisting pushes and drift.</li>
 *   <li><b>Dynamic Angle Snap (Left Trigger)</b>: Snaps and holds the nearest 90° angle while held.</li>
 *   <li><b>Cardinal Snaps (D-Pad)</b>: 1-touch snaps to 0°, 90°, 180°, 270°.</li>
 * </ul>
 *
 * <p><b>Nota Biobuzz:</b> o Right Trigger foi liberado desta classe (não faz mais
 * {@code lockHeading}) porque agora é reservado para o {@link KinematicAimDriveCommand}
 * (mira cinemática na CÉLULA ativa do HIVE), agendado como comando de prioridade 1 a partir
 * do OpMode. Ao soltar o gatilho, aquele comando termina sozinho e o Ivy retoma este
 * comando contínuo automaticamente.
 *
 * @author LucasDiegoHD - TechMaker (#23069)
 */
public final class TeleOpDriveCommand {

    private TeleOpDriveCommand() {
    }

    public static Command teleOpDrive(DrivetrainSubsystem drivetrain, Gamepad driverGamepad) {
        final AllianceEnum alliance = DataStorage.alliance;
        final double headingOffset = (alliance == AllianceEnum.Blue) ? Math.toRadians(180) : 0.0;
        class DriverAssistState {
            boolean prevRightBumper = false;
            boolean prevBButton = false;
            boolean wasLeftTriggerHeld = false;
        }
        final DriverAssistState state = new DriverAssistState();

        return Command.build()
                .setStart(() -> {
                    drivetrain.setTeleOp(true);
                    drivetrain.stopHoldPose();
                    state.prevRightBumper = false;
                    state.prevBButton = false;
                    state.wasLeftTriggerHeld = false;
                })
                .setExecute(() -> {
                    boolean currentRightBumper = driverGamepad.right_bumper;
                    if (currentRightBumper && !state.prevRightBumper) {
                        if (drivetrain.isKicking()) {
                            drivetrain.cancelKick();
                            driverGamepad.rumble(80);
                        } else {
                            drivetrain.kick();
                            driverGamepad.rumble(200);
                        }
                    }
                    state.prevRightBumper = currentRightBumper;

                    boolean currentBButton = driverGamepad.b;
                    if (currentBButton && !state.prevBButton) {
                        boolean nowHolding = drivetrain.toggleHoldPose();
                        if (nowHolding) {
                            driverGamepad.rumble(150);
                        } else {
                            driverGamepad.rumble(80);
                        }
                    }
                    state.prevBButton = currentBButton;

                    if (driverGamepad.left_trigger > 0.2) {
                        if (!state.wasLeftTriggerHeld) {
                            drivetrain.snapToNearestCardinal();
                            state.wasLeftTriggerHeld = true;
                        }
                    } else if (state.wasLeftTriggerHeld) {
                        drivetrain.unlockHeading();
                        state.wasLeftTriggerHeld = false;
                    }

                    if (driverGamepad.dpad_up) {
                        drivetrain.lockHeading(0.0);
                        state.wasLeftTriggerHeld = false;
                    } else if (driverGamepad.dpad_right) {
                        drivetrain.lockHeading(Math.toRadians(90.0));
                        state.wasLeftTriggerHeld = false;
                    } else if (driverGamepad.dpad_down) {
                        drivetrain.lockHeading(Math.toRadians(180.0));
                        state.wasLeftTriggerHeld = false;
                    } else if (driverGamepad.dpad_left) {
                        drivetrain.lockHeading(Math.toRadians(-90.0));
                        state.wasLeftTriggerHeld = false;
                    }

                    drivetrain.arcadeDrive(driverGamepad, headingOffset);
                })
                .setDone(() -> false)
                .requiring(drivetrain)
                .setPriority(0)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND);
    }
}
