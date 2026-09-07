package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.paths.BiobuzzAutoPaths;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

/**
 * Universal Autonomous Selector for the Biobuzz Season.
 *
 * <p>Allows driver selection of alliance color and strategy on gamepad2 during {@link #init_loop()}.
 * The selected routine is prepared, and the robot starting pose is loaded into the follower
 * so localizers (Pinpoint / OTOS) settle prior to match start.
 *
 * @author LucasDiegoHD - Team #23069
 */
@Autonomous(name = "⭐️ Autonomous Selector (Biobuzz)", group = "Competition")
public class Autos extends RobotOpMode {

    enum Strategy {
        BIOBUZZ_FULL,
        BIOBUZZ_PRELOAD_PARK
    }

    private AllianceEnum selectedAlliance = AllianceEnum.Red;
    private Strategy selectedStrategy = Strategy.BIOBUZZ_FULL;
    private boolean isConfigured = false;

    private Command autonomousCommand;

    @Override
    public void init_loop() {
        if (!isConfigured) {
            readConfiguration();
            showConfiguration();
        }

        super.init_loop();
    }

    private void readConfiguration() {
        if (gamepad2.xWasPressed()) selectedAlliance = AllianceEnum.Blue;
        if (gamepad2.bWasPressed()) selectedAlliance = AllianceEnum.Red;

        if (gamepad2.dpadUpWasPressed()) selectedStrategy = Strategy.BIOBUZZ_FULL;
        if (gamepad2.dpadDownWasPressed()) selectedStrategy = Strategy.BIOBUZZ_PRELOAD_PARK;

        if (gamepad2.a) {
            isConfigured = true;
            prepareRoutine();
        }
    }

    private void showConfiguration() {
        telemetry.addData("=== BIOBUZZ AUTO CONFIGURATION ===", "");
        telemetry.addData("Alliance [X / B]",
                selectedAlliance == AllianceEnum.Red ? "🔴 RED" : "🔵 BLUE");

        String stratText = (selectedStrategy == Strategy.BIOBUZZ_FULL)
                ? "🔼 1. Full Cycle (Preload + Floor Intake + Score + Park)"
                : "🔽 2. Simple (Preload Score + Park)";

        telemetry.addData("Routine [D-PAD UP/DOWN]", stratText);
        telemetry.addData("--------------------------------", "");
        telemetry.addData(">> PRESS 'A' TO CONFIRM <<", "");
        telemetry.update();
    }

    private void prepareRoutine() {
        DataStorage.alliance = selectedAlliance;

        Pose startPose = BiobuzzAutoPaths.getStartPose();

        if (selectedStrategy == Strategy.BIOBUZZ_FULL) {
            autonomousCommand = AutoRoutines.biobuzzFullRoutine(robot);
        } else {
            autonomousCommand = AutoRoutines.biobuzzPreloadAndPark(robot);
        }

        robot.setAutoStartPose(startPose);

        telemetry.addData("Status", "✅ Localizer Initialized! Ready for PLAY ▶️");
        telemetry.update();
    }

    @Override
    public void start() {
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }
}
