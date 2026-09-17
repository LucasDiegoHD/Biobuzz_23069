package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.paths.BiobuzzAutoPaths;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.BiobuzzFieldRenderer;
import org.firstinspires.ftc.teamcode.utils.DataStorage;
import org.firstinspires.ftc.teamcode.utils.HiveTargets;

/**
 * Universal Autonomous Selector for the Biobuzz Season.
 *
 * @author LucasDiegoHD - Team #23069
 */
@Autonomous(name = "⭐️ Autonomous Selector (Biobuzz)", group = "Competition")
public class Autos extends RobotOpMode {

    enum Strategy {
        BIOBUZZ_FULL,
        BIOBUZZ_PRELOAD_PARK,
        BIOBUZZ_PARK_ONLY
    }

    private AllianceEnum selectedAlliance = AllianceEnum.Red;
    private Strategy selectedStrategy = Strategy.BIOBUZZ_FULL;
    private boolean isConfigured = false;
    private boolean prevX, prevB, prevUp, prevDown, prevLeft, prevA;
    private Command autonomousCommand;

    @Override
    public void init() {
        super.init();
        robot.setAutoStartPose(BiobuzzAutoPaths.getStartPose());
    }

    @Override
    public void init_loop() {
        super.init_loop();

        if (!isConfigured) {
            readConfiguration();
            showConfiguration();
            BiobuzzFieldRenderer.drawAutoPreview(selectedAlliance);
        } else {
            telemetry.addData("STATUS", "✅ CONFIGURADO E PRONTO!");
            telemetry.addData("Aliança", selectedAlliance);
            telemetry.addData("Estratégia", selectedStrategy);
            telemetry.addData("Dica", "Pressione 'B' para reconfigurar se necessário.");

            if (gamepad2.b && !prevB) {
                isConfigured = false;
            }
            prevB = gamepad2.b;
            telemetry.update();
        }
    }

    private void readConfiguration() {
        boolean currX = gamepad2.x;
        boolean currB = gamepad2.b;
        boolean currUp = gamepad2.dpad_up;
        boolean currDown = gamepad2.dpad_down;
        boolean currLeft = gamepad2.dpad_left;
        boolean currA = gamepad2.a;

        if (currX && !prevX) selectedAlliance = AllianceEnum.Blue;
        if (currB && !prevB) selectedAlliance = AllianceEnum.Red;

        if (currUp && !prevUp) selectedStrategy = Strategy.BIOBUZZ_FULL;
        if (currDown && !prevDown) selectedStrategy = Strategy.BIOBUZZ_PRELOAD_PARK;
        if (currLeft && !prevLeft) selectedStrategy = Strategy.BIOBUZZ_PARK_ONLY;

        if (currA && !prevA) {
            isConfigured = true;
            prepareRoutine();
        }

        prevX = currX;
        prevB = currB;
        prevUp = currUp;
        prevDown = currDown;
        prevLeft = currLeft;
        prevA = currA;
    }

    private void showConfiguration() {
        telemetry.addData("=== BIOBUZZ AUTO CONFIGURATION ===", "");
        telemetry.addData("Alliance [X = BLUE | B = RED]",
                selectedAlliance == AllianceEnum.Red ? "🔴 RED" : "🔵 BLUE");

        String stratText;
        switch (selectedStrategy) {
            case BIOBUZZ_FULL:
                stratText = "🔼 1. Full Cycle (Preload + Floor + Score + Park)";
                break;
            case BIOBUZZ_PRELOAD_PARK:
                stratText = "🔽 2. Simple (Preload Score + Park)";
                break;
            default:
                stratText = "◀️ 3. Park Only (LEAVE + PARK)";
                break;
        }

        telemetry.addData("Routine [D-PAD UP/DOWN/LEFT]", stratText);
        telemetry.addData("--------------------------------", "");
        telemetry.addData(">> PRESSIONA 'A' PARA CONFIRMAR <<", "");
        telemetry.update();
    }

    private void prepareRoutine() {
        DataStorage.alliance = selectedAlliance;
        HiveTargets.resetForMatchStart(selectedAlliance);

        Pose startPose = BiobuzzAutoPaths.getStartPose(selectedAlliance);

        robot.drivetrain.getFollower().setPose(startPose);

        switch (selectedStrategy) {

            case BIOBUZZ_PRELOAD_PARK:
                autonomousCommand = AutoRoutines.biobuzzPreloadAndPark(robot);
                break;
            default:
                autonomousCommand = AutoRoutines.biobuzzStartToPark(robot);
                break;
        }
    }

    @Override
    public void start() {
        super.start();

        // Biobuzz fix: sem isso, DrivetrainSubsystem.update() nunca roda o follower.update()
        // completo durante o autônomo (só atualiza a pose) — o path nunca recebe correção/
        // frenagem e o robô passa direto pelo alvo. isTeleOp precisa ser false aqui.
        robot.drivetrain.setTeleOp(false);

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void loop() {
        super.loop();

        telemetry.addData("Pedro Pathing Busy", robot.drivetrain.getFollower().isBusy());
        telemetry.addData("Pose Atual X", robot.drivetrain.getFollower().pose().x());
        telemetry.addData("Pose Atual Y", robot.drivetrain.getFollower().pose().y());
        telemetry.addData("Heading", Math.toDegrees(robot.drivetrain.getFollower().pose().heading()));
        telemetry.update();
    }
}
