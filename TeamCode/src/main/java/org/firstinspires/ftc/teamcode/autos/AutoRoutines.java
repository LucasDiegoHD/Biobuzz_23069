package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import org.firstinspires.ftc.teamcode.autos.commands.GoToPoseCommand;
import org.firstinspires.ftc.teamcode.autos.paths.BiobuzzAutoPaths;
import org.firstinspires.ftc.teamcode.autos.paths.FrontAutoPaths;
import org.firstinspires.ftc.teamcode.autos.paths.PosesNames;
import org.firstinspires.ftc.teamcode.autos.paths.RearAutoPaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.List;

/**
 * Autonomous routines structured as clean Ivy command compositions.
 * Executes drivetrain trajectory paths using Pedro Pathing.
 *
 * @author LucasDiegoHD - Team #23069
 */
public final class AutoRoutines {

    private AutoRoutines() {
    }

    private static Command withTimeout(Command command, double milliseconds) {
        return command.raceWith(Commands.waitMs(milliseconds));
    }

    /**
     * Official Biobuzz Season Autonomous Routine Template.
     * Full sequence: Preload delivery -> Cycle 1 Intake & Score -> Park.
     */
//    public static Command biobuzzFullRoutine(Robot robot) {
//        return Groups.sequential(
//                new GoToPoseCommand(robot.drivetrain, BiobuzzAutoPaths.getPreloadScore())
//                        .toCommand(),
//
//                Commands.waitMs(600),
//
//                new GoToPoseCommand(robot.drivetrain, true,
//                        BiobuzzAutoPaths.CYCLE_1_INTAKE_CONTROL.getPose(),
//                        BiobuzzAutoPaths.getCycle1Intake())
//                        .withTangentHeading()
//                        .toCommand(),
//
//                Commands.waitMs(600),
//
//                // 3. Return to Scoring Position
//                new GoToPoseCommand(robot.drivetrain, BiobuzzAutoPaths.getCycle1Score())
//                        .toCommand(),
//
//                // TODO: Insert piece delivery action here
//                Commands.waitMs(600),
//
//                // 4. Drive to Parking Zone
//                new GoToPoseCommand(robot.drivetrain, BiobuzzAutoPaths.getParkPose())
//                        .toCommand()
//        );
//    }

    /**
     * Rotina mínima: sai da parede (LEAVE) e vai direto pra LOADING ZONE (PARK).
     * Sem disparo nenhum — baseline de segurança pra garantir LEAVE (3) + PARK (5) = 8 pts
     * mesmo se o resto do robô ainda não estiver pronto.
     */
    public static Command biobuzzStartToPark(Robot robot) {
        return Groups.sequential(
                new GoToPoseCommand(robot.drivetrain, BiobuzzAutoPaths.getParkPose())
                        .withConstantHeading(BiobuzzAutoPaths.getParkPose().heading())
                        .toCommand()
        );
    }

    /**
     * Simplified Biobuzz Season Routine: Deliver Preload and Park.
     * Reliable baseline routine for testing and early matches.
     */
    public static Command biobuzzPreloadAndPark(Robot robot) {
        return Groups.sequential(
                new GoToPoseCommand(robot.drivetrain, BiobuzzAutoPaths.getPreloadScore())
                        .toCommand(),

                Commands.waitMs(800),

                new GoToPoseCommand(robot.drivetrain, BiobuzzAutoPaths.getParkPose())
                        .toCommand()
        );
    }
}
