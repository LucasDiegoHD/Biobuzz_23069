package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
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
    public static Command biobuzzFullRoutine(Robot robot) {
        return Groups.sequential(
                // 1. Move to Preload Score Position
                new GoToPoseCommand(robot.drivetrain, BiobuzzAutoPaths.getPreloadScore())
                        .setConstraints(Constants.autoShootConstraints)
                        .toCommand(),

                // TODO: Insert preload scoring subsystem action here
                Commands.waitMs(600),

                // 2. Drive along curved path to Cycle 1 Intake
                new GoToPoseCommand(robot.drivetrain, true,
                        BiobuzzAutoPaths.CYCLE_1_INTAKE_CONTROL.getPose(),
                        BiobuzzAutoPaths.getCycle1Intake())
                        .setConstraints(Constants.autoTransitConstraints)
                        .withTangentHeading()
                        .toCommand(),

                // TODO: Insert floor intake subsystem action here
                Commands.waitMs(600),

                // 3. Return to Scoring Position
                new GoToPoseCommand(robot.drivetrain, BiobuzzAutoPaths.getCycle1Score())
                        .setConstraints(Constants.autoShootConstraints)
                        .toCommand(),

                // TODO: Insert piece delivery action here
                Commands.waitMs(600),

                // 4. Drive to Parking Zone
                new GoToPoseCommand(robot.drivetrain, BiobuzzAutoPaths.getParkPose())
                        .setConstraints(Constants.pathConstraints)
                        .toCommand()
        );
    }

    /**
     * Simplified Biobuzz Season Routine: Deliver Preload and Park.
     * Reliable baseline routine for testing and early matches.
     */
    public static Command biobuzzPreloadAndPark(Robot robot) {
        return Groups.sequential(
                // 1. Move to Preload Score Position
                new GoToPoseCommand(robot.drivetrain, BiobuzzAutoPaths.getPreloadScore())
                        .setConstraints(Constants.autoShootConstraints)
                        .toCommand(),

                // TODO: Insert preload scoring subsystem action here
                Commands.waitMs(800),

                // 2. Drive to Parking Zone
                new GoToPoseCommand(robot.drivetrain, BiobuzzAutoPaths.getParkPose())
                        .setConstraints(Constants.pathConstraints)
                        .toCommand()
        );
    }

    /** Rear trajectory with gate transit paths. */
    public static Command rearNormal(Robot robot) {
        return Groups.sequential(
                // Move to shoot position 1
                new GoToPoseCommand(robot.drivetrain, RearAutoPaths.getPose(PosesNames.GoToShoot1))
                        .setConstraints(Constants.autoShootConstraints)
                        .toCommand(),
                Commands.waitMs(500),

                // Line 1 collection path
                withTimeout(new GoToPoseCommand(robot.drivetrain, true,
                        RearAutoPaths.getPose(PosesNames.GoToLine1),
                        RearAutoPaths.getPose(PosesNames.CatchLine1))
                        .setConstraints(Constants.autoTransitConstraints)
                        .withNoDeceleration()
                        .withConstantHeading()
                        .toCommand(), 4000),

                // Return to shoot position 1
                new GoToPoseCommand(robot.drivetrain, RearAutoPaths.getPose(PosesNames.GoToShoot1))
                        .setConstraints(Constants.autoShootConstraints)
                        .withConstantHeading()
                        .toCommand(),
                Commands.waitMs(500),

                // Line 3 collection path
                withTimeout(new GoToPoseCommand(robot.drivetrain, true,
                        RearAutoPaths.getPose(PosesNames.GoToLine3),
                        RearAutoPaths.getPose(PosesNames.CatchLine3))
                        .setConstraints(Constants.autoTransitConstraints)
                        .withNoDeceleration()
                        .withConstantHeading()
                        .toCommand(), 4000),

                // Gate transit waypoint
                withTimeout(new GoToPoseCommand(robot.drivetrain, RearAutoPaths.getPose(PosesNames.GatePose))
                        .setConstraints(Constants.autoShootConstraints)
                        .toCommand(), 800),

                // Return to shoot position 1
                new GoToPoseCommand(robot.drivetrain, RearAutoPaths.getPose(PosesNames.GoToShoot1))
                        .setConstraints(Constants.autoTransitConstraints)
                        .withConstantHeading()
                        .toCommand(),
                Commands.waitMs(500),

                // Line 2 collection path
                withTimeout(new GoToPoseCommand(robot.drivetrain, true,
                        RearAutoPaths.getPose(PosesNames.GoToLine2),
                        RearAutoPaths.getPose(PosesNames.CatchLine2))
                        .setConstraints(Constants.autoTransitConstraints)
                        .withNoDeceleration()
                        .withConstantHeading()
                        .toCommand(), 2000),

                // Shoot position 2
                new GoToPoseCommand(robot.drivetrain, RearAutoPaths.getPose(PosesNames.GoToShoot2))
                        .setConstraints(Constants.autoShootConstraints)
                        .toCommand(),
                Commands.waitMs(500),

                // Line 2 cycle
                withTimeout(new GoToPoseCommand(robot.drivetrain, true,
                        RearAutoPaths.getPose(PosesNames.GoToLine2),
                        RearAutoPaths.getPose(PosesNames.CatchLine2))
                        .setConstraints(Constants.autoTransitConstraints)
                        .withNoDeceleration()
                        .withConstantHeading()
                        .toCommand(), 4000),

                new GoToPoseCommand(robot.drivetrain, RearAutoPaths.getPose(PosesNames.GoToShoot2))
                        .setConstraints(Constants.autoShootConstraints)
                        .toCommand(),
                Commands.waitMs(500),

                // Park at end pose
                new GoToPoseCommand(robot.drivetrain, RearAutoPaths.getPose(PosesNames.EndPose)).toCommand()
        );
    }

    public static Command rearNormal(Robot robot, List<Pose> unusedPoses) {
        return rearNormal(robot);
    }

    /** Front trajectory with large triangular sweeps. */
    public static Command front(Robot robot) {
        return Groups.sequential(
                withTimeout(new GoToPoseCommand(robot.drivetrain, FrontAutoPaths.getPose(PosesNames.GoToShoot1))
                        .toCommand(), 1500),
                Commands.waitMs(500),

                withTimeout(new GoToPoseCommand(robot.drivetrain, true,
                        FrontAutoPaths.getPose(PosesNames.GoToLine2),
                        FrontAutoPaths.getPose(PosesNames.CatchLine2)).toCommand(), 4000),

                new GoToPoseCommand(robot.drivetrain, FrontAutoPaths.getPose(PosesNames.GoToShoot2)).toCommand(),
                Commands.waitMs(800),

                withTimeout(new GoToPoseCommand(robot.drivetrain, true,
                        FrontAutoPaths.getPose(PosesNames.GoToLine1),
                        FrontAutoPaths.getPose(PosesNames.CatchLine1)).toCommand(), 4000),

                withTimeout(new GoToPoseCommand(robot.drivetrain, FrontAutoPaths.getPose(PosesNames.GoToShoot2))
                        .toCommand(), 2000),
                Commands.waitMs(800),

                withTimeout(new GoToPoseCommand(robot.drivetrain, FrontAutoPaths.getPose(PosesNames.GatePose))
                        .toCommand(), 2000),

                withTimeout(new GoToPoseCommand(robot.drivetrain, FrontAutoPaths.getPose(PosesNames.CatchLine3))
                        .toCommand(), 2000),
                Commands.waitMs(600),

                withTimeout(new GoToPoseCommand(robot.drivetrain, FrontAutoPaths.getPose(PosesNames.GoToShoot1))
                        .toCommand(), 2000),
                Commands.waitMs(800),

                withTimeout(new GoToPoseCommand(robot.drivetrain, FrontAutoPaths.getPose(PosesNames.EndPose))
                        .toCommand(), 2000)
        );
    }

    public static Command front(Robot robot, List<Pose> unusedPoses) {
        return front(robot);
    }

    /** Rear trajectory without gate transit (direct line cycles). */
    public static Command rearNoGate(Robot robot) {
        return Groups.sequential(
                new GoToPoseCommand(robot.drivetrain, RearAutoPaths.getPose(PosesNames.GoToShoot1))
                        .setConstraints(Constants.autoShootConstraints)
                        .toCommand(),
                Commands.waitMs(500),

                withTimeout(new GoToPoseCommand(robot.drivetrain, true,
                        RearAutoPaths.getPose(PosesNames.GoToLine1),
                        RearAutoPaths.getPose(PosesNames.CatchLine1))
                        .setConstraints(Constants.autoTransitConstraints)
                        .withNoDeceleration()
                        .withConstantHeading()
                        .toCommand(), 3000),

                new GoToPoseCommand(robot.drivetrain, RearAutoPaths.getPose(PosesNames.GoToShoot1))
                        .setConstraints(Constants.autoShootConstraints)
                        .withConstantHeading()
                        .toCommand(),
                Commands.waitMs(500),

                withTimeout(new GoToPoseCommand(robot.drivetrain, true,
                        RearAutoPaths.getPose(PosesNames.GoToLine2),
                        RearAutoPaths.getPose(PosesNames.CatchLine2))
                        .setConstraints(Constants.autoTransitConstraints)
                        .withNoDeceleration()
                        .withConstantHeading()
                        .toCommand(), 2000),

                new GoToPoseCommand(robot.drivetrain, RearAutoPaths.getPose(PosesNames.GoToShoot2))
                        .setConstraints(Constants.autoShootConstraints)
                        .toCommand(),
                Commands.waitMs(500),

                withTimeout(new GoToPoseCommand(robot.drivetrain, true,
                        RearAutoPaths.getPose(PosesNames.GoToLine2),
                        RearAutoPaths.getPose(PosesNames.CatchLine2))
                        .setConstraints(Constants.autoTransitConstraints)
                        .withNoDeceleration()
                        .withConstantHeading()
                        .toCommand(), 2000),

                new GoToPoseCommand(robot.drivetrain, RearAutoPaths.getPose(PosesNames.GoToShoot2))
                        .setConstraints(Constants.autoShootConstraints)
                        .toCommand(),
                Commands.waitMs(500),

                withTimeout(new GoToPoseCommand(robot.drivetrain, true,
                        RearAutoPaths.getPose(PosesNames.GoToLine1),
                        RearAutoPaths.getPose(PosesNames.CatchLine1))
                        .setConstraints(Constants.autoTransitConstraints)
                        .withNoDeceleration()
                        .withConstantHeading()
                        .toCommand(), 4000),

                new GoToPoseCommand(robot.drivetrain, RearAutoPaths.getPose(PosesNames.GoToShoot2))
                        .setConstraints(Constants.autoShootConstraints)
                        .withConstantHeading()
                        .toCommand(),
                Commands.waitMs(500),

                new GoToPoseCommand(robot.drivetrain, RearAutoPaths.getPose(PosesNames.EndPose)).toCommand()
        );
    }

    public static Command rearNoGate(Robot robot, List<Pose> unusedPoses) {
        return rearNoGate(robot);
    }
}
