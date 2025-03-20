package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ScoringLevel;
import frc.robot.Constants.kAutoAlign;
import frc.robot.Constants.kEndEffector;
import frc.robot.Constants.kAutoAlign.kReef;
import frc.robot.commands.scoring.IdleCommand;
import frc.robot.commands.scoring.RemoveAlgae;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.arm.ArmPivot;
import frc.robot.subsystems.collector.EndEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AlignHelper;
import frc.robot.util.CaseCommand;
import frc.robot.util.DebugCommand;
import frc.robot.util.SelectorCommand;
import frc.robot.util.AlignHelper.kClosestType;
import frc.robot.util.AlignHelper.kDirection;

public class AutoCommands {
    public static enum kReefPosition {
        FAR,
        FAR_LEFT,
        FAR_RIGHT,
        CLOSE,
        CLOSE_LEFT,
        CLOSE_RIGHT
    }

    private static final PathConstraints CONSTRAINTS = kAutoAlign.PATH_FIND_CONSTRAINTS;
    private static final LinearVelocity REEF_END_VELOCITY = MetersPerSecond.of(1.5);
    
    public static GenericEntry scoreRight;
    public static LoggedDashboardChooser<kDirection> stationChooser;
    public static kReefPosition target = kReefPosition.CLOSE_LEFT;

    private AutoCommands() {}

    public static void setupNodeChooser() {
        for (kReefPosition reef : kReefPosition.values()) {
            DebugCommand.register(reef.name(), Commands.runOnce(() -> target = reef));
        }

        scoreRight = Shuffleboard.getTab("Debug")
                        .add("Score Right", false)
                        .getEntry();


        stationChooser = new LoggedDashboardChooser<>("Station Select");

        stationChooser.addDefaultOption("Automatic", kDirection.BOTH );
        stationChooser.addOption(       "Left",      kDirection.LEFT );
        stationChooser.addOption(       "Right",     kDirection.RIGHT);
    }

    public static Command pathfindTo(Drive drive, Supplier<Pose2d> targetPose, boolean acceptVelocity, boolean selector) {
        List<Pose2d> poses = new ArrayList<>();
        poses.addAll(AlignHelper.getStationPoses(kDirection.BOTH));
        poses.addAll(kReef.TARGETS.values());

        BooleanSupplier[] suppliers = new BooleanSupplier[poses.size()];
        Command[] commands = new Command[poses.size()];

        for (int i = 0; i < poses.size(); i++) {
            final Pose2d pose = poses.get(i);

            suppliers[i] = () -> pose.equals(targetPose.get());

            if (acceptVelocity)
                commands[i] = AutoBuilder.pathfindToPoseFlipped(pose, CONSTRAINTS).raceWith(new InstantCommand()).andThen(
                    new ConditionalCommand(
                        AutoBuilder.pathfindToPoseFlipped(pose, CONSTRAINTS, REEF_END_VELOCITY),
                        AutoBuilder.pathfindToPoseFlipped(pose, CONSTRAINTS),
                        () -> {
                            PathPlannerPath path = Pathfinding.getCurrentPath(
                                CONSTRAINTS,
                                new GoalEndState(
                                    MetersPerSecond.of(0.0), 
                                    pose.getRotation()
                                )
                            );

                            if (AutoBuilder.shouldFlip())
                                path = path.flipPath();

                            List<PathPoint> points = path.getAllPathPoints();

                            if (points.size() <= 1) return false;

                            PathPoint p1 = points.get(points.size() - 2);
                            PathPoint p2 = points.get(points.size() - 1);

                            Rotation2d robotTravel = Rotation2d.fromRadians(
                                Math.atan2(
                                    p2.position.getY() - p1.position.getY(),
                                    p2.position.getX() - p1.position.getX()
                                )
                            );

                            Translation2d target = pose
                                .transformBy(scoreRight.getBoolean(false) ? kReef.RIGHT_OFFSET_TO_BRANCH : kReef.LEFT_OFFSET_TO_BRANCH)
                                .getTranslation();

                            Rotation2d robotGoal = Rotation2d.fromRadians(
                                Math.atan2(
                                    target.getY() - p2.position.getY(),
                                    target.getX() - p2.position.getX()
                                )
                            );

                            Angle difference = AlignHelper.rotationDifference(robotTravel, robotGoal);

                            return difference.lte(Degrees.of(45));
                        }
                    )
                );
            else
                commands[i] = AutoBuilder.pathfindToPoseFlipped(pose, CONSTRAINTS);
        }

        if (selector)
            return CaseCommand.buildSelector(suppliers, commands, Commands.print("ERROR WITH AUTO-TELE"));
        else
            return CaseCommand.buildCondtional(suppliers, commands, Commands.print("ERROR WITH AUTO-TELE"));
    }

    public static Command pathFindToNearestStation(Drive drive) {
        return pathfindTo(
            drive, 
            () -> AlignHelper.getClosestStation(
                drive.getBlueSidePose(), 
                kClosestType.DISTANCE, 
                stationChooser.get()
            ),
            false,
            false
        ).beforeStarting(() -> AlignHelper.reset(new ChassisSpeeds()))
        .andThen(Commands.waitSeconds(0.5));
    }

    public static Command pathFindToReef(Drive drive, Supplier<kReefPosition> reef) {
        return pathfindTo(
            drive,
            () -> kReef.TARGETS.getOrDefault(
                reef.get(), 
                new Pose2d()
            ),
            true,
            true
        );
    }

    public static Command alignToBranch(Drive drive, Supplier<kDirection> direction) {
        return DriveCommands.alignToPoint(
            drive, 
            () -> AlignHelper.getClosestBranch(drive.getBlueSidePose(), kClosestType.DISTANCE, direction.get())
        ).beforeStarting(() -> AlignHelper.reset(new ChassisSpeeds()));
    }

    public static Command alignToAlgae(Drive drive) {
        return DriveCommands.alignToPoint(
            drive,
            () -> AlignHelper.getClosestReef(drive.getBlueSidePose()).transformBy(new Transform2d(0.385, 0.0, new Rotation2d()))
        ).beforeStarting(() -> AlignHelper.reset(new ChassisSpeeds()));
    }

    public static Command backOffFromAlgae(Drive drive, Rotation2d rotationOffset) {
        return DriveCommands.alignToPoint(
            drive, 
            () -> AlignHelper.getClosestReef(drive.getBlueSidePose()).transformBy(new Transform2d(0, 0, rotationOffset))
        ).beforeStarting(() -> AlignHelper.reset(new ChassisSpeeds()));
    }

    public static Command automaticAlgae(Drive sys_drive, EndEffector sys_endEffector, Elevator sys_elevator, ArmPivot sys_armPivot) {
        Command idealAlgae = 
            Commands.sequence(
                AutoCommands.alignToAlgae(sys_drive),
                Commands.waitUntil(() -> sys_endEffector.getCurrent() >= kEndEffector.ALGAE_CURRENT),
                AutoCommands.backOffFromAlgae(sys_drive, new Rotation2d()).withTimeout(0.5)
            ).alongWith(
                new SelectorCommand(
                    new RemoveAlgae(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL2_ALGAE), 
                    new RemoveAlgae(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL3_ALGAE), 
                    () -> AlignHelper.getAlgaeHeight(sys_drive.getBlueSidePose()) == ScoringLevel.LEVEL2_ALGAE
                )
            );

        Command L4Algae = 
            Commands.sequence(
                AutoCommands.alignToAlgae(sys_drive),
                new SelectorCommand(
                    new RemoveAlgae(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL2_ALGAE), 
                    new RemoveAlgae(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL3_ALGAE), 
                    () -> AlignHelper.getAlgaeHeight(sys_drive.getBlueSidePose()) == ScoringLevel.LEVEL2_ALGAE
                ).raceWith(Commands.waitUntil(() -> sys_endEffector.getCurrent() >= kEndEffector.ALGAE_CURRENT)),
                AutoCommands.backOffFromAlgae(sys_drive, new Rotation2d()).withTimeout(0.5)
            );
        
        Command tooCloseAlgae = 
            AutoCommands.backOffFromAlgae(sys_drive, new Rotation2d()).andThen(
                Commands.sequence(
                    AutoCommands.alignToAlgae(sys_drive),
                    Commands.waitUntil(() -> sys_endEffector.getCurrent() >= kEndEffector.ALGAE_CURRENT),
                    AutoCommands.backOffFromAlgae(sys_drive, new Rotation2d()).withTimeout(0.5)
                ).alongWith(
                    new SelectorCommand(
                        new RemoveAlgae(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL2_ALGAE), 
                        new RemoveAlgae(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL3_ALGAE), 
                        () -> AlignHelper.getAlgaeHeight(sys_drive.getBlueSidePose()) == ScoringLevel.LEVEL2_ALGAE
                    )
                )
            );

        return Commands.either(
            idealAlgae, 
            Commands.either(
                tooCloseAlgae, 
                L4Algae, 
                () -> sys_elevator.getPosition().lte(ScoringLevel.LEVEL3.elevatorSetpoint.plus(Centimeters.of(5.0)))
            ), 
            Drive::isSafe
        );
    }

    public static Command telopAutoCommand(Drive drive, Elevator sys_elevator, ArmPivot sys_pivot, EndEffector sys_endeffector, Command scoringCommand, BooleanSupplier isAlgae, BooleanSupplier waitBeforeScoring) {
        Command waitForAlgae = RobotBase.isReal() ? 
            Commands.waitUntil(() -> sys_endeffector.getCurrent() >= 20.0) :
            Commands.waitSeconds(0.5);

        return Commands.sequence(
            // TODO: TEST AT MULTIPLE AUTO POSITIONS
            Commands.parallel(
                new IdleCommand(sys_elevator, sys_pivot, sys_endeffector),
                pathFindToNearestStation(drive).unless(sys_endeffector::coralDetected)
            ),
            pathFindToReef(drive, () -> target),
            alignToBranch(drive, () -> (scoreRight.getBoolean(false) ? kDirection.RIGHT : kDirection.LEFT))
                .alongWith(scoringCommand),
            backOffFromAlgae(drive, new Rotation2d())
                .andThen(alignToAlgae(drive))
                .alongWith(
                    new ConditionalCommand(
                        new RemoveAlgae(sys_elevator, sys_pivot, sys_endeffector, ScoringLevel.LEVEL2_ALGAE), 
                        new RemoveAlgae(sys_elevator, sys_pivot, sys_endeffector, ScoringLevel.LEVEL3_ALGAE), 
                        () -> AlignHelper.getAlgaeHeight(drive.getBlueSidePose()) == ScoringLevel.LEVEL2_ALGAE
                    )
                )
                .andThen(
                    waitForAlgae,
                    backOffFromAlgae(drive, Rotation2d.fromDegrees(180.0))
                ).onlyIf(isAlgae)
        ).repeatedly();
    }
}
