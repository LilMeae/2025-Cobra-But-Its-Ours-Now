// package frc.robot.commands;

// import static edu.wpi.first.units.Units.*;

// import java.util.ArrayList;
// import java.util.List;
// import java.util.function.BooleanSupplier;
// import java.util.function.Supplier;
// import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPoint;
// import com.pathplanner.lib.pathfinding.Pathfinding;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.LinearVelocity;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.ConditionalCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.Constants.ScoringLevel;
// import frc.robot.Constants.kAutoAlign;
// import frc.robot.Constants.kEndEffector;
// import frc.robot.Constants.kAutoAlign.kReef;
// import frc.robot.commands.scoring.IdleCommand;
// import frc.robot.commands.scoring.RemoveAlgae;
// import frc.robot.subsystems.Elevator.Elevator;
// import frc.robot.subsystems.arm.ArmPivot;
// import frc.robot.subsystems.collector.EndEffector;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.util.AlignHelper;
// import frc.robot.util.CaseCommand;
// import frc.robot.util.DebugCommand;
// import frc.robot.util.SelectorCommand;
// import frc.robot.util.AlignHelper.kClosestType;
// import frc.robot.util.AlignHelper.kDirection;

// public class AutoCommands {
//     public static enum kReefPosition {
//         FAR,
//         FAR_LEFT,
//         FAR_RIGHT,
//         CLOSE,
//         CLOSE_LEFT,
//         CLOSE_RIGHT
//     }

//     private static final PathConstraints CONSTRAINTS = kAutoAlign.PATH_FIND_CONSTRAINTS;
//     private static final LinearVelocity REEF_END_VELOCITY = MetersPerSecond.of(1.5);
    
//     public static GenericEntry scoreRight;
//     public static LoggedDashboardChooser<kDirection> stationChooser;
//     public static kReefPosition target = kReefPosition.CLOSE_LEFT;

//     private AutoCommands() {}

//     public static void setupNodeChooser() {
//         for (kReefPosition reef : kReefPosition.values()) {
//             DebugCommand.register(reef.name(), Commands.runOnce(() -> target = reef));
//         }

//         scoreRight = Shuffleboard.getTab("Debug")
//                         .add("Score Right", false)
//                         .getEntry();


//         stationChooser = new LoggedDashboardChooser<>("Station Select");

//         stationChooser.addDefaultOption("Automatic", kDirection.BOTH );
//         stationChooser.addOption(       "Left",      kDirection.LEFT );
//         stationChooser.addOption(       "Right",     kDirection.RIGHT);
//     }

//     /**
//      * Path finds to specific point on the field
//      * @param drive the drive subsystem
//      * @param targetPose the target pose on the field
//      * @param acceptVelocity if it should take in your velocity
//      * @param selector if this command should be a selector command, meaning if the target pose changes should the path change or wait to finish
//      * @return A command that path finds
//      * @apiNote Only used for telop auto
//      */
//     public static Command pathfindTo(Drive drive, Supplier<Pose2d> targetPose, boolean acceptVelocity, boolean selector) {
//         List<Pose2d> poses = new ArrayList<>();
//         poses.addAll(AlignHelper.getStationPoses(kDirection.BOTH));
//         poses.addAll(kReef.TARGETS.values());

//         BooleanSupplier[] suppliers = new BooleanSupplier[poses.size()];
//         Command[] commands = new Command[poses.size()];

//         for (int i = 0; i < poses.size(); i++) {
//             final Pose2d pose = poses.get(i);

//             suppliers[i] = () -> pose.equals(targetPose.get());

//             if (acceptVelocity)
//                 commands[i] = AutoBuilder.pathfindToPoseFlipped(pose, CONSTRAINTS).raceWith(new InstantCommand()).andThen(
//                     new ConditionalCommand(
//                         AutoBuilder.pathfindToPoseFlipped(pose, CONSTRAINTS, REEF_END_VELOCITY),
//                         AutoBuilder.pathfindToPoseFlipped(pose, CONSTRAINTS),
//                         () -> {
//                             PathPlannerPath path = Pathfinding.getCurrentPath(
//                                 CONSTRAINTS,
//                                 new GoalEndState(
//                                     MetersPerSecond.of(0.0), 
//                                     pose.getRotation()
//                                 )
//                             );

//                             if (AutoBuilder.shouldFlip())
//                                 path = path.flipPath();

//                             List<PathPoint> points = path.getAllPathPoints();

//                             if (points.size() <= 1) return false;

//                             PathPoint p1 = points.get(points.size() - 2);
//                             PathPoint p2 = points.get(points.size() - 1);

//                             Rotation2d robotTravel = Rotation2d.fromRadians(
//                                 Math.atan2(
//                                     p2.position.getY() - p1.position.getY(),
//                                     p2.position.getX() - p1.position.getX()
//                                 )
//                             );

//                             Translation2d target = pose
//                                 .transformBy(scoreRight.getBoolean(false) ? kReef.RIGHT_OFFSET_TO_BRANCH : kReef.LEFT_OFFSET_TO_BRANCH)
//                                 .getTranslation();

//                             Rotation2d robotGoal = Rotation2d.fromRadians(
//                                 Math.atan2(
//                                     target.getY() - p2.position.getY(),
//                                     target.getX() - p2.position.getX()
//                                 )
//                             );

//                             Angle difference = AlignHelper.rotationDifference(robotTravel, robotGoal);

//                             return difference.lte(Degrees.of(45));
//                         }
//                     )
//                 );
//             else
//                 commands[i] = AutoBuilder.pathfindToPoseFlipped(pose, CONSTRAINTS);
//         }

//         if (selector)
//             return CaseCommand.buildSelector(suppliers, commands, Commands.print("ERROR WITH AUTO-TELE"));
//         else
//             return CaseCommand.buildCondtional(suppliers, commands, Commands.print("ERROR WITH AUTO-TELE"));
//     }

//     /**
//      * Pathfinds to the nearest coral station
//      * @param drive the drive subsystem
//      * @return A command that drives to the nearest station
//      */
//     public static Command pathFindToNearestStation(Drive drive) {
//         return pathfindTo(
//             drive, 
//             () -> AlignHelper.getClosestStation(
//                 drive.getBlueSidePose(), 
//                 kClosestType.DISTANCE, 
//                 stationChooser.get()
//             ),
//             false,
//             false
//         ).beforeStarting(() -> AlignHelper.reset(new ChassisSpeeds()))
//         .andThen(Commands.waitSeconds(0.5));
//     }

//     /**
//      * Path finds to the nearest reef face
//      * @param drive the drive station
//      * @param reef a supplier of which face to pathfind to
//      * @return
//      */
//     public static Command pathFindToReef(Drive drive, Supplier<kReefPosition> reef) {
//         return pathfindTo(
//             drive,
//             () -> kReef.TARGETS.getOrDefault(
//                 reef.get(), 
//                 new Pose2d()
//             ),
//             true,
//             true
//         );
//     }

//     /**
//      * Aligns to the nearest left or right branch
//      * @param drive drive subsystem
//      * @param direction the left or right branch to score on
//      * @return A command that aligns to the given branch
//      */
//     public static Command alignToBranch(Drive drive, Supplier<kDirection> direction) {
//         return DriveCommands.alignToPoint(
//             drive, 
//             () -> AlignHelper.getClosestBranch(drive.getBlueSidePose(), kClosestType.DISTANCE, direction.get())
//         ).beforeStarting(() -> AlignHelper.reset(new ChassisSpeeds()));
//     }

//     /**
//      * Aligns to the nearest algae position
//      * @param drive drive subsystem
//      * @return A command that aligns to algae removale position
//      */
//     public static Command alignToAlgae(Drive drive) {
//         return DriveCommands.alignToPoint(
//             drive,
//             () -> AlignHelper.getClosestReef(drive.getBlueSidePose()).transformBy(new Transform2d(0.385, 0.0, new Rotation2d()))
//         ).beforeStarting(() -> AlignHelper.reset(new ChassisSpeeds()));
//     }

//     /**
//      * Backs off 0.385 metters from the facee of the reef
//      * @param drived rive subsystem
//      * @param rotationOffset rotation offset from the algae, this should usually be 0
//      * @return A backoff command
//      */
//     public static Command backOffFromAlgae(Drive drive, Rotation2d rotationOffset) {
//         return DriveCommands.alignToPoint(
//             drive, 
//             () -> AlignHelper.getClosestReef(drive.getBlueSidePose()).transformBy(new Transform2d(0, 0, rotationOffset))
//         ).beforeStarting(() -> AlignHelper.reset(new ChassisSpeeds()));
//     }

//     /**
//      * Fully automatic algae removal. This will account for ALL robot states: if the robot is too close, if its perfect, it the elevator is too high. It will also account for algae height
//      * @param sys_drive drive subsystem
//      * @param sys_endEffector end effector subsystem
//      * @param sys_elevator elevator subsystem
//      * @param sys_armPivot pivot subsystem
//      * @return A command that automatically removes algae from the nearest reef face
//      */
//     public static Command automaticAlgae(Drive sys_drive, EndEffector sys_endEffector, Elevator sys_elevator, ArmPivot sys_armPivot) {
//         Command idealAlgae = 
//             Commands.sequence(
//                 AutoCommands.alignToAlgae(sys_drive),
//                 Commands.waitUntil(() -> sys_endEffector.getCurrent() >= kEndEffector.ALGAE_CURRENT),
//                 AutoCommands.backOffFromAlgae(sys_drive, new Rotation2d()).withTimeout(0.5)
//             ).alongWith(
//                 new SelectorCommand(
//                     new RemoveAlgae(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL2_ALGAE), 
//                     new RemoveAlgae(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL3_ALGAE), 
//                     () -> AlignHelper.getAlgaeHeight(sys_drive.getBlueSidePose()) == ScoringLevel.LEVEL2_ALGAE
//                 )
//             );

//         // Used to be used but then we added L1 scoring which removed this option
//         // Command L4Algae = 
//         //     Commands.sequence(
//         //         AutoCommands.alignToAlgae(sys_drive),
//         //         new SelectorCommand(
//         //             new RemoveAlgae(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL2_ALGAE), 
//         //             new RemoveAlgae(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL3_ALGAE), 
//         //             () -> AlignHelper.getAlgaeHeight(sys_drive.getBlueSidePose()) == ScoringLevel.LEVEL2_ALGAE
//         //         ).raceWith(Commands.waitUntil(() -> sys_endEffector.getCurrent() >= kEndEffector.ALGAE_CURRENT)),
//         //         AutoCommands.backOffFromAlgae(sys_drive, new Rotation2d()).withTimeout(0.5)
//         //     );
        
//         Command tooCloseAlgae = 
//             AutoCommands.backOffFromAlgae(sys_drive, new Rotation2d()).andThen(
//                 Commands.sequence(
//                     AutoCommands.alignToAlgae(sys_drive),
//                     Commands.waitUntil(() -> sys_endEffector.getCurrent() >= kEndEffector.ALGAE_CURRENT),
//                     AutoCommands.backOffFromAlgae(sys_drive, new Rotation2d()).withTimeout(0.5)
//                 ).alongWith(
//                     new SelectorCommand(
//                         new RemoveAlgae(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL2_ALGAE), 
//                         new RemoveAlgae(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL3_ALGAE), 
//                         () -> AlignHelper.getAlgaeHeight(sys_drive.getBlueSidePose()) == ScoringLevel.LEVEL2_ALGAE
//                     )
//                 )
//             );

//         // return Commands.either(
//         //     idealAlgae, 
//         //     Commands.either(
//         //         tooCloseAlgae, 
//         //         L4Algae, 
//         //         () -> sys_elevator.getPosition().lte(ScoringLevel.LEVEL3.elevatorSetpoint.plus(Centimeters.of(5.0)))
//         //     ), 
//         //     Drive::isSafe
//         // );

//         return Commands.either(
//             idealAlgae, 
//             tooCloseAlgae,
//             Drive::isSafe
//         );
//     }

//     /**
//      * Full auto command
//      * @param drive drive subsystem
//      * @param sys_elevator elevator subsystem
//      * @param sys_pivot pivot subsystem
//      * @param sys_endeffector end effector subsystem
//      * @param scoringCommand the score command
//      * @param isAlgae if removing algae
//      * @param waitBeforeScoring if the robot should wait before scoring
//      * @return Full auto command
//      */
//     public static Command telopAutoCommand(Drive drive, Elevator sys_elevator, ArmPivot sys_pivot, EndEffector sys_endeffector, Command scoringCommand, BooleanSupplier isAlgae, BooleanSupplier waitBeforeScoring) {
//         return Commands.sequence(
//             Commands.parallel(
//                 new IdleCommand(sys_elevator, sys_pivot, sys_endeffector),
//                 Commands.sequence(
//                     pathFindToNearestStation(drive),
//                     Commands.runOnce(() -> drive.driveForward(-0.75), drive),
//                     Commands.idle(drive)
//                 )
//             ).unless(sys_endeffector::coralDetected).until(sys_endeffector::coralDetected),
//             Commands.waitSeconds(1.0),
//             pathFindToReef(drive, () -> target).onlyWhile(Drive::isSafe).onlyWhile(sys_endeffector::coralDetected),
//             Commands.waitSeconds(1.0),
//             alignToBranch(drive, () -> (scoreRight.getBoolean(false) ? kDirection.RIGHT : kDirection.LEFT))
//                 .alongWith(scoringCommand),
//             automaticAlgae(drive, sys_endeffector, sys_elevator, sys_pivot).onlyIf(isAlgae)
//         ).repeatedly();
//     }
// }
