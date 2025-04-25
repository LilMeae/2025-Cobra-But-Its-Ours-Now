// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ScoringLevel;
import frc.robot.Constants.kArmPivot;
import frc.robot.Constants.kAuto;
import frc.robot.Constants.kAutoAlign;
import frc.robot.Constants.kDrive;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.AutoCommands.kReefPosition;
import frc.robot.commands.scoring.IdleCommand;
import frc.robot.commands.scoring.RemoveAlgae;
import frc.robot.commands.scoring.ScoreCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.ArmPivot;
import frc.robot.subsystems.arm.ArmPivotIO;
import frc.robot.subsystems.arm.ArmPivotIOSim;
import frc.robot.subsystems.arm.ArmPivotIOTalonFX;
import frc.robot.Constants.kElevator;
import frc.robot.Constants.kEndEffector;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.collector.EndEffector;
import frc.robot.subsystems.collector.EndEffectorIO;
import frc.robot.subsystems.collector.EndEffectorIOSim;
import frc.robot.subsystems.collector.EndEffectorIOTalonFx;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.AlignHelper;
import frc.robot.util.AutoTimer;
import frc.robot.util.CaseCommand;
import frc.robot.util.DebugCommand;
import frc.robot.util.OpponentRobot;
import frc.robot.util.AlignHelper.kClosestType;
import frc.robot.util.AlignHelper.kDirection;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    // Subsystems
    protected final Drive       sys_drive;
    protected final Vision      sys_vision;
    protected final Elevator    sys_elevator;
    protected final EndEffector sys_endEffector;
    protected final ArmPivot    sys_armPivot;

    public static SwerveDriveSimulation simConfig;

    // Commands
    protected final Command telopAutoCommand;
  
    private ScoringLevel selectedScoringLevel = ScoringLevel.LEVEL4;

    // Controller
    private final CommandXboxController primaryController   = new CommandXboxController(0);
    private final CommandXboxController secondaryController = new CommandXboxController(1);

    /** If the robot is/should be running fully autonomously */
    public static boolean isTelopAuto = false;

    private boolean removeAlgae = false;

    @SuppressWarnings("unused")
    private static final boolean runOpponent = false
        && Constants.currentMode == Mode.SIM;

    // Dashboard inputs
    protected final LoggedDashboardChooser<Command> autoChooser;
    protected final BooleanSupplier runTelop;

    // Alerts
    private final Alert primaryDisconnectedAlert = new Alert(
        "Primary Controller Disconnected!",
        AlertType.kError
    );
    private final Alert secondaryDisconnectedAlert = new Alert(
        "Secondary Controller Disconnected!",
        AlertType.kError
    );

    private final Alert leftTriggerAlert = new Alert(
        "Left Trigger never tested!",
        AlertType.kWarning
    );

    private final Alert rightTriggerAlert = new Alert(
        "Right Trigger never tested!",
        AlertType.kWarning
    );

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        DriverStation.silenceJoystickConnectionWarning(true);

        switch (Constants.currentMode) {
            case REAL -> {
                // Real robot, instantiate hardware IO implementations
                sys_vision = new Vision(new VisionIOLimelight());
                sys_drive =
                    new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight),
                        sys_vision);
              
                sys_armPivot = new ArmPivot(new ArmPivotIOTalonFX(kArmPivot.FALCON_ID, kArmPivot.CANCODER_ID));
                sys_elevator = new Elevator(new ElevatorIOTalonFX(kElevator.MAIN_MOTOR_ID, kElevator.FOLLOWER_MOTOR_ID));
                sys_endEffector = new EndEffector(new EndEffectorIOTalonFx(kEndEffector.ENDEFFECTOR_MOTOR_ID));
            }
            case SIM -> {
                // Sim robot, instantiate physics sim IO implementations
                final DriveTrainSimulationConfig driveConfig = DriveTrainSimulationConfig.Default()
                    .withGyro(COTS.ofPigeon2())
                    .withRobotMass(kDrive.ROBOT_FULL_MASS)
                    .withTrackLengthTrackWidth(Meters.of(0.578), Meters.of(0.578))
                    .withBumperSize(Meters.of(0.881), Meters.of(0.881))
                    .withSwerveModule(
                        COTS.ofMark4i(
                            DCMotor.getKrakenX60Foc(1),
                            DCMotor.getKrakenX60Foc(1),
                            kDrive.WHEEL_COF,
                            2
                        )
                    );
              
                sys_armPivot = new ArmPivot(new ArmPivotIOSim());
                sys_elevator = new Elevator(new ElevatorIOSim());
                sys_endEffector = new EndEffector(new EndEffectorIOSim());
                
                simConfig = new SwerveDriveSimulation(
                    driveConfig,
                    new Pose2d(3, 3, new Rotation2d())
                );

                SimulatedArena.getInstance().addDriveTrainSimulation(simConfig);
                SimulatedArena.getInstance().resetFieldForAuto();

                sys_vision = new Vision(new VisionIOSim(simConfig));
                sys_drive =
                    new Drive(
                        new GyroIOSim(simConfig.getGyroSimulation()),
                        new ModuleIOSim(simConfig.getModules()[0]),
                        new ModuleIOSim(simConfig.getModules()[1]),
                        new ModuleIOSim(simConfig.getModules()[2]),
                        new ModuleIOSim(simConfig.getModules()[3]),
                        sys_vision);

                if (runOpponent) {
                    final OpponentRobot sys_opponent = 
                        new OpponentRobot(new Pose2d(3, 3, Rotation2d.fromDegrees(0.0)));
                    sys_opponent.setDefaultCommand(sys_opponent.joystickDrive(secondaryController));
                }
            }
            default -> {
                // Replayed robot, disable IO implementations
                sys_vision = new Vision(new VisionIO() {});
                sys_drive =
                    new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        sys_vision);
                sys_armPivot = new ArmPivot(new ArmPivotIO() {});
                sys_elevator = new Elevator(new ElevatorIO(){});
                sys_endEffector = new EndEffector(new EndEffectorIO() {});
            }
        }

        AutoCommands.setupNodeChooser();
        registerCommands();

        leftTriggerAlert.set(true);
        rightTriggerAlert.set(true);

        // Commands
        telopAutoCommand = AutoCommands.telopAutoCommand(
            sys_drive,
            sys_elevator,
            sys_armPivot,
            sys_endEffector,
            getLevelSelectorCommand(true),
            () -> removeAlgae,
            () -> secondaryController.getHID().getPOV() != -1
        ).onlyWhile(() -> isTelopAuto);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        autoChooser.addDefaultOption("None", Commands.none());

        for (String auto : AutoBuilder.getAllAutoNames()) {
            if (auto.endsWith("[M]")) {
                String autoName = auto.replace("[M]", "");
                autoChooser.addOption("{L} - " + autoName, new PathPlannerAuto(auto, false));
                autoChooser.addOption("{R} - " + autoName, new PathPlannerAuto(auto, true ));
            } else {
                autoChooser.addOption(auto, new PathPlannerAuto(auto));
            }
        }

        if (Constants.TUNNING) {
            // Set up SysId routines
            autoChooser.addOption(
                    "Drive Wheel Radius Characterization",
                    DriveCommands.wheelRadiusCharacterization(sys_drive));
            autoChooser.addOption(
                    "Drive Simple FF Characterization",
                    DriveCommands.feedforwardCharacterization(sys_drive));
            autoChooser.addOption(
                    "Drive SysId (Quasistatic Forward)",
                    sys_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption(
                    "Drive SysId (Quasistatic Reverse)",
                    sys_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            autoChooser.addOption(
                    "Drive SysId (Dynamic Forward)",
                    sys_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption(
                    "Drive SysId (Dynamic Reverse)",
                    sys_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        }

        runTelop = DebugCommand.putNumber("Run Telop Auto", false)::get;

        if (kAuto.RESET_ODOM_ON_CHANGE)
            autoChooser
                .getSendableChooser()
                .onChange(path -> resetPose());

        DebugCommand.register("Reset", Commands.runOnce(this::resetPose));

        // Configure the button bindings
        configureButtonBindings();

        // Controller Alerts
        new Trigger(() -> !primaryController.isConnected())
                .onChange(
                    Commands.runOnce(() -> primaryDisconnectedAlert.set(!primaryController.isConnected()))
                        .ignoringDisable(true)
                );

        new Trigger(() -> !secondaryController.isConnected())
                .onChange(
                    Commands.runOnce(() -> secondaryDisconnectedAlert.set(!secondaryController.isConnected()))
                        .ignoringDisable(true)
                );

        // When DS connects check joystick connections
        new Trigger(DriverStation::isDSAttached).onTrue(
            Commands.waitSeconds(1.0).andThen(
                Commands.runOnce(() -> {
                    primaryDisconnectedAlert.set(!primaryController.isConnected());
                    secondaryDisconnectedAlert.set(!secondaryController.isConnected());

                    resetPose();
                }).ignoringDisable(true)
            )
        );
    }

    /**
     * Updates telop auto scoring position. If the controller is tilted far right, the robot will go to the close left face. (Yes I like inverted controls)
     */
    public void updateScoringPosition() {
        if (!secondaryController.isConnected()) return;

        if (Math.hypot(secondaryController.getRightX(), secondaryController.getRightY()) < 0.6) return;

        Angle targetAngle = Radians.of(Math.atan2(secondaryController.getRightY(), secondaryController.getRightX()));

        double degrees = targetAngle.in(Degrees);
        if (degrees > -120 && degrees <= -60)
            AutoCommands.target = kReefPosition.FAR;
        else if (degrees > -60 && degrees <= 0)
            AutoCommands.target = kReefPosition.FAR_RIGHT;
        else if (degrees > 0 && degrees <= 60)
            AutoCommands.target = kReefPosition.CLOSE_RIGHT;
        else if (degrees > 60 && degrees <= 120)
            AutoCommands.target = kReefPosition.CLOSE;
        else if (degrees > 120 && degrees <= 180)
            AutoCommands.target = kReefPosition.CLOSE_LEFT;
        else
            AutoCommands.target = kReefPosition.FAR_LEFT;

        Logger.recordOutput("Scoring Position", AutoCommands.target);
    }

    /**
     * Updates sim positions of algae, coral and robot poses
     */
    public void updateSim() {
        SimulatedArena.getInstance().simulationPeriodic();

        Logger.recordOutput("Simulation/RobotPose", simConfig.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "Simulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        Logger.recordOutput(
                "Simulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    }

    /**
     * Resets the pose of our robot to the starting pose of auto
     */
    private void resetPose() {
        Pose2d startingPose = getStartingPose();
        if (Constants.currentMode == Mode.SIM)
            simConfig.setSimulationWorldPose(startingPose);

        sys_drive.setPose(startingPose);
    }

    /**
     * Creates a command that will run the a score command that is currently selected. If it gets changed it will update to that new level
     * @param ends if it should end when it has reached it position or should it keep updating
     * @return The level selector command
     */
    private Command getLevelSelectorCommand(boolean ends) {
        ScoringLevel[] levels = ScoringLevel.values();
        BooleanSupplier[] conditionals = new BooleanSupplier[levels.length];
        Command[] commands = new Command[levels.length];

        for (int i = 0; i < conditionals.length; i++) {
            final int index = i;
            conditionals[i] = () -> levels[index] == selectedScoringLevel;
            commands[i] = new ScoreCommand(sys_elevator, sys_armPivot, sys_endEffector, levels[index], ends ? DriveCommands::isAligned : () -> false);
        }

        return CaseCommand.buildSelector(
            conditionals, 
            commands, 
            Commands.print("Error: Couldn't find selected level to go to")
        );
    }

    /**
     * @deprecated never used
     */
    public Command getLevelSelectorDistCommand(boolean ends) {
        ScoringLevel[] levels = ScoringLevel.values();
        BooleanSupplier[] conditionals = new BooleanSupplier[4];
        Command[] commands = new Command[4];

        for (int i = 0; i < conditionals.length; i++) {
            final int index = i;
            conditionals[i] = () -> levels[index] == selectedScoringLevel;
            commands[i] = new ScoreCommand(sys_elevator, sys_armPivot, sys_endEffector, levels[index + 4], ends ? DriveCommands::isAligned : () -> false);
        }

        return CaseCommand.buildSelector(
            conditionals, 
            commands, 
            Commands.print("Error: Couldn't find selected level to go to")
        );
    }

    /**
     * Full automatic scoring command. Extends elevator while driving to closest branch and automatically scores when confident
     * @param side Which side to score on [left/right]
     * @param level Which level to score on
     * @return A fully automatic scoring command
     */
    private Command fullAutoScore(kDirection side, ScoringLevel level) {
        return Commands.parallel(
            DriveCommands.alignToPoint(
                sys_drive, 

                () -> {
                    if ((level == null ? selectedScoringLevel : level) == ScoringLevel.LEVEL1)
                        return AlignHelper.getClosestL1(sys_drive.getBlueSidePose(), kClosestType.DISTANCE)
                        .transformBy(new Transform2d(Feet.of(0.8).in(Meters), 0, new Rotation2d()));
                    else
                        return AlignHelper.getClosestBranch(sys_drive.getBlueSidePose(), kClosestType.DISTANCE, side);
                },

                () -> (level == null ? selectedScoringLevel : level) == ScoringLevel.LEVEL4 
                    ? kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_SLOW
                    : kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_FAST,

                () -> (level == null ? selectedScoringLevel : level) == ScoringLevel.LEVEL4
                    ? kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_SLOW
                    : kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_FAST
            ).beforeStarting(() -> AlignHelper.reset(sys_drive.getFieldRelativeSpeeds())),
            level == null ? 
            getLevelSelectorCommand(true) : 
            new ScoreCommand(sys_elevator, sys_armPivot, sys_endEffector, level, DriveCommands::isAligned)
        );
    }

    /**
     * @return Pose2d of the starting pose of the robot
     */
    @AutoLogOutput(key = "Odometry/StartingPose")
    public Pose2d getStartingPose() {
        String autoName = autoChooser.getSendableChooser().getSelected();

        boolean mirror = false;
        if ((mirror = autoName.startsWith("{R}")) || autoName.startsWith("{L}")) {
            autoName = autoName.substring(6) + "[M]";
        }

        if (autoName.equals("None"))
            return new Pose2d();

        try {
            PathPlannerPath path = PathPlannerAuto.getPathGroupFromAutoFile(autoName)
                    .get(0);
    
            if (mirror)
                path = path.mirrorPath();
                    
            Pose2d pose = path.getStartingHolonomicPose().get();

            return AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(pose) : pose;
        } catch (IOException | ParseException | IndexOutOfBoundsException e) {
            return new Pose2d();
        }
    }

    /**
     * Registers DebugCommands and NamedCommands
     */
    private void registerCommands() {
        // DEBUG COMMANDS
        DebugCommand.register("L1", new ScoreCommand(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL1, () -> true));
        DebugCommand.register("L2", new ScoreCommand(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL2, () -> true));
        DebugCommand.register("L3", new ScoreCommand(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL3, () -> true));
        DebugCommand.register("L4", new ScoreCommand(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL4, () -> true));

        DebugCommand.register("Algae L2", new RemoveAlgae(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL2_ALGAE));
        DebugCommand.register("Algae L3", new RemoveAlgae(sys_elevator, sys_armPivot, sys_endEffector, ScoringLevel.LEVEL3_ALGAE));

        DebugCommand.register("Idle", new IdleCommand(sys_elevator, sys_armPivot, sys_endEffector));

        DebugCommand.register("Setup Auto [R]", 
            Commands.runOnce(sys_drive::coastMode, sys_drive).andThen(
                Commands.run(() -> sys_drive.pointWheelsToward(Rotation2d.fromDegrees(60)), sys_drive).withTimeout(3.0)
            )
        );

        DebugCommand.register("Setup Auto [L]", 
            Commands.runOnce(sys_drive::coastMode, sys_drive).andThen(
                Commands.run(() -> sys_drive.pointWheelsToward(Rotation2d.fromDegrees(-60)), sys_drive).withTimeout(3.0)
            )
        );

        DebugCommand.register("CORAL COAST", Commands.runOnce(sys_endEffector::coast));
        DebugCommand.register("CORAL BRAKE", Commands.runOnce(sys_endEffector::brake));

        // NAMED COMMANDS

        // Next time try to use for loop for this...
        NamedCommands.registerCommand(
                "SCORE_LEFT_L4",
                new ConditionalCommand(
                    fullAutoScore(kDirection.LEFT, ScoringLevel.LEVEL4),
                    fullAutoScore(kDirection.RIGHT, ScoringLevel.LEVEL4),
                    () -> !autoChooser.getSendableChooser().getSelected().startsWith("{R}")
                )
                .beforeStarting(() -> AlignHelper.reset(new ChassisSpeeds()))
                .withTimeout(2.0)
                .onlyIf(sys_endEffector::coralDetected)
            );

        NamedCommands.registerCommand(
                "SCORE_RIGHT_L4",
                new ConditionalCommand(
                    fullAutoScore(kDirection.RIGHT, ScoringLevel.LEVEL4),
                    fullAutoScore(kDirection.LEFT, ScoringLevel.LEVEL4),
                    () -> !autoChooser.getSendableChooser().getSelected().startsWith("{R}")
                )
                .beforeStarting(() -> AlignHelper.reset(new ChassisSpeeds()))
                .withTimeout(2.0)
                .onlyIf(sys_endEffector::coralDetected)
            );

        NamedCommands.registerCommand(
                "SCORE_LEFT_L3",
                new ConditionalCommand(
                    fullAutoScore(kDirection.LEFT, ScoringLevel.LEVEL3),
                    fullAutoScore(kDirection.RIGHT, ScoringLevel.LEVEL3),
                    () -> !autoChooser.getSendableChooser().getSelected().startsWith("{R}")
                )
                .beforeStarting(() -> AlignHelper.reset(new ChassisSpeeds()))
                .withTimeout(2.0)
                .onlyIf(sys_endEffector::coralDetected)
            );

        NamedCommands.registerCommand(
                "SCORE_RIGHT_L3",
                new ConditionalCommand(
                    fullAutoScore(kDirection.RIGHT, ScoringLevel.LEVEL3),
                    fullAutoScore(kDirection.LEFT, ScoringLevel.LEVEL3),
                    () -> !autoChooser.getSendableChooser().getSelected().startsWith("{R}")
                )
                .beforeStarting(() -> AlignHelper.reset(new ChassisSpeeds()))
                .withTimeout(2.0)
                .onlyIf(sys_endEffector::coralDetected)
            );

        NamedCommands.registerCommand(
                "SCORE_LEFT_L2",
                new ConditionalCommand(
                    fullAutoScore(kDirection.LEFT, ScoringLevel.LEVEL2),
                    fullAutoScore(kDirection.RIGHT, ScoringLevel.LEVEL2),
                    () -> !autoChooser.getSendableChooser().getSelected().startsWith("{R}")
                )
                .beforeStarting(() -> AlignHelper.reset(new ChassisSpeeds()))
                .withTimeout(2.0)
                .onlyIf(sys_endEffector::coralDetected)
            );

        NamedCommands.registerCommand(
                "SCORE_RIGHT_L2",
                new ConditionalCommand(
                    fullAutoScore(kDirection.RIGHT, ScoringLevel.LEVEL2),
                    fullAutoScore(kDirection.LEFT, ScoringLevel.LEVEL2),
                    () -> !autoChooser.getSendableChooser().getSelected().startsWith("{R}")
                )
                .beforeStarting(() -> AlignHelper.reset(new ChassisSpeeds()))
                .withTimeout(2.0)
                .onlyIf(sys_endEffector::coralDetected)
            );

        NamedCommands.registerCommand(
                "SCORE_LEFT_L1",
                new ConditionalCommand(
                    fullAutoScore(kDirection.LEFT, ScoringLevel.LEVEL1),
                    fullAutoScore(kDirection.RIGHT, ScoringLevel.LEVEL1),
                    () -> !autoChooser.getSendableChooser().getSelected().startsWith("{R}")
                )
                .beforeStarting(() -> AlignHelper.reset(new ChassisSpeeds()))
                .withTimeout(2.0)
                .onlyIf(sys_endEffector::coralDetected)
            );

        NamedCommands.registerCommand(
                "SCORE_RIGHT_L1",
                new ConditionalCommand(
                    fullAutoScore(kDirection.RIGHT, ScoringLevel.LEVEL1),
                    fullAutoScore(kDirection.LEFT, ScoringLevel.LEVEL1),
                    () -> !autoChooser.getSendableChooser().getSelected().startsWith("{R}")
                )
                .beforeStarting(() -> AlignHelper.reset(new ChassisSpeeds()))
                .withTimeout(2.0)
                .onlyIf(sys_endEffector::coralDetected)
            );

        NamedCommands.registerCommand("IDLE",
            new IdleCommand(sys_elevator, sys_armPivot, sys_endEffector)
                .finallyDo(() -> sys_endEffector.io.setVoltage(0.0))
        );

        NamedCommands.registerCommand("PREP_ELEVATOR", 
            sys_armPivot.moveArm(kArmPivot.MOVEMENT_SETPOINT).alongWith(
                Commands.sequence(
                    Commands.waitUntil(() -> sys_armPivot.getPosition().isNear(kArmPivot.MOVEMENT_SETPOINT, Degrees.of(2.0))),
                    sys_elevator.elevatorGo(kElevator.ELEVATOR_PREP_HEIGHT)
                )
            )
        );
      
        // NamedCommands.registerCommand("END_WHEN_COLLECTED", Commands.waitUntil(sys_endEffector::coralDetected).withTimeout(1.75));

        // Saves lots of time by detecting when the game piece has entered using current detection
        NamedCommands.registerCommand("END_WHEN_COLLECTED", Commands.waitUntil(() ->
            sys_endEffector.getCurrent() >= 30 || sys_endEffector.coralDetected()
        ).withTimeout(2.0));

        NamedCommands.registerCommand("DRIVE_FORWARD", Commands.runOnce(() -> sys_drive.driveForward(-0.75), sys_drive));
        NamedCommands.registerCommand("BUMP", Commands.runOnce(() -> sys_drive.driveForward(-2.0), sys_drive));

        NamedCommands.registerCommand("REMOVE_ALGAE", 
            AutoCommands.automaticAlgae(sys_drive, sys_endEffector, sys_elevator, sys_armPivot)
            .andThen(Commands.runOnce(sys_drive::stop, sys_drive))
        );

        NamedCommands.registerCommand("BACKOFF", AutoCommands.backOffFromAlgae(sys_drive, new Rotation2d()));

        NamedCommands.registerCommand("CORAL_COAST", Commands.runOnce(sys_endEffector::coast));

        NamedCommands.registerCommand("AUTO_END", AutoTimer.end(kAuto.PRINT_AUTO_TIME));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        primaryController.leftTrigger(0.75)
            .onTrue(
                Commands.runOnce(() -> leftTriggerAlert.set(false))
                    .ignoringDisable(true)
            );

        primaryController.rightTrigger(0.75)
            .onTrue(
                Commands.runOnce(() -> rightTriggerAlert.set(false))
                    .ignoringDisable(true)
            );

        // Default command, normal field-relative drive
        sys_drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                sys_drive,
                () -> -primaryController.getLeftY(),
                () -> -primaryController.getLeftX(),
                () -> -(primaryController.getRightTriggerAxis() - primaryController.getLeftTriggerAxis())
            )
        );

        primaryController.start()
            .onTrue(
                Commands.runOnce(sys_drive::coastMode)
                    .ignoringDisable(true)
            ).onFalse(
                Commands.runOnce(sys_drive::brakeMode)
                    .ignoringDisable(true)
            );

        // Button to start full auto cycles
        // primaryController
        //     .back()
        //     .or(() -> secondaryController.getHID().getBackButton())
        //         .whileFalse(
        //             Commands.runOnce(() -> isTelopAuto = !isTelopAuto)
        //                 .andThen(telopAutoCommand)
        //         );

        // Manual scoring if Vision doesn't work
        primaryController.a()
            .onTrue(
                getLevelSelectorCommand(false)
            )
            .onFalse(
                Commands.either(
                    sys_endEffector.runUntilCoralNotDetected(() -> selectedScoringLevel.voltage),
                    sys_endEffector.setVoltage(() -> selectedScoringLevel.voltage).withTimeout(1.0),
                    sys_endEffector::coralDetected
                )
                .andThen(
                    Commands.waitUntil(Drive::isSafe),
                    new IdleCommand(sys_elevator, sys_armPivot, sys_endEffector)
                )
            );

        // L1 pickup
        primaryController.y()
            .whileTrue(
                Commands.sequence(
                    sys_endEffector.setVoltage(2.5),
                    sys_armPivot.moveArm(kArmPivot.MOVEMENT_SETPOINT),
                    sys_elevator.elevatorGo(Meters.of(0.123)),
                    sys_armPivot.moveArm(kArmPivot.L1_PICKUP_ANGLE)
                )
            ).onFalse(
                Commands.sequence(
                    sys_armPivot.moveArm(Degrees.of(100.0)),
                    sys_elevator.elevatorGo(Meters.of(0.01))
                )
            );

        // Algae removal button
        primaryController.x()
            .whileTrue(
                new ConditionalCommand(
                    Commands.sequence(
                        Commands.runOnce(sys_drive::stop, sys_drive),
                        Commands.runOnce(() -> primaryController.setRumble(RumbleType.kBothRumble, 0.4))
                            .alongWith(Commands.waitSeconds(1.0)),
                        AutoCommands.automaticAlgae(sys_drive, sys_endEffector, sys_elevator, sys_armPivot)
                    ),
                    AutoCommands.automaticAlgae(sys_drive, sys_endEffector, sys_elevator, sys_armPivot),
                    sys_endEffector::coralDetected
                ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .finallyDo(() -> primaryController.setRumble(RumbleType.kBothRumble, 0.0))
            )
            .onFalse(new IdleCommand(sys_elevator, sys_armPivot, sys_endEffector));

        // Idle button
        primaryController.b()
            .onTrue(new IdleCommand(sys_elevator, sys_armPivot, sys_endEffector));

        // Align left branch
        primaryController
            .leftBumper()
            .and(() -> !isTelopAuto)
                .whileTrue(
                    Commands.sequence(
                        fullAutoScore(kDirection.LEFT, null),
                        new IdleCommand(sys_elevator, sys_armPivot, sys_endEffector)
                    ).deadlineFor(
                        Commands.repeatingSequence(
                            Commands.waitUntil(() -> !sys_vision.hasTarget()),
                            Commands.runOnce(() -> primaryController.setRumble(RumbleType.kBothRumble, 0.5)),
                            Commands.waitUntil(sys_vision::hasTarget),
                            Commands.runOnce(() -> primaryController.setRumble(RumbleType.kBothRumble, 0.0))
                        ).finallyDo(() -> primaryController.setRumble(RumbleType.kBothRumble, 0.0))
                    )
                ).onFalse(
                    Commands.sequence(
                        Commands.waitSeconds(0.5).onlyIf(() -> selectedScoringLevel == ScoringLevel.LEVEL1),
                        new IdleCommand(sys_elevator, sys_armPivot, sys_endEffector)
                    )
                );

        // Align right branch
        primaryController
            .rightBumper()
            .and(() -> !isTelopAuto)
                .whileTrue(
                    Commands.sequence(
                        fullAutoScore(kDirection.RIGHT, null),
                        new IdleCommand(sys_elevator, sys_armPivot, sys_endEffector)
                    ).deadlineFor(
                        Commands.repeatingSequence(
                            Commands.waitUntil(() -> !sys_vision.hasTarget()),
                            Commands.runOnce(() -> primaryController.setRumble(RumbleType.kBothRumble, 0.5)),
                            Commands.waitUntil(sys_vision::hasTarget),
                            Commands.runOnce(() -> primaryController.setRumble(RumbleType.kBothRumble, 0.0))
                        ).finallyDo(() -> primaryController.setRumble(RumbleType.kBothRumble, 0.0))
                    )
                ).onFalse(
                    Commands.sequence(
                        Commands.waitSeconds(0.5).onlyIf(() -> selectedScoringLevel == ScoringLevel.LEVEL1),
                        new IdleCommand(sys_elevator, sys_armPivot, sys_endEffector)
                    )
                );

        primaryController.leftBumper()
            .and(() -> isTelopAuto)
            .onTrue(Commands.runOnce(() -> AutoCommands.scoreRight.setBoolean(false)).ignoringDisable(true));

        primaryController.rightBumper()
            .and(() -> isTelopAuto)
            .onTrue(Commands.runOnce(() -> AutoCommands.scoreRight.setBoolean(true )).ignoringDisable(true));

        primaryController.povLeft()
            .and(() -> !isTelopAuto)
            .whileTrue(
                DriveCommands.alignToPoint(
                    sys_drive, 
                    () -> AlignHelper.getClosestStation(sys_drive.getBlueSidePose())
                ).beforeStarting(() -> AlignHelper.reset(sys_drive.getFieldRelativeSpeeds()))
            );

        primaryController.povUp()
            .onTrue(sys_endEffector.setVoltage(kEndEffector.IDLE_VOLTAGE, false))
            .onFalse(sys_endEffector.setVoltage(0.0, false));

        primaryController.povDown()
            .onTrue(sys_endEffector.setVoltage(-kEndEffector.IDLE_VOLTAGE, false))
            .onFalse(sys_endEffector.setVoltage(0.0, false));

        // SECONDARY CONTROLLER

        secondaryController.a()
            .onTrue(prepLevelCommand(ScoringLevel.LEVEL1));
        secondaryController.b()
            .onTrue(prepLevelCommand(ScoringLevel.LEVEL2));
        secondaryController.x()
            .onTrue(prepLevelCommand(ScoringLevel.LEVEL3));
        secondaryController.y()
            .onTrue(prepLevelCommand(ScoringLevel.LEVEL4));

        secondaryController.leftBumper()
            .onTrue(Commands.runOnce(() -> AutoCommands.scoreRight.setBoolean(false)).ignoringDisable(true));

        secondaryController.rightBumper()
            .onTrue(Commands.runOnce(() -> AutoCommands.scoreRight.setBoolean(true )).ignoringDisable(true));

        secondaryController.start()
            .onTrue(Commands.runOnce(() -> removeAlgae = true).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> removeAlgae = false).ignoringDisable(true));
    }

    public Command prepLevelCommand(ScoringLevel level) {
        return Commands.runOnce(() -> {
            Logger.recordOutput("Scoring Level", level);
            selectedScoringLevel = level;
        }).ignoringDisable(true);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutoTimer.start()
            .alongWith(autoChooser.get())
            .andThen(
                Commands.either(
                    AutoCommands.telopAutoCommand(sys_drive, sys_elevator, sys_armPivot, sys_endEffector, getLevelSelectorCommand(true), () -> removeAlgae, () -> false), 
                    Commands.none(),
                    runTelop // If should run telop auto
                )
            );
    }
}
