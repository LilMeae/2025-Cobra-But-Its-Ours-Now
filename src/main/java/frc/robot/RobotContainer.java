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
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.Constants.kAuto;
import frc.robot.Constants.kDrive;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
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
import frc.robot.util.DebugCommand;

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

    public static SwerveDriveSimulation simConfig;

    // 
    // Controller
    private final CommandXboxController primaryController   = new CommandXboxController(0);
    private final CommandXboxController secondaryController = new CommandXboxController(1);

    /** If the robot is/should be running fully autonomously */
    public static boolean isTelopAuto = false;


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
            }
        }


        leftTriggerAlert.set(true);
        rightTriggerAlert.set(true);

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

    /**



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

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
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
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
}
