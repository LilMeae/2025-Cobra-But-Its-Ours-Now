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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.kAuto;
import frc.robot.Constants.kAutoAlign.kReef;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.AlignHelper;
import frc.robot.util.WaitThen;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive sys_drive;

  // Controller
  private final CommandXboxController primaryController   = new CommandXboxController(0);
  private final CommandXboxController secondaryController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Alerts
  private final Alert primaryDisconnectedAlert   = new Alert("Primary Controller Disconnected!"  , AlertType.kError);
  private final Alert secondaryDisconnectedAlert = new Alert("Secondary Controller Disconnected!", AlertType.kError);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    switch (Constants.currentMode) {
      case REAL -> {
        // Real robot, instantiate hardware IO implementations
        sys_drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
      }
      case SIM -> {
        // Sim robot, instantiate physics sim IO implementations
        sys_drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
      }
      default -> {
        // Replayed robot, disable IO implementations
        sys_drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
      }
    }

    registerCommands();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    if (kAuto.RESET_ODOM_ON_CHANGE)
      autoChooser.getSendableChooser().onChange((path) -> sys_drive.setPose(getStartingPose()));
      
    SmartDashboard.putData("Reset", 
      Commands.runOnce(
        () -> sys_drive.setPose(getStartingPose())
      ).ignoringDisable(true)
    );

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(sys_drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(sys_drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        sys_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        sys_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", sys_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", sys_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    // Controller Alerts
    new Trigger(() -> !primaryController.isConnected())
        .onTrue(Commands.runOnce(() -> {
          secondaryController.setRumble(RumbleType.kBothRumble, 0.50);
          primaryDisconnectedAlert.set(true);
        }).ignoringDisable(true).andThen(
          new WaitThen(0.5, Commands.runOnce(() -> secondaryController.setRumble(RumbleType.kBothRumble, 0.0))).ignoringDisable(true)
        )).onFalse(
          Commands.runOnce(() -> primaryDisconnectedAlert.set(false)).ignoringDisable(true)
        );

    new Trigger(() -> !secondaryController.isConnected())
        .onTrue(Commands.runOnce(() -> {
          primaryController.setRumble(RumbleType.kBothRumble, 0.50);
          secondaryDisconnectedAlert.set(true);
        }).ignoringDisable(true).andThen(
          new WaitThen(0.5, Commands.runOnce(() -> primaryController.setRumble(RumbleType.kBothRumble, 0.0))).ignoringDisable(true)
        )).onFalse(
          Commands.runOnce(() -> secondaryDisconnectedAlert.set(false)).ignoringDisable(true)
        );

    // TODO: fix on real robot
    new Trigger(DriverStation::isDSAttached)
        .onTrue(Commands.runOnce(() -> {
            primaryDisconnectedAlert  .set(!primaryController  .isConnected());
            secondaryDisconnectedAlert.set(!secondaryController.isConnected());
          }).ignoringDisable(true)
        );
  }

  @AutoLogOutput(key = "Odometry/StartingPose")
  public Pose2d getStartingPose() {
    String path = autoChooser.getSendableChooser().getSelected();

    if (path.equals("None")) return new Pose2d();
        
    try {
      Pose2d pose = PathPlannerAuto.getPathGroupFromAutoFile(path).get(0).getStartingHolonomicPose().get();

      return AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(pose) : pose;
    } catch (IOException | ParseException e) {
      return new Pose2d();
    }
  }

  private void registerCommands() {
    NamedCommands.registerCommand("ALIGN_LEFT" , DriveCommands.alignToPoint(sys_drive, () -> AlignHelper.getClosestReef(sys_drive.getPose()).transformBy(kReef.LEFT_OFFSET_TO_BRANCH )));
    NamedCommands.registerCommand("ALIGN_RIGHT", DriveCommands.alignToPoint(sys_drive, () -> AlignHelper.getClosestReef(sys_drive.getPose()).transformBy(kReef.RIGHT_OFFSET_TO_BRANCH)));

    // TODO: Finish Commands
    NamedCommands.registerCommand("PREPARE_STATION", Commands.none());
    NamedCommands.registerCommand("STATION_PICKUP", Commands.waitSeconds(0.35));
    NamedCommands.registerCommand("SCORE_CORAL", Commands.waitSeconds(0.45));
  }

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

    primaryController.x()
      .onTrue(
        DriveCommands.setSpeedHigh(sys_drive)
      );

    primaryController.y()
      .onTrue(
        DriveCommands.setSpeedLow(sys_drive) 
      );

    // primaryController.x().onTrue(DriveCommands.increaseSpeed(sys_drive));

    // primaryController.y().onTrue(DriveCommands.decreaseSpeed(sys_drive));
   
    // Reset gyro to 0° when Start button is pressed
    primaryController.start()
        .onTrue(
            Commands.runOnce(
                    () ->
                    sys_drive.setPose(
                            new Pose2d(sys_drive.getPose().getTranslation(), new Rotation2d())),
                    sys_drive
                ).ignoringDisable(true));


    // primaryController.leftBumper()
    //     .whileTrue(
    //         DriveCommands.alignToPoint(drive, () -> AlignHelper.getClosestReef(drive.getPose()))
    //     );

    primaryController.leftBumper()
        .whileTrue(
            DriveCommands.alignToPoint(sys_drive, () -> AlignHelper.getClosestReef(sys_drive.getPose()).transformBy(kReef.LEFT_OFFSET_TO_BRANCH))
        );

    primaryController.rightBumper()
        .whileTrue(
            DriveCommands.alignToPoint(sys_drive, () -> AlignHelper.getClosestReef(sys_drive.getPose()).transformBy(kReef.RIGHT_OFFSET_TO_BRANCH))
        );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Drive getDrive() {
    return sys_drive;
  }
}
