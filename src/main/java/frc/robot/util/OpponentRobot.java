package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * A class that creates a simple opponent robot to play defence
 * @author Alexander Szura team 5409
 */
public class OpponentRobot extends SubsystemBase {

    private static final LinearVelocity MAX_ROBOT_SPEED = MetersPerSecond.of(4.56);
    private static final AngularVelocity MAX_TURN_SPEED = RadiansPerSecond.of(Math.PI * 2.0);

    protected static DriveTrainSimulationConfig opponentConfig;
    protected static RobotConfig PPConfig;

    private final SelfControlledSwerveDriveSimulation driveSimulation;

    public OpponentRobot(Pose2d fieldPose) {
        if (opponentConfig == null) {
            opponentConfig = DriveTrainSimulationConfig.Default()
            .withGyro(() -> new GyroSimulation(0.0, 0.0)) // No drift because no vision
            .withRobotMass(Pounds.of(114.9))
            .withTrackLengthTrackWidth(Meters.of(0.578), Meters.of(0.578))
            .withBumperSize(Meters.of(0.881), Meters.of(0.881))
            .withSwerveModule(
                COTS.ofMark4i(
                    DCMotor.getKrakenX60Foc(1),
                    DCMotor.getKrakenX60Foc(1),
                    1.20,
                    2
                )
            );

        }

        final SwerveDriveSimulation sim = new SwerveDriveSimulation(
            opponentConfig, 
            fieldPose
        );

        driveSimulation = new SelfControlledSwerveDriveSimulation(sim);

        SimulatedArena.getInstance().addDriveTrainSimulation(sim);
    }

    /**
     * Normal joystick drive command to drive the robot manually
     * @param controller the controller to drive with
     * @return A command that drives the robot
     */
    public Command joystickDrive(CommandXboxController controller) {
        return Commands.run(() -> {
            final ChassisSpeeds joystickSpeeds = new ChassisSpeeds(
                MetersPerSecond.of(-controller.getLeftY() * MAX_ROBOT_SPEED.in(MetersPerSecond)), 
                MetersPerSecond.of(-controller.getLeftX() * MAX_ROBOT_SPEED.in(MetersPerSecond)), 
                RadiansPerSecond.of(-(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) * MAX_TURN_SPEED.in(RadiansPerSecond))
            );

            final ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    joystickSpeeds,
                    driveSimulation.getActualPoseInSimulationWorld().getRotation()
                    // .plus(Rotation2d.k180deg) // Uncomment if running mutliple camera angles
            );

            driveSimulation.runChassisSpeeds(fieldRelativeSpeeds, new Translation2d(), true, true);
        }, this);
    }

    /**
     * Follows a path from pathplanner for the robot to follow 
     * @param path The path planner path to follow
     * @return a command that follows the path
     * @implNote Not tested
     */
    public Command followDrive(PathPlannerPath path) {
        final PPHolonomicDriveController driveController =
            new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0));

        return new PathfindingCommand(
            path, 
            path.getGlobalConstraints(),
            () -> driveSimulation.getActualPoseInSimulationWorld(),
            () -> driveSimulation.getActualSpeedsRobotRelative(), 
            (speeds, ff) -> driveSimulation.runChassisSpeeds(speeds, new Translation2d(), false, false),
            driveController,
            PPConfig,
            () -> true,
            this
        );
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Simulation/OpponentRobot", driveSimulation.getActualPoseInSimulationWorld());
    }
}
