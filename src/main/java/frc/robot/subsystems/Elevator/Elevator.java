// 5409: The Chargers
// http://github.com/FRC5409

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kElevator;

public class Elevator extends SubsystemBase{

    // IO
    private final ElevatorIO io;
    private final ElevatorInputsAutoLogged inputs;

    // Initiazlie Pose3d object
    private static Pose3d elevatorPose;
    private static Pose3d elevatorPoseStage2;

    //Setup alerts for elevator motors connection
    private final Alert leftElevatorAlert  = new Alert("The Left Elevator Motor is Disconnected " + kElevator.MAIN_MOTOR_ID, AlertType.kError);
    private final Alert rightElevatorAlert = new Alert("The Right Elevator Motor is Disconnected " + kElevator.FOLLOWER_MOTOR_ID, AlertType.kError);
    public Elevator(ElevatorIO io) {
        this.io = io;
        inputs = new ElevatorInputsAutoLogged();

        elevatorPose = new Pose3d();
        elevatorPoseStage2 = new Pose3d();
    }

    public Command startManualMove(double voltage) {
        return Commands.runOnce(() -> io.setMotorVoltage(voltage), this);
    }

    public Command zeroEncoder() {
        return Commands.runOnce(() -> io.zeroEncoder(), this);
    }

    public Command elevatorGo(Distance setpoint) {
        return Commands.sequence(
            Commands.runOnce(() -> io.setSetpoint(setpoint), this),
            Commands.waitUntil(() -> setpoint.isNear(getPosition(), Meters.of(0.025)))
        );
    }

    public Command stopAll() {
        return Commands.runOnce(() -> io.stopMotor(), this);
    }
    
    public Distance getPosition() {
        return io.getPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        leftElevatorAlert.set(!inputs.mainMotorConnection);
        rightElevatorAlert.set(!inputs.followerMotorConnection);
        elevatorPose = new Pose3d(0,0,inputs.mainMotorPosition, new Rotation3d());
        elevatorPoseStage2 = new Pose3d(0,0,2*inputs.mainMotorPosition, new Rotation3d());
        Logger.recordOutput("Components/Elevator", elevatorPose);
        Logger.recordOutput("Components/Elevator Stage 2", elevatorPoseStage2);
    }

    public static Pose3d getElevatorStage2Pose3dPose() {
        return elevatorPoseStage2;
    }
}
