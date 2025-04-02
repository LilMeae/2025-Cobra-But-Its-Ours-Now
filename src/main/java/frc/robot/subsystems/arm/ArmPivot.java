package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.Elevator;

public class ArmPivot extends SubsystemBase {

    private ArmPivotIO io;
    private static ArmPivotInputsAutoLogged inputs;

    private final Alert motorConnectedAlert = new Alert("Arm Motor Not Connected"     , AlertType.kError);
    private final Alert cancoderConnected   = new Alert("CANCoder [Arm] Not Connected", AlertType.kError);
    private final Alert CANCoderAlertOrange = new Alert("CANCoder [Arm] Orange"       , AlertType.kWarning);
    private final Alert CANCoderAlertRed    = new Alert("CANCoder [Arm] Red"          , AlertType.kError);

  
    public ArmPivot(ArmPivotIO io) {
        this.io = io;
        inputs = new ArmPivotInputsAutoLogged();
    }

    public Angle getPosition() {
        return io.getPosition();
    }

    public Command moveArm(Angle positionRad) {
        return Commands.sequence(
            Commands.runOnce(() -> io.setSetpoint(positionRad), this),
            Commands.waitUntil(() -> getPosition().isNear(positionRad, Degrees.of(1.5)))
        );
    }

    public Command setVoltage(double volatge){
        return Commands.runOnce(() -> io.setVoltage(volatge), this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
        Logger.recordOutput("Components/End Effector", new Pose3d(new Translation3d(0.231, -0.023, 0.155 + Elevator.getElevatorStage2Pose3dPose().getZ()), new Rotation3d(0, -getPosition().in(Radians), 0)));

        motorConnectedAlert.set(!inputs.connected);
        cancoderConnected.set(!inputs.cancoderConnected || inputs.magnetHealth == MagnetHealthValue.Magnet_Invalid);
        CANCoderAlertOrange.set(inputs.magnetHealth == MagnetHealthValue.Magnet_Orange);
        CANCoderAlertRed.set(inputs.magnetHealth == MagnetHealthValue.Magnet_Red);
    }
}