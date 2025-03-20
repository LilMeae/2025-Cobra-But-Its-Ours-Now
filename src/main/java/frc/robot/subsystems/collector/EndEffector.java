package frc.robot.subsystems.collector;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.kEndEffector;
import frc.robot.util.DebugCommand;
import frc.robot.util.WaitThen;

public class EndEffector extends SubsystemBase {

    public final EndEffectorIO io;
    private final EndEffectorInputsAutoLogged inputs = new EndEffectorInputsAutoLogged();

    private final LoggedNetworkNumber tofRange;

    private int timer = 0;

    private Alert alert = new Alert("End Effector Motor Not Connected", AlertType.kError);

    public EndEffector(EndEffectorIO io) {
        this.io = io;

        tofRange = new LoggedNetworkNumber("ToF Range", kEndEffector.TIMEOFFLIGHT_DISTANCE_VALIDATION.in(Millimeters));

        DebugCommand.register("EndEffector Run Forward", setVoltage(3));
        DebugCommand.register("EndEffector Run Backwards", setVoltage(-3));
        DebugCommand.register("EndEffector Stop", setVoltage(0));
    }

    // Run until coral is detected then wait 50ms then stop
    public Command runUntilCoralDetected(double voltage) {
        if (Constants.currentMode == Mode.SIM)
            return Commands.sequence(
                Commands.runOnce(
                    () -> io.setVoltage(voltage), 
                    this
                ),
                Commands.waitSeconds(0.8),
                Commands.runOnce(
                    () -> io.setVoltage(0), 
                    this
                )
            );
        else
            return Commands.repeatingSequence(
                setVoltage(voltage).alongWith(Commands.runOnce(() -> timer = 0)),
                new WaitThen(
                    0.2,
                    Commands.waitUntil(() -> {
                        if (io.getMotorCurrent() >= kEndEffector.CURRENT_LIMIT - 3)
                            return ++timer > 5;
                        else
                            timer = 0;

                        return false;
                    })
                ),
                setVoltage(0.0),
                Commands.waitSeconds(0.1)
            ).raceWith(
                Commands.sequence(
                    Commands.waitUntil(this::coralDetected),
                    Commands.waitSeconds(0.075)
                )
            ).unless(this::coralDetected)
            .finallyDo(() -> io.setVoltage(0.0));

    }
    // Run until coral is no longer detected, if spike in current, wait then rerun
    public Command runUntilCoralNotDetected(double voltage) {
        if (Constants.currentMode == Mode.SIM)
            return Commands.sequence(
                Commands.runOnce(
                    () -> io.setVoltage(voltage), 
                    this
                ),
                Commands.waitSeconds(0.2),
                Commands.runOnce(
                    () -> io.setVoltage(0), 
                    this
                )
            );
        else
            return Commands.repeatingSequence(
                setVoltage(voltage),
                new WaitThen(
                    0.2,
                    Commands.waitUntil(() -> io.getMotorCurrent() >= kEndEffector.CURRENT_LIMIT - 3)
                ),
                setVoltage(-kEndEffector.IDLE_VOLTAGE),
                Commands.waitSeconds(0.15),
                setVoltage(0.0),
                Commands.waitSeconds(0.25)
            ).onlyWhile(this::coralDetected)
            .finallyDo(() -> io.setVoltage(0.0));
    }

    public Command setVoltage(double voltage){
        return Commands.runOnce(
            () -> io.setVoltage(voltage), 
            this
        );
    }

    public boolean coralDetected(){
        return io.getTofRange().lte(Millimeters.of(tofRange.get()));
    }

    public double getCurrent() {
        return io.getMotorCurrent();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        SmartDashboard.putBoolean("Coral Status", coralDetected());
        SmartDashboard.putNumber("ToF Distance", io.getTofRange().in(Millimeters));

        // Logging
        io.updateInputs(inputs);
        Logger.processInputs("EndEffector", inputs);
        Logger.recordOutput("EndEffector/Coral Detected", coralDetected());

        // System.out.println(inputs.tofDistance); --- Use when testing

        // Alert if motor is not connected
        alert.set(!inputs.endEffectorConnection);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}