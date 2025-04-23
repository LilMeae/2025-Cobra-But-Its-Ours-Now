package frc.robot.subsystems.collector;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;
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
    private final LoggedNetworkNumber voltageOffset;

    private int timer = 0;
    // Create an end effector motor not connected alert
    private Alert alert = new Alert("End Effector Motor Not Connected", AlertType.kError);

    public EndEffector(EndEffectorIO io) {
        this.io = io;
        // Logging
        tofRange = new LoggedNetworkNumber("ToF Range", kEndEffector.TIMEOFFLIGHT_DISTANCE_VALIDATION.in(Millimeters));
        voltageOffset = new LoggedNetworkNumber("Voltage offset", 0.0);

        // Debug commands
        DebugCommand.register("EndEffector Run Forward", setVoltage(3));
        DebugCommand.register("EndEffector Run Backwards", setVoltage(-3));
        DebugCommand.register("EndEffector Stop", setVoltage(0));
    }

    /**
     * Set motor to given voltage. If current spikes for more than 5 seconds set voltage to 0 and 
     * retry until coral detected unless coral is detected earlier, then it sets voltage to 0
     */
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
            ).until(this::coralDetected)
            .andThen(
                setVoltage(2.0),
                Commands.waitSeconds(0.175)
            ).unless(this::coralDetected)
            .finallyDo(() -> io.setVoltage(0.0));
    }

    /**
     * Run runUntilCoralNotDetected command with a set voltage 
     */
    public Command runUntilCoralNotDetected(double voltage) {
        return runUntilCoralNotDetected(() -> voltage);
    }
    /**
     * Sets motor to given voltage minus the voltage offset 
     * then wait for a current spike (Hit a branch) and retract the coral 
     * if its still in the end effector
     */
    public Command runUntilCoralNotDetected(DoubleSupplier voltage) {
        if (Constants.currentMode == Mode.SIM)
            return Commands.sequence(
                Commands.runOnce(
                    () -> io.setVoltage(voltage.getAsDouble()), 
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
                setVoltage(() -> voltage.getAsDouble() - voltageOffset.get()),
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

    /**
     * Set given voltage to io 
     */
    public Command setVoltage(DoubleSupplier voltageSupplier) {
        return Commands.runOnce(
            () -> io.setVoltage(voltageSupplier.getAsDouble()),
            this
        );
    }

    /**
     * Set given voltage to io with an interupt
     * @param voltage
     */
    public Command setVoltage(double voltage){
        return setVoltage(voltage, true);
    }
    /**
     * Set voltage with/without an interupt
     */
    public Command setVoltage(double voltage, boolean interupt) {
        if (interupt)
            return Commands.runOnce(
                () -> io.setVoltage(voltage), 
                this
            );

        return Commands.runOnce(() -> io.setVoltage(voltage));
    }
    /**
     * 
     * @return
     *      True or false if object detected within range of the ToF Sensor
     */
    public boolean coralDetected(){
        return io.getTofRange().lte(Millimeters.of(tofRange.get()));
    }
    /**
     * 
     * @return
     *       IO motor current
     */
    public double getCurrent() {
        return io.getMotorCurrent();
    }
    /**
     * Set IO motor to coast mode
     */
    public void coast() {
        io.coastMode();
    }
    /**
     * Set IO motor to brake mode
     */
    public void brake() {
        io.brakeMode();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Put coral status and ToF Distance to elastic
        SmartDashboard.putBoolean("Coral Status", coralDetected());
        SmartDashboard.putNumber("ToF Distance", io.getTofRange().in(Millimeters));

        // Log Proccessing
        io.updateInputs(inputs);
        Logger.processInputs("EndEffector", inputs);
        Logger.recordOutput("EndEffector/Coral Detected", coralDetected());

        // Alert if motor is not connected
        alert.set(!inputs.endEffectorConnection);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}