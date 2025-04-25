package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.kElevator;

public class ElevatorIOSim implements ElevatorIO {

    private boolean running;
    private ElevatorSim elevatorSim;
    private PIDController PID;

    public ElevatorIOSim() {

        //Creating sim elevator object
        elevatorSim = new ElevatorSim(
            DCMotor.getFalcon500(2), 
            kElevator.kGearing, 
            kElevator.ELEVATOR_MASS.in(Kilograms), 
            kElevator.ELEVATOR_DRUMRADIUS.in(Meters), 
            kElevator.ELEVATOR_MIN_HEIGHT, 
            kElevator.ELEVATOR_MAX_HEIGHT, 
            true, 
            kElevator.ELEVATOR_MIN_HEIGHT
        );
        PID = new PIDController(kElevator.SIM_PID.kP, kElevator.SIM_PID.kI, kElevator.SIM_PID.kD);
        running = false;

    }

    /**
     * Set voltage of the motor to assigned voltage 
     *@param voltage voltage value
     */
    @Override
    public void setMotorVoltage(double voltage) {
        elevatorSim.setInputVoltage(voltage);
    }

    /**
     * Stops the motor
     */
    @Override
    public void stopMotor() {
        elevatorSim.setInputVoltage(0.0);
        running = false;
    }

    /**
     * sets the elevator to the assigned setpoint
     * @param setpoint setpoint value
     */
    @Override
    public void setSetpoint(Distance setpoint) {
        PID.setSetpoint(setpoint.in(Meters));
        running = true;
    }

    /**
     * returns the elevator encoders value
     * @return values of the encoder
     */
    @Override
    public Distance getPosition() {
        return Meters.of(elevatorSim.getPositionMeters());
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        double volts = 0.0;
        double current = 0.0;
        if (running) {
            volts = MathUtil.clamp(
                PID.calculate(elevatorSim.getPositionMeters()) * 12, 
                -RoboRioSim.getVInVoltage(), 
                RoboRioSim.getVInVoltage()
            );
            current = elevatorSim.getCurrentDrawAmps();
        }
        elevatorSim.setInputVoltage (volts);
        elevatorSim.update(0.02);

        inputs.mainMotorConnection = true;
        inputs.mainAppliedVoltage = volts;
        inputs.mainAppliedCurrent = Math.abs(current);
        inputs.mainMotorTemperature = 0.0;
        inputs.mainMotorPosition = elevatorSim.getPositionMeters();

        inputs.followerMotorConnection = true;
        inputs.followerAppliedVoltage = volts;
        inputs.followerAppliedCurrent = Math.abs(current);
        inputs.followerMotorTemperature = 0.0;
        inputs.followerMotorPosition = elevatorSim.getPositionMeters();
    }
}