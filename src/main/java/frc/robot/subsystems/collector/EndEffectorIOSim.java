package frc.robot.subsystems.collector;

public class EndEffectorIOSim implements EndEffectorIO {

    private double voltage = 0.0;

    public EndEffectorIOSim() {}

    /**
     * Set voltage to volts
     */
    @Override
    public void setVoltage(double volts) {
        voltage = volts;
    }
    /**
     * Make the logged volts to the voltage
     */
    @Override
    public void updateInputs(EndEffectorInputs inputs) {
        inputs.endEffectorVolts = voltage;
    }
}
