package frc.robot.subsystems.collector;

public class EndEffectorIOSim implements EndEffectorIO {

    private double voltage = 0.0;

    public EndEffectorIOSim() {}

    @Override
    public void setVoltage(double volts) {
        voltage = volts;
    }

    @Override
    public void updateInputs(EndEffectorInputs inputs) {
        inputs.endEffectorVolts = voltage;
    }
}
