package frc.robot.subsystems.collector;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.kEndEffector;

public class EndEffectorIOTalonFx implements EndEffectorIO {
    
    private TalonFX endEffectorMotor;
    private TalonFXConfigurator endEffectorConfig;
    private CurrentLimitsConfigs currentConfig;

    private StatusSignal<Voltage> deviceVoltage;
    private StatusSignal<Current> deviceCurrent;
    private StatusSignal<Temperature> deviceTemp;
    private StatusSignal<AngularVelocity> deviceVelocity;
    private StatusSignal<Angle> devicePosition;
    
    private final TimeOfFlight tof = new TimeOfFlight(kEndEffector.TIMOFFLIGHT_SENSORID);

    public EndEffectorIOTalonFx(int ID) {
        // Creating Objects
        endEffectorMotor = new TalonFX(ID);
        endEffectorConfig = endEffectorMotor.getConfigurator();
        currentConfig = new CurrentLimitsConfigs();

        // Setting Configs
        currentConfig.SupplyCurrentLimit = kEndEffector.CURRENT_LIMIT;
        currentConfig.SupplyCurrentLimitEnable = true;
        endEffectorConfig.apply(currentConfig);

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

        endEffectorConfig.apply(motorOutputConfigs);

        // Getting Status Signals
        deviceVoltage = endEffectorMotor.getMotorVoltage();
        deviceCurrent = endEffectorMotor.getSupplyCurrent();
        deviceTemp = endEffectorMotor.getDeviceTemp();
        deviceVelocity = endEffectorMotor.getVelocity();
        devicePosition = endEffectorMotor.getPosition();
        // Make it so these status signals aren't touched by optimization
        BaseStatusSignal.setUpdateFrequencyForAll(
            50, 
            deviceVoltage,
            deviceCurrent,
            deviceTemp,
            deviceVelocity,
            devicePosition
        );
        // Optimize the CanBus
        endEffectorMotor.optimizeBusUtilization();

        tof.setRangingMode(RangingMode.Short, 50);
    }

    @Override 
    public void setVoltage(double volts) {
        endEffectorMotor.setVoltage(volts);
    }

    @Override
    public Distance getTofRange(){
        return Millimeters.of(tof.getRange());
    }

    @Override
    public double getMotorCurrent() {
        return deviceCurrent.getValueAsDouble();
    }

    @Override
    public void coastMode() {
        endEffectorMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void brakeMode() {
        endEffectorMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override 
    public void updateInputs(EndEffectorInputs inputs) {
        inputs.endEffectorConnection = BaseStatusSignal.refreshAll(
            deviceVoltage,
            deviceCurrent,
            deviceTemp
        ).isOK();
        
        inputs.endEffectorVolts = deviceVoltage.getValueAsDouble();
        inputs.endEffectorCurrent = Math.abs(deviceCurrent.getValueAsDouble());
        inputs.endEffectorTemp = deviceTemp.getValueAsDouble();
        inputs.tofDistance = getTofRange().in(Millimeters);
        inputs.endEffectorVelocity = deviceVelocity.getValueAsDouble();
        inputs.endEffectorPosition = devicePosition.getValueAsDouble();
    }
}
