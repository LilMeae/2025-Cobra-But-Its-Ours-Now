package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.kArmPivot;
import frc.robot.util.PhoenixUtil;

public class ArmPivotIOTalonFX implements ArmPivotIO {
    private TalonFX armMotor;
    private CANcoder canCoderSensor;
    private final PositionVoltage positionVoltage;

    // Status signals
    private StatusSignal<Angle> positionSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private StatusSignal<Voltage> deviceVoltage;
    private StatusSignal<Current> deviceCurrent;
    private StatusSignal<Temperature> deviceTemp;
    private StatusSignal<AngularVelocity> deviceVelocity;
    private StatusSignal<MagnetHealthValue> magneticHealth;

    public ArmPivotIOTalonFX(int canID, int sensorID) {

        armMotor = new TalonFX(canID);
        canCoderSensor = new CANcoder(sensorID);

        // Apply/configure settings to CANCoder
        canCoderSensor.getConfigurator().apply(
            new CANcoderConfiguration().MagnetSensor
            .withAbsoluteSensorDiscontinuityPoint(Degrees.of(180))
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(kArmPivot.MAGNET_SENSOR_OFFSET)
        );

        TalonFXConfigurator configurator = armMotor.getConfigurator();

        // Apple/configure current settings
        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.SupplyCurrentLimit = 30;
        limitConfigs.SupplyCurrentLimitEnable = true;

        configurator.apply(limitConfigs);

        // Apply/configure motor output
        MotorOutputConfigs outputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

        configurator.apply(outputConfigs);

        // Apply/configure CANCoder settings
        FeedbackConfigs feedBackConfig = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withRemoteCANcoder(canCoderSensor)
            .withSensorToMechanismRatio(1.0)
            .withRotorToSensorRatio(kArmPivot.ARM_GEARING);
            
        configurator.apply(feedBackConfig);

        positionVoltage = new PositionVoltage(0).withSlot(0);

        // Apply PID and feedforward
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = kArmPivot.kP;
        slot0Configs.kI = kArmPivot.kI;
        slot0Configs.kD = kArmPivot.kD;
        slot0Configs.kG = kArmPivot.kG;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        armMotor.getConfigurator().apply(slot0Configs);

        // Set status signals
        positionSignal = armMotor.getPosition();
        velocitySignal = armMotor.getVelocity();
        deviceVoltage = armMotor.getMotorVoltage();
        deviceCurrent = armMotor.getSupplyCurrent();
        deviceTemp = armMotor.getDeviceTemp();
        deviceVelocity = armMotor.getVelocity();
        magneticHealth = canCoderSensor.getMagnetHealth();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            positionSignal,
            velocitySignal,
            deviceVoltage,
            deviceCurrent,
            deviceTemp,
            deviceVelocity,
            magneticHealth
        );

        armMotor.optimizeBusUtilization();
        // TODO: Look into this
        // canCoderSensor.optimizeBusUtilization();
    }

    /*
     * Set voltage
     * @param volts
     */
    @Override
    public void setVoltage(double volts) {
        armMotor.setVoltage(volts);
    }
    
    /*
     * Set arm's setpoint
     * @param armPosiitonRad
     */
    @Override
    public void setSetpoint(Angle armPositionRad) {
        PhoenixUtil.tryUntilOk(3, () -> armMotor.setControl(positionVoltage.withPosition(armPositionRad)));
    }

    /*
     * Get arm's current position
     * @return arm's position
     */
    @Override
    public Angle getPosition() {
        return armMotor.getPosition().getValue();
    }

    /*
     * Update inputs with hardware data
     * @param inputs
     */
    @Override
    public void updateInputs(ArmPivotInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
            deviceVoltage,
            deviceCurrent,
            deviceTemp,
            deviceVelocity
        ).isOK();

        inputs.positionAngles = positionSignal.getValue().in(Degrees);
        inputs.speed = velocitySignal.getValue().in(RadiansPerSecond);
        inputs.positionRad = Units.degreesToRadians(inputs.positionAngles);
        inputs.voltage = deviceVoltage.getValueAsDouble();
        inputs.current = Math.abs(deviceCurrent.getValueAsDouble());
        inputs.temperature = deviceTemp.getValueAsDouble();
        inputs.speed = deviceVelocity.getValueAsDouble();

        inputs.cancoderConnected = BaseStatusSignal.refreshAll(
            magneticHealth
        ).isOK();

        inputs.magnetHealth = magneticHealth.getValue();
    }
}