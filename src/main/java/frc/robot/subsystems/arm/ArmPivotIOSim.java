package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants.kArmPivot;

public class ArmPivotIOSim implements ArmPivotIO{

    // private static final ShuffleboardTab sb_sim = Shuffleboard.getTab("SIM");

    private boolean isRunning;

    private final SingleJointedArmSim armSim;
    private final PIDController controller;

    private final Mechanism2d mech;
    private final MechanismRoot2d root;
    private final MechanismLigament2d stand;
    private final MechanismLigament2d arm;

    public ArmPivotIOSim() {
        // Create single jointed arm object
        armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            kArmPivot.ARM_GEARING,
            kArmPivot.ARM_MOI.in(KilogramSquareMeters),
            kArmPivot.ARM_LENGTH.in(Meters),
            kArmPivot.minAngles.in(Radians),
            kArmPivot.maxAngles.in(Radians),
            true,
            Math.PI
        );
        armSim.update(0.02);

        // Create sim PID controller object
        controller = new PIDController(
            kArmPivot.SIMULATED_PID_VALUES.kP,
            kArmPivot.SIMULATED_PID_VALUES.kI,
            kArmPivot.SIMULATED_PID_VALUES.kD
        );

        // Initialize new mechanism and base object
        mech = new Mechanism2d(0.6, 10.0);
        root = mech.getRoot("Base", 0.3, 0.1);

        // Add new stand object to base
        stand = root.append(
            new MechanismLigament2d(
                "Stand",
                 0.7,
                 90
            )
        );

        // Add new arm object to stand
        arm = stand.append(
            new MechanismLigament2d(
                "Arm",
                 0.5,
                 90
            )
        );

        // SmartDashboard.putData("Mech2d", mech);

        // sb_sim.add("Arm Mech", mech);

        setSetpoint(Degrees.of(90));
    }

    /*
     * Set setpoint in radians
     * @param angle
     */
    @Override
    public void setSetpoint(Angle angle) {
        controller.setSetpoint(angle.in(Radians));
        isRunning = true;
    }

    /*
     * Get position of the arm in radians
     * @return arm position in radians
     */
    @Override
    public Angle getPosition() {
        return Radians.of(armSim.getAngleRads());
    }

    /*
     * Stop arm movement
     */
    @Override
    public void stop() {
        armSim.setInputVoltage(0.0);
        controller.reset();
        isRunning = false;
    }

    /*
     * Update inputs with simulated data
     * @param inputs
     */
    @Override
    public void updateInputs(ArmPivotInputs inputs) {
        double volatge = 0.0;
        double current = 0.0;

        // Set voltage based on simulated PID values
        if (isRunning) {
            volatge = MathUtil.clamp(
                controller.calculate(armSim.getAngleRads()) * RoboRioSim.getVInVoltage(),
                -12,
                12
            );
            
            armSim.setInputVoltage(volatge);
            armSim.update(0.02);

            arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));

            current = armSim.getCurrentDrawAmps();
        }

        // Update inputs
        inputs.connected = true;
        inputs.positionRad = armSim.getAngleRads();
        inputs.positionAngles = Units.radiansToDegrees(inputs.positionRad);
        inputs.targetAngle = controller.getSetpoint();
        inputs.voltage = volatge;
        inputs.current = Math.abs(current);
        inputs.temperature = 0.0;
    }
}