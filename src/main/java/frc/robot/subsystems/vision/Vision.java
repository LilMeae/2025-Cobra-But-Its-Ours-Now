package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kVision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers.PoseEstimate;

/**
 * @author Logan, Alexander Szura team 5409
 */
public class Vision extends SubsystemBase {

    private final VisionIO io;
    private final VisionInputsAutoLogged inputs;

    private final Alert disconnectedAlert = new Alert("Limelight appears to be disconnected. (TIMEOUT)", Alert.AlertType.kError);
    private final Alert tempAlert = new Alert("LL Temp", AlertType.kWarning);

    public Vision(VisionIO io) {
        this.io = io;
        inputs = new VisionInputsAutoLogged();

        io.setCameraOffset();
    }

    /**
     * Estimates the robot pose given the AprilTags on the field
     * @param drive Drive subsystem to adjust odometry of
     */
    public void addPoseEstimate(Drive drive) {
        PoseEstimate estimate = io.estimatePose(drive);

        if (estimate == null)
            return;

        if (estimate.tagCount < kVision.FIDUCIAL_TRUST_THRESHOLD)
            return;

        drive.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
    }

    /**
     * Sets the rotation of the robot, used for LL MT2
     * @param rotation The rotation of the robot
     */
    public void setRotation(Rotation2d rotation) {
        io.setRotation(rotation);
    }

    /**
     * @return true if vision has a target
     */
    public boolean hasTarget() {
        return inputs.hasTarget;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);

        disconnectedAlert.set(!inputs.isConnected && Constants.currentMode != Constants.Mode.SIM);
        tempAlert.set(inputs.sysTemp >= 70.0);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        io.simulationPeriodic();
    }
}