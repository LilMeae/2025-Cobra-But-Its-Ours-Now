package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kVision;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.drive.Drive;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionInputsAutoLogged inputs;

    private final ShuffleboardTab sTab = Shuffleboard.getTab("Vision");
    private final GenericEntry sTagCount;
    private final Field2d sEstimatedPose;

    public Vision(VisionIO io) {
        this.io = io;
        inputs = new VisionInputsAutoLogged();

        LimelightHelpers.setCameraPose_RobotSpace(kVision.CAM_NAME, 0.171919, 0, 0.629752, 0, 0, 0);

        sTagCount = sTab.add("Tag Count", 0).getEntry();
        sEstimatedPose = new Field2d();
        sTab.add("Estimated Pose", sEstimatedPose);
    }

    /**
     * Estimates the robot pose given the apriltags on the field
     */
    public void addPoseEstimate(Drive drive) {
        Rotation2d rot = drive.getRotation();
        LimelightHelpers.SetRobotOrientation(kVision.CAM_NAME, rot.getDegrees(),
                drive.getChassisSpeeds().omegaRadiansPerSecond, 0, 0, 0, 0);
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kVision.CAM_NAME);

        if (estimate == null)
            return;

        sTagCount.setInteger(estimate.tagCount);
        sEstimatedPose.setRobotPose(estimate.pose);

        if (estimate.tagCount < kVision.FIDUCIAL_TRUST_THRESHOLD)
            return;

        drive.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, VecBuilder.fill(.5, .5, 9999999));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}