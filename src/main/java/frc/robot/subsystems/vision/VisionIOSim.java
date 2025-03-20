package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;

public class VisionIOSim implements VisionIO {
    private final SwerveDriveSimulation sim_drive;
    private final VisionSystemSim sim_vision;
    private final PhotonPoseEstimator poseEstimator;
    private final PhotonCamera cam;

    private Optional<Pose3d> prevEstPose = Optional.empty();

    private boolean hasTarget = false;

    private static volatile VisionIOSim globalThis = null;

    private static final Transform3d ROBOT_CAM_OFFSET = new Transform3d(new Translation3d(-0.118, 0.218, 0.38), new Rotation3d(0, 0, Units.degreesToRadians(-20)));

    public VisionIOSim(SwerveDriveSimulation sim) {
        if (globalThis != null) throw new IllegalArgumentException("You cannot create more than one VisionIOSim!");

        this.sim_drive = sim;
        this.sim_vision = new VisionSystemSim("PV-simsystem");

        SimCameraProperties prop = new SimCameraProperties();
        prop.setCalibration(1280, 800, Rotation2d.fromDegrees(97.6524449259));
        prop.setCalibError(0.25, 0.08);
        prop.setFPS(20);
        prop.setAvgLatencyMs(35);
        prop.setLatencyStdDevMs(5);

        this.cam = new PhotonCamera("PV-simcamera");
        PhotonCameraSim cameraSim = new PhotonCameraSim(cam, prop);

        try {
            AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
            sim_vision.addAprilTags(tagLayout);

            poseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOT_CAM_OFFSET);
        } catch (IOException e) {
            throw new RuntimeException("Failed to load PV AprilTag layout '" + AprilTagFields.kDefaultField + "': " + AprilTagFields.kDefaultField.m_resourceFile);
        }

        sim_vision.addCamera(cameraSim, ROBOT_CAM_OFFSET);

        globalThis = this;
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.isConnected = true;
        inputs.fps         = 20;
        inputs.prxLatency  = 35;
        inputs.hasTarget = hasTarget;
    }

    @Override
    public PoseEstimate estimatePose(Drive drive) {
        if (prevEstPose.isPresent()) poseEstimator.setReferencePose(prevEstPose.get());

        List<PhotonPipelineResult> results = cam.getAllUnreadResults();
        if (results.isEmpty()) return null;
        Optional<EstimatedRobotPose> pe = poseEstimator.update(results.get(results.size() - 1));

        if (!pe.isPresent()) {
            hasTarget = false;
            return null;
        }

        hasTarget = true;

        prevEstPose = Optional.of(pe.get().estimatedPose);

        if (pe.get().targetsUsed.size() < 2) return null;   // minimum tag threshold

        return new PoseEstimate(
            pe.get().estimatedPose.toPose2d(), 
            pe.get().timestampSeconds, 
            0, 
            pe.get().targetsUsed.size(), 
            0, 
            0, 
            0, 
            pe.get().targetsUsed.stream().map(
                t -> new RawFiducial(
                    t.fiducialId, 
                    t.bestCameraToTarget.getX(), 
                    t.getBestCameraToTarget().getY(), 
                    t.area, 
                    t.bestCameraToTarget.getTranslation().getNorm(), 
                    t.bestCameraToTarget.getTranslation().plus(ROBOT_CAM_OFFSET.getTranslation()).getNorm(), 
                    t.poseAmbiguity)
            ).toList().toArray(new RawFiducial[]{}),
        false);
    }

    @Override
    public void simulationPeriodic() {
        sim_vision.update(sim_drive.getSimulatedDriveTrainPose());
        Logger.recordOutput("Simulation/Vision/RobotPose", sim_vision.getRobotPose());
    }
}
