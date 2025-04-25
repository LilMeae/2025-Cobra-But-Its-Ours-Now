package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kVision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.DebugCommand;
import frc.robot.util.LimelightHelpers;

/**
 * @author Logan, Alexander Szura team 5409
 */
public class VisionIOLimelight implements VisionIO {

    public enum kIMU_MODE {
        EXTERNAL(0),
        FUSED(1),
        INTERNAL(2);

        public final int ID;

        private kIMU_MODE(int num) {
            ID = num;
        }
    }

    private double lastPrxLatency = 0;
    private double disconnectedFrames = 0;

    public VisionIOLimelight() {
        new Trigger(DriverStation::isDisabled)
            .onTrue(
                Commands.parallel(
                    setIMUMode(kIMU_MODE.FUSED),
                    setThrottle(kVision.THROTTLE_DISABLED)
                )
            ).onFalse(
                Commands.parallel(
                    setIMUMode(kIMU_MODE.INTERNAL),
                    setThrottle(0)
                )
            );
        
        DebugCommand.register("No Throttle LL", setThrottle(0));
        DebugCommand.register("Throttle LL", setThrottle(kVision.THROTTLE_DISABLED));
        DebugCommand.register("Fused LL", setIMUMode(kIMU_MODE.FUSED));
        DebugCommand.register("Internal LL", setIMUMode(kIMU_MODE.INTERNAL));

        LimelightHelpers.SetThrottle(kVision.CAM_NAME, kVision.THROTTLE_DISABLED);
        LimelightHelpers.SetIMUMode(kVision.CAM_NAME, kIMU_MODE.FUSED.ID);
    }

    private Command setThrottle(int throttle) {
        return Commands.runOnce(
            () -> LimelightHelpers.SetThrottle(kVision.CAM_NAME, throttle)
        ).ignoringDisable(true);
    }

    private Command setIMUMode(kIMU_MODE mode) {
        return Commands.runOnce(
            () -> LimelightHelpers.SetIMUMode(kVision.CAM_NAME, mode.ID)
        ).ignoringDisable(true);
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.tx           = LimelightHelpers.getTX(               kVision.CAM_NAME);
        inputs.ty           = LimelightHelpers.getTY(               kVision.CAM_NAME);
        inputs.ta           = LimelightHelpers.getTA(               kVision.CAM_NAME);
        inputs.hasTarget    = LimelightHelpers.getTV(               kVision.CAM_NAME);
        inputs.targetId     = LimelightHelpers.getFiducialID(       kVision.CAM_NAME);
        inputs.imgLatency   = LimelightHelpers.getLatency_Capture(  kVision.CAM_NAME);
        inputs.prxLatency   = LimelightHelpers.getLatency_Pipeline( kVision.CAM_NAME);

        Double[] system = LimelightHelpers.getLimelightNTTableEntry(kVision.CAM_NAME, "hw")
                                                    .getDoubleArray(new Double[] {0.0, 0.0, 0.0, 0.0});

        try {
            inputs.fps      = system[3];
            inputs.cpuTemp  = system[2];
            inputs.ramUsage = system[1];
            inputs.sysTemp  = system[0];
        } catch (Exception e) {
            inputs.fps       = -1.0;
            inputs.cpuTemp   = -1.0;
            inputs.ramUsage  = -1.0;
            inputs.sysTemp   = -1.0;
        }

        // check if disconnected by comparing prx latency
        if (lastPrxLatency != inputs.prxLatency) {
            disconnectedFrames = 0;
            inputs.isConnected = true;
        } else {
            inputs.isConnected = ++disconnectedFrames <= kVision.DISCONNECTION_TIMEOUT;
        }

        lastPrxLatency = inputs.prxLatency;
    }

    @Override
    public void setCameraOffset() {
        LimelightHelpers.setCameraPose_RobotSpace(kVision.CAM_NAME,
                                                  kVision.OFFSET_FROM_ROBOT_ORIGIN.getTranslation().getX(), 
                                                  kVision.OFFSET_FROM_ROBOT_ORIGIN.getTranslation().getY(), 
                                                  kVision.OFFSET_FROM_ROBOT_ORIGIN.getTranslation().getZ(), 
                                                  kVision.OFFSET_FROM_ROBOT_ORIGIN.getRotation().getMeasureX().in(Degrees), 
                                                  kVision.OFFSET_FROM_ROBOT_ORIGIN.getRotation().getMeasureY().in(Degrees), 
                                                  kVision.OFFSET_FROM_ROBOT_ORIGIN.getRotation().getMeasureZ().in(Degrees));
    }

    private void logMode(String mode) {
        Logger.recordOutput("Vision/Gyro-Mode", mode);
    }

    @Override
    public LimelightHelpers.PoseEstimate estimatePose(Drive drive) {
        ChassisSpeeds speeds = drive.getChassisSpeeds();

        Rotation2d robotYaw = drive.getRotation();
        if (DriverStation.isEnabled()) {
            if (LimelightHelpers.getTA(kVision.CAM_NAME) >= 1.5 && Math.abs(speeds.vxMetersPerSecond) < 0.1 && Math.abs(speeds.vyMetersPerSecond) < 0.1 && Math.abs(speeds.omegaRadiansPerSecond) < 0.1) {
                LimelightHelpers.SetIMUMode(kVision.CAM_NAME, kIMU_MODE.FUSED.ID);
                robotYaw = LimelightHelpers.getBotPoseEstimate_wpiBlue(kVision.CAM_NAME).pose.getRotation();
                logMode("FUSED");
            } else {
                // Never got internal gyro to work properly, wasn't resetting... Looking back at this might of been because we were throtling the LL.
                // LimelightHelpers.SetIMUMode(kVision.CAM_NAME, kIMU_MODE.INTERNAL.ID);
                // logMode("INTERNAL");
            }
        }

        LimelightHelpers.SetRobotOrientation(kVision.CAM_NAME, robotYaw.getDegrees(), 0, 0, 0, 0, 0);

        // Issue with resetting it's position in auto. If it sees a tag then it would reset back to the wrong position, this would happen only when the LL was throttled.
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kVision.CAM_NAME);
    }

    @Override
    public void setRotation(Rotation2d rotation) {
        LimelightHelpers.SetIMUMode(kVision.CAM_NAME, kIMU_MODE.FUSED.ID);
        LimelightHelpers.SetRobotOrientation(kVision.CAM_NAME, rotation.getDegrees(), 0, 0, 0, 0, 0);
    }

    /**
     * Forward limelight ports (5800-5809) so it can be used over USB
     */
    public static void forwardLimelightPorts() {
        for (int i = 5800; i <= 5809; i++) 
            PortForwarder.add(i, kVision.CAM_NAME + ".local", i);
    }
}
