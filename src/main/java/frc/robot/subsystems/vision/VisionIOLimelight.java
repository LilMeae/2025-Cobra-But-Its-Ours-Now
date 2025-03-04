package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kVision;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.DebugCommand;

public class VisionIOLimelight implements VisionIO {
    private double lastPrxLatency = 0;
    private double disconnectedFrames = 0;

    public VisionIOLimelight() {
        new Trigger(DriverStation::isDisabled)
            .onTrue(
                setThrottle(kVision.THROTTLE_DISABLED)
            ).onFalse(
                setThrottle(0)
            );

        LimelightHelpers.setLimelightNTDouble(kVision.CAM_NAME, "throttle_set", kVision.THROTTLE_DISABLED);
        
        DebugCommand.register("No Throttle LL", setThrottle(0));
        DebugCommand.register("Throttle LL", setThrottle(kVision.THROTTLE_DISABLED));
    }

    private Command setThrottle(int throttle) {
        return Commands.runOnce(
            () -> LimelightHelpers.setLimelightNTDouble(kVision.CAM_NAME, "throttle_set", throttle)
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
            inputs.fps = 0.0;
            inputs.cpuTemp = 0.0;
            inputs.ramUsage = 0.0;
            inputs.sysTemp = 0.0;
        }

        // check if disconnected by comparing prx latency
        if (lastPrxLatency != inputs.prxLatency) {
            disconnectedFrames = 0;
            inputs.isConnected = true;
        } else {
            disconnectedFrames++;
            inputs.isConnected = disconnectedFrames <= kVision.DISCONNECTION_TIMEOUT;
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

    @Override
    public LimelightHelpers.PoseEstimate estimatePose(Drive drive) {
        Rotation2d rot = drive.getRotation();
        LimelightHelpers.SetRobotOrientation(kVision.CAM_NAME, rot.getDegrees(),
                Units.radiansToDegrees(drive.getChassisSpeeds().omegaRadiansPerSecond), 0, 0, 0, 0);
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(kVision.CAM_NAME);
    }

    /**
     * Forward limelight ports (5800-5809) so it can be used over USB
     */
    public static void forwardLimelightPorts() {
        for (int i = 5800; i <= 5809; i++) 
            PortForwarder.add(i, kVision.CAM_NAME+".local", i);
    }
}
