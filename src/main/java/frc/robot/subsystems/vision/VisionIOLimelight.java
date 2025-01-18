package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.kVision;

public class VisionIOLimelight implements VisionIO {

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.tx = LimelightHelpers.getTX(kVision.CAM_NAME);
        inputs.ty = LimelightHelpers.getTY(kVision.CAM_NAME);
        inputs.ta = LimelightHelpers.getTA(kVision.CAM_NAME);
        inputs.hasTarget = LimelightHelpers.getTV(kVision.CAM_NAME);
        inputs.targetId = LimelightHelpers.getFiducialID(kVision.CAM_NAME);
    }

}
