package frc.robot.util;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * @author Alexander Szura 5409
 */
public class FieldMirror {
    
    private FieldMirror() {}

    /**
     * Mirrors the pose to the left/right side of the field, used for left/right pose generation
     * @param pose The pose to flip
     * @return TThe flipped pose
     */
    public static Pose2d mirrorPose(Pose2d pose) {
        return new Pose2d(new Translation2d(pose.getX(), FlippingUtil.fieldSizeY - pose.getY()), new Rotation2d(Math.PI * 2).minus(pose.getRotation()));
    }
}
