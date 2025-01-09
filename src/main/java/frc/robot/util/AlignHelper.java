package frc.robot.util;

import java.util.Collection;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.FlippingUtil.FieldSymmetry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.kAutoAlign.kReef;

public class AlignHelper {
    public static Pose2d getClosestReef(Pose2d robotPose) {
        Collection<Pose2d> reef = kReef.TARGETS.values();

        boolean shouldFlip = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

        FlippingUtil.symmetryType = FieldSymmetry.kRotational;

        double minDist = -1;
        Pose2d closest = null;

        for (Pose2d branch : reef) {
            Pose2d pose = branch;
            if (shouldFlip) 
                pose = FlippingUtil.flipFieldPose(pose);

            double dist = robotPose.getTranslation().getDistance(pose.getTranslation());
            if (minDist == -1 || dist < minDist) {
                minDist = dist;
                closest = pose;
            }
        }

        return closest;
    }
}
