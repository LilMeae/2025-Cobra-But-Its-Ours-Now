package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map.Entry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.ScoringLevel;
import frc.robot.Constants.kAutoAlign;
import frc.robot.Constants.kAutoAlign.kReef;
import frc.robot.Constants.kAutoAlign.kStation;

public class AlignHelper {

    public static enum kClosestType {
        DISTANCE,
        ROTATION
    }

    public static enum kDirection {
        LEFT,
        RIGHT,
        BOTH
    }

    private static ChassisSpeeds speeds = new ChassisSpeeds();
    private static int timer = kAutoAlign.TIME_ADJUSTMENT_TIMEOUT;

    private AlignHelper() {}

    public static void reset(ChassisSpeeds speeds) {
        AlignHelper.speeds = speeds;
        timer = kAutoAlign.TIME_ADJUSTMENT_TIMEOUT;
    }

    public static ScoringLevel getAlgaeHeight(Pose2d robotPose) {
        return kReef.ALGAE_HEIGHTS.getOrDefault(getClosestReef(robotPose), ScoringLevel.LEVEL2_ALGAE);
    }

    public static Pose2d getClosestReef(Pose2d robotPose) {
        return calculator(robotPose, kClosestType.DISTANCE, kReef.TARGETS.values());
    }

    public static Pose2d getClosestBranch(Pose2d robotPose) {
        return getClosestBranch(robotPose, kClosestType.DISTANCE);
    }

    public static Pose2d getClosestBranch(Pose2d robotPose, kClosestType type) {
        return getClosestBranch(robotPose, type, kDirection.BOTH);
    }

    public static Pose2d getClosestBranch(Pose2d robotPose, kClosestType type, kDirection direction) {
        return calculator(robotPose, type, getBranchPoses(direction));
    }

    public static Pose2d getClosestStation(Pose2d robotPose) {
        return getClosestStation(robotPose, kClosestType.DISTANCE, kDirection.BOTH);
    }

    public static Pose2d getClosestStation(Pose2d robotPose, kClosestType type, kDirection station) {        
        return calculator(robotPose, type, getStationPoses(station));
    }

    public static Pose2d getClosestElement(Pose2d robotPose) {
        return getClosestElement(robotPose, kDirection.BOTH);
    }

    public static Pose2d getClosestElement(Pose2d robotPose, kDirection direction) {
        Collection<Pose2d> poses = getStationPoses(direction);
        poses.addAll(getBranchPoses(direction));

        return calculator(robotPose, kClosestType.DISTANCE, poses);
    }

    public static Collection<Pose2d> getStationPoses(kDirection station) {
        List<Pose2d> poses = new ArrayList<>();
        for (int i = -1; i < 4; i++) {
            if (station == kDirection.LEFT  || station == kDirection.BOTH)
                poses.add(kStation.LEFT_STATION .transformBy(kStation.STATIONS_OFFSET.times( i)));
            
            if (station == kDirection.RIGHT || station == kDirection.BOTH)
                poses.add(kStation.RIGHT_STATION.transformBy(kStation.STATIONS_OFFSET.times(-i)));
        }

        return poses;
    }

    public static Collection<Pose2d> getBranchPoses(kDirection branch) {
        List<Pose2d> poses = new ArrayList<>();
        for (Entry<String, Pose2d> entry : kReef.BRANCHES.entrySet()) {
            if (branch == kDirection.BOTH) {
                poses.add(entry.getValue());
                continue;
            }

            if (branch == kDirection.LEFT  && entry.getKey().endsWith("L"))
                poses.add(entry.getValue());

            if (branch == kDirection.RIGHT && entry.getKey().endsWith("R"))
                poses.add(entry.getValue());
        }

        return poses;
    }

    public static Pose2d getClosestL1(Pose2d robotPose, kClosestType type) {
        return calculator(robotPose, type, kReef.L1_POSES);
    }

    private static Pose2d calculator(Pose2d robotPose, kClosestType type, Collection<Pose2d> poses) {
        Pose2d estimatedPose = robotPose.plus(
            new Transform2d(
                speeds.vxMetersPerSecond * kAutoAlign.VELOCITY_TIME_ADJUSTEDMENT.in(Seconds),
                speeds.vyMetersPerSecond * kAutoAlign.VELOCITY_TIME_ADJUSTEDMENT.in(Seconds),
                robotPose.getRotation().plus(
                    Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * kAutoAlign.VELOCITY_TIME_ADJUSTEDMENT.in(Seconds))
                )
            )
        );

        if (timer-- <= 0)
            speeds = new ChassisSpeeds();
        
        double min = -1;
        Pose2d closest = null;

        for (Pose2d pose : poses) {
            if (type == kClosestType.DISTANCE) {
                double dist = estimatedPose.getTranslation().getDistance(pose.getTranslation());
                if (min == -1 || dist < min) {
                    min = dist;
                    closest = pose;
                }
            } else if (type == kClosestType.ROTATION) {
                double rotation = Math.abs(estimatedPose.getRotation().minus(pose.getRotation()).getRadians());
                if (min == -1 || rotation < min) {
                    min = rotation;
                    closest = pose;
                }
            } else throw new IllegalArgumentException("AlignHelper Recieved an invalid type");
        }

        return closest;
    }

    public static Angle rotationDifference(Rotation2d r1, Rotation2d r2) {
        double difference = r1.getRadians() - r2.getRadians();

        difference = (difference + Math.PI) % (2 * Math.PI) - Math.PI;

        while (difference < -Math.PI)
            difference += 2 * Math.PI;

        while (difference > Math.PI)
            difference -= 2 * Math.PI;

        return Radians.of(Math.abs(difference));
    }
}
