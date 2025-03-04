package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Inspired by FRC Team <a href="https://github.dev/Mechanical-Advantage/RobotCode2024">6328</a> FudgeFactors */
public class FieldFactors {
    public static class AllianceTransform {
        private Transform2d blueAlliance = new Transform2d();
        private Transform2d redAlliance  = new Transform2d();

        public Transform2d getTransform() {
            return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? blueAlliance : redAlliance; 
        }

        public AllianceTransform withRedFactor(double forwardFactor, double strafeFactor, double angleFactor) {
            return withRedFactor(Meters.of(forwardFactor), Meters.of(strafeFactor), Degrees.of(angleFactor));
        }

        public AllianceTransform withRedFactor(Distance forwardFactor, Distance strafeFactor, Angle angleFacor) {
            return withRedFactor(new Transform2d(forwardFactor, strafeFactor, new Rotation2d(angleFacor)));
        }

        public AllianceTransform withRedFactor(Transform2d factor) {
            redAlliance = factor;
            return this;
        }

        public AllianceTransform withBlueFactor(double forwardFactor, double strafeFactor, double angleFactor) {
            return withBlueFactor(Meters.of(forwardFactor), Meters.of(strafeFactor), Degrees.of(angleFactor));
        }

        public AllianceTransform withBlueFactor(Distance forwardFactor, Distance strafeFactor, Angle angleFacor) {
            return withBlueFactor(new Transform2d(forwardFactor, strafeFactor, new Rotation2d(angleFacor)));
        }

        public AllianceTransform withBlueFactor(Transform2d factor) {
            blueAlliance = factor;
            return this;
        }
    }

    private FieldFactors() {}

    public static final AllianceTransform PROCESSOR     = new AllianceTransform();

    public static final AllianceTransform LEFT_STATION  = new AllianceTransform();

    public static final AllianceTransform RIGHT_STATION = new AllianceTransform();
}
