// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.Map.Entry;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.commands.AutoCommands.kReefPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldMirror;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final boolean TUNNING = false;

  public static final class kDrive {
    public static final Mass ROBOT_FULL_MASS = Kilograms.of(60.27789);
    public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(2.881);
    public static final double WHEEL_COF = 1.916;
  }

  public static final class kAuto {
    public static final boolean PRINT_AUTO_TIME = false;

    /** When this is true the robot will set it's position where the path starts when the auto is selected. */
    public static final boolean RESET_ODOM_ON_CHANGE = true;

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants ROTATION_PID    = new PIDConstants(5.0, 0.0, 0.0);
  }

  public static final class kAutoAlign {
    public static final PIDConstants ALIGN_PID = new PIDConstants(4.9, 0.0, 0.28);

    public static final LinearVelocity     MAX_AUTO_ALIGN_VELOCITY_SLOW     = MetersPerSecond         .of(2.00);
    public static final LinearVelocity     MAX_AUTO_ALIGN_VELOCITY_FAST     = MetersPerSecond         .of(2.75);
    public static final LinearAcceleration MAX_AUTO_ALIGN_ACCELERATION_SLOW = MetersPerSecondPerSecond.of(8.00);
    public static final LinearAcceleration MAX_AUTO_ALIGN_ACCELERATION_FAST = MetersPerSecondPerSecond.of(25.0);

    public static final Distance TRANSLATION_TOLERANCE;
    public static final Angle    ROTATION_TOLERANCE   ;
    public static final LinearVelocity VELOCITY_TOLERANCE = MetersPerSecond.of(0.1);
    static {
        if (TUNNING) {
            TRANSLATION_TOLERANCE = Centimeters.of(0.00);
            ROTATION_TOLERANCE    = Degrees    .of(0.00);
        } else {
            TRANSLATION_TOLERANCE = Centimeters.of(1.75);
            ROTATION_TOLERANCE    = Degrees    .of(1.25);
        }
    }

    public static final PathConstraints PATH_FIND_CONSTRAINTS = new PathConstraints(
        TunerConstants.kSpeedAt12Volts,
        MAX_AUTO_ALIGN_ACCELERATION_FAST,
        RadiansPerSecond.of(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / Drive.DRIVE_BASE_RADIUS),
        DegreesPerSecondPerSecond.of(720.0)
    );

    public static final Pose2d PROCESSOR_TARGET = new Pose2d(11.568, 7.500, Rotation2d.fromDegrees(-90.000));

    public static final Time VELOCITY_TIME_ADJUSTEDMENT = Milliseconds.of(250);
    public static final int  TIME_ADJUSTMENT_TIMEOUT = 10;

    public static final class kReef {
      public static final Transform2d LEFT_OFFSET_TO_BRANCH  = new Transform2d(0.35, 0.18, new Rotation2d());
      public static final Transform2d RIGHT_OFFSET_TO_BRANCH = new Transform2d(0.35, -0.18, new Rotation2d());

      private static final Pose2d generatePose(Rotation2d rotation) {
        final double mx = 4.48945;
        final double my = FlippingUtil.fieldSizeY / 2.0;
        final double r = 1.64;

        return new Pose2d(r * -rotation.getCos() + mx, r * -rotation.getSin() + my, rotation);
      } 

      public static final HashMap<kReefPosition, Pose2d> TARGETS = new HashMap<>();
      static {
        TARGETS.put(kReefPosition.CLOSE_LEFT,   generatePose(Rotation2d.fromDegrees(-60.000)));
        TARGETS.put(kReefPosition.FAR_LEFT,     generatePose(Rotation2d.fromDegrees(-120.000)));
        TARGETS.put(kReefPosition.FAR,          generatePose(Rotation2d.fromDegrees(180.000)));
        TARGETS.put(kReefPosition.CLOSE,        generatePose(Rotation2d.fromDegrees(0.000)));
        
        TARGETS.put(kReefPosition.CLOSE_RIGHT,  FieldMirror.mirrorPose(TARGETS.get(kReefPosition.CLOSE_LEFT)));
        TARGETS.put(kReefPosition.FAR_RIGHT,    FieldMirror.mirrorPose(TARGETS.get(kReefPosition.FAR_LEFT)));
      }

      public static final HashMap<Pose2d, ScoringLevel> ALGAE_HEIGHTS = new HashMap<>();
      static {
        ALGAE_HEIGHTS.put(TARGETS.get(kReefPosition.CLOSE_LEFT ), ScoringLevel.LEVEL2_ALGAE);
        ALGAE_HEIGHTS.put(TARGETS.get(kReefPosition.CLOSE      ), ScoringLevel.LEVEL3_ALGAE);
        ALGAE_HEIGHTS.put(TARGETS.get(kReefPosition.CLOSE_RIGHT), ScoringLevel.LEVEL2_ALGAE);
        ALGAE_HEIGHTS.put(TARGETS.get(kReefPosition.FAR_RIGHT  ), ScoringLevel.LEVEL3_ALGAE);
        ALGAE_HEIGHTS.put(TARGETS.get(kReefPosition.FAR        ), ScoringLevel.LEVEL2_ALGAE);
        ALGAE_HEIGHTS.put(TARGETS.get(kReefPosition.FAR_LEFT   ), ScoringLevel.LEVEL3_ALGAE);
      }

      public static final HashMap<String, Pose2d> BRANCHES = new HashMap<>();
      static {
        for (Entry<kReefPosition, Pose2d> entry : TARGETS.entrySet()) {
            BRANCHES.put(entry.getKey().name() + ".L", entry.getValue().transformBy( LEFT_OFFSET_TO_BRANCH));
            BRANCHES.put(entry.getKey().name() + ".R", entry.getValue().transformBy(RIGHT_OFFSET_TO_BRANCH));
        }
      }
    }

    public static final class kStation {
        private static final Distance DISTANCE_RAMPS = Inches .of( 8.000);
        private static final Angle    STATION_ANGLE  = Degrees.of(-54.000);

        public static final Pose2d LEFT_STATION  = new Pose2d(1.498, 7.274, Rotation2d.fromDegrees(STATION_ANGLE.in(Degrees)));
        public static final Pose2d RIGHT_STATION = FieldMirror.mirrorPose(LEFT_STATION);

        public static final Transform2d STATIONS_OFFSET = new Transform2d(
            0.0,
            -DISTANCE_RAMPS.in(Meters),
            new Rotation2d()
        );
    }
  }

  public static final class kArmPivot {
    public static final int FALCON_ID = 22;
    public static final int CANCODER_ID = 24;

    public static final double MAGNET_SENSOR_OFFSET = -0.344727;

    public static final double kP = 112.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0075;
    public static final double kG = 0.025 * 12.0;

    public static final double   ARM_GEARING      = (72/22) * 4*9;
    public static final Distance ARM_DRUM_RADIUS  = Inches.of(0.944);
    public static final MomentOfInertia ARM_MOI   = KilogramSquareMeters.of(0.724982551);
    public static final Distance ARM_LENGTH       = Inches.of(12.0);
    public static final Mass     ARM_MASS         = Pounds.of(8.75);

    public static final Angle minAngles = Degrees.of(-90);
    public static final Angle maxAngles = Degrees.of(120);

    public static final Angle MOVEMENT_SETPOINT = Degrees.of(88);
    public static final Angle PICKUP_ANGLE  = Degrees.of(104.5);

    public static final PIDConstants SIMULATED_PID_VALUES = new PIDConstants(3.25, 0.0, 0.3);
  }

    public static enum ScoringLevel {
        LEVEL1(      Meters.of(0.043), Degrees.of(100.), 12.),
        LEVEL2(      Meters.of(0.170), Degrees.of(83.0), 6.5),
        LEVEL3(      Meters.of(0.370), Degrees.of(83.0), 6.5),
        LEVEL4(      Meters.of(0.665), Degrees.of(87.0), 7.6),

        // TODO: Tune these values
        LEVEL1_DIST( Meters.of(0.030), Degrees.of(100.), 2.0),
        LEVEL2_DIST( Meters.of(0.170), Degrees.of(83.0), 6.5),
        LEVEL3_DIST( Meters.of(0.370), Degrees.of(83.0), 6.5),
        LEVEL4_DIST( Meters.of(0.665), Degrees.of(87.0), 7.6),

        LEVEL2_ALGAE(Meters.of(0.220), Degrees.of(70.5), 0.0), // No voltages, stored in algae voltage
        LEVEL3_ALGAE(Meters.of(0.420), Degrees.of(70.5), 0.0), // No voltages, stored in algae voltage
        /** Not Implemented */
        BARGE(       Meters.of(0.650), Degrees.of(100.), 0.0),
        /** Not Implemented */
        PROCESSOR(   Meters.of(0.050), Degrees.of(80.0), 0.0);

        public final Distance elevatorSetpoint;
        public final Angle pivotAngle;
        public final double voltage;

        private ScoringLevel(Distance elevatorSetpoint, Angle pivotAngle, double voltage) {
            this.elevatorSetpoint = elevatorSetpoint;
            this.pivotAngle = pivotAngle;
            this.voltage = voltage;
        }
    }

  public static final class kEndEffector {
      public static final int ENDEFFECTOR_MOTOR_ID = 23;
      public static final int CURRENT_LIMIT = 40;
      public static final int TIMOFFLIGHT_SENSORID = 28;
      public static final Distance TIMEOFFLIGHT_DISTANCE_VALIDATION = Millimeters.of(140);

      public static final double IDLE_VOLTAGE  =  5.5;
      public static final double ALGAE_VOLTAGE = -4.0;

      public static final double ALGAE_CURRENT = 20.0;
  }

  public static final class kElevator {
    public static final int MAIN_MOTOR_ID = 20;
    public static final int FOLLOWER_MOTOR_ID = 21;
    public static final double CURRENT_LIMIT = 30.0;
    public static final double kGearing = 12.0/1.0;
    public static final Distance ELEVATOR_DRUMRADIUS = Inches.of(1.751/2.0);
    public static final double kCircumfrence = 2 * Math.PI * ELEVATOR_DRUMRADIUS.in(Meters);
    public static final double kRotationConverter = kCircumfrence / kGearing;
    public static final PIDConstants TALONFX_PID = new PIDConstants(100, 0, 0);
    public static final PIDConstants SIM_PID = new PIDConstants(10, 0, 0);
    public static final Mass ELEVATOR_MASS = Pound.of(52.95);
    public static final double ELEVATOR_MIN_HEIGHT = 0.0;
    public static final double ELEVATOR_MAX_HEIGHT = 0.652587890625;

    public static final Distance ELEVATOR_PREP_HEIGHT = Meters.of(0.175);

    public static final Distance IDLING_HEIGHT = Meters.of(0.022);
  }

  public static final class kVision {
    public static final String CAM_NAME = "limelight";

    public static final int FIDUCIAL_TRUST_THRESHOLD = 1;

    /**
     * Frames allowed without latency update before flagged as disconnected
     */
    public static final int DISCONNECTION_TIMEOUT = 5;

    public static final int THROTTLE_DISABLED = 200;

    public static final Transform3d OFFSET_FROM_ROBOT_ORIGIN = new Transform3d(
                                                                        new Translation3d(0.1324, -0.2169, 0.3804),
                                                                        new Rotation3d(   0,  0,   Units.degreesToRadians(-20)));
  }
}