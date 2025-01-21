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
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.FieldMirror;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics
 * sim) and "replay"
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

  public static final class kDrive {
    public static final Mass ROBOT_FULL_MASS = Pounds.of(125.0);
    public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(9.2437679288);
    public static final double WHEEL_COF = 1.2;
  }

  public static final class kAuto {
    /** @throws IllegalArgumentException If this the auto command is ran twice */
    public static final boolean PRINT_AUTO_TIME = false;

    /** When this is true the robot will set it's position where the path starts when the auto is selected. */
    public static final boolean RESET_ODOM_ON_CHANGE = true;

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5.0 , 0.0, 0.0);
    public static final PIDConstants ROTATION_PID    = new PIDConstants(5.0 , 0.0, 0.0);
  }

  public static final class kAutoAlign {
    public static final PIDConstants ALIGN_PID = new PIDConstants(12.0, 0.0, 0.5);

    public static final LinearVelocity     MAX_AUTO_ALIGN_VELOCITY     = MetersPerSecond         .of(3.5);
    public static final LinearAcceleration MAX_AUTO_ALIGN_ACCELERATION = MetersPerSecondPerSecond.of(8.0);

    public static final Distance TRANSLATION_TOLERANCE = Centimeters.of(2.0);
    public static final Angle    ROTATION_TOLERANCE    = Degrees    .of(1.0);

    public static final Pose2d PROCESSOR_TARGET = new Pose2d(11.568, 7.500, Rotation2d.fromDegrees(-90.000));

    public static final class kReef {
      public static final HashMap<String, Pose2d> TARGETS = new HashMap<>();
      static {
        TARGETS.put("BL", new Pose2d(3.668, 5.428, Rotation2d.fromDegrees(-60.000)));
        TARGETS.put("FL", new Pose2d(5.335, 5.392, Rotation2d.fromDegrees(-120.000)));
        TARGETS.put("F", new Pose2d(6.150, 4.026, Rotation2d.fromDegrees(180.000)));
        TARGETS.put("B", new Pose2d(2.850, 4.026, Rotation2d.fromDegrees(0.000)));
        TARGETS.put("BR", FieldMirror.mirrorPose(TARGETS.get("BL")));
        TARGETS.put("FR", FieldMirror.mirrorPose(TARGETS.get("FL")));
      }

      public static final Transform2d LEFT_OFFSET_TO_BRANCH = new Transform2d(0.315, 0.167, new Rotation2d());
      public static final Transform2d RIGHT_OFFSET_TO_BRANCH = new Transform2d(0.315, -0.167, new Rotation2d());
    }
  }

  public static final class kVision {
    public static final String CAM_NAME = "limelight";

    public static final int FIDUCIAL_TRUST_THRESHOLD = 1;

    /**
     * Frames allowed without latency update before flagged as disconnected
     */
    public static final int DISCONNECTION_TIMEOUT = 5;
  }
}
