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
//eeee

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.kAutoAlign;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AlignHelper;
import frc.robot.util.ProfiledController;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double TRIGGER_DEADBAND = 0.01;
  private static final double ANGLE_KP = 7.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 1.0; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  public static double speedModifier = 1;

  private static boolean aligned = false;

  private DriveCommands() {}

  public static boolean isAligned() {
    return aligned;
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.clamp(MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND), -1, 1);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  // Increase drive speed
  public static Command setSpeedHigh(Drive drive) {
    return Commands.run(
            () -> {
              speedModifier = 1.0;
            });
  }

  // Decrease drive speed
  public static Command setSpeedLow(Drive drive) {
    return Commands.run(
            () -> {
              speedModifier = 0.5;
            });
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), TRIGGER_DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(Math.pow(Math.abs(omega), 2.0), omega);

          if (Math.abs(omega) >= TRIGGER_DEADBAND)
            omega += Math.copySign(0.05, omega);

          // Convert to field relative speeds & send command

          ChassisSpeeds  speeds = new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec()*speedModifier,
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec()*speedModifier,
                  omega * drive.getMaxAngularSpeedRadPerSec()*speedModifier);
          
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
                // Get linear velocity
                Translation2d linearVelocity =
                    getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());


                // Calculate angular speed
                double omega =
                angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());


              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec()*speedModifier,
                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec()*speedModifier,
                        omega);
              boolean isFlipped =
                    DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        isFlipped
                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                            : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }
    @SuppressWarnings("resource")
    public static Command alignToPoint(Drive drive, Supplier<Pose2d> target, Supplier<LinearVelocity> maxVelo, Supplier<LinearAcceleration> maxAccel) {
        ProfiledController translationController =
            new ProfiledController(
                kAutoAlign.ALIGN_PID,
                maxVelo.get().in(MetersPerSecond),
                maxAccel.get().in(MetersPerSecondPerSecond)
            );

        PIDController angleController =
            new PIDController(
                ANGLE_KP,
                0.0,
                ANGLE_KD
            );
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.sequence(
            Commands.runOnce(() -> {
                aligned = false;

                ChassisSpeeds speeds = drive.getFieldRelativeSpeeds();

                translationController.reset(-Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
                angleController.reset();
            }),
            Commands.run(() -> {
                translationController.setContraints(maxVelo.get().in(MetersPerSecond), maxAccel.get().in(MetersPerSecondPerSecond));
                
                Pose2d robotPose = drive.getPose();
                Pose2d targetPose = target.get();

                if (AutoBuilder.shouldFlip())
                    targetPose = FlippingUtil.flipFieldPose(targetPose);
        
                double xDiff = targetPose.getX() - robotPose.getX();
                double yDiff = targetPose.getY() - robotPose.getY();

                double totalDiff = Math.hypot(xDiff, yDiff);

                double speed = Math.abs(translationController.calculate(totalDiff, 0.0));
        
                double omega =
                angleController.calculate(
                    robotPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
                                                                                           
                double speedX = speed * (xDiff / totalDiff);
                double speedY = speed * (yDiff / totalDiff);
                
                drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, omega, drive.getRotation()));

                Logger.recordOutput("AutoAlign/Target", targetPose);
                Logger.recordOutput("AutoAlign/SpeedOutput", speed);
                Logger.recordOutput("AutoAlign/OmegaOutput", omega);
                Logger.recordOutput("AutoAlign/MaxVelo [m per s]", translationController.getMaxVelocity());
                Logger.recordOutput("AutoAlign/MaxAccel [m per s^2]", translationController.getMaxAcceleration());
            }, drive)
        ).until(() -> {
            Pose2d robotPose = drive.getPose();
            Pose2d targetPose = target.get();
            if (AutoBuilder.shouldFlip())
                targetPose = FlippingUtil.flipFieldPose(targetPose);

            Angle difference = AlignHelper.rotationDifference(targetPose.getRotation(), robotPose.getRotation());

            Distance distance = Meters.of(Math.hypot(robotPose.getX() - targetPose.getX(), robotPose.getY() - targetPose.getY()));

            ChassisSpeeds speeds = drive.getChassisSpeeds();
            LinearVelocity robotSpeed = MetersPerSecond.of(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));

            Logger.recordOutput("AutoAlign/Distance To Alignment [cm]", distance.in(Centimeters));
            Logger.recordOutput("AutoAlign/Angle To Alignment [degrees]", difference.in(Degrees));
            Logger.recordOutput("AutoAlign/Velocity [m per s]", robotSpeed.in(MetersPerSecond));
        
            if (DriverStation.isAutonomous())
                return
                    distance.lte(kAutoAlign.TRANSLATION_TOLERANCE) &&
                    difference.lte(kAutoAlign.ROTATION_TOLERANCE) &&
                    robotSpeed.lte(kAutoAlign.VELOCITY_TOLERANCE);
            else
                return
                    distance.lte(kAutoAlign.TRANSLATION_TOLERANCE) &&
                    difference.lte(kAutoAlign.ROTATION_TOLERANCE) &&
                    robotSpeed.lte(kAutoAlign.AUTO_VELOCITY_TOLERANCE);
        }
    ).andThen(
        Commands.runOnce(() -> {
            drive.stop();
            aligned = true;
        }, drive)
    );
    }

    public static Command alignToPoint(Drive drive, Supplier<Pose2d> target) {
        return alignToPoint(drive, target, () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_FAST, () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_FAST);
    }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();


    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),


        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),


        // Start timer
        Commands.runOnce(timer::restart),


        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)


            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);


                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }


  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();


    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),


            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),


        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),


            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),


            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })


                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;


                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}