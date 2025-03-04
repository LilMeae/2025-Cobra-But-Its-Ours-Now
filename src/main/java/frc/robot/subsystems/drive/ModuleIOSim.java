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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.generated.TunerConstants;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim implements ModuleIO {
  // TunerConstants doesn't support separate sim constants, so they are declared locally
  private static final double DRIVE_KP = 0.05;
  private static final double DRIVE_KD = 0.0;
  
  private static final double TURN_KP = 8.0;
  private static final double TURN_KD = 0.0;

  private final SwerveModuleSimulation moduleSim;
  private final GenericMotorController driveMotor, turnMotor;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
    private PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;
    private double desiredWheelVelocityRadPerSec = 0.0;
    private Rotation2d desiredSteerFacing = new Rotation2d();

    public ModuleIOSim(SwerveModuleSimulation moduleSim) {
        this.moduleSim = moduleSim;

        this.driveMotor = 
                moduleSim.useGenericMotorControllerForDrive()
                .withCurrentLimit(TunerConstants.kSlipCurrent);

        this.turnMotor = 
                moduleSim.useGenericControllerForSteer()
                .withCurrentLimit(TunerConstants.steerInitialConfigs.CurrentLimits.getStatorCurrentLimitMeasure());

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI);
        SimulatedArena.getInstance().addCustomSimulation((delta) -> runControlLoops(delta));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update drive inputs
        inputs.driveConnected = true;
        inputs.drivePositionRad = moduleSim.getDriveWheelFinalPosition().in(Radians);
        inputs.driveVelocityRadPerSec = moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond);
        inputs.driveAppliedVolts = moduleSim.getDriveMotorAppliedVoltage().in(Volts);
        inputs.driveCurrentAmps = Math.abs(moduleSim.getDriveMotorStatorCurrent().in(Amps));

        // Update turn inputs
        inputs.turnConnected = true;
        inputs.turnEncoderConnected = true;
        inputs.turnAbsolutePosition = moduleSim.getSteerAbsoluteFacing();
        inputs.turnPosition = moduleSim.getSteerAbsoluteFacing();
        inputs.turnVelocityRadPerSec = moduleSim.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
        inputs.turnAppliedVolts = moduleSim.getSteerMotorAppliedVoltage().in(Volts);
        inputs.turnCurrentAmps = Math.abs(moduleSim.getSteerMotorStatorCurrent().in(Amps));

        inputs.magnetHealth = MagnetHealthValue.Magnet_Green;

        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
        inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
        inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
        inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
    }

    public void runControlLoops(double delta) {
        // Run control loops if activated
        if (driveClosedLoop) calculateDriveControlLoops();
        else driveController.reset();
        if (turnClosedLoop) calculateSteerControlLoops();
        else turnController.reset();

        // Feed voltage to motor simulation
        driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
        turnMotor.requestVoltage(Volts.of(turnAppliedVolts));
    }

    public void calculateDriveControlLoops() {
        DCMotor motor = moduleSim.config.driveMotorConfigs.motor;

        double frictionTorque =
            motor.getTorque(motor.getCurrent(0, TunerConstants.FrontLeft.DriveFrictionVoltage))
                        * Math.signum(desiredWheelVelocityRadPerSec);

        double velocityFeedforwardVolts = motor.getVoltage(
                frictionTorque, desiredWheelVelocityRadPerSec * moduleSim.config.DRIVE_GEAR_RATIO);

        double feedBackVolts = driveController.calculate(
                moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond), desiredWheelVelocityRadPerSec);

        driveAppliedVolts = velocityFeedforwardVolts + feedBackVolts;
    }

    public void calculateSteerControlLoops() {
        turnAppliedVolts = turnController.calculate(
                moduleSim.getSteerAbsoluteFacing().getRadians(), desiredSteerFacing.getRadians());
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        driveClosedLoop = true;
        desiredWheelVelocityRadPerSec = velocityRadPerSec;
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        desiredSteerFacing = rotation;
    }
}
