package frc.robot.commands.scoring;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ScoringLevel;
import frc.robot.Constants.kArmPivot;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.arm.ArmPivot;
import frc.robot.subsystems.collector.EndEffector;

/**
 * A score command that gets ready to score coral at a specified location
 * @author Alexander Szura team 5409
 */
public class ScoreCommand extends SequentialCommandGroup {

    /**
     * Creates score command
     * @param sys_elevator elevator subsystem
     * @param sys_pivot pivot subsystem
     * @param sys_score end effector subsystem
     * @param level The level to score on
     * @param scoring when the robot should score
     */
    public ScoreCommand(Elevator sys_elevator, ArmPivot sys_pivot, EndEffector sys_score, ScoringLevel level, BooleanSupplier scoring) {
        super(
            sys_pivot.moveArm(kArmPivot.MOVEMENT_SETPOINT),
            sys_elevator.elevatorGo(level.elevatorSetpoint),
            sys_pivot.moveArm(level.pivotAngle),
            Commands.waitUntil(scoring),
            Commands.either(
                sys_score.runUntilCoralNotDetected(level.voltage),
                sys_score.setVoltage(level.voltage).withTimeout(0.25),
                sys_score::coralDetected
            )
        );
    }

    /**
     * A score command with no scoring, gets the robot in position to score
     * @param sys_elevator elevator subsystem
     * @param sys_pivot pivot subsystem
     * @param level the level to score on
     */
    public ScoreCommand(Elevator sys_elevator, ArmPivot sys_pivot, ScoringLevel level) {
        super(
            sys_pivot.moveArm(kArmPivot.MOVEMENT_SETPOINT),
            sys_elevator.elevatorGo(level.elevatorSetpoint),
            sys_pivot.moveArm(level.pivotAngle)
        );
    }
}
