package frc.robot.commands.scoring;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ScoringLevel;
import frc.robot.Constants.kArmPivot;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.arm.ArmPivot;
import frc.robot.subsystems.collector.EndEffector;

public class ScoreCommand extends SequentialCommandGroup {
    public ScoreCommand(Elevator sys_elevator, ArmPivot sys_pivot, EndEffector sys_score, ScoringLevel level, BooleanSupplier scoring) {
        super(
            sys_pivot.moveArm(kArmPivot.MOVEMENT_SETPOINT),
            sys_elevator.elevatorGo(level.elevatorSetpoint),
            sys_pivot.moveArm(level.pivotAngle),
            Commands.waitUntil(scoring),
            Commands.waitSeconds(0.1).onlyIf(DriverStation::isTeleopEnabled),
            sys_score.runUntilCoralNotDetected(level.voltage)
        );
    }
}
