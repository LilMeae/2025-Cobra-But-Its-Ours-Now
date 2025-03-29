package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ScoringLevel;
import frc.robot.Constants.kArmPivot;
import frc.robot.Constants.kEndEffector;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.arm.ArmPivot;
import frc.robot.subsystems.collector.EndEffector;
import frc.robot.util.WaitThen;

public class RemoveAlgae extends SequentialCommandGroup {

    public RemoveAlgae(Elevator sys_elevator, ArmPivot sys_arm, EndEffector sys_endEffector, ScoringLevel level) {
        super(
            sys_arm.moveArm(kArmPivot.MOVEMENT_SETPOINT),
            Commands.parallel(
                sys_elevator.elevatorGo(level.elevatorSetpoint),
                new WaitThen(
                    () -> DriveCommands.isAligned(),
                    sys_arm.moveArm(level.pivotAngle)
                ),
                sys_endEffector.setVoltage(kEndEffector.ALGAE_VOLTAGE)
            )
        );
    }
}
