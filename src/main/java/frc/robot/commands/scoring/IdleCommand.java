package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kArmPivot;
import frc.robot.Constants.kElevator;
import frc.robot.Constants.kEndEffector;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.arm.ArmPivot;
import frc.robot.subsystems.collector.EndEffector;
import frc.robot.subsystems.drive.Drive;

public class IdleCommand extends SequentialCommandGroup {
    public IdleCommand(Elevator sys_elevator, ArmPivot sys_pivot, EndEffector sys_endeffector) {
        this(sys_elevator, sys_pivot, sys_endeffector, kEndEffector.IDLE_VOLTAGE);
    }

    public IdleCommand(Elevator sys_elevator, ArmPivot sys_pivot, EndEffector sys_endeffector, double endEffectorVoltage) {
        super(
            sys_pivot.moveArm(kArmPivot.MOVEMENT_SETPOINT),
            Commands.waitUntil(Drive::isSafe),
            sys_elevator.elevatorGo(kElevator.IDLING_HEIGHT),
            Commands.deadline(
                sys_endeffector.runUntilCoralDetected(endEffectorVoltage),
                sys_pivot.moveArm(kArmPivot.PICKUP_ANGLE)
            ),
            sys_pivot.moveArm(kArmPivot.MOVEMENT_SETPOINT)
        );
    }
}
