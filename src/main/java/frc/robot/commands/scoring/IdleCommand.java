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

/**
 * Brings the elevator and arm into intaking position and waiting for a game piece
 * @author Alexander Szura team 5409
 */
public class IdleCommand extends SequentialCommandGroup {

    /**
     * Creates an idle command that runs the end effector with the intaking voltage
     * @param sys_elevator elevator subsystem
     * @param sys_pivot pivot subsystem
     * @param sys_endeffector end effector subsystem
     */
    public IdleCommand(Elevator sys_elevator, ArmPivot sys_pivot, EndEffector sys_endeffector) {
        this(sys_elevator, sys_pivot, sys_endeffector, kEndEffector.IDLE_VOLTAGE);
    }

    /**
     * Creates an idle command
     * @param sys_elevator elevator subsystem
     * @param sys_pivot pivot subsystem
     * @param sys_endeffector end effector subsystem
     * @param endEffectorVoltage the voltage to run the end effector at
     */
    public IdleCommand(Elevator sys_elevator, ArmPivot sys_pivot, EndEffector sys_endeffector, double endEffectorVoltage) {
        super(
            Commands.waitUntil(Drive::isSafe),
            sys_pivot.moveArm(kArmPivot.MOVEMENT_SETPOINT),
            sys_elevator.elevatorGo(kElevator.IDLING_HEIGHT),
            Commands.deadline(
                sys_endeffector.runUntilCoralDetected(endEffectorVoltage),
                sys_pivot.moveArm(kArmPivot.PICKUP_ANGLE)
            ),
            sys_pivot.moveArm(kArmPivot.MOVEMENT_SETPOINT)
        );
    }
}
