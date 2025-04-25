package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ScoringLevel;
import frc.robot.Constants.kArmPivot;
import frc.robot.Constants.kEndEffector;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.arm.ArmPivot;
import frc.robot.subsystems.collector.EndEffector;

/**
 * Brings the elevator and arm into position for removing algae, spins the end effector at algae removing speeds
 * @author Alexander Szura team 5409
 */
public class RemoveAlgae extends SequentialCommandGroup {

    /**
     * Remove algae command
     * @param sys_elevator elevator subsystem
     * @param sys_arm arm subsystem
     * @param sys_endEffector end effector subsystem
     * @param level The level to remove algae at
     */
    public RemoveAlgae(Elevator sys_elevator, ArmPivot sys_arm, EndEffector sys_endEffector, ScoringLevel level) {
        super(
            sys_arm.moveArm(kArmPivot.MOVEMENT_SETPOINT),
            Commands.parallel(
                sys_elevator.elevatorGo(level.elevatorSetpoint),
                sys_arm.moveArm(level.pivotAngle),
                sys_endEffector.setVoltage(kEndEffector.ALGAE_VOLTAGE)
            )
        );
    }
}
