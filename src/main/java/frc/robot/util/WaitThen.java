package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class WaitThen extends SequentialCommandGroup {

    /**
     * Waits then runs a command
     * @param waitTime the time to wait in seconds
     * @param command the command to run after sometime
     */
    public WaitThen(double waitTime, Command command) {
        super(new WaitCommand(waitTime), command);
    }
}
