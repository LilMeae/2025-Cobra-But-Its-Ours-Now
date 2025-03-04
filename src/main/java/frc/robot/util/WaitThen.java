package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class WaitThen extends SequentialCommandGroup {

    /**
     * Waits then runs a command
     * @param waitTime the time to wait in seconds
     * @param command the command to run after sometime
     */
    public WaitThen(double waitTime, Command command) {
        this(Seconds.of(waitTime), command);
    }

    /**
     * Waits then runs a command
     * @param waitTime the time to wait
     * @param command the command to run after sometime
     */
    public WaitThen(Time waitTime, Command command) {
        super(new WaitCommand(waitTime), command);
    }

    /**
     * Waits until a condition is met then runs a command
     * @param conditionalSupplier the condition to be satisfied
     * @param command the command to run after sometime
     */
    public WaitThen(BooleanSupplier conditionalSupplier, Command command) {
        super(new WaitUntilCommand(conditionalSupplier), command);
    }
}
