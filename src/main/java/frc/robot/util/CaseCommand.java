package frc.robot.util;

import java.util.Arrays;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

/**
 * Case command class, replaces selector command by building using boolean conditional instead of a hashmap.
 * @author Alexander Szura 5409
 */
public class CaseCommand {

    private CaseCommand() {}

    /**
     * Builds a conditional command that takes in multiple conditionals with multiple commands
     * @param conditionals Array of Boolean(Suppliers) conditions
     * @param commands Array of Commands
     * @param otherwise If none of the conditional are matches the fallback command
     * @return A large Conditional Command with all of the conditionals and commands built together
     */
    public static final Command buildCondtional(BooleanSupplier[] conditionals, Command[] commands, Command otherwise) {
        if (conditionals.length != commands.length) throw new IllegalArgumentException("Case Command recieved a different length for conditionals and commands");

        if (conditionals.length == 0) return otherwise;

        return new ConditionalCommand(
            commands[0],
            buildCondtional(
                Arrays.copyOfRange(
                    conditionals,
                    1, conditionals.length
                ),
                Arrays.copyOfRange(
                    commands,
                    1, commands.length
                ),
                otherwise
            ),
            conditionals[0]
        );
    }

    /**
     * Builds a {@link SelectorCommand} that takes in multiple conditionals with multiple commands
     * @param conditionals Array of Boolean(Suppliers) conditions
     * @param commands Array of Commands
     * @param otherwise If none of the conditional are matches the fallback command
     * @return A large {@link SelectorCommand} with all of the conditionals and commands built together
     */
    public static final Command buildSelector(BooleanSupplier[] conditionals, Command[] commands, Command otherwise) {
        if (conditionals.length != commands.length) throw new IllegalArgumentException("Case Command recieved a different length for conditionals and commands");

        if (conditionals.length == 0) return otherwise;

        return new SelectorCommand(
            commands[0],
            buildSelector(
                Arrays.copyOfRange(
                    conditionals,
                    1, conditionals.length
                ),
                Arrays.copyOfRange(
                    commands,
                    1, commands.length
                ),
                otherwise
            ),
            conditionals[0]
        );
    }
}
