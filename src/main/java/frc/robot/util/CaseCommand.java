package frc.robot.util;

import java.util.Arrays;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class CaseCommand {
    private CaseCommand() {}

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
