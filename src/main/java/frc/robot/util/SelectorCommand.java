package frc.robot.util;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * A Selector Command will run either command A or command B, if the supplier changes then it will cancel the other command and run the selected one.
 * This command ends when the currently selected command ends
 * @author Alexander Szura Team 5409
 */
public class SelectorCommand extends Command {

    private final Command onTrue;
    private final Command onFalse;
    private final BooleanSupplier condition;

    private Command currentCommand;
    private boolean lastEntry;

    /**
     * Creates a new selector command. This command will run onTrue when the condition is true, when the condition changes the command will change. Ends when the currently running command ends
     * @param onTrue The command to run when true
     * @param onFalse The command to run when false
     * @param condition The condition
     */
    public SelectorCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        this.onTrue = requireNonNullParam(onTrue, "onTrue", "SelectorCommand");
        this.onFalse = requireNonNullParam(onFalse, "onFalse", "SelectorCommand");
        this.condition = requireNonNullParam(condition, "condition", "SelectorCommand");

        CommandScheduler.getInstance().registerComposedCommands(onTrue, onFalse);
        
        addRequirements(onTrue.getRequirements());
        addRequirements(onFalse.getRequirements());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        lastEntry = condition.getAsBoolean();
        if (lastEntry)
            currentCommand = onTrue;
        else
            currentCommand = onFalse;

        currentCommand.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        boolean currentEntry = condition.getAsBoolean();
        if (lastEntry != currentEntry) {
            currentCommand.end(true);
            initialize();
        }

        currentCommand.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        currentCommand.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return currentCommand.isFinished();
    }

}
