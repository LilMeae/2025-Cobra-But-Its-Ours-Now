package frc.robot.util;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoTimer {
    private static long startTime = -1;
    
    private AutoTimer() {}

    /**
     * Starts the timer for auto
     * @return An instant command that starts the timer
     */
    public static Command start() {
        return Commands.runOnce(() -> startTime = System.currentTimeMillis());
    }

    /**
     * Ends the timer a logs it
     * @param print if this command should print auto time to the console
     * @return An instant command that does the above
     */
    public static Command end(boolean print) {
        return Commands.runOnce(() -> {
            if (startTime == -1) return;

            double time = (System.currentTimeMillis() - startTime) / 1000.0;

            if (print)
                System.out.println("Auto Took: " + time + " Seconds!");

            Logger.recordOutput("Auto Time", time);
        });
    }
}
