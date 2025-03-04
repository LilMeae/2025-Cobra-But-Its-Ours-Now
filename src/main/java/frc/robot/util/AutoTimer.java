package frc.robot.util;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoTimer {
    private static long startTime;
    
    private AutoTimer() {}

    public static Command start() {
        return Commands.runOnce(() -> startTime = System.currentTimeMillis());
    }

    public static Command end(boolean print) {
        return Commands.runOnce(() -> {
            double time = (System.currentTimeMillis() - startTime) / 1000.0;

            if (print)
                System.out.println("Auto Took: " + time + " Seconds!");

            Logger.recordOutput("Auto Time", time);
        });
    }
}
