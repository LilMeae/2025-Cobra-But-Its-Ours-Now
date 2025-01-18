package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionInputs {
        public double tx = 0;
        public double ty = 0;
        public double ta = 0;
        public boolean hasTarget = false;
        public double targetId = 0;
    }

    public default void updateInputs(VisionInputs inputs) {
    }
}
