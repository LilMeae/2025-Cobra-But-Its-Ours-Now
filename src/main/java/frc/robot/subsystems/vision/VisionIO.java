package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.drive.Drive;

public interface VisionIO {
    @AutoLog
    class VisionInputs {
        public boolean isConnected = false;
        /**
         * Horizontal distance to target
         */
        public double tx = 0;
        /**
         * Vertical distance to target
         */
        public double ty = 0;
        /**
         * How much camera space the target takes up (%)
         */
        public double ta = 0;
        public boolean hasTarget = false;
        public double targetId = 0;

        /**
         * Latency for camera to capture image in milliseconds
         */
        public double imgLatency = 0;
        /**
         * Camera pipeline and processing latency in milliseconds
         */
        public double prxLatency = 0;

        public double fps = 0;
        public double cpuTemp = 0;
        public double ramUsage = 0;
        public double sysTemp = 0;
    }

    default void updateInputs(VisionInputs inputs) {}

    /**
     * Sets the camera offset and return nothing
     */
    default void setCameraOffset() {}

    /**
     * Estimate the current pose using AprilTags
     * @param drive Drive subsystem to get rotation from
     * @return PoseEstimate
     */
    default PoseEstimate estimatePose(Drive drive) {
        return new PoseEstimate();
    }
}
