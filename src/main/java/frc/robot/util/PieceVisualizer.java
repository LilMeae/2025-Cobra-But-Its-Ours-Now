package frc.robot.util;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Robot;

public class PieceVisualizer {
    public static class GamePiece {
        private static int pieceId = 0;

        private Pose3d pose;

        private boolean attached;
        private Supplier<Transform3d> pieceOffset;
        private Supplier<Pose2d> robotPose;

        private StructPublisher<Pose3d> posePublisher;

        private GamePiece(Supplier<Transform3d> pieceOffset, Supplier<Pose2d> robotPose) {
            this.pieceOffset = pieceOffset;
            this.robotPose = robotPose;

            this.pose = new Pose3d(robotPose.get()).transformBy(pieceOffset.get());

            this.posePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("Coral-" + (pieceId++), Pose3d.struct).publish();

            attached = true;
        }

        public void updatePose() {
            if (!attached) return;

            this.pose = new Pose3d(robotPose.get()).transformBy(pieceOffset.get());

            posePublisher.accept(this.pose);
        }

        public Pose3d getPose() {
            return pose;
        }

        public void attach() {
            attached = true;
        }

        public void detach() {
            attached = false;
        }
    }

    private static Supplier<Pose2d> robotPose = null;
    private static ArrayList<GamePiece> gamePieces;

    /**
     * Confirues and sets up the Game Piece Visualizer.
     * @param robotPose The robots pose
     */
    public static void configure(Supplier<Pose2d> robotPose) {
        PieceVisualizer.robotPose = robotPose;
        PieceVisualizer.gamePieces = new ArrayList<>();
    }

    /**
     * Adds a Coral to NetworkTables for simulation purposes
     * @param pieceOffset The Supplier for the game piece
     * @return An {@link Optional} Game Piece if the robot is simulated, otherwise empty
     */
    public static Optional<GamePiece> addGamePiece(Supplier<Transform3d> pieceOffset) {
        if (Robot.isReal()) return Optional.empty();

        if (robotPose == null) throw new RuntimeException("CoralVisulizer was not configured!");

        GamePiece piece = new GamePiece(pieceOffset, robotPose);

        gamePieces.add(piece);

        return Optional.of(piece);
    }

    /**
     * Updates the pose of all of the game pieces
     */
    public static void update() {
        for (GamePiece piece : gamePieces)
            piece.updatePose();
    }
}
