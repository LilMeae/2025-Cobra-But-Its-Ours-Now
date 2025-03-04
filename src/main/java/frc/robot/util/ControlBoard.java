package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControlBoard extends CommandGenericHID {

    // TODO: VALIDATE THESE
    public static enum kButton {
        CLOSE_LEFT_LEFT   (0),
        CLOSE_LEFT_RIGHT  (1),
        CLOSE_MIDDLE_LEFT (2),
        CLOSE_MIDDLE_RIGHT(3),
        CLOSE_RIGHT_LEFT  (4),
        CLOSE_RIGHT_RIGHT (5),
        FAR_LEFT_LEFT     (6),
        FAR_LEFT_RIGHT    (7),
        FAR_MIDDLE_LEFT   (8),
        FAR_MIDDLE_RIGHT  (9),
        FAR_RIGHT_LEFT    (10),
        FAR_RIGHT_RIGHT   (11),
        LEVEL_1           (12),
        LEVEL_2           (13),
        LEVEL_3           (14),
        LEVEL_4           (15);

        public final int id;
        private kButton(int id) {
            this.id = id + 1;
        }
    }

    /**
     * Creates a new Control board
     * @param port The driverstation port of the control board
     */
    public ControlBoard(int port) {
        super(port);
    }

    /**
     * Gets the button attached to the button
     * @param button The button to get
     * @return The trigger related to that button
     */
    public Trigger button(kButton button) {
        return super.button(button.id);
    }
}
