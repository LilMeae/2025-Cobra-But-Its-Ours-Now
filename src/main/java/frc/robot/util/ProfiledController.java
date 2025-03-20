package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Time;

public class ProfiledController {

    private final PIDController controller;
    private double maxVelocity;
    private double maxAcceleration;

    private double lastPoint;

    public ProfiledController(PIDConstants PID, double maxVelocity, double maxAcceleration, Time period) {
        controller = new PIDController(PID.kP, PID.kI, PID.kD, period.in(Seconds));
        controller.setIZone(PID.iZone);

        setContraints(maxVelocity, maxAcceleration);
    }
    

    public ProfiledController(PIDConstants PID, double maxVelocity, double maxAcceleration) {
        this(PID, maxVelocity, maxAcceleration, Seconds.of(0.02));
    }

    public ProfiledController(double kP, double kI, double kD, double maxVelocity, double maxAcceleration) {
        this(new PIDConstants(kP, kI, kD), maxVelocity, maxAcceleration);
    }

    public void setContraints(double velo, double accel) {
        setMaxVelocity(velo);
        setMaxAcceleration(accel);
    }

    public void setMaxVelocity(double velo) {
        maxVelocity = Math.max(velo, 0.0);
    }

    public void setMaxAcceleration(double accel) {
        maxAcceleration = Math.max(accel, 0.0);
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public void reset() {
        reset(0.0);
    }

    public void reset(double velocity) {
        controller.reset();
        lastPoint = velocity;
    }

    public double calculate(double measurement, double setpoint) {
        double dt = controller.getPeriod();
    
        double output = controller.calculate(measurement, setpoint);
    
        output = Math.max(-maxVelocity, Math.min(maxVelocity, output));
    
        double deltaVelocity = output - lastPoint;
        double constrainedVelocity = lastPoint + Math.max(-maxAcceleration * dt, Math.min(maxAcceleration * dt, deltaVelocity));
    
        lastPoint = constrainedVelocity;
        
        return constrainedVelocity;
    }
    
}
