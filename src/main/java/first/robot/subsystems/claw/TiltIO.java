package first.robot.subsystems.claw;

import org.wpilib.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TiltIO {
    @AutoLog
    public static class TiltIOInputs {
        public boolean tiltConnected = false;
        public Rotation2d tiltPosition = new Rotation2d();
        public double tiltVelocity = 0;
        public double tiltAppliedVoltage = 0;
        public double tiltCurrentDraw = 0;
        public double tiltTemperature = 0;
        public double tiltBusVoltage = 0;
    }
    
    default void updateInputs(TiltIOInputs inputs) {}
 
    /** Runs closed-loop control to the specified position, using the onboard Spark Flex PID controller. */
    default void setPosition(Rotation2d position) {}
 
    /** Runs the pivot at the specified open-loop voltage (bypasses closed-loop control). */
    default void setVoltage(double volts) {}
 
    /** Stops the pivot. */
    default void stop() {}
}

