package first.robot.subsystems.pivot;
 
import org.wpilib.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
 
public interface PivotIO {
    @AutoLog
    class PivotIOInputs {
        public boolean leaderConnected = false;
        public boolean followerConnected = false;
 
        // Position/velocity, from the through-bore absolute encoder
        public Rotation2d position = new Rotation2d();
        public double velocityRadPerSec = 0.0;
 
        // Electrical, for logging/diagnostics
        public double appliedVolts = 0.0;
        public double leaderCurrentAmps = 0.0;
        public double followerCurrentAmps = 0.0;
        public double leaderTempCelsius = 0.0;
        public double followerTempCelsius = 0.0;
    }
 
    default void updateInputs(PivotIOInputs inputs) {}
 
    /** Runs closed-loop control to the specified position, using the onboard Spark Flex PID controller. */
    default void setPosition(Rotation2d position) {}
 
    /** Runs the pivot at the specified open-loop voltage (bypasses closed-loop control). */
    default void setVoltage(double volts) {}
 
    /** Stops the pivot. */
    default void stop() {}
 
    /** Sets the onboard PID gains. */
    default void setPID(double kP, double kI, double kD) {}
 
    /** Enables or disables brake mode on both motors. */
    default void setBrakeMode(boolean enabled) {}
}


