package first.robot.subsystems.elevator;
 
import org.wpilib.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
 
public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public boolean elevatorLeaderConnected = false;
        public boolean elevatorFollowerConnected = false;
 
        // Position/velocity, from the through-bore absolute encoder
        public Rotation2d elevatorPosition = new Rotation2d();
        public double elevatorVelocityRadPerSec = 0.0;
 
        // Electrical, for logging/diagnostics
        public double elevatorAppliedVolts = 0.0;
        public double elevatorLeaderCurrentAmps = 0.0;
        public double elevatorFollowerCurrentAmps = 0.0;
        public double elevatorLeaderTempCelsius = 0.0;
        public double elevatorFollowerTempCelsius = 0.0;
    }
 
    default void updateInputs(ElevatorIOInputs inputs) {}
 
    /** Runs closed-loop control to the specified position, using the onboard Spark Flex PID controller. */
    default void setPosition(Rotation2d position) {}
 
    /** Runs the elevator at the specified open-loop voltage (bypasses closed-loop control). */
    default void setVoltage(double volts) {}
 
    /** Stops the elevator. */
    default void stop() {}
 
    /** Sets the onboard PID gains. */
    default void setPID(double kP, double kI, double kD) {}
 
    /** Enables or disables brake mode on both motors. */
    default void setBrakeMode(boolean enabled) {}
}