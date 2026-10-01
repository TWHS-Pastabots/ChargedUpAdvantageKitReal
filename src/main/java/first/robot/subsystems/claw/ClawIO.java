package first.robot.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

import org.wpilib.math.geometry.Rotation2d;

public interface ClawIO {
    @AutoLog
    public static class ClawIOInputs {
        public boolean intakeConnected = false;
        public Rotation2d intakePosition = new Rotation2d();
        public double intakeVelocity = 0;
        public double intakeAppliedVoltage = 0;
        public double intakeCurrentDraw = 0;
        public double intakeTemperature = 0;
        public double intakeBusVoltage = 0;
    }
    
    default void updateInputs(ClawIOInputs inputs) {}
 
    /** Runs closed-loop control to the specified position, using the onboard Spark Flex PID controller. */
    default void setPosition(Rotation2d position) {}
 
    /** Runs the pivot at the specified open-loop voltage (bypasses closed-loop control). */
    default void setVoltage(double volts) {}

    default void setPower(double power) {}
 
    /** Stops the pivot. */
    default void stop() {}
}
