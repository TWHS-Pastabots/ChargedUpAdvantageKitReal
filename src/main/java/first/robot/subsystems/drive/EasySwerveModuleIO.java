package first.robot.subsystems.drive;

import static org.wpilib.units.Units.*;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.*;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/** Interface to define common swerve module input/output functionality. */
public interface EasySwerveModuleIO {
  @AutoLog
  public static class EasySwerveModuleIOInputs {
    public double drivePosition = 0.0;
    public double driveVelocity = 0.0;
    public double driveAppliedVoltage = 0.0;
    public double driveCurrentAmps = 0.0;
    public double driveTemperature = 0.0;
    public boolean driveConnected = false;

    public double turnAbsoluteRotation = 0.0;
    public double turnAngularVelocity = 0.0;
    public double turnAppliedVoltage = 0.0;
    public double turnCurrentAmps = 0.0;
    public double turnTemperature = 0.0;
    public boolean turnConnected = false;

    
    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /**
   * Updates the system with new inputs from the module.
   *
   * @param inputs The new inputs.
   */
  public default void updateInputs(EasySwerveModuleIOInputs inputs) {}

  /**
   * Sets the target drive velocity and acceleration for the module.
   *
   * @param velocity target drive velocity in m/s.
   * @param acceleration target drive acceleration in m/s^2.
   */
  public default void setDriveState(double velocity, double acceleration) {}

  /**
   * Sets the target turn position and velocity for the module.
   *
   * @param rotation target turn position in radians.
   * @param angularVelocity target turn velocity in radians per second.
   */
  public default void setTurnState(double rotation, double angularVelocity) {}

  /**
   * Sets the drive output to a voltage.
   *
   * @param voltage The output voltage.
   */
  public default void setDriveOpenLoop(double voltage) {}

  /**
   * Sets the turn output to a voltage.
   *
   * @param voltage The output voltage.
   */
  public default void setTurnOpenLoop(double voltage) {}

  /**
   * Set the current limit for the drive motors
   *
   * @param currentLimit The current limit in amps
   */
  public default void setDriveCurrentLimit(int currentLimit) {}

  public default void setDriveVelocity(double velocityRadPerSec){}

  public default void setTurnPosition(Rotation2d rotation){}

  public default void setDesiredState(SwerveModuleVelocity desiredState){}

  //void resetEncoders();
}
