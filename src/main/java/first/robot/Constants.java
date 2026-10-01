// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package first.robot;

import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.Units;


/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class ModuleConstants {
    // The EasySwerve module can only be configured with one pinion gears: 12T.
    public static final int kDrivingMotorPinionTeeth = 12;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 30 teeth on the first-stage spur gear,
    // 15 teeth on the bevel pinion
    public static final double kDrivingWheelBevelGearTeeth = 45.0;
    public static final double kDrivingWheelFirstStageSpurGearTeeth = 30.0;
    public static final double kDrivingMotorBevelPinionTeeth = 15.0;
    public static final double kDrivingMotorReduction =
        (kDrivingWheelBevelGearTeeth * kDrivingWheelFirstStageSpurGearTeeth)
            / (kDrivingMotorPinionTeeth * kDrivingMotorBevelPinionTeeth);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    
    public static final double maxSpeedMetersPerSec = 4.8;
    public static final double maxAngularSpeed = 2 * Math.PI;
    public static final double odometryFrequency = 100.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(30);
    public static final double wheelBase = Units.inchesToMeters(30);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
    };

      public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2, trackWidth / 2),
        new Translation2d(wheelBase / 2, -trackWidth / 2),
        new Translation2d(-wheelBase / 2, trackWidth / 2),
        new Translation2d(-wheelBase / 2, -trackWidth / 2)
      );
        

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

    // Device CAN IDs
    public static final int pigeonCanId = 9;

    public static final int frontLeftDriveCanId = 1;
    public static final int backLeftDriveCanId = 3;
    public static final int frontRightDriveCanId = 5;
    public static final int backRightDriveCanId = 7;

    public static final int frontLeftTurnCanId = 2;
    public static final int backLeftTurnCanId = 4;
    public static final int frontRightTurnCanId = 6;
    public static final int backRightTurnCanId = 8;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 60;
    public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
    public static final double driveMotorReduction =
            (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth and 22 spur teeth
    public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor =
            2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor =
            (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    public static final double driveKp = 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.1;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;


    //Change this
    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 60;
    public static final double turnMotorReduction = 9424.0 / 203.0;
    public static final DCMotor turnGearbox = DCMotor.getNeoVortex(1);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double turnKp = 2.0;
    public static final double turnKd = 0.0;
    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // // PathPlanner configuration
    // public static final double robotMassKg = 45;
    // public static final double robotMOI = 6.883;
    // public static final double wheelCOF = 1.2;
    // public static final RobotConfig ppConfig = new RobotConfig(
    //         robotMassKg,
    //         robotMOI,
    //         new ModuleConfig(
    //                 wheelRadiusMeters,
    //                 maxSpeedMetersPerSec,
    //                 wheelCOF,
    //                 driveGearbox.withReduction(driveMotorReduction),
    //                 driveMotorCurrentLimit,
    //                 1),
    //         moduleTranslations);

//     public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
//             .withCustomModuleTranslations(moduleTranslations)
//             .withRobotMass(Kilogram.of(robotMassKg))
//             .withGyro(COTS.ofPigeon2())
//             .withSwerveModule(new SwerveModuleSimulationConfig(
//                     driveGearbox,
//                     turnGearbox,
//                     driveMotorReduction,
//                     turnMotorReduction,
//                     Volts.of(0.1),
//                     Volts.of(0.1),
//                     Meters.of(wheelRadiusMeters),
//                     KilogramSquareMeters.of(0.02),
//                     wheelCOF));
  }

  
public static final class ClawConstants {
    public static final int tiltCanID = 7;
    public static final int intakeCanID = 8;
    public static final boolean tiltInverted = false;
    public static final boolean intakeInverted = true;
    public static final double encoderZeroOffset = 0;
    public static final int currentLimitAmps = 60;
    public static final double kP = 0.0; // TODO: tune
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double minAngle = 0;
    public static final double maxAngle = Math.PI;
    public static final double kS = 0.0; // TODO: tune
    public static final double kG = 0.0; // TODO: tune
    public static final double kV = 0.0; // TODO: tune

    public enum TiltState{
          TRANSITION(Rotation2d.fromDegrees(0)),
          CONESCORE(Rotation2d.fromDegrees(0)),
          CUBESCORE(Rotation2d.fromDegrees(0)),
          CONEINTAKEFRONT(Rotation2d.fromDegrees(0)),
          CONEINTAKEBACK(Rotation2d.fromDegrees(0)),
          CUBEINTAKEFRONT(Rotation2d.fromDegrees(0)),
          CUBEINTAKEBACK(Rotation2d.fromDegrees(0));

          public final Rotation2d angle;

          TiltState(Rotation2d angle){
              this.angle = angle;
          }
  }
}

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class PivotConstants {
      public static final int leaderCanID = 5;
      public static final int followerCanID = 6;
      public static final boolean leaderInverted = false;
      public static final boolean followerInverted = true;
      public static final boolean followerOpposesLeader = true;
      public static final boolean encoderInverted = false;
      public static final double encoderZeroOffset = 0;
      public static final int currentLimitAmps = 60;
      public static final double kP = 0.0; // TODO: tune
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double minAngle = 0;
      public static final double maxAngle = Math.PI;
      public static final double kS = 0.0; // TODO: tune
      public static final double kG = 0.0; // TODO: tune
      public static final double kV = 0.0; // TODO: tune

      public enum PivotState{
          TRANSITION(Rotation2d.fromDegrees(0)),
          CONESCORE(Rotation2d.fromDegrees(0)),
          CUBESCORE(Rotation2d.fromDegrees(0)),
          CONEINTAKEFRONT(Rotation2d.fromDegrees(0)),
          CONEINTAKEBACK(Rotation2d.fromDegrees(0)),
          CUBEINTAKEFRONT(Rotation2d.fromDegrees(0)),
          CUBEINTAKEBACK(Rotation2d.fromDegrees(0));

          public final Rotation2d angle;

          PivotState(Rotation2d angle){
              this.angle = angle;
          }
      }
  }
    public static final class ElevatorConstants {
      public static final int leaderCanID = 5;
      public static final int followerCanID = 6;
      public static final boolean leaderInverted = false;
      public static final boolean followerInverted = true;
      public static final boolean followerOpposesLeader = true;
      public static final boolean encoderInverted = false;
      public static final double encoderZeroOffset = 0;
      public static final int currentLimitAmps = 60;
      public static final double kP = 0.0; // TODO: tune
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double minAngle = 0;
      public static final double maxAngle = Math.PI;
      public static final double kS = 0.0; // TODO: tune
      public static final double kG = 0.0; // TODO: tune
      public static final double kV = 0.0; // TODO: tune

      public enum ElevatorState{
          TRANSITION(Rotation2d.fromDegrees(0)),
          CONESCORE(Rotation2d.fromDegrees(0)),
          CUBESCORE(Rotation2d.fromDegrees(0)),
          CONEINTAKE(Rotation2d.fromDegrees(0)),
          CUBEINTAKE(Rotation2d.fromDegrees(0));

          public final Rotation2d angle;

          ElevatorState(Rotation2d angle){
              this.angle = angle;
          }
      }
  }

}

