// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package first.robot.subsystems.drive;

import static org.wpilib.units.Units.*;

import static first.robot.Constants.ModuleConstants;
import static first.robot.Constants.ModuleConstants.driveBaseRadius;
import static first.robot.Constants.ModuleConstants.moduleTranslations;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.pathfinding.Pathfinding;
// import com.pathplanner.lib.util.PathPlannerLogging;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.HAL.*;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command3.*;
import org.wpilib.sysid.SysIdRoutineLog;
import first.robot.Constants;
import first.robot.Constants.Mode;
import first.robot.Constants.ModuleConstants;

// import first.robot.subsystems.vision.Vision;
// import first.robot.util.LocalADStarAK;
import java.util.ResourceBundle;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.MatchState;

// public class Drive implements Mechanism , Vision.VisionConsumer {
public class Drive implements Mechanism {
    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final EasySwerveModule[] modules = new EasySwerveModule[4]; // FL, FR, BL, BR
    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected Gyro","Disconnected gyro, using kinematics as fallback.", Alert.Level.HIGH);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
    private Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
    private final SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
    private final Consumer<Pose2d> resetSimulationPoseCallBack;

    public Drive(
            GyroIO gyroIO,
            EasySwerveModuleIO flModuleIO,
            EasySwerveModuleIO frModuleIO,
            EasySwerveModuleIO blModuleIO,
            EasySwerveModuleIO brModuleIO,
            Consumer<Pose2d> resetSimulationPoseCallBack) {
        this.gyroIO = gyroIO;
        this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
        modules[0] = new EasySwerveModule(flModuleIO, 0);
        modules[1] = new EasySwerveModule(frModuleIO, 1);
        modules[2] = new EasySwerveModule(blModuleIO, 2);
        modules[3] = new EasySwerveModule(brModuleIO, 3);

        //(tInstances, tInstances.kRobotDriveSwerve_AdvantageKit);
        HAL.reportUsage("drive", 18, "drive data");

        // Start odometry thread
        SparkOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        // AutoBuilder.configure(
        //         this::getPose,
        //         this::resetOdometry,
        //         this::getChassisVelocities,
        //         this::runVelocity,
        //         new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        //         ppConfig,
        //         () -> MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED,
        //         this);
        // Pathfinding.setPathfinder(new LocalADStarAK());
        // PathPlannerLogging.setLogActivePathCallback((activePath) -> {
        //     Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        // });
        // PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
        //     Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        // });

        // Configure SysId
        // sysId = new SysIdRoutine(
        //         new SysIdRoutine.Config(
        //                 null, null, null, (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        //         new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
        Scheduler.getDefault().addPeriodic(this::periodic);
    }

    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (RobotState.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (RobotState.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleVelocity[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleVelocity[] {});
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distance - lastModulePositions[moduleIndex].distance,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
        
    }

    public void drive(double x, double y, double omega, boolean fieldRelative){

        double xSpeedDelivered = x * ModuleConstants.maxSpeedMetersPerSec;
        double ySpeedDelivered = y * ModuleConstants.maxSpeedMetersPerSec;
        double rotDelivered = omega * ModuleConstants.maxAngularSpeed;

        ChassisVelocities chassisVelocities = new ChassisVelocities(xSpeedDelivered, ySpeedDelivered, rotDelivered);

        SwerveModuleVelocity[] swerveModuleVelocities = ModuleConstants.driveKinematics.toSwerveModuleVelocities(
        fieldRelative
            ? chassisVelocities.toFieldRelative(
              // xSpeedDelivered, ySpeedDelivered, rotDelivered,
                getYaw())
            : new ChassisVelocities(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    swerveModuleVelocities = SwerveDriveKinematics.desaturateWheelVelocities(
        swerveModuleVelocities, ModuleConstants.maxSpeedMetersPerSec);
    modules[0].runSetpoint(swerveModuleVelocities[0]);
    modules[1].runSetpoint(swerveModuleVelocities[1]);
    modules[2].runSetpoint(swerveModuleVelocities[2]);
    modules[3].runSetpoint(swerveModuleVelocities[3]);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisVelocities speeds) {
        // Calculate module setpoints
        speeds = speeds.discretize(0.02);
        SwerveModuleVelocity[] setpointStates = kinematics.toSwerveModuleVelocities(speeds);
        setpointStates = SwerveDriveKinematics.desaturateWheelVelocities(setpointStates, ModuleConstants.maxSpeedMetersPerSec);

        // Log unoptimized setpoints
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisVelocities/Setpoints", speeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisVelocities());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will return to their
     * normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = moduleTranslations[i].getAngle().get();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    // }

    // /** Returns a command to run a dynamic test in the specified direction. */
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    // }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleVelocity[] getModuleStates() {
        SwerveModuleVelocity[] states = new SwerveModuleVelocity[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisVelocities/Measured")
    private ChassisVelocities getChassisVelocities() {
        return kinematics.toChassisVelocities(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rad/sec. */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public Rotation2d getYaw(){
        return gyroInputs.yawPosition;
    }

      public void setX() {
        modules[0].runSetpoint(new SwerveModuleVelocity(0, Rotation2d.fromDegrees(45)));
        modules[1].runSetpoint(new SwerveModuleVelocity(0, Rotation2d.fromDegrees(-45)));
        modules[2].runSetpoint(new SwerveModuleVelocity(0, Rotation2d.fromDegrees(-45)));
        modules[3].runSetpoint(new SwerveModuleVelocity(0, Rotation2d.fromDegrees(45)));
  }

    

    /** Resets the current odometry pose. */
    public void resetOdometry(Pose2d pose) {
        resetSimulationPoseCallBack.accept(pose);
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    // @Override
    // public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
    //     poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    // }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return ModuleConstants.maxSpeedMetersPerSec;
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return ModuleConstants.maxSpeedMetersPerSec / driveBaseRadius;
    }


}