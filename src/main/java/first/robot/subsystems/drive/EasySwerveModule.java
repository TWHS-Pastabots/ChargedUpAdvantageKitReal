// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;
import org.wpilib.util.Alert;
import org.wpilib.command3.Scheduler;
//import org.wpilib.hardware.hal.AlertJNI;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.*;


import first.robot.subsystems.drive.EasySwerveModuleIO.EasySwerveModuleIOInputs;

import  first.robot.subsystems.drive.EasySwerveModuleIO.EasySwerveModuleIOInputs;

public class EasySwerveModule {
    private final EasySwerveModuleIO io;
    private final EasySwerveModuleIOInputsAutoLogged inputs = new EasySwerveModuleIOInputsAutoLogged();
    private final int index;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public EasySwerveModule(EasySwerveModuleIO io, int index) {
        this.io = io;
        this.index = index;
        driveDisconnectedAlert =
                new Alert("Drive Disconnect","Disconnected drive motor on module " + Integer.toString(index) + ".", Alert.Level.HIGH);
        turnDisconnectedAlert =
                new Alert("Turn Disconnect","Disconnected turn motor on module " + Integer.toString(index) + ".", Alert.Level.HIGH);
        Scheduler.getDefault().addPeriodic(this::periodic);
    }


  /** Call this periodically from the Drive subsystem's periodic() method. */
public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * .0508;
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
        
    }


    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
    public void runSetpoint(SwerveModuleVelocity state) {
        // Optimize velocity setpoint
        state = state.optimize(getAngle());
        state = state.cosineScale(new Rotation2d(inputs.turnAbsoluteRotation));

        // Apply setpoints
        io.setDriveVelocity(state.velocity / .0508);
        io.setTurnPosition(state.angle);
    }

    /** Runs the module with the specified output while controlling to zero degrees. */
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(new Rotation2d());
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setTurnOpenLoop(0.0);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return new Rotation2d(inputs.turnAbsoluteRotation);
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return inputs.drivePosition * .0508;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocity * .0508;
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleVelocity getState() {
        return new SwerveModuleVelocity(getVelocityMetersPerSec(), getAngle());
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePosition;
    }

    public double getFFCharacterizationVelocity() {
        return inputs.driveVelocity;
    }

    public double getDriveTemp(){
        return inputs.drivePosition;
    }

    public double getTurnTemp(){
      return inputs.turnTemperature;
    }
}
