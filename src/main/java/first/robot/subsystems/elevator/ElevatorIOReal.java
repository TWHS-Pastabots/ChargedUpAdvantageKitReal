package first.robot.subsystems.elevator;

import static first.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import first.robot.Constants.PivotConstants;

import first.robot.Constants.ElevatorConstants;


import first.robot.util.SparkUtil;

import org.wpilib.math.util.*;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;
import java.util.Queue;
import java.util.function.DoubleSupplier;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.hardware.bus.CANPort;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller, and duty cycle
 * absolute encoder.
 */
public class ElevatorIOReal implements ElevatorIO {
    //private final Rotation2d zeroRotation;

    // Hardware objects
    private final SparkFlex followerSpark;
    private final SparkFlex leaderSpark;

    private final RelativeEncoder relativeEncoder;
    private final ArmFeedforward ff = new ArmFeedforward(PivotConstants.kS, PivotConstants.kG, PivotConstants.kV);

    // Connection debouncers
    private final Debouncer followerConnectedDebounce = new Debouncer(0.5);
    private final Debouncer leaderConnectedDebounce = new Debouncer(0.5);

    private final SparkClosedLoopController leaderClosedLoopController;

    public ElevatorIOReal(){
        leaderSpark = new SparkFlex(CANPort.CAN_D1, ElevatorConstants.leaderCanID, MotorType.kBrushless);
        leaderClosedLoopController = leaderSpark.getClosedLoopController();

        followerSpark = new SparkFlex(CANPort.CAN_D1, ElevatorConstants.followerCanID, MotorType.kBrushless);
        
        relativeEncoder = leaderSpark.getEncoder();

        var leaderConfig = new SparkFlexConfig();

        leaderConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ElevatorConstants.currentLimitAmps)
            .inverted(ElevatorConstants.leaderInverted);
        // leaderConfig
        //     .encoder
        //     .positionConversionFactor(2*Math.PI)
        //     .velocityConversionFactor(2*Math.PI / 60.0)  // Convert RPM to rad/s
        //     .uvwMeasurementPeriod(10)
        //     .uvwAverageDepth(2);
        leaderConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD)
            .outputRange(-1, 1)
            .positionWrappingEnabled(false);
        leaderSpark.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        var followerConfig = new SparkFlexConfig();
        followerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ElevatorConstants.currentLimitAmps)
            .follow(leaderSpark, ElevatorConstants.followerOpposesLeader);
        followerSpark.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Read sensors
        SparkUtil.sparkStickyFault = false;
        SparkUtil.ifOk(leaderSpark, () -> relativeEncoder.getPosition().get(), pos -> inputs.elevatorPosition = new Rotation2d(pos));
        SparkUtil.ifOk(leaderSpark, () -> relativeEncoder.getVelocity().get(), vel -> inputs.elevatorVelocityRadPerSec = vel);
    
        SparkUtil.ifOk(leaderSpark, () -> leaderSpark.getAppliedOutput().get() * leaderSpark.getBusVoltage().get(), volts -> inputs.elevatorAppliedVolts = volts);
        SparkUtil.ifOk(leaderSpark, () -> leaderSpark.getOutputCurrent().get(), amps -> inputs.elevatorLeaderCurrentAmps = amps);
        SparkUtil.ifOk(leaderSpark, () -> leaderSpark.getMotorTemperature().get(), temp -> inputs.elevatorLeaderTempCelsius = temp);

        inputs.elevatorLeaderConnected = leaderConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);
        SparkUtil.sparkStickyFault = false;

    // Follower Motor Inputs
        SparkUtil.ifOk(followerSpark, () -> followerSpark.getOutputCurrent().get(), amps -> inputs.elevatorFollowerCurrentAmps = amps);
        SparkUtil.ifOk(followerSpark, () -> followerSpark.getMotorTemperature().get(), temp -> inputs.elevatorFollowerTempCelsius = temp);

        inputs.elevatorFollowerConnected = followerConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);
    }

    @Override 
    public void setPosition(Rotation2d position) {
        leaderClosedLoopController.setSetpoint(position.getRadians(), ControlType.kPosition, ClosedLoopSlot.kSlot0, ff.calculate(relativeEncoder.getPosition().get(), 0));
    }

    @Override
    public void setVoltage(double volts) {
        leaderSpark.setVoltage(volts);
    }

    @Override
    public void stop() {
        leaderSpark.stopMotor();
        followerSpark.stopMotor();
    }
}