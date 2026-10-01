package first.robot.subsystems.claw;

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

import first.robot.Constants.ClawConstants;
import first.robot.Constants.PivotConstants;

import first.robot.util.SparkUtil;

import org.wpilib.math.util.*;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;
import java.util.Queue;
import java.util.function.DoubleSupplier;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.ArmFeedforward;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller, and duty cycle
 * absolute encoder.
 */
public class TiltIOReal implements TiltIO {
    //private final Rotation2d zeroRotation;

    // Hardware objects
    private final SparkFlex tiltSpark;

    private final SparkAbsoluteEncoder tiltEncoder;
    private final ArmFeedforward ff = new ArmFeedforward(PivotConstants.kS, PivotConstants.kG, PivotConstants.kV);

    // Connection debouncers
    private final Debouncer followerConnectedDebounce = new Debouncer(0.5);
    private final Debouncer leaderConnectedDebounce = new Debouncer(0.5);

    private final SparkClosedLoopController leaderClosedLoopController;

    public TiltIOReal(){
        tiltSpark = new SparkFlex(CANPort.CAN_D1, ClawConstants.tiltCanID, MotorType.kBrushless);
        leaderClosedLoopController = tiltSpark.getClosedLoopController();
        
        tiltEncoder = tiltSpark.getAbsoluteEncoder();

        var tiltConfig = new SparkFlexConfig();

        tiltConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ClawConstants.currentLimitAmps)
            .inverted(ClawConstants.tiltInverted);
        // tiltConfig
        //     .encoder
        //     .positionConversionFactor(2*Math.PI)
        //     .velocityConversionFactor(2*Math.PI / 60.0)  // Convert RPM to rad/s
        //     .uvwMeasurementPeriod(10)
        //     .uvwAverageDepth(2);
        tiltConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(ClawConstants.kP, ClawConstants.kI, ClawConstants.kD)
            .outputRange(-1, 1)
            .positionWrappingEnabled(false);
        tiltSpark.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    @Override
    public void updateInputs(TiltIOInputs inputs) {
        // Read sensors
        SparkUtil.sparkStickyFault = false;
        SparkUtil.ifOk(tiltSpark, () -> tiltEncoder.getPosition().get(), pos -> inputs.tiltPosition = new Rotation2d(pos));
        SparkUtil.ifOk(tiltSpark, () -> tiltEncoder.getVelocity().get(), vel -> inputs.tiltVelocity = vel);

        SparkUtil.ifOk(tiltSpark, () -> tiltSpark.getAppliedOutput().get() * tiltSpark.getBusVoltage().get(), volts -> inputs.tiltAppliedVoltage = volts);
        SparkUtil.ifOk(tiltSpark, () -> tiltSpark.getOutputCurrent().get(), amps -> inputs.tiltCurrentDraw = amps);
        SparkUtil.ifOk(tiltSpark, () -> tiltSpark.getMotorTemperature().get(), temp -> inputs.tiltTemperature = temp);

        inputs.tiltConnected = leaderConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);
        SparkUtil.sparkStickyFault = false;

    // Follower Motor Inputs
    }

    @Override 
    public void setPosition(Rotation2d position) {
        leaderClosedLoopController.setSetpoint(position.getRadians(), ControlType.kPosition, ClosedLoopSlot.kSlot0, ff.calculate(tiltEncoder.getPosition().get(), 0));
    }

    @Override
    public void setVoltage(double volts) {
        tiltSpark.setVoltage(volts);
    }

    @Override
    public void stop() {
        tiltSpark.stopMotor();
    }
}
