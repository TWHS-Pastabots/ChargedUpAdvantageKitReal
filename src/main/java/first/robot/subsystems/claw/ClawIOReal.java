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
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import first.robot.Constants.PivotConstants;
import first.robot.Constants.ClawConstants;

import first.robot.util.SparkUtil;

import org.wpilib.math.util.*;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;
import java.util.Queue;
import java.util.function.DoubleSupplier;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.rotation.Encoder;
import org.wpilib.math.controller.ArmFeedforward;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller, and duty cycle
 * absolute encoder.
 */
public class ClawIOReal implements ClawIO {
    //private final Rotation2d zeroRotation;

    // Hardware objects
    private final SparkFlex intakeSpark;

    private final RelativeEncoder intakeEncoder;


    // Connection debouncers
    private final Debouncer intakeConnectedDebounce = new Debouncer(0.5);


    private final SparkClosedLoopController intakeClosedLoopController;

    public ClawIOReal(){
        intakeSpark = new SparkFlex(CANPort.CAN_D1, ClawConstants.intakeCanID, MotorType.kBrushless);
        intakeClosedLoopController = intakeSpark.getClosedLoopController();
        intakeEncoder = intakeSpark.getEncoder();

        var intakeConfig = new SparkFlexConfig();

        intakeConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ClawConstants.currentLimitAmps)
            .inverted(ClawConstants.intakeInverted);
      //  intakeConfig
       //     .encoder
           // .positionConversionFactor(2*Math.PI)
           // .velocityConversionFactor(2*Math.PI / 60.0)  // Convert RPM to rad/s
        //    .uvwMeasurementPeriod(10)
        //    .uvwAverageDepth(2);
        intakeConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(ClawConstants.kP, ClawConstants.kI, ClawConstants.kD)
            .outputRange(-1, 1)
            .positionWrappingEnabled(false);
        intakeSpark.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    @Override
    public void updateInputs(ClawIOInputs inputs) {
        // Read sensors
        SparkUtil.sparkStickyFault = false;
        SparkUtil.ifOk(intakeSpark, () -> intakeEncoder.getPosition().get(), pos -> inputs.intakePosition = new Rotation2d(pos));
        SparkUtil.ifOk(intakeSpark, () -> intakeEncoder.getVelocity().get(), vel -> inputs.intakeVelocity = vel);
    
        SparkUtil.ifOk(intakeSpark, () -> intakeSpark.getAppliedOutput().get() * intakeSpark.getBusVoltage().get(), volts -> inputs.intakeAppliedVoltage = volts);
        SparkUtil.ifOk(intakeSpark, () -> intakeSpark.getOutputCurrent().get(), amps -> inputs.intakeCurrentDraw = amps);
        SparkUtil.ifOk(intakeSpark, () -> intakeSpark.getMotorTemperature().get(), temp -> inputs.intakeTemperature = temp);

        inputs.intakeConnected = intakeConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);
    }

    @Override 
    public void setPosition(Rotation2d position) {
        intakeClosedLoopController.setSetpoint(position.getRadians(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setVoltage(double volts) {
        intakeSpark.setVoltage(volts);
    }

    @Override
    public void setPower(double power) {
        intakeSpark.setThrottle(power);
    }

    @Override
    public void stop() {
        intakeSpark.stopMotor();
    }
}
