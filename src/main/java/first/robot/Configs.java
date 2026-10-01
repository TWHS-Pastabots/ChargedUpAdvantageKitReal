package first.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import static first.robot.Constants.ModuleConstants.*;

public final class Configs {
  public static final class EasySwerveModule {
    public static final SparkFlexConfig driveConfig = new SparkFlexConfig();
    public static final SparkFlexConfig turnConfig = new SparkFlexConfig();

    static {        

        driveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(driveMotorCurrentLimit)
                .voltageCompensation(12.0);
        // driveConfig
        //         .encoder
        //         .positionConversionFactor(driveEncoderPositionFactor)
        //         .velocityConversionFactor(driveEncoderVelocityFactor)
        //         .uvwMeasurementPeriod(10)
        //         .uvwAverageDepth(2);
        driveConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(
                        driveKp, 0.0,
                        driveKd);
        driveConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        turnConfig
                .inverted(turnInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(turnMotorCurrentLimit)
                .voltageCompensation(12.0);
        // turnConfig
        //         .absoluteEncoder
        //         .inverted(turnEncoderInverted)
        //         .positionConversionFactor(turnEncoderPositionFactor)
        //         .velocityConversionFactor(turnEncoderVelocityFactor)
        //         .averageDepth(2);
        turnConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(true)
                .pid(turnKp, 0.0, turnKd);
        turnConfig
                .signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
    }
  }
}
