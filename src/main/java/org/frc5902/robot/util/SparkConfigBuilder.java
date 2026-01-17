package org.frc5902.robot.util;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;
import lombok.ToString;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

@Getter
@Setter
@ToString
public class SparkConfigBuilder implements LoggableInputs {

    public SparkMaxConfig SPARK_MAX_CONFIG;
    public String type;

    @Builder
    public SparkConfigBuilder(
            String type,
            IdleMode idleMode,
            int smartCurrentLimit,
            FeedbackSensor feedbackSensor,
            boolean inverted,
            double encoderPositionConversionFactor,
            double encoderVelocityConversionFactor,
            PID pid) {
        SPARK_MAX_CONFIG = new SparkMaxConfig();
        SPARK_MAX_CONFIG.idleMode(idleMode);
        SPARK_MAX_CONFIG.smartCurrentLimit(smartCurrentLimit);
        SPARK_MAX_CONFIG.closedLoop.feedbackSensor(feedbackSensor);
        SPARK_MAX_CONFIG.closedLoop.pid(pid.getProportional(), pid.getIntegral(), pid.getDeriviative());
        SPARK_MAX_CONFIG.inverted(inverted);

        EncoderConfig ENCODER_CONFIGURATION = new EncoderConfig();
        ENCODER_CONFIGURATION.positionConversionFactor(encoderPositionConversionFactor);
        ENCODER_CONFIGURATION.velocityConversionFactor(encoderVelocityConversionFactor);

        SPARK_MAX_CONFIG.apply(ENCODER_CONFIGURATION);

        this.type = type;
    }

    @Override
    public void toLog(LogTable table) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'toLog'");
    }

    @Override
    public void fromLog(LogTable table) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'fromLog'");
    }
}
