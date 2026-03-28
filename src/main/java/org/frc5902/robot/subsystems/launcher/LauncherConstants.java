package org.frc5902.robot.subsystems.launcher;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import lombok.AllArgsConstructor;
import lombok.Builder;
import org.frc5902.robot.util.motorutil.PID;

public class LauncherConstants {
    // create flywheel constants builder class
    @Builder
    @AllArgsConstructor
    public static class FlywheelConstants {
        @Builder.Default
        public int FlywheelCANID = -1;

        @Builder.Default
        public boolean inverted = false;

        @Builder.Default
        public final double reduction = 1;

        @Builder.Default
        public int StallLimit = 0;

        @Builder.Default
        public int FreeLimit = 0;

        @Builder.Default
        public PID flywheelPID = PID.builder().build();

        @Builder.Default
        public double flywheelPositionConversionFactor = 1.0;

        @Builder.Default
        public double flywheelVelocityConversionFactor = 1.0 / 60;

        @Builder.Default
        public IdleMode idleMode = IdleMode.kBrake;
    }

    public static FlywheelConstants FlywheelLeftConstants = FlywheelConstants.builder()
            .FlywheelCANID(51)
            .inverted(false)
            .reduction(1)
            .StallLimit(70)
            .FreeLimit(45)
            .flywheelPID(PID.builder().proportional(0.01).deriviative(0.2).build())
            .flywheelPositionConversionFactor(1.0)
            .flywheelVelocityConversionFactor(1.0 / 60)
            .idleMode(IdleMode.kCoast)
            .build();

    public static FlywheelConstants FlywheelRightConstants = FlywheelConstants.builder()
            .FlywheelCANID(50)
            .inverted(true)
            .reduction(1)
            .StallLimit(70)
            .FreeLimit(45)
            .flywheelPID(PID.builder().proportional(0.01).deriviative(0.2).build())
            .flywheelPositionConversionFactor(1.0)
            .flywheelVelocityConversionFactor(1.0 / 60)
            .idleMode(IdleMode.kCoast)
            .build();

    public class InserterConstants {
        public static int InserterCANID = 55;
        public static boolean inverted = true;
        public static final double reduction = 1;
        public static int StallLimit = 65;
        public static int FreeLimit = 46;
        public static PID inserterPID =
                PID.builder().proportional(0.25).deriviative(0.1).build();
        public static double inserterPositionConversionFactor = 1.0 / 3.0;
        public static double inserterVelocityConversionFactor = inserterPositionConversionFactor / 60;
        public static IdleMode idleMode = IdleMode.kBrake;
    }
}
