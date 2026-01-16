package org.frc5902.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import lombok.Builder;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

@Builder(builderClassName = "")
public class ModuleConfiguration implements LoggableInputs {
    public String ModuleName;

    @Builder.Default
    public int TurningMotorAbsoluteOffset = 0;

    @Builder.Default
    public int DrivingID = 0;

    @Builder.Default
    public int TurningID = 0;

    @Builder.Default
    public boolean DrivingMotorInverted = false;

    @Builder.Default
    public boolean TurningMotorInverted = true;

    @Builder.Default
    public boolean TurnSensorInvert = true;

    @Builder.Default
    public Translation2d ModuleOffset = new Translation2d(0, 0);

    public ModuleConfiguration(
            String ModuleName,
            int TurningMotorAbsoluteOffset,
            int DrivingID,
            int TurningID,
            boolean DrivingPowerInverted,
            boolean TurningMotorInverted,
            boolean TurnSensorInvert,
            Translation2d ModuleOffset) {}

    public static ModuleConfigurationBuilder builder(String ModuleName) {
        return new ModuleConfigurationBuilder().ModuleName(ModuleName);
    }

    @Override
    public void toLog(LogTable table) {
        table.put(ModuleName + "_TurningMotorAbsoluteOffset", TurningMotorAbsoluteOffset);
        table.put(ModuleName + "_DrivingID", DrivingID);
        table.put(ModuleName + "_TurningID", TurningID);
        table.put(ModuleName + "_DrivingMotorInverted", DrivingMotorInverted);
        table.put(ModuleName + "_TurningMotorInverted", TurningMotorInverted);
        table.put(ModuleName + "_TurnSensorInvert", TurnSensorInvert);
    }

    @Override
    public void fromLog(LogTable table) {
        TurningMotorAbsoluteOffset = table.get(ModuleName + "_TurningMotorAbsoluteOffset", 0);
        DrivingID = table.get(ModuleName + "_DrivingID", 0);
        TurningID = table.get(ModuleName + "_TurningID", 0);
        DrivingMotorInverted = table.get(ModuleName + "_DrivingMotorInverted", false);
        TurningMotorInverted = table.get(ModuleName + "_, TurningMotorInverted", false);
        TurnSensorInvert = table.get(ModuleName + "_TurnSensorInvert", false);
    }
}
