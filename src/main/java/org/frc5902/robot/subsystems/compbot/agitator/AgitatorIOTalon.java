package org.frc5902.robot.subsystems.compbot.agitator;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.DoubleSupplier;

import static org.frc5902.robot.util.motorutil.SparkUtil.tryUntilOk;

public class AgitatorIOTalon implements AgitatorIO {

    // hardware
    public final TalonSRX agitator;
    // signals
    public final DoubleSupplier appliedVolts;
    public final DoubleSupplier temp;
    public final DoubleSupplier busVoltage;

    // outputs
    public final Debouncer agitatorConnectedDebounce = new Debouncer(0.5, DebounceType.kFalling);

    public AgitatorIOTalon() {
        agitator = new TalonSRX(AgitatorConstants.AgitatorCANID);
        agitator.setInverted(AgitatorConstants.inverted);
        agitator.clearStickyFaults();
        
        appliedVolts = () -> agitator.getMotorOutputVoltage();

        temp = () -> agitator.getTemperature();
        busVoltage = () -> agitator.getBusVoltage();
    }

    @Override
    public void updateInputs(AgitatorIOInputs inputs) {
        inputs.data = new AgitatorIOData(
                agitatorConnectedDebounce.calculate(agitator.getLastError() != ErrorCode.OK),
                appliedVolts.getAsDouble(),
                busVoltage.getAsDouble(),
                temp.getAsDouble());
    }

    @Override
    public void runVolts(double volts) {
        // agitator.setVoltage(volts);
        // calculation: percent output * bus voltage ==> output (volts)
        // To reverse, output / bus voltage = percent output
        agitator.set(TalonSRXControlMode.PercentOutput, volts / busVoltage.getAsDouble());
    }


    @Override
    public void stop() {
        agitator.set(ControlMode.PercentOutput, 0.0);
    }
}
