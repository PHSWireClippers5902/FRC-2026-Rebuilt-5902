package org.frc5902.robot.subsystems.kitbot.intake;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.filter.Debouncer;
import org.frc5902.robot.Constants.IntakeConstants;

public class IntakeIOTalonSRX implements IntakeIO {

    private final TalonSRX intakeTalon;
    private final TalonSRX feederTalon;
    private final Debouncer intakeConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer feederConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    public IntakeIOTalonSRX() {
        intakeTalon = new TalonSRX(IntakeConstants.IntakeCANID);
        intakeTalon.setInverted(IntakeConstants.IntakeInverted);

        feederTalon = new TalonSRX(IntakeConstants.FeederCANID);
        feederTalon.setInverted(IntakeConstants.FeederInverted);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeConnected = intakeConnectedDebounce.calculate(intakeTalon.getLastError() == ErrorCode.OK);
        inputs.feederConnected = feederConnectedDebounce.calculate(feederTalon.getLastError() == ErrorCode.OK);

        inputs.intakePercentOutput = intakeTalon.getMotorOutputPercent();
        inputs.feederPercentOutput = feederTalon.getMotorOutputPercent();
    }

    @Override
    public void setIntakePercentageOutput(double percent) {
        intakeTalon.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void setFeederPercentageOutput(double percent) {
        feederTalon.set(ControlMode.PercentOutput, percent);
    }
}
