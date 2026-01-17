package org.frc5902.robot.subsystems.drive;

import java.util.Queue;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;

public class GyroIO_ADXRS implements GyroIO {
    public final ADXRS450_Gyro ADXRS_Gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    
    public GyroIO_ADXRS() {
        yawPositionQueue = 
    }

    public void updateInputs(GyroIOInputs inputs) {}
}
