package org.frc5902.robot.subsystems.drive;

import java.util.Queue;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class GyroIO_ADXRS implements GyroIO {
    public final ADXRS450_Gyro ADXRS_Gyro = new ADXRS450_Gyro();
    
    public GyroIO_ADXRS(){
        
    }
    
    public void updateInputs(GyroIOInputs inputs){

    }


}
