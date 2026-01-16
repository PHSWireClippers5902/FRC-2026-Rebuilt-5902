package org.frc5902.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class KitbotRobotContainer extends RobotContainer{

    private final CommandXboxController m_XboxController = new CommandXboxController(0);

    
 
    private void configureBindings() {
        if (Robot.isSimulation()){
            DriverStation.silenceJoystickConnectionWarning(true);
        }
        
        


    }

    public 

    public Pose2d getInitialPose(){
        return null;
    }

    public Command getAutonomousCommand(){
        return null;
    }

}
