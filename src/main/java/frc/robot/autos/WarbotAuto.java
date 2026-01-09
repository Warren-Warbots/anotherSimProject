package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_manager.RobotManager;

abstract class WarbotAuto {
    RobotManager robotManager;
    
    public abstract Command getAutoCommand();

    public void setRobotManager(RobotManager robotManager){
        this.robotManager = robotManager;
    }

    
}
