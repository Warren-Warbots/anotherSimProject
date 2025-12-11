package frc.robot.autos;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_manager.RobotState;

public class DriveForwardAuto extends WarbotAuto{
    @Override
    public Command getAutoCommand() {
        return robotManager.setModeCommand(RobotState.STOW_NO_GP);
    }
}
