package frc.robot.autos;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_manager.RobotState;
import frc.robot.util.FieldUtil;

public class DriveForwardAuto extends WarbotAuto{
    @Override
    public Command getAutoCommand() {
        return robotManager.driveToPose(FieldUtil.getExamplePose(),  0.1, 3,
         10, 3, 5);
    }
}
