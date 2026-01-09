package frc.robot.autos;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class FollowPathAuto extends WarbotAuto{
    @Override
    public Command getAutoCommand() {
        return Commands.sequence(
            robotManager.driveToPose(new Pose2d(1,1,Rotation2d.fromDegrees(0)), 0.1, 4, 10, 3, 5),
            robotManager.driveToPose(new Pose2d(2,1,Rotation2d.fromDegrees(0)), 0.1, 4, 10, 3, 5),
            robotManager.driveToPose(new Pose2d(2,2,Rotation2d.fromDegrees(0)), 0.1, 4, 10, 3, 5),
            robotManager.driveToPose(new Pose2d(1,2,Rotation2d.fromDegrees(0)), 0.1, 4, 10, 3, 5),
            robotManager.driveToPose(new Pose2d(1,1,Rotation2d.fromDegrees(0)), 0.1, 4, 10, 3, 5)


        );


     
    }
}
