package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class DoNothingAuto extends WarbotAuto{
    @Override
    public Command getAutoCommand() {
        return Commands.none();
    }
}
