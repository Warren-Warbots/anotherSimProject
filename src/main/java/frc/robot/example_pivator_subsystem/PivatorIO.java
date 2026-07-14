package frc.robot.example_pivator_subsystem;

import org.littletonrobotics.junction.AutoLog;

public interface PivatorIO {
    @AutoLog
    public class PivatorIOInputs {
        double pivatorPosition = 0.0;
        boolean atGoal = true;
        double currentHeight = 0.0;
    }

    default void setPivatorPosition(double position) {}

    default void updateInputs(PivatorIOInputs inputs){}

}