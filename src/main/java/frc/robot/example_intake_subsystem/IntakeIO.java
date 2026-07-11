package frc.robot.example_intake_subsystem;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs{
        public boolean canDetected = false;
        public double intakeSpeed = 0.0;
    }
    default void updateInputs(IntakeIOInputs inputs){}

    default void setVoltage(double volts){}
}
