package frc.robot.example_intake_subsystem;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.simulation.PhysicsSim;
import frc.robot.simulation.TalonFXSimProfile;
import frc.robot.util.TalonFxUtils;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX intakeMotor;
    private final CANrange canRange;
    private final TalonFXSimProfile intakeSimMotor;
    private final double rotorInertia = 0.02;

    public IntakeIOReal(){
        intakeMotor = new TalonFX(Constants.intakeMotorId);
        TalonFxUtils.configureTalon(intakeMotor, IntakeConstants.intakeMotorConfig);
        intakeSimMotor = new TalonFXSimProfile(intakeMotor, rotorInertia);
        PhysicsSim.getInstance().addSimProfile(intakeSimMotor);
        canRange = new CANrange(Constants.intakeCANrangeId);
    }
    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.canDetected = canRange.getIsDetected().getValue();
        inputs.intakeSpeed = intakeMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts){
        intakeMotor.setControl(IntakeConstants.intakeVoltageOut.withOutput(volts));
    }

}
