package frc.robot.example_pivator_subsystem;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants;
import frc.robot.simulation.SimMech;
import frc.robot.simulation.TalonFXSimProfile;
import frc.robot.util.TalonFxUtils;

public class PivatorIOReal implements PivatorIO {
    TalonFX pivotMotor;
    TalonFX elevatorFrontMotor;
    TalonFX elevatorBackMotor;
    CANcoder pivotCANcoder;

    private final SimMech simMech = new SimMech();
    public static final double rotorInertia = 0.02;
    public TalonFXSimProfile pivotSimProfile;
    public TalonFXSimProfile frontElevatorSimProfile;

    public PivatorIOReal(){
        pivotMotor = new TalonFX(Constants.pivotMotorId);
        TalonFxUtils.configureTalon(pivotMotor, PivatorConstants.pivotMotorConfig);

        elevatorFrontMotor = new TalonFX(Constants.elevatorFrontMotorId);
        TalonFxUtils.configureTalon(elevatorFrontMotor, PivatorConstants.elevatorFrontMotorConfig);

        elevatorBackMotor = new TalonFX(Constants.elevatorBackMotorId);
        TalonFxUtils.configureTalon(elevatorBackMotor, PivatorConstants.elevatorBackMotorConfig);

        pivotCANcoder = new CANcoder(Constants.pivotCANcoderId);

        elevatorFrontMotor.setPosition(0);
        elevatorBackMotor.setPosition(0);
        elevatorBackMotor.setControl(new Follower(elevatorFrontMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        pivotSimProfile = new TalonFXSimProfile(pivotMotor, rotorInertia);
        frontElevatorSimProfile = new TalonFXSimProfile(elevatorFrontMotor, rotorInertia);
    }

    @Override
    public void updateInputs(PivatorIOInputs inputs){
        inputs.pivatorPosition = pivotMotor.getPosition().getValueAsDouble();
        if (Utils.isSimulation()) {
            simMech.updatePivot(pivotMotor.getPosition(), elevatorFrontMotor.getPosition());
        }
    }

    @Override
    public void setPivatorPosition(double position){
        pivotMotor.setControl(PivatorConstants.pivotPositionVoltage.withPosition(position));
    }
}
