package frc.robot.simulation;

import java.lang.module.Configuration;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.example_pivator_subsystem.PivatorSubsystem;
import frc.robot.simulation.PhysicsSim.SimProfile;

/**
 * Holds information about a simulated TalonFX.
 */
public class TalonFXSimProfile extends SimProfile {
    private static final double kMotorResistance = 0.002; // Assume 2mOhm resistance for voltage drop calculation

    private final DCMotorSim _motorSim;
    private final TalonFXSimState _talonFXSimState;

    /*
     * @param talonFX
     * The TalonFX device
     * 
     * @param rotorInertia
     * Rotational Inertia of the mechanism at the rotor
     */
    public TalonFXSimProfile(final TalonFX talonFX, final double rotorInertia) {
        var gearbox = DCMotor.getKrakenX60Foc(1);
        this._motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, rotorInertia, 1.0), gearbox);
        this._talonFXSimState = talonFX.getSimState();
    }

    public void run() {
        /// DEVICE SPEED SIMULATION
        _motorSim.setInputVoltage(_talonFXSimState.getMotorVoltage());

        // elevator_motorSim.setInput(elevator_talonFXSimState.getMotorVoltage());

        _motorSim.update(getPeriod());
        // elevator_motorSim.update(getPeriod());

        /// SET SIM PHYSICS INPUTS
        final double pivot_position_rot = _motorSim.getAngularPositionRotations();
        final double pivot_velocity_rps = Units.radiansToRotations(_motorSim.getAngularVelocityRadPerSec());

        // final double elevator_position_rot =
        // elevator_motorSim.getAngularPositionRotations();
        // final double elevator_velocity_rps =
        // Units.radiansToRotations(elevator_motorSim.getAngularVelocityRadPerSec());

        // pivot
        _talonFXSimState.setRawRotorPosition(pivot_position_rot);
        _talonFXSimState.setRotorVelocity(pivot_velocity_rps);

        _talonFXSimState.setSupplyVoltage(12 - _talonFXSimState.getSupplyCurrent() * kMotorResistance);

        // elevator
        // elevator_talonFXSimState.setRawRotorPosition(elevator_position_rot);
        // elevator_talonFXSimState.setRotorVelocity(elevator_velocity_rps);

        // elevator_talonFXSimState.setSupplyVoltage(12 -
        // elevator_talonFXSimState.getSupplyCurrent() * kMotorResistance);

    }

}