package frc.robot.simulation;

import java.util.ArrayList;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.example_pivator_subsystem.PivatorSubsystem;

/**
 * Manages physics simulation for CTRE products.
 */

// in here is gonna be the elevator and pivot simulation
public class PhysicsSim {
    private static final PhysicsSim PhysicsSim = new PhysicsSim();

    /**
     * Gets the robot simulator instance.
     */
    public static PhysicsSim getInstance() {
        return PhysicsSim;
    }

    /**
     * Adds a TalonFX controller to the simulator.
     * 
     * @param talonFX
     *                     The TalonFX device
     * @param rotorInertia
     *                     Rotational Inertia of the mechanism at the rotor
     */

    public void addSimProfile(TalonFXSimProfile talonFXSimProfile) {
        if (talonFXSimProfile != null) {
            _simProfiles.add(talonFXSimProfile);
        }

    }

    // public void addPivot(TalonFX talonFX, final double rotorInertia) {
    // if (talonFX != null) {
    // PivotSimProfile simPivot = new PivotSimProfile(talonFX, rotorInertia);
    // _simProfiles.add(simPivot);
    // }
    // }

    // private final LinearSystem<N1, N1, N1> m_flywheelPlant =
    // LinearSystemId.createFlywheelSystem(
    // DCMotor.getKrakenX60Foc(1), GroundPivotSimProfile.kFlywheelMomentOfInertia,
    // GroundPivotSimProfile.kFlywheelGearing);

    // public final KalmanFilter<N1, N1, N1> m_observer = new KalmanFilter<>(
    // Nat.N1(),
    // Nat.N1(),
    // m_flywheelPlant,
    // VecBuilder.fill(3.0), // How accurate we think our model is
    // VecBuilder.fill(0.01), // How accurate we think our encoder
    // // data is
    // 0.020);

    // public void addGroundPivot(TalonFX talonFX, final double rotorInertia) {
    // if (talonFX != null) {
    // GroundPivotSimProfile simGroundPivot = new GroundPivotSimProfile(talonFX,
    // rotorInertia);
    // _simProfiles.add(simGroundPivot);
    // }
    // }

    /**
     * Runs the simulator:
     * - enable the robot
     * - simulate sensors
     */
    public void run() {
        // Simulate devices
        for (SimProfile simProfile : _simProfiles) {
            simProfile.run();

            // simProfile.runPivot();
            // simProfile.runGroundPivot();
        }
        // System.out.println("Number of SimProfiles: " + _simProfiles.size()); to check
        // the # of simprofiles (2)
    }

    private final ArrayList<SimProfile> _simProfiles = new ArrayList<SimProfile>();

    /**
     * Holds information about a simulated device.
     */
    static class SimProfile {
        private double _lastTime;
        private boolean _running = false;

        /**
         * Runs the simulation profile.
         * Implemented by device-specific profiles.
         */

        public void run() {
        }

        public void runGroundPivot() {
        }

        /**
         * Returns the time since last call, in seconds.
         */
        protected double getPeriod() {
            // set the start time if not yet running
            if (!_running) {
                _lastTime = Utils.getCurrentTimeSeconds();
                _running = true;
            }

            double now = Utils.getCurrentTimeSeconds();
            final double period = now - _lastTime;
            _lastTime = now;

            return period;
        }
    }

}
