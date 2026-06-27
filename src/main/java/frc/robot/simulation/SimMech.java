package frc.robot.simulation;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.example_intake_subsystem.IntakeSubsystem;
import frc.robot.example_pivator_subsystem.PivatorSubsystem;

/**
 * Class to keep all the mechanism-specific objects together and out of the main
 * example
 */
public class SimMech {
    private final double HEIGHT = 100; // Controls the height of the mech2d SmartDashboard
    private final double WIDTH = 100; // Controls the height of the mech2d SmartDashboard

    // PivatorSubsystem pivot = new PivatorSubsystem(); // -- causes sim to crash
    // IntakeSubsystem intake;

    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);

    public SimMech() {
    }

    // }
    // public void getfrontElevatorPosition() {
    // frontElevatorSim.getPositionMeters();
    // }

    Mechanism2d mech2d = new Mechanism2d(WIDTH, HEIGHT);
    // MechanismLigament2d pivotArm = mech2d
    // .getRoot("startPoint", 44, 50)
    // .append(new MechanismLigament2d("pivotArm", 25, 0, 6, new
    // Color8Bit(Color.kGreen)))

    // MechanismLigament2d stick = mech2d.getRoot("stick", 44, 0)
    // .append(new MechanismLigament2d("stick",
    // // (PivatorSubsystem.frontElevatorMotor.getPositionMeters())
    // 10, 90, 6,
    // new Color8Bit(Color.kPurple)));

    MechanismLigament2d groundIntakePivot = mech2d.getRoot("groundIntakePivot", 55, 2)
            .append(new MechanismLigament2d("L_groundIntakePivot", 6, 90, 6,
                    new Color8Bit(Color.kBurlywood)));

    MechanismLigament2d right_elevatorStick = mech2d
            .getRoot("right_elevatorStick Root", 49, 0)
            .append(
                    new MechanismLigament2d("right_elevatorStick",
                            // PivatorSubsystem.frontElevatorSim.getPositionMeters()
                            10, 90, 6, new Color8Bit(Color.kCyan)));

    MechanismLigament2d tippy_top_elevatorStick = right_elevatorStick
            .append(new MechanismLigament2d("tippy_top_elevatorStick", 9, 90, 6, new Color8Bit(
                    Color.kPink)));

    MechanismLigament2d pivotArm = right_elevatorStick
            // .getRoot("startPoint", 44, 50)
            .append(new MechanismLigament2d("notPivotArm", 1, 270, 6, new Color8Bit(Color.kGreen)))

            .append(new MechanismLigament2d("pivotArm", 25, 0, 6, new Color8Bit(Color.kGreen)));

    MechanismLigament2d left_elevatorStick = mech2d
            .getRoot("leftElevatorStick root", 40, 0)
            .append(
                    new MechanismLigament2d("leftElevatorStick",
                            // PivatorSubsystem.frontElevatorSim.getPositionMeters(),
                            10, 90, 6, new Color8Bit(Color.kCyan)));

    MechanismLigament2d drivetrain = mech2d
            .getRoot("drivebase", 30, 2)
            .append(new MechanismLigament2d("drive", 34, 0, 12, new Color8Bit(Color.kDarkRed)));

    public double wrapAngle(double angle) {
        return ((angle + 180) % 360 + 360) % 360 - 180;

    }

    public double wrapTo0_2PI(double angle) {
        return (angle % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);

    }

    // }

    public void updatePivot(StatusSignal<Angle> position, StatusSignal<Angle> angle, StatusSignal<Angle> height) {

        pivotArm.setLength(25);
        pivotArm.setAngle(wrapAngle(position.getValueAsDouble() * 360));

        // stick.setLength(height.getValueAsDouble());

        groundIntakePivot.setLength(8);
        groundIntakePivot.setAngle(wrapAngle(angle.getValueAsDouble() * 360));

        right_elevatorStick.setLength(height.getValueAsDouble()); // height.getValueAsDouble()
        left_elevatorStick.setLength(height.getValueAsDouble());

        // right_elevatorStick.setLength(PivatorSubsystem.targetHeight);
        // left_elevatorStick.setLength(PivatorSubsystem.targetHeight);

        SmartDashboard.putData("mech2d", mech2d); // Creates mech2d in SmartDashboard
    }
}
