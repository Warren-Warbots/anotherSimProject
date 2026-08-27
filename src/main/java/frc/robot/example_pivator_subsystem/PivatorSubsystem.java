// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.example_pivator_subsystem;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.simulation.SimMech;
import frc.robot.simulation.TalonFXSimProfile;
import frc.robot.util.TalonFxUtils;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class PivatorSubsystem {
  /** Creates a new PivatorSubsystem. */
  public WantedState wantedState = WantedState.STOW;
  private SystemState systemState = SystemState.STOWED;

  private double timestampAtSetState = Timer.getFPGATimestamp();

  private final SimMech simMech = new SimMech();
  public static final double rotorInertia = 0.02;
  public PivatorIO io;

  private PivatorIOInputsAutoLogged inputs = new PivatorIOInputsAutoLogged();
  double targetRotation = 0.0;
  double targetHeight = 0.0;
  double currentRotation = 0.0;
  double currentHeight = 0.0;

  boolean atGoal = false;
  boolean pivotAtGoal = false;
  boolean elevatorAtGoal = false;

  public PivatorSubsystem(PivatorIO io) {
    /*
     * initialize motors here
     * step 1 is make config object for each motor in subsystem constants folder
     * step 2 is to use configure talon function to apply config to that motor
     * intakeMotor = new TalonFX(Constants.intake_Motor_ID);
     */
    this.io = io;
  }

  public enum WantedState {
    STOW,
    LVL4;
  }

  private enum SystemState {
    STOWED,
    LVL4;
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;

  }

  public boolean atPosition() {
    return atGoal;
  }

  private void collectInputs() {
    io.updateInputs(inputs);
    atGoal = pivotAtGoal && elevatorAtGoal;
    Logger.recordOutput("ExamplePivatorSubsystem/currentRotation", currentRotation);
    Logger.recordOutput("ExamplePivatorSubsystem/currentHeight", currentHeight);

    // add logging here
    Logger.recordOutput("ExamplePivatorSubsystem/systemState", systemState.name());
  }

  // this handles simple, 1:1 transitions (see robot manager for more complex
  // transitions)
  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case STOW -> SystemState.STOWED;
      case LVL4 -> SystemState.LVL4;
    };
  }

  // this applies function based on systemState
  private void applyStates() {
    switch (systemState) {
      case STOWED -> stow();
      case LVL4 -> scoreLVL4();
    }
  }

  /*
   * add functions IF NEEDED, try not to add to many
   * some examples could be:
   * public functions so that other parts of the robot can check things like:
   * at Goal
   */

  public void periodic() {
    collectInputs();
    systemState = handleStateTransition();
    applyStates();
    double timeInState = Timer.getFPGATimestamp() - timestampAtSetState;
    io.setPivatorPosition(targetRotation);
  }

  private void stow() {
    targetHeight = Constants.IS_COMP_BOT ? 7.52 : 0.0;
    targetRotation = Constants.IS_COMP_BOT ? 0.26 : 0.0;
  }

  private void scoreLVL4() {
    targetHeight = Constants.IS_COMP_BOT ? 31.2 : 56;
    targetRotation = Constants.IS_COMP_BOT ? 0.52 : 0.437763;
  }

  @AutoLogOutput(key = "ExamplePivatorSubsystem/whatIfRotation")
  public double pivotWhatIf(){
    return inputs.pivatorPosition + 0;
    //edit the value by however
  }

   @AutoLogOutput(key = "ExamplePivatorSubsystem/whatIfHeight")
  public double elevatorWhatIf(){
    return inputs.currentHeight + 0;
    //edit the value by however
  }

}
