// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.example_pivator_subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TalonFxUtils;

public class PivatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public WantedState wantedState = WantedState.STOW;
  private SystemState systemState = SystemState.STOWED;

  private double timestampAtSetState = Timer.getFPGATimestamp();
  TalonFX pivotmotor;
  TalonFX elevatorMotorFront;
  TalonFX elevatorMotorBack;
  CANcoder pivotCANcoder;

  double targetPivotRotation = 0.0;
  double targetHeight = 0.0;
  double currentRotation = 0.0;
  double currentHeight = 0.0;

  double angleError;
  double heightError;

  boolean atGoal = false;
  boolean pivotAtGoal = false;
  boolean elevatorAtGoal = false;

  public PivatorSubsystem() {
    // initialize motors here
    // step 1 is make config object for each motor in subsystem constants folder
    // step 2 is to use configure talon function to apply config to that motor
    pivotCANcoder = new CANcoder(Constants.pivotCANcoder);

    pivotmotor = new TalonFX(Constants.pivotMotorId);
    TalonFxUtils.configureTalon(pivotmotor, PivatorConstants.intakeMotorConfig);

    elevatorMotorFront = new TalonFX(Constants.elevatorMotorFrontId);
    TalonFxUtils.configureTalon(elevatorMotorFront, PivatorConstants.intakeMotorConfig);

    elevatorMotorBack = new TalonFX(Constants.elevatorMotorBackId);
    TalonFxUtils.configureTalon(elevatorMotorBack, PivatorConstants.intakeMotorConfig);

  }

  public enum WantedState {
    STOW,
    LVL3,
    LVL4;
  }

  private enum SystemState {
    STOWED,
    LVL3,
    LVL4;
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;

  }

  public boolean atPosition() {
    return atGoal;
  }

  /*
   * add functions IF NEEDED, try not to add to many
   * some examples could be:
   * public functions so that other parts of the robot can check things like:
   * at Goal
   */

  public SystemState handleStateTransition() {
    return switch (wantedState) {
      case STOW -> SystemState.STOWED;
      case LVL3 -> SystemState.LVL3;
      case LVL4 -> SystemState.LVL4;
    };
  }

  public void apply() {
    switch (systemState) {
      case STOWED -> stow();
      case LVL3 -> scoreLVL3();
      case LVL4 -> scoreLVL4();
    }
  }

  private void stow() {

  }

  private void scoreLVL4() {

  }

  private void scoreLVL3() {

  }

  @Override
  public void periodic() {
    // This is where your state machine lives
    double timeInState = Timer.getFPGATimestamp() - timestampAtSetState;
    heightError = Math.abs(targetHeight - currentHeight);
    angleError = Math.abs(targetPivotRotation - currentRotation);
    atGoal = pivotAtGoal && elevatorAtGoal;
    // DogLog.log("ExampleSubsystem/state", state);

  }

}
