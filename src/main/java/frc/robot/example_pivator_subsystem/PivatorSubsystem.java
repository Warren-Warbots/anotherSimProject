// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.example_pivator_subsystem;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TalonFxUtils;

public class PivatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public WantedState wantedState = WantedState.STOW;
  private SystemState systemState = SystemState.STOWED;

  private double timestampAtSetState = Timer.getFPGATimestamp();
  TalonFX pivotMotor;
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

    pivotCANcoder = new CANcoder(Constants.pivotCANcoder);

    pivotMotor = new TalonFX(Constants.pivotMotorId);
    TalonFxUtils.configureTalon(pivotMotor, PivatorConstants.pivotMotorConfig);

    elevatorMotorFront = new TalonFX(Constants.elevatorMotorFrontId);
    TalonFxUtils.configureTalon(elevatorMotorFront, PivatorConstants.elevatorMotorFrontConfig);

    elevatorMotorBack = new TalonFX(Constants.elevatorMotorBackId);
    TalonFxUtils.configureTalon(elevatorMotorBack, PivatorConstants.elevatorMotorBackConfig);

    elevatorMotorFront.setPosition(0);
    elevatorMotorBack.setPosition(0);

    elevatorMotorBack.setControl(new Follower(elevatorMotorFront.getDeviceID(), MotorAlignmentValue.Opposed));

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

  public void applyStates() {
    switch (systemState) {
      case STOWED -> stow();
      case LVL3 -> scoreLVL3();
      case LVL4 -> scoreLVL4();
    }
  }

  private void stow() {
    targetHeight = Constants.IS_COMP_BOT ? 7.52 : 0.0;
    targetPivotRotation = Constants.IS_COMP_BOT ? 0.26 : 0.0;
  }

  private void scoreLVL3() {
    targetHeight = Constants.IS_COMP_BOT ? 17.25 : 26.4;
    targetPivotRotation = Constants.IS_COMP_BOT ? 0.4898 : 0.452881;
  }

  private void scoreLVL4() {
    targetHeight = Constants.IS_COMP_BOT ? 31.2 : 56;
    targetPivotRotation = Constants.IS_COMP_BOT ? 0.52 : 0.437763;
  }

  @Override
  public void periodic() {
    currentHeight = elevatorMotorFront.getPosition().getValueAsDouble();
    currentRotation = pivotMotor.getPosition().getValueAsDouble();

    targetHeight = MathUtil.clamp(targetHeight, PivatorConstants.minElevatorHeight,
        PivatorConstants.maxElevatorHeight);
    targetPivotRotation = MathUtil.clamp(targetPivotRotation, PivatorConstants.minPivotRot,
        PivatorConstants.maxPivotRot);
    boolean pivotAtGoal = angleError < PivatorConstants.pivotTolerance;
    boolean elevatorAtGoal = heightError < PivatorConstants.elevatorTolerance;

    // This is where your state machine lives
    double timeInState = Timer.getFPGATimestamp() - timestampAtSetState;
    heightError = Math.abs(targetHeight - currentHeight);
    angleError = Math.abs(targetPivotRotation - currentRotation);
    atGoal = pivotAtGoal && elevatorAtGoal;
    // DogLog.log("ExampleSubsystem/state", state);

  }

}
