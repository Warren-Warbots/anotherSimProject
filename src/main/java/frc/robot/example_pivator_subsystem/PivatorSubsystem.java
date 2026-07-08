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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.simulation.SimMech;
import frc.robot.simulation.TalonFXSimProfile;
import frc.robot.util.TalonFxUtils;

public class PivatorSubsystem {
  /** Creates a new ExampleSubsystem. */
  public WantedState wantedState = WantedState.STOW;
  private SystemState systemState = SystemState.STOWED;

  private double timestampAtSetState = Timer.getFPGATimestamp();

  private final SimMech simMech = new SimMech();
  public static final double rotorInertia = 0.02;
  public TalonFXSimProfile pivotSimProfile;
  public TalonFXSimProfile frontElevatorSimProfile;

  TalonFX pivotMotor;
  TalonFX elevatorMotorFront;
  TalonFX elevatorMotorBack;
  CANcoder pivotCANcoder;

  double targetRotation = 0.0;
  double targetHeight = 0.0;
  double currentRotation = 0.0;
  double currentHeight = 0.0;

  double angleError;
  double heightError;

  boolean atGoal = false;
  boolean pivotAtGoal = false;
  boolean elevatorAtGoal = false;

  public PivatorSubsystem() {
    /*
     * initialize motors here
     * step 1 is make config object for each motor in subsystem constants folder
     * step 2 is to use configure talon function to apply config to that motor
     * intakeMotor = new TalonFX(Constants.intake_Motor_ID);
     */
    pivotMotor = new TalonFX(Constants.pivot_Motor_ID);
    TalonFxUtils.configureTalon(pivotMotor, PivatorConstants.pivotMotorConfig);
    elevatorMotorFront = new TalonFX(Constants.elevator_Front_Motor_ID);
    TalonFxUtils.configureTalon(elevatorMotorFront, PivatorConstants.elevatorMotorFrontConfig);
    elevatorMotorBack = new TalonFX(Constants.elevator_Back_Motor_ID);
    TalonFxUtils.configureTalon(elevatorMotorBack, PivatorConstants.elevatorMotorBackConfig);
    pivotCANcoder = new CANcoder(Constants.pivot_CANcoder_ID);

    elevatorMotorFront.setPosition(0);
    elevatorMotorBack.setPosition(0);
    elevatorMotorBack.setControl(new Follower(elevatorMotorFront.getDeviceID(), MotorAlignmentValue.Opposed));

    pivotSimProfile = new TalonFXSimProfile(pivotMotor, rotorInertia);
    frontElevatorSimProfile = new TalonFXSimProfile(elevatorMotorFront, rotorInertia);
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

  private void collectInputs() {
    currentRotation = pivotMotor.getPosition().getValueAsDouble();
    currentHeight = elevatorMotorFront.getPosition().getValueAsDouble();

    pivotAtGoal = angleError < PivatorConstants.pivotTolerance;
    elevatorAtGoal = heightError < PivatorConstants.elevatorTolerance;
    heightError = Math.abs(targetHeight - currentHeight);
    angleError = Math.abs(targetRotation - currentRotation);
    atGoal = pivotAtGoal && elevatorAtGoal;

    // add logging here
    DogLog.log("ExamplePivatorSubsystem/systemState", systemState.name());

  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case STOW -> SystemState.STOWED;
      case LVL3 -> SystemState.LVL3;
      case LVL4 -> SystemState.LVL4;
    };
  }

  private void applyStates() {
    switch (systemState) {
      case STOWED -> stow();
      case LVL3 -> scoreLVL3();
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
    pivotMotor.setControl(PivatorConstants.pivotPositionVoltage.withPosition(targetRotation));
    elevatorMotorFront.setControl(PivatorConstants.elevatorMotionMagicVoltage.withPosition(targetHeight));
    if (Utils.isSimulation()) {
      simMech.updatePivot(pivotMotor.getPosition(), elevatorMotorFront.getPosition());
    }
  }

  private void stow() {
    targetHeight = Constants.IS_COMP_BOT ? 7.52 : 0.0;
    targetRotation = Constants.IS_COMP_BOT ? 0.26 : 0.0;
  }

  private void scoreLVL3() {
    targetHeight = Constants.IS_COMP_BOT ? 17.25 : 26.4;
    targetRotation = Constants.IS_COMP_BOT ? 0.4898 : 0.452881;
  }

  private void scoreLVL4() {
    targetHeight = Constants.IS_COMP_BOT ? 31.2 : 56;
    targetRotation = Constants.IS_COMP_BOT ? 0.52 : 0.437763;
  }

}
