// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.example_pivator_subsystem.PivatorSubsystem;
import frc.robot.example_intake_subsystem.IntakeSubsystem;
import frc.robot.lights_subsystem.LightsSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.FieldUtil;

public class RobotManager extends SubsystemBase {
  public WantedRobotState wantedState = WantedRobotState.STOW;
  public CurrentRobotState currentState = CurrentRobotState.STOW;

  public SwerveSubsystem swerve;
  public LightsSubsystem lights;
  public PivatorSubsystem pivot;
  public IntakeSubsystem intake;
  private double timestampAtSetState = Timer.getFPGATimestamp();

  public boolean hasGP = false;

  public RobotManager(SwerveSubsystem swerve, LightsSubsystem lights, PivatorSubsystem pivot, IntakeSubsystem intake) {
    this.swerve = swerve;
    this.lights = lights;
    this.pivot = pivot;
    this.intake = intake;

  }

  public void setWantedRobotState(WantedRobotState state) {
    DogLog.log("Robot/wantedState", state);
    timestampAtSetState = Timer.getFPGATimestamp();
    this.wantedState = state;

  }

  public Command setWantedRobotStateCommand(WantedRobotState wantedState) {
    return Commands.runOnce(() -> setWantedRobotState(wantedState));
  }

  public Command waitForStateCommand(WantedRobotState waitState) {
    return Commands.waitUntil(() -> this.wantedState == waitState);
  }

  public void startDriveToPose(Pose2d desiredPose, double translationToleranceMeters, double maxSpeed,
      double rotationToleranceDegrees, double maxAngularSpeed) {
    swerve.setDriveToPose(desiredPose, translationToleranceMeters, maxSpeed, rotationToleranceDegrees, maxAngularSpeed);
    swerve.setWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POSE);
  }

  public Command driveToPose(Pose2d desiredPose, double translationToleranceMeters, double maxSpeed,
      double rotationToleranceDegrees, double maxAngularSpeed, double timeout) {
    return Commands
        .runOnce(() -> startDriveToPose(desiredPose, translationToleranceMeters, maxSpeed, rotationToleranceDegrees,
            maxAngularSpeed))
        .andThen(Commands.waitUntil(() -> swerve.isAtDriveToPoseSetpoint()).withTimeout(timeout));
  }

  @Override
  public void periodic() {
    double timeInState = Timer.getFPGATimestamp() - timestampAtSetState;
    lights.setRobotState(wantedState);
    collectInputs();
    currentState = handleStateTransitions();
    applyStates();
    DogLog.log("Robot/currentState", currentState);

  }

  public void collectInputs() {
    DogLog.log("Robot/hasGP", intake.getSensor());
    hasGP = intake.getSensor();

  }

  private CurrentRobotState handleStateTransitions() {
    return switch (wantedState) {
      case STOW: {
        yield CurrentRobotState.STOW; // always go to stow when wanted state is stow.
      }
      case INTAKE: {
        yield CurrentRobotState.INTAKE;
      }
      case AUTO_SCORE_L4: {
        // this is where specific conditions/logic and/or safety code lives
        if (pivot.atPosition() && hasGP && swerve.isAtDriveToPoseSetpoint()) {
          yield CurrentRobotState.SCORE_L4;
        } else {
          yield CurrentRobotState.PREPARE_SCORE_L4;
        }
      }
    };
  }

  private void applyStates() {
    switch (currentState) {
      case STOW -> stow();
      case INTAKE -> intake();
      case PREPARE_SCORE_L4 -> prepareScoreL4();
      case SCORE_L4 -> scoreL4();
    }
    ;
  }

  private void stow() {
    intake.setWantedState(IntakeSubsystem.WantedState.STOP);
    pivot.setWantedState(PivatorSubsystem.WantedState.STOW);

  }

  private void intake() {
    intake.setWantedState(IntakeSubsystem.WantedState.INTAKE);
    pivot.setWantedState(PivatorSubsystem.WantedState.STOW);

  }

  private void prepareScoreL4() {
    intake.setWantedState(IntakeSubsystem.WantedState.STOP);
    pivot.setWantedState(PivatorSubsystem.WantedState.LVL4);
    startDriveToPose(FieldUtil.getExamplePose(), 0.05, 3.0, 1, 2.0);
    // transition to actively scoring is handled in handleStateTransitions()
  }

  private void scoreL4() {
    intake.setWantedState(IntakeSubsystem.WantedState.OUTTAKE);
    pivot.setWantedState(PivatorSubsystem.WantedState.LVL4);
    if (!hasGP) {
      setWantedRobotState(WantedRobotState.STOW);
    }
  }

}
