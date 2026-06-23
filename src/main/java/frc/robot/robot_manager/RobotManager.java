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
import frc.robot.example_intake_subsystem.IntakeSubsystem.WantedState;
import frc.robot.lights_subsystem.LightsSubsystem;
import frc.robot.swerve.SwerveState;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.FieldUtil;

public class RobotManager extends SubsystemBase {
  public WantedRobotState wantedState = WantedRobotState.STOW;
  public WantedRobotState lastWantedState = WantedRobotState.STOW;

  public CurrentRobotState currentState = CurrentRobotState.STOW;
  public CurrentRobotState lastCurrentState = CurrentRobotState.STOW;

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

  public void setWantedState(WantedRobotState state) {
    DogLog.log("Robot/wantedState", state);
    timestampAtSetState = Timer.getFPGATimestamp();
    this.wantedState = state;

  }

  public Command setWantedStateCommand(WantedRobotState wantedState) {
    return Commands.runOnce(() -> setWantedState(wantedState));
  }

  public Command waitForStateCommand(WantedRobotState waitState) {
    return Commands.waitUntil(() -> this.wantedState == waitState);
  }

  public void startDriveToPose(Pose2d desiredPose, double translationToleranceMeters, double maxSpeed,
      double rotationToleranceDegrees, double maxAngularSpeed) {
    swerve.setDriveToPose(desiredPose, translationToleranceMeters, maxSpeed, rotationToleranceDegrees, maxAngularSpeed);
    swerve.setState(SwerveState.DRIVE_TO_POSE);
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
    hasGP = intake.getSensor();
    lights.setRobotState(wantedState);
    DogLog.log("Robot/currentState", currentState);

    lastWantedState = wantedState;

  }

  public CurrentRobotState handleStateTransitions(WantedRobotState wantedState) {
    return switch (wantedState) {
      case STOW: {
        yield CurrentRobotState.STOW; // always go to stow when wanted state is stow.
      }
      case INTAKE: {
        yield CurrentRobotState.INTAKE;
      }
      case AUTO_SCORE: {
        if (true) {
          yield CurrentRobotState.AUTO_SCORE; // specific conditions to transition
        }
      }
      case MANUAL_SCORE: {
        yield CurrentRobotState.MANUAL_SCORE;
      }
    };
  }

  public void applyStates(CurrentRobotState currentState) {
    switch (currentState) {
      case STOW -> stow();
      case INTAKE -> intake();
      case AUTO_SCORE -> autoScore();
      case MANUAL_SCORE -> manualScore();
    }
    ;
  }

  public void stow() {
    intake.setWantedState(WantedState.STOP);

  }

  public void intake() {
    intake.setWantedState(WantedState.INTAKE);

  }

  public void autoScore() {
    // hi

  }

  public void manualScore() {
    // hi

  }

}
