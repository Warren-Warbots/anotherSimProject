// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.SwerveState;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.FieldUtil;

public class RobotManager extends SubsystemBase {

  
  public RobotState state = RobotState.STOW_HAS_GP;
  public RobotState lastState = RobotState.STOW_NO_GP;
  public SwerveSubsystem swerve;

  private double timestampAtSetState = Timer.getFPGATimestamp();

  public RobotManager(SwerveSubsystem swerve) {
    this.swerve = swerve;

  }

  public void setState(RobotState state) {
    DogLog.log("Robot/state", state);
    timestampAtSetState = Timer.getFPGATimestamp();
    this.state = state;
  }

  public Command setModeCommand(RobotState state) {    
      return Commands.runOnce(() -> setState(state));
  }

  public Command waitForStateCommand(RobotState waitState) {
    return Commands.waitUntil(() -> this.state == waitState);
  }

  public void startDriveToPose(Pose2d desiredPose,double translationToleranceMeters,double maxSpeed, double rotationToleranceDegrees, double maxAngularSpeed){
    swerve.setDriveToPose(desiredPose, translationToleranceMeters, maxSpeed, rotationToleranceDegrees, maxAngularSpeed);
    swerve.setState(SwerveState.DRIVE_TO_POSE);
  }

  public Command driveToPose(Pose2d desiredPose,double translationToleranceMeters,double maxSpeed, double rotationToleranceDegrees, double maxAngularSpeed,double timeout){
    return Commands.runOnce(()->startDriveToPose(desiredPose, translationToleranceMeters, maxSpeed, rotationToleranceDegrees, maxAngularSpeed))
    .andThen(Commands.waitUntil(()->swerve.isAtDriveToPoseSetpoint()).withTimeout(timeout));
  }
  


  @Override
  public void periodic() {

    double timeInState = Timer.getFPGATimestamp() - timestampAtSetState;

   

    switch (state) {
      case STOW_HAS_GP:
        break;
      case STOW_NO_GP:
        break;
      default:
        break;

    }

    lastState = state;

  }
}
