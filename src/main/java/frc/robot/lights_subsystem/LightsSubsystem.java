// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lights_subsystem;

import com.ctre.phoenix6.hardware.CANdle;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot_manager.RobotState;

public class LightsSubsystem extends SubsystemBase {
  private RobotState state;
  private double timestampAtSetState = Timer.getFPGATimestamp();
  CANdle candle;
  public LightsSubsystem() {
    candle = new CANdle(LightsConstants.candleId);
    candle.getConfigurator().apply(LightsConstants.candleConfig);
  }

  public void setRobotState(RobotState robotState){
    state=robotState;
  }

  @Override
  public void periodic() {
    // This is where your state machine lives
    double timeInState = Timer.getFPGATimestamp() - timestampAtSetState;
    DogLog.log("LightsSubsystem/state", state);

    switch (state) {

      case STOW_HAS_GP:
        candle.setControl(LightsConstants.blue);
        break;
      case STOW_NO_GP:
        candle.setControl(LightsConstants.pink);
        break;
      case STATE:
      case STATE1:
      case STATE2:
      case STATE3:
        candle.setControl(LightsConstants.rainbow);
        break;

    }

  }

}
