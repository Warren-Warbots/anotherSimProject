// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lights_subsystem;

import com.ctre.phoenix6.hardware.CANdle;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.robot_manager.CurrentRobotState;
import frc.robot.robot_manager.WantedRobotState;
import org.littletonrobotics.junction.Logger;

public class LightsSubsystem {
  private CurrentRobotState state;
  private double timestampAtSetState = Timer.getFPGATimestamp();
  CANdle candle;

  public LightsSubsystem() {
    candle = new CANdle(Constants.lightsId);
    candle.getConfigurator().apply(LightsConstants.candleConfig);
  }

  public void setLightState(CurrentRobotState robotState) {
    state = robotState;
  }

  public void periodic() {
    // This is where your state machine lives
    double timeInState = Timer.getFPGATimestamp() - timestampAtSetState;
    Logger.recordOutput("LightsSubsystem/state", state);

    switch (state) {

      case STOW:
        candle.setControl(LightsConstants.blue);
        break;
      case INTAKE:
        candle.setControl(LightsConstants.pink);
        break;
      case PREPARE_SCORE_L4:
        candle.setControl(LightsConstants.rainbow);
        break;
      case SCORE_L4:
        candle.setControl(LightsConstants.white);
        break;

    }

  }

}
