// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lights_subsystem;

import com.ctre.phoenix6.hardware.CANdle;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.robot_manager.WantedRobotState;

public class LightsSubsystem extends SubsystemBase {
  private WantedRobotState state;
  private double timestampAtSetState = Timer.getFPGATimestamp();
  CANdle candle;

  public LightsSubsystem() {
    candle = new CANdle(Constants.Lights_ID);
    candle.getConfigurator().apply(LightsConstants.candleConfig);
  }

  public void setRobotState(WantedRobotState robotState) {
    state = robotState;
  }

  @Override
  public void periodic() {
    // This is where your state machine lives
    double timeInState = Timer.getFPGATimestamp() - timestampAtSetState;
    DogLog.log("LightsSubsystem/state", state);

    switch (state) {

      case STOW:
        candle.setControl(LightsConstants.blue);
        break;
      case INTAKE:
        candle.setControl(LightsConstants.pink);
        break;
      case AUTO_SCORE_L4:
        candle.setControl(LightsConstants.rainbow);
        break;

    }

  }

}
