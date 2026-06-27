// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.autos.Autos;
import frc.robot.example_pivator_subsystem.PivatorSubsystem;
import frc.robot.example_intake_subsystem.IntakeSubsystem;
import frc.robot.lights_subsystem.LightsSubsystem;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.WantedRobotState;
import frc.robot.swerve.SwerveSubsystem;

public class Robot extends TimedRobot {
  private XboxController driverController = new XboxController(0);
  private SwerveSubsystem swerve = new SwerveSubsystem(driverController);
  private LightsSubsystem lights = new LightsSubsystem();
  private PivatorSubsystem pivot = new PivatorSubsystem();
  private IntakeSubsystem intake = new IntakeSubsystem();

  private final RobotManager manager = new RobotManager(swerve, lights, pivot, intake);
  private Autos autos = new Autos(manager);

  public Robot() {

    DogLog.setOptions(
        new DogLogOptions().withCaptureNt(false)
            .withCaptureDs(true)
            .withNtPublish(!Constants.IS_AT_COMP));
  }

  @Override
  public void robotPeriodic() {
    manager.periodic();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    autos.preloadAuto();
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("TimeLeft", DriverStation.getMatchTime());
    boolean leftTrigger = driverController.getLeftTriggerAxis() > 0.5;
    boolean rightTrigger = driverController.getRightTriggerAxis() > 0.5;
    boolean povLeft = driverController.getPOV() == 270;
    boolean povRight = driverController.getPOV() == 90;
    boolean startPressed = driverController.getStartButton();
    boolean rightBumper = driverController.getRightBumper();
    boolean leftBumper = driverController.getLeftBumper();

    if (leftTrigger) {
      manager.setWantedRobotState(WantedRobotState.AUTO_SCORE_L4);
    } else if (rightTrigger) {
      manager.setWantedRobotState(WantedRobotState.INTAKE);
    } else if (rightBumper) {
      manager.setWantedRobotState(WantedRobotState.STOW);
    } else if (leftBumper) {
      manager.setWantedRobotState(WantedRobotState.DRIVE_WITH_VELOCITY);
    }

  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
