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
import frc.robot.autos.DriveForwardAuto;
import frc.robot.autos.WarbotAuto;
import frc.robot.example_pivator_subsystem.PivatorSubsystem;
import frc.robot.example_intake_subsystem.IntakeSubsystem;
import frc.robot.lights_subsystem.LightsSubsystem;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.WantedRobotState;
import frc.robot.simulation.PhysicsSim;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.FmsUtil;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


public class Robot extends LoggedRobot {
  private XboxController driverController;
  private SwerveSubsystem swerve;
  private LightsSubsystem lights;
  private PivatorSubsystem pivot;
  private IntakeSubsystem intake;
  private RobotManager manager;
  private Autos autos;

  public Robot() {
//    DogLog.setOptions(
//        new DogLogOptions().withCaptureNt(false)
//            .withCaptureDs(true)
//            .withNtPublish(!Constants.IS_AT_COMP));

//   AdvantageKit stuff

  Logger.recordMetadata("anotherSimProject", "MyProject");
switch(Constants.robotMode){
  case REAL, SIM:
    Logger.addDataReceiver(new WPILOGWriter());
    Logger.addDataReceiver(new NT4Publisher());
    break;

    case REPLAY:
      setUseTiming(false);
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    break;
}

    Logger.start();
    DogLog.setOptions(
        new DogLogOptions().withCaptureNt(false)
            .withCaptureDs(true)
            .withNtPublish(false));


//Subsystem and other Definitions
    boolean inReplay = !isReal();
    driverController = new XboxController(0);
    swerve = new SwerveSubsystem(driverController);
    lights = new LightsSubsystem();
    pivot = new PivatorSubsystem();
    intake = new IntakeSubsystem();
    manager = new RobotManager(swerve, lights, pivot, intake);
    autos = new Autos(manager);
  }

  @Override
  public void robotPeriodic() {
    manager.periodic();
    swerve.periodic();
    pivot.periodic();
    intake.periodic();
    lights.periodic();
  }

  @Override
  public void robotInit() {
    Logger.recordOutput("IsCompBot", Constants.IS_AT_COMP);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    autos.updateMirror();
    autos.preloadAuto();
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    autos.updateMirror();
    autos.preloadAuto();
    autos.init();
  }

  @Override
  public void autonomousPeriodic() {
    autos.periodic();
    Logger.recordOutput("isFinished", autos.isFinished());
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
    // This is were we call all the driver and operator controllers for use during
    // teleop
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
  public void simulationInit() {
    // this initializes the physics sim in simulation
    PhysicsSim.getInstance().addSimProfile(pivot.pivotSimProfile);
    PhysicsSim.getInstance().addSimProfile(pivot.frontElevatorSimProfile);

  }

  @Override
  public void simulationPeriodic() {
    // this continuously runs the physics sim in simulation
    PhysicsSim.getInstance().run();

  }
}
