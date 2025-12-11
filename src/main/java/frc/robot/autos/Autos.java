// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.Optional;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_manager.RobotManager;

// TODO configure pathplanner configs here and make localization subsystem

/** Add your docs here. */
public class Autos {
    private RobotManager manager;
    private final SendableChooser<WarbotAuto> autoChooser = new SendableChooser<>();
    private WarbotAuto selectedAuto;

    public Autos(RobotManager robotManager) {
        this.manager = robotManager;

    
        autoChooser.setDefaultOption("DoNothing", new DoNothingAuto());

        SmartDashboard.putData(autoChooser);


        selectedAuto = new DoNothingAuto();
        selectedAuto.setRobotManager(manager);
    }


    public void preloadAuto(){
        selectedAuto=autoChooser.getSelected();
        selectedAuto.setRobotManager(manager);
    }

    public Command getAutoCommand() {
        return selectedAuto.getAutoCommand();
    }


    
}