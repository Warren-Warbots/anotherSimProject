package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.robot_manager.RobotManager;

/** Add your docs here. */
public class Autos extends WarbotAuto {
    private RobotManager manager;
    private final SendableChooser<WarbotAuto> chooser1 = new SendableChooser<>();
    private final SendableChooser<WarbotAuto> chooser2 = new SendableChooser<>();
    private final SendableChooser<Boolean> mirrorAuto = new SendableChooser<>();
    private WarbotAuto path1;
    private WarbotAuto path2;
    private boolean path1Finished = false;

    public Autos(RobotManager robotManager) {
        this.manager = robotManager;
        chooser1.setDefaultOption("DoNothing", new DoNothingAuto());
        chooser2.setDefaultOption("DoNothing", new DoNothingAuto());
        SmartDashboard.putData("chooser1", chooser1);
        SmartDashboard.putData("chooser2", chooser2);

        path1 = new DoNothingAuto();
        path1.setManager(manager);
        path2 = new DoNothingAuto();
        path2.setManager(manager);

        mirrorAuto.setDefaultOption("(Right) Normal", false);
        mirrorAuto.addOption("(Right) Normal", false);
        mirrorAuto.addOption("(Left) Mirrored", true);
        SmartDashboard.putData("MirrorAuto", mirrorAuto);

        chooser1.addOption("DriveForwardAuto", new DriveForwardAuto());

        chooser2.addOption("DriveForwardAuto", new DriveForwardAuto());

    }

    public void preloadAuto() {
        WarbotAuto selectedPath1 = chooser1.getSelected();
        if (selectedPath1 != null) {
            path1 = selectedPath1;
        }
        WarbotAuto selectedPath2 = chooser2.getSelected();
        if (selectedPath2 != null) {
            path2 = selectedPath2;
        }

        path1.setManager(manager);
        path2.setManager(manager);
    }

    public void init() {
        path1Finished = false;
        path1.init();
        path2.init();
    }

    public void periodic() {
        if (!path1Finished) {
            path1.periodic();
            if (path1.isFinished()) {
                path1Finished = true;
            }
        } else {
            path2.periodic();
        }
    }

    public void updateMirror() {
        WarbotAuto.mirror = mirrorAuto.getSelected();
    }

    public boolean getMirror() {
        return WarbotAuto.mirror;
    }
}