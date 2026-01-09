// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//todo:
// add skew comp? https://github.com/FRCTeam2910/2025CompetitionRobot-Public/blob/1b9e161c7719da0522ec826e2a48d2afb63232a5/src/main/java/org/frc2910/robot/subsystems/drive/SwerveSubsystem.java#L400
// try out friction compensation
/*
things to support:
driving to a point
  in a certain robot state, we set swerve state to DRIVE TO POSE and set the target pose
facing a target
facing a constant angle
limiting speed based on conditions outside of the swerve sub 
precise align style driving (ie moving using data that is not the pose estimator)
*/
package frc.robot.swerve;

import java.util.HashMap;
import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import dev.doglog.DogLog;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.swerve.generated.CompTunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.ControllerHelpers;
import frc.robot.util.FieldUtil;
import frc.robot.util.FmsUtil;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;


public class SwerveSubsystem extends SubsystemBase {
    public TunerSwerveDrivetrain drivetrain;
    private SwerveRequest.FieldCentric drive_field_rel;
    private SwerveRequest.ApplyRobotSpeeds drive_robot_rel;
    private SwerveRequest.FieldCentricFacingAngle drive_snap;
    private SwerveRequest.FieldCentricFacingAngle driveMaintainHeading;
    private SwerveRequest.RobotCentric drive_robot_centric;
    private SwerveState state = SwerveState.TELEOP_DRIVE;
    private double currTopSpeedPercent = 1.0;
    private double currTopRotationSpeedPercent = 1.0;
    private ChassisSpeeds driverDesiredSpeeds = new ChassisSpeeds();

    public SwerveDriveState swerveDriveState = new SwerveDriveState();
    public SwerveDriveState lastSwerveDriveState = new SwerveDriveState();



    private Matrix<N3, N1> stdDevs;
    private static final double kSimLoopPeriod = 0.005;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Telemetry telem = new Telemetry(SwerveConstants.maxSpeed);
    private CommandXboxController driverXboxController;
    private Optional<Rotation2d> lastMaintainHeadingAngle = Optional.empty();
    private double rotationJoystickLastTouched = -1;
    private double highSpeedLastTime = -1;



    private Pose2d driveToPoseTargetPose = new Pose2d();
    private double driveToPoseMaxSpeed;
    private double driveToPoseMaxAngularSpeed;
    private double driveToPoseTranslationToleranceMeters;
    private double driveToPoseRotationToleranceDegrees;
  
    private Rotation2d snapAngle = new Rotation2d();
    private Translation2d snapPoint = new Translation2d();
  
  
    private boolean atGoal = false;
    private boolean timerHasBeenEnabled = false;


    private String[] limelightNames = { "limelight" };

    private HashMap<String, Double> lastAddedVisionTimestampMap = new HashMap<String, Double>();

    public SwerveSubsystem(CommandXboxController driverXboxController) {
        drivetrain = new TunerSwerveDrivetrain(SwerveConstants.swerveDrivetrainConstants,
                SwerveConstants.FrontLeft,
                SwerveConstants.FrontRight,
                SwerveConstants.BackLeft,
                SwerveConstants.BackRight);

        drive_robot_rel = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);
        drive_field_rel = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity)
                .withDeadband(0.08)
                .withRotationalDeadband(0.06 * SwerveConstants.maxRotSpeed);
        drive_snap = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity)
                .withDeadband(0.08)
                .withRotationalDeadband(0.06 * SwerveConstants.maxRotSpeed);
        drive_snap.HeadingController = SwerveConstants.snapController;
        drive_snap.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        drive_snap.HeadingController.setTolerance(SwerveConstants.snapTolerance);

    

        drive_robot_centric = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity)
                .withDeadband(0.08)
                .withRotationalDeadband(0.06 * SwerveConstants.maxRotSpeed);

        this.driverXboxController = driverXboxController;


        if (Utils.isSimulation()) {
            startSimThread();
        }
  
        drivetrain.registerTelemetry(telem::telemeterize);
    }

    public ChassisSpeeds getTeleopDriveSpeeds() {
        double forward = ControllerHelpers.getExponent(
                ControllerHelpers.getDeadbanded(driverXboxController.getLeftY(), SwerveConstants.leftYDeadband),
                SwerveConstants.leftYExponent);
        double strafe = ControllerHelpers.getExponent(
                ControllerHelpers.getDeadbanded(driverXboxController.getLeftX(), SwerveConstants.leftXDeadband),
                SwerveConstants.leftXExponent);
        double rotate = -1.0 * ControllerHelpers.getExponent(
                ControllerHelpers.getDeadbanded(driverXboxController.getRightX(), SwerveConstants.rightXDeadband),
                SwerveConstants.rightXExponent);
        if (!FmsUtil.isRedAlliance()) {
            strafe *= -1.0;
            forward *= -1.0;
        }
        driverDesiredSpeeds.vxMetersPerSecond = forward;
        driverDesiredSpeeds.vyMetersPerSecond = strafe;
        driverDesiredSpeeds.omegaRadiansPerSecond = rotate;

        return driverDesiredSpeeds;

    }

    

    public void setState(SwerveState newState) {
        state = newState;
    }

    public SwerveState getState() {
        return state;
    }



    public Pose2d getPose() {
        return swerveDriveState.Pose;
    }

    public ChassisSpeeds getRobotSpeeds() {
        return swerveDriveState.Speeds;
    }

    public double getRobotRotationSpeed() {
        return currTopRotationSpeedPercent * SwerveConstants.maxRotSpeed;
    }

    public double getRobotTopSpeed() {
        return currTopSpeedPercent * SwerveConstants.maxSpeed;
    }

    public void setRobotTopSpeeds(double newTopSpeed, double newTopRotationSpeed) {
        this.currTopSpeedPercent = newTopSpeed;
        this.currTopRotationSpeedPercent = newTopRotationSpeed;
    }

    public Command setGyroToZero() {
        return Commands.runOnce(() -> drivetrain.getPigeon2().setYaw(0));
    }



    public void setSnapPoint(Translation2d snapPoint){
      //sets the point to look at while in snap
      this.snapPoint=snapPoint;
    }

    public void setSnapAngle(Rotation2d snapAngle){
      this.snapAngle=snapAngle;

    }
    
    public void setDriveToPose(Pose2d targetPose,double translationToleranceMeters, double maxSpeed, double rotationToleranceDegrees, double maxAngularSpeed){
      this.driveToPoseTargetPose=targetPose;
      this.driveToPoseTranslationToleranceMeters=translationToleranceMeters;
      this.driveToPoseMaxSpeed=maxSpeed;
      this.driveToPoseMaxAngularSpeed=maxAngularSpeed;
      this.driveToPoseRotationToleranceDegrees = rotationToleranceDegrees;
    }

    public void setDriveToFieldRelativeOffset(Transform2d vectorToMove,double translationToleranceMeters, double maxSpeed, double rotationToleranceDegrees, double maxAngularSpeed){
      this.driveToPoseTargetPose=swerveDriveState.Pose.plus(vectorToMove);
      setDriveToPose(driveToPoseTargetPose, translationToleranceMeters, maxSpeed, rotationToleranceDegrees, maxAngularSpeed);
    }

    public void setDriveToRobotRelativeOffset(Transform2d vectorToMoveRobotFrame,double translationToleranceMeters, double maxSpeed, double rotationToleranceDegrees, double maxAngularSpeed){
      this.driveToPoseTargetPose=swerveDriveState.Pose.plus(vectorToMoveRobotFrame.plus(new Transform2d(Translation2d.kZero,swerveDriveState.Pose.getRotation())));
      setDriveToPose(driveToPoseTargetPose, translationToleranceMeters, maxSpeed, rotationToleranceDegrees, maxAngularSpeed);
    }

 
    //todo add distance to target 
    
    public boolean isAtDriveToPoseSetpoint() {

        DogLog.log("Swerve/isAlignAtGoal/distance",
                getPose().getTranslation().getDistance(driveToPoseTargetPose.getTranslation()));
        DogLog.log("Swerve/isAlignAtGoal/angle",
                FieldUtil.angleBetweenRotation2ds(driveToPoseTargetPose.getRotation(), getPose().getRotation()));

        DogLog.log("Swerve/isAlignAtGoal/distanceBool",
                getPose().getTranslation().getDistance(driveToPoseTargetPose.getTranslation()) < driveToPoseTranslationToleranceMeters);
        DogLog.log("Swerve/isAlignAtGoal/angleBool", FieldUtil.angleBetweenRotation2ds(driveToPoseTargetPose.getRotation(),
                getPose().getRotation()) < driveToPoseRotationToleranceDegrees);

        return getPose().getTranslation().getDistance(driveToPoseTargetPose.getTranslation()) < driveToPoseTranslationToleranceMeters
                && FieldUtil.angleBetweenRotation2ds(driveToPoseTargetPose.getRotation(),
                        getPose().getRotation()) < driveToPoseRotationToleranceDegrees;

    }

    

  

    @Override
    public void periodic() {

        if (SwerveConstants.useLimelight) {

            addVisionPosesToPoseEstimator();
        }

        swerveDriveState = drivetrain.getState();
        lastSwerveDriveState = swerveDriveState;
        swerveDriveState = drivetrain.getState();

        double currentTime = Timer.getFPGATimestamp();
        double robotSpeed = new Translation2d(swerveDriveState.Speeds.vxMetersPerSecond,
                swerveDriveState.Speeds.vyMetersPerSecond).getNorm();
        DogLog.log("Swerve/ModuleStates", swerveDriveState.ModuleStates);
        DogLog.log("Swerve/EstimatedPose", swerveDriveState.Pose);
        DogLog.log("Swerve/TopSpeedPercent", currTopSpeedPercent);
        DogLog.log("Swerve/TopRotationSpeedPercent", currTopRotationSpeedPercent);
        DogLog.log("Swerve/State", state);

        DogLog.log("Swerve/Speeds", swerveDriveState.Speeds);
        
        

        getTeleopDriveSpeeds();

        if (Math.abs(driverDesiredSpeeds.omegaRadiansPerSecond) > SwerveConstants.rightXDeadband) {
            rotationJoystickLastTouched = currentTime;
            
            if (state == SwerveState.SNAP || state == SwerveState.SNAP_POINT ) {
                state = SwerveState.TELEOP_DRIVE;
            }

        }

        if (robotSpeed > 1) {
            highSpeedLastTime = currentTime;
        }
        if (state != SwerveState.TELEOP_DRIVE) {
            lastMaintainHeadingAngle = Optional.empty();
        }

        DogLog.log("Swerve/TeleopDesiredSpeeds", driverDesiredSpeeds);
      
        switch (state) {
            case TELEOP_DRIVE:
                if (Math.abs(driverDesiredSpeeds.omegaRadiansPerSecond) > SwerveConstants.rightXDeadband
                        || lastMaintainHeadingAngle.isEmpty()
                        || ((currentTime - highSpeedLastTime) > 0.1)
                        || ((currentTime - rotationJoystickLastTouched < 0.2))) {

                    drivetrain
                            .setControl(drive_field_rel
                                    .withVelocityX(driverDesiredSpeeds.vxMetersPerSecond * getRobotTopSpeed())
                                    .withVelocityY(driverDesiredSpeeds.vyMetersPerSecond * getRobotTopSpeed())
                                    .withRotationalRate(
                                            driverDesiredSpeeds.omegaRadiansPerSecond * getRobotRotationSpeed()));
                    lastMaintainHeadingAngle = Optional.of(swerveDriveState.Pose.getRotation());

                } else {
                    drivetrain.setControl(drive_snap
                            .withVelocityX(driverDesiredSpeeds.vxMetersPerSecond * getRobotTopSpeed())
                            .withVelocityY(driverDesiredSpeeds.vyMetersPerSecond * getRobotTopSpeed())
                            .withTargetDirection(lastMaintainHeadingAngle.get()));

                }

                timerHasBeenEnabled = false;

                break;
            case SNAP:
                   drivetrain.setControl(drive_snap
                            .withVelocityX(driverDesiredSpeeds.vxMetersPerSecond * getRobotTopSpeed())
                            .withVelocityY(driverDesiredSpeeds.vyMetersPerSecond * getRobotTopSpeed())
                            .withTargetDirection(snapAngle));
                break;

            case SNAP_POINT:
                   drivetrain.setControl(drive_snap
                            .withVelocityX(driverDesiredSpeeds.vxMetersPerSecond * getRobotTopSpeed())
                            .withVelocityY(driverDesiredSpeeds.vyMetersPerSecond * getRobotTopSpeed())
                            .withTargetDirection(FieldUtil.getFieldRelativeAngleToPose(swerveDriveState.Pose,snapPoint)));
                break;
            case DRIVE_TO_POSE:
                Translation2d error = driveToPoseTargetPose.getTranslation().minus(swerveDriveState.Pose.getTranslation());
                double distanceToGoal = error.getNorm();
                Rotation2d directionOfTravel = error.getAngle();
                double velocityOutput = 0.0;
                if (DriverStation.isAutonomous()){
                  velocityOutput = Math.min(
                            Math.abs(SwerveConstants.autoDriveToPoseController.calculate(distanceToGoal, 0)) ,
                            driveToPoseMaxSpeed);
                } else {
                    velocityOutput = Math.min(
                            Math.abs(SwerveConstants.teleopDriveToPoseController.calculate(distanceToGoal, 0)) ,
                            driveToPoseMaxSpeed);
                }
                double xComponent = velocityOutput * directionOfTravel.getCos();
                double yComponent = velocityOutput * directionOfTravel.getSin();

                DogLog.log("Swerve/DriveToPoint/xVelocitySetpoint", xComponent);
                DogLog.log("Swerve/DriveToPoint/yVelocitySetpoint", yComponent);
                DogLog.log("Swerve/DriveToPoint/velocityOutput", velocityOutput);
                DogLog.log("Swerve/DriveToPoint/linearDistance", distanceToGoal);
                DogLog.log("Swerve/DriveToPoint/directionOfTravel", directionOfTravel);
                DogLog.log("Swerve/DriveToPoint/desiredPoint", driveToPoseTargetPose);


                drivetrain.setControl(drive_snap
                  .withVelocityX(xComponent)
                  .withVelocityY(yComponent)
                  .withTargetDirection(driveToPoseTargetPose.getRotation())
                  .withMaxAbsRotationalRate(driveToPoseMaxAngularSpeed)
                );
                break;
            case CALIBRATION:

                break;

            default:
                break;

        }
    }

    public void addVisionPosesToPoseEstimator() {

        for (String limelightName : limelightNames) {

            LimelightHelpers.SetRobotOrientation(limelightName, swerveDriveState.Pose.getRotation().getDegrees(), 0,
                    0, 0,
                    0, 0);

            boolean isDisabled = DriverStation.isDisabled();
            if (isDisabled) {
                LimelightHelpers.SetFiducialDownscalingOverride(limelightName, 1);
                LimelightHelpers.SetThrottle(limelightName, 100);
            } else {
                LimelightHelpers.SetFiducialDownscalingOverride(limelightName, 0);
                LimelightHelpers.SetThrottle(limelightName, 0);
            }
            DogLog.log("Swerve/" + limelightName + "HeartBeat",
                    NetworkTableInstance.getDefault().getTable(limelightName).getEntry("hb").getDouble(0));
            DogLog.log("Swerve/" + limelightName + "TV", LimelightHelpers.getTV(limelightName));
            if (limelightName.equals("limelight-hp")) {

                continue;

            }
            PoseEstimate estimatePoseMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
            PoseEstimate estimatePoseMT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
            PoseEstimate poseToAdd;

            if (estimatePoseMT1 == null || estimatePoseMT2 == null) {
                continue;
            }

            // This prevents pose estimator from having crazy poses if the Limelight loses
            // power
            if (estimatePoseMT2.tagCount == 0
                    || estimatePoseMT2.pose.getX() == 0.0 && estimatePoseMT2.pose.getY() == 0.0) {
                continue;
            }
            if (estimatePoseMT2.timestampSeconds == lastAddedVisionTimestampMap.getOrDefault(limelightName, 0.0)) {
                continue;
            }

            DogLog.log("Swerve/" + limelightName + "_avgTagArea", estimatePoseMT1.avgTagArea);
            if (estimatePoseMT1.avgTagArea >= (isDisabled ? 0.1 : 1.5) && !limelightName.equals("limelight-hp")) {

                poseToAdd = estimatePoseMT1;
                stdDevs = isDisabled ? SwerveConstants.megaTag1DisabledstdDev : SwerveConstants.megaTag1stdDev;

            } else {
                poseToAdd = estimatePoseMT2;
                stdDevs = SwerveConstants.megaTag2stdDev;
            }

            DogLog.log("Swerve/" + limelightName + "_PoseMT2", estimatePoseMT2.pose);
            DogLog.log("Swerve/" + limelightName + "_PoseMT1", estimatePoseMT1.pose);
            DogLog.log("Swerve/" + limelightName + "_lastAddedPose", poseToAdd.pose);

            drivetrain.addVisionMeasurement(poseToAdd.pose, Utils.fpgaToCurrentTime(poseToAdd.timestampSeconds),
                    stdDevs);
            lastAddedVisionTimestampMap.put(limelightName, Utils.fpgaToCurrentTime(poseToAdd.timestampSeconds));

        }

    }

    



    private void startSimThread() {
        System.out.println("Starting Sim thread");
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            drivetrain.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

}