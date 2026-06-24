// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.example_pivator_subsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class PivatorConstants {

        public static double minElevatorHeight = 0;
        public static double maxElevatorHeight = 31.375;

        public static double minPivotRot = -0.1;
        public static double maxPivotRot = 0.7;

        public static double elevatorTolerance = 0.5;
        public static double pivotTolerance = 1.0 / 360;

        public static TalonFXConfiguration elevatorMotorFrontConfig = new TalonFXConfiguration()
                        .withCurrentLimits(
                                        new CurrentLimitsConfigs()
                                                        .withSupplyCurrentLimit(Constants.IS_COMP_BOT ? 85 : 85)
                                                        .withStatorCurrentLimit(Constants.IS_COMP_BOT ? 85 : 45)
                                                        .withSupplyCurrentLimitEnable(
                                                                        Constants.IS_COMP_BOT ? true : true)
                                                        .withStatorCurrentLimitEnable(
                                                                        Constants.IS_COMP_BOT ? true : true))
                        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(5 / (Math.PI * 1.751)))
                        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
                                        .withNeutralMode(NeutralModeValue.Brake))
                        .withVoltage(new VoltageConfigs().withPeakForwardVoltage(12)
                                        .withPeakReverseVoltage(-8))
                        .withSlot0(new Slot0Configs().withKG(0.74).withKP(14.0).withKD(0.0))
                        .withMotionMagic(
                                        new MotionMagicConfigs().withMotionMagicAcceleration(75)
                                                        .withMotionMagicCruiseVelocity(50));

        public static TalonFXConfiguration elevatorMotorBackConfig = new TalonFXConfiguration()
                        .withCurrentLimits(
                                        new CurrentLimitsConfigs()
                                                        .withSupplyCurrentLimit(Constants.IS_COMP_BOT ? 85 : 85)
                                                        .withStatorCurrentLimit(Constants.IS_COMP_BOT ? 85 : 45)
                                                        .withSupplyCurrentLimitEnable(
                                                                        Constants.IS_COMP_BOT ? true : true)
                                                        .withStatorCurrentLimitEnable(
                                                                        Constants.IS_COMP_BOT ? true : true))
                        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(5 / (Math.PI * 1.751)))
                        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        public static MotionMagicVoltage elevatorMotionMagicVoltage = new MotionMagicVoltage(0.0);

        public static VoltageOut elevatorVoltageOut = new VoltageOut(0.0);

        public static TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration()
                        .withCurrentLimits(
                                        new CurrentLimitsConfigs()
                                                        .withSupplyCurrentLimit(Constants.IS_COMP_BOT ? 85 : 85)
                                                        .withStatorCurrentLimit(Constants.IS_COMP_BOT ? 85 : 45)
                                                        .withSupplyCurrentLimitEnable(
                                                                        Constants.IS_COMP_BOT ? true : true)
                                                        .withStatorCurrentLimitEnable(
                                                                        Constants.IS_COMP_BOT ? true : true))

                        .withFeedback(new FeedbackConfigs().withRemoteCANcoder(new CoreCANcoder(33))
                                        .withRotorToSensorRatio(5.0 * 5.0 * (30.0 / 12.0)))
                        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                        .withSlot0(new Slot0Configs().withGravityType(GravityTypeValue.Arm_Cosine)
                                        .withKG(0.5)
                                        .withKP(100))
                        .withVoltage(new VoltageConfigs().withPeakForwardVoltage(12)
                                        .withPeakReverseVoltage(-12))
                        .withMotionMagic(
                                        new MotionMagicConfigs().withMotionMagicAcceleration(4.5)
                                                        .withMotionMagicCruiseVelocity(1));

        public static MotionMagicVoltage pivotMotionMagicVoltage = new MotionMagicVoltage(0.0);

}
