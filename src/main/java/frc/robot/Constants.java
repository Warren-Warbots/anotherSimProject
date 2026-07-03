package frc.robot;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;

public class Constants {
        public static boolean IS_AT_COMP = false;


        /*Change the second value to change different runtime modes for AdvantageKit such as
        ROBOT for the real Robot,
        SIM for physics Sim,
        REPLAY to use replay editing from AdvantageKit.
        Do Not Touch Mode.Real, only edit the value after the colon
        */
        public static Mode robotMode = Robot.isReal() ? Mode.REAL : Mode.SIM;

        // public static final String BETA_SERIAL_NUMBER = "-";// 0329F366
        // public static final String SERIAL_NUMBER = System.getenv("serialnum");

        public static final boolean IS_COMP_BOT = true;

        public static String CANBUS_NAME = "rio";

        public static final ClosedLoopRampsConfigs CLOSED_LOOP_RAMP = new ClosedLoopRampsConfigs()
                        .withDutyCycleClosedLoopRampPeriod(0.04)
                        .withTorqueClosedLoopRampPeriod(0.04)
                        .withVoltageClosedLoopRampPeriod(0.04);
        public static final OpenLoopRampsConfigs OPEN_LOOP_RAMP = new OpenLoopRampsConfigs()
                        .withDutyCycleOpenLoopRampPeriod(0.04)
                        .withTorqueOpenLoopRampPeriod(0.04)
                        .withVoltageOpenLoopRampPeriod(0.04);

        // Motor IDs here
        public static final int Lights_ID = 55;
        public static final int Intake_Motor_ID = 19;
        public static final int CANrange_ID = 45;

        // Pivator
        public static final int pivotMotorId = 20;
        public static final int pivotCANcoder = 21;

        // Elevator
        public static final int elevatorMotorFrontId = 22;
        public static final int elevatorMotorBackId = 23;


        public static enum Mode{
                REAL,
                SIM,
                REPLAY;
        }
}
