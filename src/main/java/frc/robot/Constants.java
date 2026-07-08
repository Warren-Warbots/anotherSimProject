package frc.robot;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;

public class Constants {

        public static boolean IS_AT_COMP = false;

        public static final String BETA_SERIAL_NUMBER = "-"; // 0329F366
        public static final String SERIAL_NUMBER = System.getenv("serialnum");

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

        // Intake
        public static final int intake_Motor_ID = 20;
        public static final int intake_CANrange_ID = 21;

        // Pivator
        public static final int pivot_Motor_ID = 30;
        public static final int pivot_CANcoder_ID = 31;

        // Elevator
        public static final int elevator_Front_Motor_ID = 40;
        public static final int elevator_Back_Motor_ID = 41;

        // Motor IDs here
        public static final int lights_ID = 55;

}
