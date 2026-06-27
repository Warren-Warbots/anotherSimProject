package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_manager.WantedRobotState;
import frc.robot.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class DriveForwardAuto extends WarbotAuto {

    private enum State {
        START,
        DONE;

        private static final State[] vals = values();

        public State next() {
            return vals[Math.min(this.ordinal() + 1, vals.length - 1)];
        }
    }

    private State currentState = State.START;
    private Timer stateTimer = new Timer();

    private static final Pose2d[] UNDER_TRENCH_1 = { p(4.890, 0.620, 0.1), p(6.747, 0.890, 55.1) };
    private static final Pose2d[] NOT_UNDER_TRENCH_1 = { p(7.118, 3.416, 0.1), p(6.742, 0.889, 55.1) };

    public DriveForwardAuto() {
    }

    @Override
    public void init() {
        currentState = State.START;
        stateTimer.reset();
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case START:
                resetSwervePose(PathFollower.applyFlipping(UNDER_TRENCH_1[0], mirror));
                manager.setWantedRobotState(WantedRobotState.DRIVE_WITH_VELOCITY);
                stateTimer.restart();
                currentState = currentState.next();
                break;

            case DONE:
                isFinished = true;
                break;
        }
    }
}