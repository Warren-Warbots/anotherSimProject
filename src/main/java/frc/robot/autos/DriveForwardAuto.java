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

    private static final Pose2d[] UNDER_TRENCH_1 = { p(4.89, 0.62, 0), p(6.57, 0.80, 0) };

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
                manager.setWantedRobotState(WantedRobotState.STOW);
                stateTimer.restart();
                currentState = currentState.next();
                break;

            case DONE:
                isFinished = true;
                break;
        }
    }
}