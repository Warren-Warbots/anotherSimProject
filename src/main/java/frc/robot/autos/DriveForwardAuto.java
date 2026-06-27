package frc.robot.autos;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.robot_manager.WantedRobotState;
import frc.robot.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class DriveForwardAuto extends WarbotAuto {

    private enum State {
        START,
        TEST_1,
        TEST_2,
        DONE;

        private static final State[] vals = values();

        public State next() {
            return vals[Math.min(this.ordinal() + 1, vals.length - 1)];
        }
    }

    private State currentState = State.START;
    private Timer stateTimer = new Timer();

    private static final Pose2d[] UNDER_TRENCH_1 = { p(5.151, 0.637, 0.1), p(7.370, 0.680, 2.4) };
    private static final Pose2d[] TEST_1 = { p(7.370, 0.680, 0.1), p(7.714, 2.909, 2.4) };
    private static final Pose2d[] TEST_2 = { p(7.728, 2.894, 0.1), p(5.395, 2.923, 2.4) };

    public DriveForwardAuto() {
    }

    @Override
    public void init() {
        currentState = State.START;
        stateTimer.reset();
    }

    @Override
    public void periodic() {
        DogLog.log("AutoState", currentState);
        switch (currentState) {
            case START:
                resetSwervePose(PathFollower.applyFlipping(UNDER_TRENCH_1[0], mirror));
                stateTimer.restart();
                currentState = currentState.next();
                break;

            case TEST_1:
                if (manager.drivePath(TEST_1, 2.0, 3.5, 0.4, true, mirror)) {
                    if (manager.swerve.velocityAtGoal()) {
                        currentState = currentState.next();
                    }
                }
                break;

            case TEST_2:
                if (manager.drivePath(TEST_2, 2.0, 3.5, 0.4, false, mirror)) {
                    if (manager.swerve.velocityAtGoal()) {
                        currentState = currentState.next();
                    }
                }
                break;

            case DONE:
                isFinished = true;
                break;
        }
    }
}