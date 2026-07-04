package frc.robot.autos;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.robot_manager.WantedRobotState;
import frc.robot.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

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

    private static final Pose2d[] UNDER_TRENCH_1 = { p(5.151, 0.637, 0.1), p(7.018, 0.639, -0.0) };
    private static final Pose2d[] TEST_1 = { p(7.814, 1.373, 0), p(7.874, 3.482, 0) };
    private static final Pose2d[] TEST_2 = { p(6.702, 2.779, 0), p(2.841, 2.626, 0) };

    public DriveForwardAuto() {
    }

    @Override
    public void init() {
        currentState = State.START;
        stateTimer.reset();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("AutoState", currentState);
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