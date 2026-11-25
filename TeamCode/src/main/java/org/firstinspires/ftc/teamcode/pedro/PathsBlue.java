import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class Pathsblue {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(84, 132, 270);
    private final Pose moveForwardAfterShootBLUE = poseFactory.of(84, 118, 270);
    private final Pose moveTurnTowardsGardenBLUEStart = poseFactory.of(84, 118, 270);
    private final Pose moveTurnTowardsGardenBLUE = poseFactory.of(112, 132, 360);
    private final Pose grabGardenBallsBLUE = poseFactory.of(132, 132, 360);
    private final Pose moveBackFromGardenBLUE = poseFactory.of(112, 132, 0);
    private final Pose moveFurtherFromGardenBLUEStart = poseFactory.of(112, 132, 360);
    private final Pose moveFurtherFromGardenBLUE = poseFactory.of(84, 118, 270);
    private final Pose moveBackToShootBLUE = poseFactory.of(84, 132, -90);
    private final Pose parkBLUEStart = poseFactory.of(84, 132, 270);
    private final Pose parkBLUE = poseFactory.of(132, 48, 180);
    private final Pose parkBLUEControl1 = poseFactory.of(129.938, 90.186, 0);

    public Path moveForwardAfterShootBLUE() {
        return Paths.line(start, moveForwardAfterShootBLUE).linear(start, moveForwardAfterShootBLUE);
    }

    public Path moveTurnTowardsGardenBLUE() {
        return Paths.line(moveTurnTowardsGardenBLUEStart, moveTurnTowardsGardenBLUE).linear(moveTurnTowardsGardenBLUEStart, moveTurnTowardsGardenBLUE);
    }

    public Path grabGardenBallsBLUE() {
        return Paths.line(moveTurnTowardsGardenBLUE, grabGardenBallsBLUE).constant(grabGardenBallsBLUE);
    }

    public Path moveBackFromGardenBLUE() {
        return Paths.line(grabGardenBallsBLUE, moveBackFromGardenBLUE).reverseTangent();
    }

    public Path moveFurtherFromGardenBLUE() {
        return Paths.line(moveFurtherFromGardenBLUEStart, moveFurtherFromGardenBLUE).linear(moveFurtherFromGardenBLUEStart, moveFurtherFromGardenBLUE);
    }

    public Path moveBackToShootBLUE() {
        return Paths.line(moveFurtherFromGardenBLUE, moveBackToShootBLUE).reverseTangent();
    }

    public Path parkBLUE() {
        return Paths.curve(parkBLUEStart, parkBLUEControl1, parkBLUE).linear(parkBLUEStart, parkBLUE);
    }
