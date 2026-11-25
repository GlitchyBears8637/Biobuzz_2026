package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class PathsRed {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose start = poseFactory.of(58.5, 9, 90);
    public final Pose moveForwardAfterShoot = poseFactory.of(58.5, 24, 90);
    public final Pose moveTurnTowardsGardenStart = poseFactory.of(58.5, 24, 90);
    public final Pose moveTurnTowardsGarden = poseFactory.of(36, 9, 180);
    public final Pose grabGardenBalls = poseFactory.of(9, 9, 180);
    public final Pose moveBackFromGarden = poseFactory.of(36, 9, 180);
    public final Pose moveFurtherFromGardenStart = poseFactory.of(36, 9, 180);
    public final Pose moveFurtherFromGarden = poseFactory.of(58.5, 24, 90);
    public final Pose moveBackToShoot = poseFactory.of(58.5, 9, 90);
    public final Pose parkStart = poseFactory.of(58.5, 9, 90);
    public final Pose park = poseFactory.of(9, 96, 360);
    public final Pose parkControl1 = poseFactory.of(17.7917, 43.0193, 0);

    public Path moveForwardAfterShoot() {
        return Paths.line(start, moveForwardAfterShoot).linear(start, moveForwardAfterShoot);
    }

    public Path moveTurnTowardsGarden() {
        return Paths.line(moveTurnTowardsGardenStart, moveTurnTowardsGarden).linear(moveTurnTowardsGardenStart, moveTurnTowardsGarden);
    }

    public Path grabGardenBalls() {
        return Paths.line(moveTurnTowardsGarden, grabGardenBalls).constant(grabGardenBalls);
    }

    public Path moveBackFromGarden() {
        return Paths.line(grabGardenBalls, moveBackFromGarden).reverseTangent();
    }

    public Path moveFurtherFromGarden() {
        return Paths.line(moveFurtherFromGardenStart, moveFurtherFromGarden).linear(moveFurtherFromGardenStart, moveFurtherFromGarden);
    }

    public Path moveBackToShoot() {
        return Paths.line(moveFurtherFromGarden, moveBackToShoot).reverseTangent();
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}
