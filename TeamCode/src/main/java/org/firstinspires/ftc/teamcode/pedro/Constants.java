package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig mecanumConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("motorFrontLeft");
        c.frontRightName.set("motorFrontRight");
        c.backLeftName.set("motorBackLeft");
        c.backRightName.set("motorBackRight");

        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig pinpointConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(0.0);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller forward = Controller.proportional(0.05);
        Controller strafe = Controller.proportional(0.05);

        c.forwardTranslational.set(forward);
        c.strafeTranslational.set(strafe);
        c.headingFeedback.set(Controller.proportional(0.5));

        c.coast.set(Controller.proportionalFeedforward(0.0));
        c.brake.set(Controller.proportionalFeedforward(0.0));

        c.linearBrakeCoefficients.set(Matrix.diag(0.1, 0.1));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.01, 0.01));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.1, 0.01));

        c.maxAchievableForwardVelocity.set(50.0);
        c.maxAchievableStrafeVelocity.set(40.0);
        c.naturalForwardDeceleration.set(50.0);
        c.naturalStrafeDeceleration.set(50.0);
    });

    public static Follower create(HardwareMap h) {
        Mecanum drivetrain = new Mecanum(h, mecanumConfig);
        PinpointLocalizer localizer = new PinpointLocalizer(h, pinpointConfig);
        Foresight algorithm = new Foresight(foresightConfig);

        return new Follower(localizer, drivetrain, algorithm);
    }
}
