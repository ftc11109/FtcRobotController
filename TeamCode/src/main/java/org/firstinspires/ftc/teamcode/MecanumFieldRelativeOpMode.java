package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.util.Polar;

@TeleOp(name = "mecanum field relative opmode", group = "ftc16072")
public class MecanumFieldRelativeOpMode extends OpMode {
    private Robot robot = new Robot();

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
        double strafe = gamepad1.left_stick_x;

        Polar g1RightJoystick = Polar.fromCartesian(gamepad1.right_stick_x, -gamepad1.right_stick_y);

        double r = g1RightJoystick.getR();
        if (r >= 0.1) {
            robot.nav.driveFieldRelativeAngle(strafe, forward, g1RightJoystick.getTheta());
        } else {
            robot.nav.driveFieldRelative(strafe, forward, 0.0);
        }

        telemetry.addData("direction",robot.nav.getImuAngularOrientation());
    }
}
