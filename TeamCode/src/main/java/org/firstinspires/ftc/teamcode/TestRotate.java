// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomouseMovement.AutoDrive;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.autonomouseMovement.ImuPIDTurning;


@Autonomous(name = "test rotate", group = "Exercises")
public class TestRotate extends LinearOpMode {
    private static final double TURN_SPEED = 0.5;
    private ElapsedTime timer;

    ImuPIDTurning IMU;
    AutoDrive autoDrive;
    Drive drive;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        IMU = new ImuPIDTurning(telemetry, hardwareMap);
        IMU.init();
        drive = new Drive(telemetry, hardwareMap);
        drive.init();
        autoDrive = new AutoDrive(telemetry,drive);
        timer = new ElapsedTime();

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.addData("angle", IMU.getAngle());
        telemetry.update();
        double startHeading = IMU.getAngle() + -2;
//        claw.SetPosition(Claw_Servo.CLOSED);
//
//        while () {
//        }

        autoDrive.encoderDrive(0.5,63,63,10);
        IMU.rotate(10, 0.3,10);
        IMU.rotate(startHeading - IMU.getAngle(), 0.3,10);
//        IMU.sleepAndLog(2000);
//        IMU.rotate(135, 0.5, 10);
//        IMU.sleepAndLog(2000);
//        IMU.rotate(-180, 0.40,10);
//        IMU.sleepAndLog(2000);
//        IMU.rotate(-90, 0.40,10);
//        IMU.sleepAndLog(2000);
//        IMU.rotate(0, 0.40,10);
//        IMU.sleepAndLog(2000);
//        IMU.rotate(90, 0.40,10);
//        IMU.sleepAndLog(2000);
//        IMU.rotate(180, 0.40,10);
//        IMU.sleepAndLog(2000);
    }
}