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
import org.firstinspires.ftc.teamcode.subsystem.RingSensors;
import org.firstinspires.ftc.teamcode.subsystem.RingTranstition;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.WebCam;

@Autonomous(name = "Auto Shoot", group = "Exercises")
public class TestAutoShoot extends LinearOpMode {
    private static final double TURN_SPEED = 0.5;
    private ElapsedTime timer;
    private ElapsedTime pauseTransitionTime;
    enum transitionShooterMode {
        Advancing, Pause, Ready, Shooting, ClearingFront, ClearingBack
    }

    double shot = 0;
    double webCamHeading;
    WebCam webCam;
    Shooter shooter;
    ImuPIDTurning IMU;
    AutoDrive autoDrive;
    Drive drive;
    RingTranstition ringTransition;
    RingSensors disSensors;
    masterRobot.transitionShooterMode transitionState = masterRobot.transitionShooterMode.Advancing;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        webCam = new WebCam(telemetry,hardwareMap);
        webCam.init();
        IMU = new ImuPIDTurning(telemetry, hardwareMap);
        IMU.init();
        drive = new Drive(telemetry, hardwareMap);
        drive.init();
        autoDrive = new AutoDrive(telemetry, drive);
        shooter = new Shooter(telemetry, hardwareMap);
        shooter.init();
        ringTransition = new RingTranstition(telemetry, hardwareMap);
        ringTransition.init();
        disSensors = new RingSensors(telemetry,hardwareMap);
        disSensors.init();
        timer = new ElapsedTime();
        pauseTransitionTime = new ElapsedTime();

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

        autoDrive.encoderDrive(0.5, 57, 57, 10);
        sleep(1000);
//        if (startHeading - IMU.getAngle() < 1 && startHeading - IMU.getAngle() > -1)
//        IMU.rotate(10, 0.3, 10);
//        IMU.rotate(startHeading - IMU.getAngle(), 0.3, 10);

//        timer.reset();
//        while (timer.seconds() < 5){
//            shooter.autoSetShootOn(true);
//            shooter.loop();
//            if (!shooter.isUpToSpeed()){
//                ringTransition.doNothingMode();
//                ringTransition.runMotors();
//            } else {
//                ringTransition.shootingTransitionMode();
//                ringTransition.runMotors();
//            }
//        }
//        timer.reset();
//        while (timer.milliseconds()> 2000){
//            webCam.execute();
//            webCamHeading = webCam.camHeading();
//        }
//        IMU.rotate(webCamHeading-90,0.4,2);

//        while (!shooter.isUpToSpeed()){
//            shooter.autoSetShootOn(true);
//            shooter.loop();
//        }
        shooter.setShootOn(true);
        timer.reset();
        while (shot < 3 && timer.milliseconds() < 10000) {
            if (transitionState == masterRobot.transitionShooterMode.Advancing) {
                if (disSensors.isRingInEle()) {
                    transitionState = masterRobot.transitionShooterMode.Pause;
                } else if (disSensors.isRingInForward()){
                    transitionState = masterRobot.transitionShooterMode.ClearingFront;
                } else {
                    ringTransition.advancingTransitionMode();
                }
            }
            if (transitionState == masterRobot.transitionShooterMode.Pause) {
                if (pauseTransitionTime.milliseconds() > 1500) {
                    transitionState = masterRobot.transitionShooterMode.Ready;
                } else {
                    ringTransition.doNothingMode();
                }
            } else {
                pauseTransitionTime.reset();
            }
            if (transitionState == masterRobot.transitionShooterMode.Ready) {
                if (shooter.isUpToSpeed()) {
                    transitionState = masterRobot.transitionShooterMode.Shooting;
                } else {
                    ringTransition.doNothingMode();
                }
            }
            if (transitionState == masterRobot.transitionShooterMode.Shooting) {
                if (!shooter.isUpToSpeed()) {
                    shot = shot + 1;
                    if (disSensors.isRingInForward()){
                        transitionState = masterRobot.transitionShooterMode.ClearingFront;
                    } else {
                        transitionState = masterRobot.transitionShooterMode.ClearingBack;
                    }
                } else {
                    ringTransition.shootingTransitionMode();
                }
            }
            if (transitionState == masterRobot.transitionShooterMode.ClearingFront) {
                if (disSensors.isRingInEle()) {
                    transitionState = masterRobot.transitionShooterMode.ClearingBack;
                } else {
                    ringTransition.upperTransitionOuttake();
                }
            }
            if (transitionState == masterRobot.transitionShooterMode.ClearingBack){
                if (!disSensors.isRingInEle()){
                    transitionState = masterRobot.transitionShooterMode.Advancing;
                } else {
                    ringTransition.upperTransitionOuttake();
                }
            }
            ringTransition.runMotors();
            shooter.loop();
            telemetry.addData("im HERE", true);
            telemetry.addData("trans state", transitionState.toString());
            telemetry.addData("how many shot", shot);
            telemetry.update();
        }
        shooter.setShootOn(false);
        ringTransition.doNothingMode();
        shooter.loop();
        ringTransition.runMotors();
        autoDrive.encoderDrive(0.5,8,8,10);
        IMU.rotate(-90,0.3, 2);
        autoDrive.encoderDrive(0.5,24,24,10);
        autoDrive.encoderDrive(0.5,-8,-8,10);
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