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
    private ElapsedTime timeOut;
    private ElapsedTime loopTimer;
    private ElapsedTime linningUpTime;

    enum transitionShooterMode {
        Advancing, Pause, Ready, Shooting, ClearingFront, ClearingBack, clearingMid
    }

    enum liningUpMode {
        strafing, rotating, distancing, rotating2, rotating3
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
    transitionShooterMode transitionState = transitionShooterMode.Ready;
    liningUpMode liningState = liningUpMode.strafing;

    double lastLoop;

    public double strafeCurve(double distance){
        return -0.756*distance*distance+52.689*distance+148.74;
    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        webCam = new WebCam(telemetry, hardwareMap);
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
        disSensors = new RingSensors(telemetry, hardwareMap);
        disSensors.init();
        timer = new ElapsedTime();
        pauseTransitionTime = new ElapsedTime();
        timeOut = new ElapsedTime();
        loopTimer = new ElapsedTime();
        linningUpTime = new ElapsedTime();

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.addData("angle", IMU.getAngle());
        telemetry.update();
        loopTimer.reset();
        double startHeading = IMU.getAngle() + -2;

        //        claw.SetPosition(Claw_Servo.CLOSED);
//
//        while () {
//        }

        autoDrive.encoderDrive(0.6, 60, 60, 10);
        sleep(650);
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
        timer.reset();
        linningUpTime.reset();
        shooter.setShootOn(true);
        while (timer.milliseconds() < 7000) {
            webCam.execute();
            if (webCam.isTargetVisible()) {
                if (liningState == liningUpMode.strafing) {
                    if (linningUpTime.milliseconds() > 250) {
                        double camYDif = -26.5 - webCam.camY();
                        double distanceByCam = strafeCurve(camYDif);
                        if (Math.abs(camYDif) > 0.5) {
                            autoDrive.timeStrafe(0.75 * Math.signum(camYDif), Math.abs(distanceByCam));
                        }
                        liningState = liningUpMode.rotating;
                        linningUpTime.reset();
                    }
                }
                if (liningState == liningUpMode.rotating) {
                    if (linningUpTime.milliseconds() > 500) {
                        if (Math.abs(87 - webCam.camHeading()) > 1) {
                            IMU.rotate(87 - webCam.camHeading(), 0.4, 2);
                        }
                        liningState = liningUpMode.distancing;
                        linningUpTime.reset();
                    }
                }
                if (liningState == liningUpMode.distancing) {
                    if (linningUpTime.milliseconds() > 250) {
                        double camZDif = 8 - webCam.x;
                        autoDrive.encoderDrive(0.3, camZDif, camZDif, 2);
                        liningState = liningUpMode.rotating3;
                        break;
                    }
                }
            }
        }
//        while (!shooter.isUpToSpeed()){
//            shooter.autoSetShootOn(true);
//            shooter.loop();
//        }
        transitionState = transitionShooterMode.Ready;
        timer.reset();
        timeOut.reset();
        while (timer.milliseconds() < 20500) {
            webCam.execute();
            if (transitionState == transitionShooterMode.Advancing) {
                if (disSensors.isRingInEle() || timeOut.milliseconds() > 4000) {
                    transitionState = transitionShooterMode.Pause;
                    timeOut.reset();
                } else if (disSensors.isRingInForward()) {
                    transitionState = transitionShooterMode.ClearingFront;
                } else {
                    ringTransition.advancingTransitionMode();
                }
            }
            if (transitionState == transitionShooterMode.Pause) {
                if (pauseTransitionTime.milliseconds() > 500) {
                    transitionState = transitionShooterMode.Ready;
                    timeOut.reset();
                } else {
                    ringTransition.doNothingMode();
                }
            } else {
                pauseTransitionTime.reset();
            }
            if (transitionState == transitionShooterMode.Ready) {
                if (shooter.isUpToSpeed() || timeOut.milliseconds() > 5000) {
                    transitionState = transitionShooterMode.Shooting;
                    timeOut.reset();
                } else {
                    ringTransition.doNothingMode();
                }
            }
            if (transitionState == transitionShooterMode.Shooting) {
                if (!shooter.isUpToSpeed() || timeOut.milliseconds() > 2000) {
//                    shot = shot + 1;
                    timeOut.reset();
                    if (disSensors.isRingInForward()) {
                        transitionState = transitionShooterMode.ClearingFront;
                    } else {
                        transitionState = transitionShooterMode.ClearingBack;
                    }
                } else {
                    ringTransition.shootingTransitionMode();
                }
            }
            if (transitionState == transitionShooterMode.ClearingFront) {

                if ((disSensors.isRingInEle() && !disSensors.isRingInForward()) || timeOut.milliseconds() > 2000) {
                    transitionState = transitionShooterMode.ClearingBack;
                    timeOut.reset();
                } else {
                    ringTransition.upperTransitionOuttake();
                    ringTransition.lowerDoNothing();
                }
            }
            if (transitionState == transitionShooterMode.ClearingBack) {
                if (!disSensors.isRingInEle() || timeOut.milliseconds() > 2000) {
                    transitionState = transitionShooterMode.Advancing;
                    timeOut.reset();
                } else {
                    ringTransition.upperTransitionOuttake();
                    ringTransition.lowerDoNothing();
                }
            }
            ringTransition.runMotors();
            shooter.loop();
            //            telemetry.addData("im HERE", true);
//            telemetry.addData("trans state", transitionState.toString());
//            telemetry.addData("how many shot", shot);
//            ringTransition.telemetery();
        }
        shooter.setShootOn(false);
        ringTransition.doNothingMode();
        shooter.loop();
        ringTransition.runMotors();
        autoDrive.encoderDrive(0.5, 7, 7, 10);
//        IMU.rotate(-90,0.3, 2);
//        autoDrive.encoderDrive(0.5,24,24,10);
//        autoDrive.encoderDrive(0.5,-8,-8,10);
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