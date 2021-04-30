/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomouseMovement.AutoDrive;
import org.firstinspires.ftc.teamcode.autonomouseMovement.ImuPIDTurning;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.RingIntake;
import org.firstinspires.ftc.teamcode.subsystem.RingSensors;
import org.firstinspires.ftc.teamcode.subsystem.RingTranstition;
import org.firstinspires.ftc.teamcode.subsystem.RobotControls;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.WebCam;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "master", group = "AA 11109")
public class masterRobot extends OpMode {
    // Declare OpMode members.
    private static final double INCHES_PER_100_MILLISECONDS = 1.5;

    enum transitionShooterMode {
        Advancing, Pause, Ready, Shooting, ClearingFront, ClearingBack, ClearingMid
    }

    enum liningUpMode {
        strafing, rotating, distancing, rotating2
    }

    AutoDrive autoDrive;
    RingIntake intake;
    Drive Drive;
    RingTranstition transtition;
    RingSensors disSensors;
    RobotControls controls;
    Shooter shooter;
    //    IMU imu;
    ImuPIDTurning Imu;
    WebCam webCam;


    boolean isShooterOn = false;
    boolean shooterSpinUpState = false;
    boolean lastAutoShootState = false;

    boolean isIntakeOn = false;
    boolean intakeState = false;
    boolean lastIsUpToSpeed = false;
    boolean justShot = false;
    transitionShooterMode transitionState = transitionShooterMode.Advancing;
    liningUpMode liningState = liningUpMode.rotating2;

    boolean slowMode = false;

    double startHeading;
    double camYDif;
    double distanceByCam;
    double camZDif;

    boolean lineUpState = false;
    boolean lastLineUpState = false;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime controlTime = new ElapsedTime();
    private ElapsedTime pauseTransitionTime = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private ElapsedTime polyIntakeWait = new ElapsedTime();
    private ElapsedTime timeOut = new ElapsedTime();
    private ElapsedTime linningUpTime = new ElapsedTime();
    private double lastTime = 0;

    @Override
    public void init() {
        intake = new RingIntake(telemetry, hardwareMap);
        intake.init();
        Drive = new Drive(telemetry, hardwareMap);
        Drive.init();
        disSensors = new RingSensors(telemetry, hardwareMap);
        disSensors.init();
        transtition = new RingTranstition(telemetry, hardwareMap);
        transtition.init();
        controls = new RobotControls(gamepad1, gamepad2);
        shooter = new Shooter(telemetry, hardwareMap);
        shooter.init();
        Imu = new ImuPIDTurning(telemetry, hardwareMap);
        Imu.init();
        autoDrive = new AutoDrive(telemetry, Drive);
        autoDrive.init();

        //        imu = new IMU(telemetry, hardwareMap);
//        imu.init();
        webCam = new WebCam(telemetry, hardwareMap);
        webCam.init();

        telemetry.addData("ver", "1.391");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        Imu.setStartAngle();
        runtime.reset();
    }

    public double strafeDistanceToTime(double distance){
                return  -0.756 * distance * distance + 52.7 * distance + 149;
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        ////////////drive
//        Drive.drive(controls.strafe(), controls.forward(), controls.turn(), slowMode, -imu.getHeading(AngleUnit.DEGREES) );

// working robot centric mode
        if (controls.autoLineUp() && !lastLineUpState) {
            lineUpState = !lineUpState;
            liningState = liningUpMode.rotating2;
            linningUpTime.reset();
        }
        lastLineUpState = controls.autoLineUp();

        if (lineUpState && webCam.isTargetVisible()) {
            if (liningState == liningUpMode.rotating2) {
                if (Math.abs(87 - webCam.camHeading()) > 1) {
                    Imu.rotate(87 - webCam.camHeading(), 0.3, 2);
                }
                liningState = liningUpMode.strafing;
                linningUpTime.reset();
            }
            if (liningState == liningUpMode.strafing) {
                if (linningUpTime.milliseconds() > 500) {
                    camYDif = -26 - webCam.camY();
                    distanceByCam = strafeDistanceToTime(camYDif);
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
                        Imu.rotate(87 - webCam.camHeading(), 0.3, 2);
                    }
                    liningState = liningUpMode.distancing;
                    linningUpTime.reset();
                }
            }
            if (liningState == liningUpMode.distancing) {
                if (linningUpTime.milliseconds() > 250) {
                    camZDif = 8 - webCam.x;
                    autoDrive.encoderDrive(0.3, camZDif, camZDif, 2);
                    lineUpState = false;
                }
            }

        } else {
            Drive.drive(controls.strafe(), controls.forward(), controls.turn(), slowMode, 0);
        }


        if (gamepad2.x) {
            autoDrive.timeStrafe(0.75, strafeDistanceToTime(2));
        }
        if (gamepad2.a) {
            autoDrive.timeStrafe(0.75, strafeDistanceToTime(5));
        }
        if (gamepad2.y) {
            autoDrive.timeStrafe(0.75, strafeDistanceToTime(10));
        }

        if (controls.slowMode()) {
            slowMode = true;
        }

        if (controls.speedMode()) {
            slowMode = false;
        }
        /////////////transition
        transtition.doNothingMode();

        // feeder states: Advancing, Pause, Ready
        //

        if (isShooterOn) {
            if (transitionState == transitionShooterMode.Advancing) {
                if (disSensors.isRingInEle()) {
                    transitionState = transitionShooterMode.Pause;
                    timeOut.reset();
                } else if (disSensors.isRingInForward()) {
                    transitionState = transitionShooterMode.ClearingFront;
                    timeOut.reset();
                } else {
                    transtition.advancingTransitionMode();
                }
            }
            if (transitionState == transitionShooterMode.Pause) {
                if (pauseTransitionTime.milliseconds() > 500) {
                    transitionState = transitionShooterMode.Ready;
                    timeOut.reset();
                } else {
                    transtition.doNothingMode();
                }
            } else {
                pauseTransitionTime.reset();
            }
            if (transitionState == transitionShooterMode.Ready) {
                if (shooter.isUpToSpeed() && controls.shootToggle()) {
                    transitionState = transitionShooterMode.Shooting;
                    shooter.ResetMinMax();
                    timeOut.reset();
                } else {
                    transtition.doNothingMode();
                }
            }
            if (transitionState == transitionShooterMode.Shooting) {
                if (!shooter.isUpToSpeed()) {
                    timeOut.reset();
                    if (disSensors.isRingInForward()) {
                        transitionState = transitionShooterMode.ClearingFront;
                    } else {
                        transitionState = transitionShooterMode.ClearingBack;
                    }
                } else {
                    transtition.shootingTransitionMode();
                }
            }
            if (transitionState == transitionShooterMode.ClearingFront) {
                if ((disSensors.isRingInEle() && !disSensors.isRingInForward()) || timeOut.milliseconds() > 2000) {
                    transitionState = transitionShooterMode.ClearingBack;
                    timeOut.reset();
                } else {
                    transtition.upperTransitionOuttake();
                    transtition.lowerDoNothing();
                }
            }
            if (transitionState == transitionShooterMode.ClearingBack) {
                if (!disSensors.isRingInEle() || timeOut.milliseconds() > 2000) {
                    transitionState = transitionShooterMode.Advancing;
                    timeOut.reset();
                } else {
                    transtition.upperTransitionOuttake();
                    transtition.lowerDoNothing();
                }
            }
        } else {
            transitionState = transitionShooterMode.Advancing;
            timeOut.reset();
        }


        if (disSensors.isRingInIntake() && !controls.shootToggle()) {
            if (polyIntakeWait.milliseconds() > 500) {
                transtition.lowerTranistionIntake();
            }
        } else {
            polyIntakeWait.reset();
        }
        if (disSensors.isRingInEle() && !controls.shootToggle()) {
            transtition.doNothingMode();
        }

        if (controls.lowerPolycordIntake()) {
            transtition.lowerTranistionIntake();
        }

        if (controls.spitOut()) {
            transtition.reverseIntakeTransitionMode();
        }
        if (controls.polycordIntake()) {
            transtition.shootingTransitionMode();
        }

        transtition.runMotors();

        /////////////intake

        if (controls.intakeToggle() && !intakeState) {
            if (controlTime.milliseconds() > 500) {
                isIntakeOn = !isIntakeOn;
                controlTime.reset();
            }
        }
        intakeState = controls.shooterSpinUp();
        intake.setIntakeOn(isIntakeOn);


        if (controls.outtake()) {
            intake.outtake();
        }

        ///////////shooter

//        if (!shooter.isUpToSpeed() && lastIsUpToSpeed) {
//            justShot = true;
//        }
//        lastIsUpToSpeed = shooter.isUpToSpeed();
//
//        if (controls.autoShootToggle()) {
//            autoShoot();
//        }
        if (controls.shooterSpinUp2() && !shooterSpinUpState && controlTime.milliseconds() > 250) {
            isShooterOn = !isShooterOn;
            shooter.setShootOn(isShooterOn);
            controlTime.reset();
        }
        shooterSpinUpState = controls.shooterSpinUp();


        if (controls.increaseShooterSpeed() && controlTime.milliseconds() > 500) {
            shooter.increaseSpeed();
            controlTime.reset();
        }


        if (controls.decreaseShooterSpeed() && controlTime.milliseconds() > 500) {
            shooter.decreaseSpeed();
            controlTime.reset();
        }

        shooter.loop();

        /////////////webcam
        webCam.execute();

        /////////////telemetry
//        telemetry.addData("loop time", loopTimer.milliseconds() - lastTime);
//        lastTime = loopTimer.milliseconds();
        disSensors.telemetry();
//        imu.telemetry();
        transtition.telemetery();
//        intake.telemetry();
//        shooter.telemetry();
//        telemetry.addData("trans state", transitionState);
//        telemetry.addData("just shot", justShot);
//        telemetry.addData("is up to speed", shooter.isUpToSpeed());
//        telemetry.addData("last is up", lastIsUpToSpeed);
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    @Override
    public void stop() {
    }

    public void autoShoot() {
        ElapsedTime autoTime = new ElapsedTime();
        shooter.shooterSpinUp();
        while (!shooter.isUpToSpeed()) {
            //TODO:get toggle to work with return
            if (controls.autoShootToggle()) {
                shooter.doNothing();
                return;
            }
        }
        autoTime.reset();
        transtition.shootingTransitionMode();
        while (autoTime.seconds() < 5) {
            //TODO:get toggle to work with break
            if (controls.autoShootToggle()) {
                break;
            }
        }
        shooter.doNothing();
        transtition.doNothingMode();
    }
}
