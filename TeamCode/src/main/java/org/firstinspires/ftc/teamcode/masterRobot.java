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

import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.IMU;
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
    RingIntake intake;
    Drive Drive;
    RingTranstition transtition;
    RingSensors disSensors;
    RobotControls controls;
    Shooter shooter;
    IMU imu;
    WebCam webCam;


    boolean isShooterOn = false;
    boolean shooterSpinUpState = false;
    boolean lastAutoShootState = false;

    boolean isIntakeOn = false;
    boolean intakeState = false;

    boolean slowMode = true;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime controlTime = new ElapsedTime();
    private ElapsedTime polycordTime = new ElapsedTime();

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
        shooter = new Shooter(telemetry, hardwareMap, webCam);
        shooter.init();
        imu = new IMU(telemetry, hardwareMap);
        imu.init();
        webCam = new WebCam(telemetry, hardwareMap);
        webCam.init();

        telemetry.addData("ver","1.35");
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        ////////////drive
//        Drive.drive(controls.strafe(), controls.forward(), controls.turn(), slowMode, -imu.getHeading(AngleUnit.DEGREES) );

// working robot centric mode
        Drive.drive(controls.strafe(), controls.forward(), controls.turn(), slowMode, 0);

        if (controls.slowMode()){
            slowMode = true;
        }

        if (controls.speedMode()){
            slowMode = false;
        }
        /////////////transition
        transtition.doNothingMode();
        if (disSensors.isRingInIntake()) {
            transtition.intakeTransitionMode();

        }
        if (disSensors.isRingInEle()) {
            transtition.doNothingMode();
        }

        if (controls.spitOut()) {
            transtition.reverseIntakeTransitionMode();
        }
        if (controls.polycordIntake()) {
            transtition.intakeTransitionMode();
        }
        if (controls.shootToggle() && shooter.isUpToSpeed()) {
            if (polycordTime.milliseconds() > 300){
                transtition.shootingTransitionMode();
            } else {
                transtition.reverseIntakeTransitionMode();
            }
        } else {
            polycordTime.reset();
        }

        transtition.runMotors();

        /////////////intake

        if (controls.intakeToggle() && !intakeState){
            isIntakeOn = !isIntakeOn;
        }
        intakeState = controls.shooterSpinUp();
        intake.setIntakeOn(isIntakeOn);


        if (controls.outtake()){
            intake.outtake();
        }

        ///////////shooter
        if (controls.autoShootToggle()){
            autoShoot();
        }

        if (controls.shooterSpinUp() && !shooterSpinUpState){
            isShooterOn = !isShooterOn;
            shooter.setShootOn(isShooterOn);
        }
        shooterSpinUpState = controls.shooterSpinUp();



        if (controls.increaseShooterSpeed() && controlTime.milliseconds() > 500){
            shooter.increaseSpeed();
            controlTime.reset();
        }

        if (controls.resetMinAndMax()){
            shooter.ResetMinMax();
        }

        if (controls.decreaseShooterSpeed() && controlTime.milliseconds() > 500){
            shooter.decreaseSpeed();
            controlTime.reset();
        }

        shooter.loop();

        /////////////webcam
        webCam.execute();

        /////////////telemetry
        disSensors.telemetry();
        imu.telemetry();
        transtition.telemetery();
        intake.telemetry();
        shooter.telemetry();
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    @Override
    public void stop() {
    }

    public void autoShoot(){
        ElapsedTime autoTime = new ElapsedTime();
        shooter.shooterSpinUp();
        while (!shooter.isUpToSpeed()) {
        //TODO:get toggle to work with return
            if (controls.autoShootToggle()){
                shooter.doNothing();
                return;
            }
        }
        autoTime.reset();
        transtition.shootingTransitionMode();
        while (autoTime.seconds() < 5){
            //TODO:get toggle to work with break
            if (controls.autoShootToggle()){
                break;
            }
        }
        shooter.doNothing();
        transtition.doNothingMode();
    }
}
