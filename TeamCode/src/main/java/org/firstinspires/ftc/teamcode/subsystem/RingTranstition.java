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

package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

public class RingTranstition {

    private static final double REVERSE_UPPER_BELTS_SPEED = 1000.0;
    public static final double LOWER_BELTS_SPIT_OUT_SPEED = -1000.0;

    private static final double SHOOTING_UPPER_BELTS_SPEED = -1100;
    private static final double SHOOTING_LOWER_BELTS_SPEED = 1100.0;

    private static final double INTAKE_UPPER_BELTS_SPEED = -1000.0;
    private static final double LOWER_BELT_INTAKE_SPEED = 1000.0;

    private static final double ADVANCING_UPPER_BELTS_SPEED = -1100;
    //800
    private static final double LOWER_BELT_ADVANCING_SPEED = 900;
    //585

    private double lowerMotorSpeed;
    private double upperMotorSpeed;

    public RingTranstition(Telemetry telemetry, HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    DcMotorEx eleWheelsLowerBelts;
    DcMotorEx eleBeltsRampBelts;

    /*
     * will run in master once init is hit
     */
    public void init() {
        eleBeltsRampBelts = (DcMotorEx) hardwareMap.get(DcMotor.class, "eleBeltsRampBelts");
        eleWheelsLowerBelts = (DcMotorEx) hardwareMap.get(DcMotor.class, "eleWheelsLowerBelts");
    }

    public void runMotors(){
        eleBeltsRampBelts.setVelocity(upperMotorSpeed);
        eleWheelsLowerBelts.setVelocity(lowerMotorSpeed);
    }

    private void runUpperMotor(double upperBeltsSpeed) {
        //RPM * pulley diameter * gear ratio
        upperMotorSpeed = -upperBeltsSpeed;  //upperBeltsSpeed * Math.PI * 30 * (1 / 3);
    }

    private void runLowerMotor(double lowerBeltsSpeed) {
        lowerMotorSpeed = lowerBeltsSpeed;  //lowerBeltsSpeed * Math.PI * 30 * (1 / 3);
    }


    public void advancingTransitionMode(){
        runUpperMotor(ADVANCING_UPPER_BELTS_SPEED);
        runLowerMotor(LOWER_BELT_ADVANCING_SPEED);
    }

    public void shootingTransitionMode() {
        runLowerMotor(SHOOTING_LOWER_BELTS_SPEED);
        runUpperMotor(SHOOTING_UPPER_BELTS_SPEED);
    }

    public void lowerDoNothing(){
        runLowerMotor(0);
    }

    public void lowerTranistionIntake(){
        runLowerMotor(LOWER_BELT_INTAKE_SPEED);
    }

    public void doNothingMode() {
        runLowerMotor(0);
        runUpperMotor(0);
    }

    public void upperTransitionIntake(){
        runUpperMotor(SHOOTING_UPPER_BELTS_SPEED);
    }

    public void upperTransitionOuttake(){
        runUpperMotor(REVERSE_UPPER_BELTS_SPEED);
    }


    public void intakeTransitionMode() {
        runUpperMotor(INTAKE_UPPER_BELTS_SPEED);
        runLowerMotor(LOWER_BELT_INTAKE_SPEED);
    }

    public void reverseIntakeTransitionMode() {
        runUpperMotor(REVERSE_UPPER_BELTS_SPEED);
        runLowerMotor(LOWER_BELTS_SPIT_OUT_SPEED);
    }

    public void telemetery(){
        telemetry.addData("ele polyCord speed", eleBeltsRampBelts.getVelocity());
        telemetry.addData("lower polyCord speed", eleWheelsLowerBelts.getVelocity());

    }
}
