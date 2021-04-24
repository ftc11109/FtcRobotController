/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link RingSensors} illustrates how to use the REV Robotics
 * Time-of-Flight Range Sensor.
 * <p>
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://revrobotics.com">REV Robotics Web Page</a>
 */
public class RingSensors {

    private static final double RING_WAIT = 800;

    public RingSensors(Telemetry telemetry, HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    boolean isRingIn = false;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private DistanceSensor intakeSensor;
    private DistanceSensor forwardShooterSensor;

    private DistanceSensor elevatorSensor;

    private ElapsedTime ringTimer = new ElapsedTime();
    private ElapsedTime eleTimer = new ElapsedTime();

    public void init() {
        // you can use this as a regular DistanceSensor.
        intakeSensor = hardwareMap.get(DistanceSensor.class, "int2");
        forwardShooterSensor = hardwareMap.get(DistanceSensor.class, "forward_shooter_sensor");
        elevatorSensor = hardwareMap.get(DistanceSensor.class, "eleSensor");
    }

    public double getIntakeDistance() {
        return intakeSensor.getDistance(DistanceUnit.MM);
    }

    public double getForwardSensorDistance() {
        return forwardShooterSensor.getDistance(DistanceUnit.MM);
    }

    public double getEleDistance() {
        return elevatorSensor.getDistance(DistanceUnit.MM);
    }

    public boolean isRingInIntake() {
        if (intakeSensor.getDistance(DistanceUnit.MM) < 85) {
            ringTimer.reset();
            return true;
        } else if (ringTimer.milliseconds() > RING_WAIT) {
            return false;
        } else {
            return true;
        }
    }

    public boolean isRingInEle() {
//    if (getEle2Distance() < 30){
//        if (eleTimer.milliseconds() > 500){
//            return true;
//        } else {
//            return false;
//        }
//    } else {
//        eleTimer.reset();
//        return false;
//    }
        return getEleDistance() < 45;
    }

    public boolean isRingInForward(){
        return getForwardSensorDistance() < 69;
    }
    public void telemetry() {


        // generic DistanceSensor methods.
        telemetry.addData("intake range", getIntakeDistance());
        telemetry.addData("forward range", getForwardSensorDistance());
        telemetry.addData("elevator Range", getEleDistance());

        telemetry.addData("isRingInEle", isRingInEle());
        telemetry.addData("isRing?", isRingInIntake());

    }
}
