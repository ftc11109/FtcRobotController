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

package org.firstinspires.ftc.teamcode.autonomouseMovement;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Drive;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

public class AutoDrive {

    Drive drive;

    public AutoDrive(Telemetry telemetry, Drive drive) {
        this.drive = drive;
        this.telemetry = telemetry;
    }

    // Declare OpMode members.

    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime strafeTime = new ElapsedTime();
    DcMotor leftDrive;
    DcMotor rightDrive;

    static final double COUNTS_PER_MOTOR_REV = 420; //8.5714;
    static final double WHEEL_DIAMETER_INCHES = 3.54331 * 79 / 63;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.14159);

    private boolean isDriving = false;

    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //        leftDrive = hardwareMap.get(DcMotor.class, "L_drive");
//        rightDrive = hardwareMap.get(DcMotor.class, "R_drive");
//
//        leftDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    public boolean getIsDriving() {
        return isDriving;
    }

    public void timeStrafe(double speed, double timeout) {
        strafeTime.reset();
        while (strafeTime.milliseconds() < timeout) {
            drive.drive( speed, 0, 0, false, 0);
        }
        drive.drive(0,0,0,false,0);
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeOutS) {
        double leftTargetTicks = leftInches * COUNTS_PER_INCH;
        double rightTargetTicks = rightInches * COUNTS_PER_INCH;
        double startOffset = drive.getLeftTicks();

        runtime.reset();
        while (runtime.seconds() < timeOutS) {
            double offsetGetLeftTicks = drive.getLeftTicks() - startOffset;


            if (Math.abs(offsetGetLeftTicks) < Math.abs(leftTargetTicks)) {
                drive.drive(0.0, Math.abs(speed) * -Math.signum(leftInches), 0.0, false, 0);
                telemetry.addData("voltage", 0);
                telemetry.addData("dis", "%.1f, %.1f", leftTargetTicks, offsetGetLeftTicks);
                telemetry.update();
            } else {
                drive.drive(0.0, 0.0, 0.0, false, 0);
                break;
            }

        }

    }

}