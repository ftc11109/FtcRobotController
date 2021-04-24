package org.firstinspires.ftc.teamcode.autonomouseMovement;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.subsystem.Drive;


public class ImuPIDTurning {

    private static final double P_RATE = 0.07;
    private static final double RAMP_RATE = P_RATE * 7;
    private ElapsedTime runtime = new ElapsedTime();

    Drive drive;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    DcMotor leftmotorB;
    DcMotor rightmotorB;
    DcMotor leftmotorF;
    DcMotor rightmotorF;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .10, correction;
    double startAngle;

    public ImuPIDTurning(Telemetry telemetry, HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

    }

    public void init() {

        drive = new Drive(telemetry, hardwareMap);
        drive.init();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        leftmotorB = hardwareMap.get(DcMotor.class, "left_driveB");
        leftmotorF = hardwareMap.get(DcMotor.class, "left_driveF");
        rightmotorB = hardwareMap.get(DcMotor.class, "right_driveB");
        rightmotorF = hardwareMap.get(DcMotor.class, "right_driveF");


        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();


        resetAngle();

    }


    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double getAbsoluteAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle;
    }
    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;

        else if (deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public double getDirection() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void setStartAngle(){
        startAngle = getAngle();
    }

    public double getStartAngle(){
        return startAngle;
    }
    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    private double speedByBusVoltage(double speed) {
        return speed;
    }

    public void sleepAndLog(double ms) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < ms) {
            getAngle();
        }
    }


    public boolean inRange(double max, double min, double current, double target) {
        double error = current - target;
        error = Math.abs(error);
        return error >= min && error <= max;
    }


    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param targetAngle Degrees to turn, + is left - is right
     */
    public void rotate(double targetAngle, double power, double timeOutS) {
        double leftPower, rightPower;
        double error;
        //LEFT IS + RIGHT is -
        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        power = Math.abs(power);
        if (targetAngle < 0) {   // turn right.
            leftPower = -power;
            rightPower = power;
        } else if (targetAngle > 0) {   // turn left.
            leftPower = power;
            rightPower = -power;
        } else return;

        runtime.reset();
        // set power to rotate.
        while (runtime.seconds() < timeOutS) {

            error = targetAngle - getAngle();

            leftPower = P_RATE * error;
            rightPower = P_RATE * -error;
//            leftPower = power;
//            rightPower = -power;

            leftPower = Range.clip(leftPower, -power, power);
            rightPower = Range.clip(rightPower, -power, power);

            leftmotorB.setPower(leftPower);
            rightmotorB.setPower(rightPower);
            leftmotorF.setPower(leftPower);
            rightmotorF.setPower(rightPower);

            telemetry.addData("left pow", leftPower);
            telemetry.addData("right pow", rightPower);
            telemetry.addData("angle", getAngle());
            telemetry.addData("target", targetAngle);
            telemetry.addData("error", error);
            telemetry.update();
            if (inRange(0.5, -0.5, getAngle(), targetAngle)) {
                leftmotorB.setPower(0);
                rightmotorB.setPower(0);
                leftmotorF.setPower(0);
                rightmotorF.setPower(0);
                break;
            }
        }
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotateToHeading(double degrees, double power, double timeOutS) {
        double leftPower, rightPower;
        //LEFT IS - RIGHT is +
        // restart imu movement tracking.

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        leftPower = -power;
        rightPower = power;

        runtime.reset();
        // set power to rotate.
        while (runtime.seconds() < timeOutS) {
            telemetry.addData("angle", getDirection());
            telemetry.update();
            double slowLeft = leftPower * 0.7;
            double slowRight = rightPower * 0.7;

            double error = Math.abs(getDirection()) - Math.abs(degrees);
            if (error < 0) {
                if (error > -22) {
                    leftmotorB.setPower(slowLeft);
                    rightmotorB.setPower(slowRight);
                } else {
                    leftmotorB.setPower(leftPower);
                    rightmotorB.setPower(rightPower);

                }
            } else {
                rightmotorB.setPower(0);
                leftmotorB.setPower(0);
                break;
            }
        }
    }

}