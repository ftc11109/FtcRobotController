package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMU {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    BNO055IMU imu;


    public IMU(Telemetry telemetry, HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void init() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);

    }

    public double getHeading(AngleUnit angleUnit) {
        Orientation angle;
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        return angleUnit.fromDegrees(angle.firstAngle);
    }

    public double getPitch(AngleUnit angleUnit) {
        Orientation angle;
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        return angleUnit.fromDegrees(angle.secondAngle);
    }

    public double getRoll(AngleUnit angleUnit) {
        Orientation angle;
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        return angleUnit.fromDegrees(angle.thirdAngle);
    }


    public void telemetry() {
        telemetry.addData("heading", getHeading(AngleUnit.DEGREES));
        telemetry.addData("picth", getPitch(AngleUnit.DEGREES));
        telemetry.addData("roll", getRoll(AngleUnit.DEGREES));
    }
}
