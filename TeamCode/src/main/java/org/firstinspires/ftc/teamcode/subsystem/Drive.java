package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drive {

    public static final double SLOW_MULTIPLiER = 0.4;

    public Drive(Telemetry telemetry, HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    HardwareMap hardwareMap;
    Telemetry telemetry;
    private DcMotor leftDriveBack = null;
    private DcMotor rightDriveBack = null;
    private DcMotor leftDriveFront = null;
    private DcMotor rightDriveFront = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init() {

        leftDriveBack  = hardwareMap.get(DcMotor.class, "left_driveB");
        rightDriveBack = hardwareMap.get(DcMotor.class, "right_driveB");
        rightDriveFront = hardwareMap.get(DcMotor.class, "right_driveF");
        leftDriveFront = hardwareMap.get(DcMotor.class, "left_driveF");

        leftDriveBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDriveFront.setDirection(DcMotorSimple.Direction.REVERSE);

//        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //y is the y value of the left joy stick.
    //x is the x value of the left joy stick.
    //rx is the x value of the right joy stick.
    public void drive(double y, double x, double rx) {



        double leftFP;
        double leftBP;
        double rightFP;
        double rightBP;

        leftFP = y + x + rx; // FP meaning Front Power and BP meaning Back Power
        leftBP = y - x + rx;
        rightFP = y - x - rx;
        rightBP = y + x - rx;


        if (Math.abs(leftFP) > 1 || Math.abs(leftBP) > 1 ||
                Math.abs(rightFP) > 1 || Math.abs(rightBP) > 1){
            double max = 0;
            max = Math.max(Math.abs(leftFP), Math.abs(leftBP));
            max = Math.max(Math.abs(rightFP), max);
            max = Math.max(Math.abs(rightBP), max);

            leftFP /= max;
            leftBP /= max;
            rightFP /= max;
            rightBP /= max;
        }



        leftDriveFront.setPower(leftFP);
        leftDriveBack.setPower(leftBP);
        rightDriveFront.setPower(rightFP);
        rightDriveBack.setPower(rightBP);
    }

    public void drive(double y, double x, double rx, boolean slowMode){
        if (slowMode){
          drive(y * SLOW_MULTIPLiER, x * SLOW_MULTIPLiER, rx * SLOW_MULTIPLiER);
        } else {
            drive(y, x, rx);
        }
    }

}
