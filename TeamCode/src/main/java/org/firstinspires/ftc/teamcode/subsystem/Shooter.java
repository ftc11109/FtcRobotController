package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Shooter {

    private static final double P_RATE = 0.07;

    //power shot speed 1000
    double targetShootSpeed = 1250.0;
    double autoTargetShootSpeed = 1250.0;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    WebCam webCam;

    public Shooter(Telemetry telemetry, HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.webCam = webCam;
    }

    private DcMotorEx shooterMotor;
    private double minSpeed;
    private double minAmps;
    private double maxAmps;
    private ElapsedTime rangeTime = new ElapsedTime();

    boolean stablizationMode = true;

    public void init() {

        shooterMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter");
    }

    public void doNothing() {
        shooterMotor.setVelocity(0);
    }

    public boolean isUpToSpeed() {
        if (stablizationMode) {
            if (shooterMotor.getVelocity() > targetShootSpeed - 75 && shooterMotor.getVelocity() < targetShootSpeed + 75 ) {
                if (rangeTime.milliseconds() > 250){
                    stablizationMode = false;
                    return true;
                } else{
                    return false;
                }
            } else {
                rangeTime.reset();
                return false;
            }
        } else {
            if (shooterMotor.getVelocity() < targetShootSpeed - 200) {
                stablizationMode = true;
            }
            return true;
        }

    }


    public void shooterSpinUp() {
        shooterMotor.setVelocity(targetShootSpeed);
    }

    public void setShootOn(boolean shooterOn) {
        if (shooterOn) {
            shooterSpinUp();
        } else {
            doNothing();
        }
    }

    public void increaseSpeed() {
        targetShootSpeed = targetShootSpeed + 25;
        shooterMotor.setVelocity(targetShootSpeed);
    }

    public void decreaseSpeed() {
        targetShootSpeed = targetShootSpeed - 25;
        shooterMotor.setVelocity(targetShootSpeed);
    }


    public void ResetMinMax() {
        minAmps = shooterMotor.getCurrent(CurrentUnit.AMPS);
        minSpeed = shooterMotor.getVelocity();
        maxAmps = shooterMotor.getCurrent(CurrentUnit.AMPS);
    }
    public void autoSetShootOn(boolean shooterOn) {
        if (shooterOn) {
            autoShooterSpinUp();
        } else {
            doNothing();
        }
    }

    public void autoShooterSpinUp() {
        shooterMotor.setVelocity(autoTargetShootSpeed);
    }

    public void loop() {
        minAmps = Math.min(minAmps, shooterMotor.getCurrent(CurrentUnit.AMPS));
        minSpeed = Math.min(minSpeed, shooterMotor.getVelocity());
        maxAmps = Math.max(maxAmps, shooterMotor.getCurrent(CurrentUnit.AMPS));
        isUpToSpeed();
    }

    public void autoShootStart() {
        shooterSpinUp();
    }

    public void telemetry() {
        telemetry.addData("shooter speed", shooterMotor.getVelocity());
        telemetry.addData("target shooter speed", targetShootSpeed);
        telemetry.addData("shooter current", shooterMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("min shooter amps", minAmps);
        telemetry.addData("min shooter speed", minSpeed);
        telemetry.addData("max shooter amps", maxAmps);
        telemetry.addData("is stableizaion", stablizationMode);
    }
}
