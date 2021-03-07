package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Shooter {
    double targetShootSpeed = 1200.0;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Shooter(Telemetry telemetry, HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    private DcMotorEx shooterMotor;
    private double minSpeed;
    private double minAmps;
    private double maxAmps;



    public void init(){

        shooterMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter");
    }

    public void doNothing(){
        shooterMotor.setVelocity(0);
    }

    public void shooterSpinUp(){
        shooterMotor.setVelocity(targetShootSpeed);
    }

    public void setShootOn(boolean shooterOn){
        if (shooterOn){
            shooterSpinUp();
        } else {
            doNothing();
        }
    }

    public void increaseSpeed(){
        targetShootSpeed = targetShootSpeed + 100;
    }

    public void decreaseSpeed(){
        targetShootSpeed = targetShootSpeed - 100;
    }

    public void ResetMinMax(){
        minAmps = shooterMotor.getCurrent(CurrentUnit.AMPS);
        minSpeed = shooterMotor.getVelocity();
        maxAmps = shooterMotor.getCurrent(CurrentUnit.AMPS);
    }

    public void loop (){
        minAmps = Math.min(minAmps, shooterMotor.getCurrent(CurrentUnit.AMPS));
        minSpeed = Math.min(minSpeed, shooterMotor.getVelocity());
        maxAmps = Math.max(maxAmps, shooterMotor.getCurrent(CurrentUnit.AMPS));
    }

    public void telemetry(){
        telemetry.addData("shooter speed", shooterMotor.getVelocity());
        telemetry.addData("target shooter speed", targetShootSpeed);
        telemetry.addData("shooter current", shooterMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("min shooter amps", minAmps);
        telemetry.addData("min shooter speed", minSpeed);
        telemetry.addData("max shooter amps", maxAmps);
    }
    }
