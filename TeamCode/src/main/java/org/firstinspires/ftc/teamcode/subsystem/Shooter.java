package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    double targetShootSpeed = 1200.0;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Shooter(Telemetry telemetry, HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    private DcMotorEx shooterMotor;



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
    //wont go past base shooter speed
    public void increaseSpeed(){
        targetShootSpeed = targetShootSpeed + 100;
    }

    public void decreaseSpeed(){
        targetShootSpeed = targetShootSpeed - 100;
    }

    public void telemetry(){
        telemetry.addData("shooter speed", shooterMotor.getVelocity());
        telemetry.addData("target shooter speed", targetShootSpeed);
    }
    }
