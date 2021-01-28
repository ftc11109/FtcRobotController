package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    private static final double SHOOTER_RPM = 3000.0;
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
        shooterMotor.setVelocity(SHOOTER_RPM);
    }
}
