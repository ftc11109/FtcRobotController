package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;

public class RobotControls {
    Gamepad gamepad1;
    Gamepad gamepad2;

    public RobotControls(Gamepad gamepad1, Gamepad gamepad2){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public double forward(){
        return gamepad1.left_stick_y;
    }
    public double strafe(){
        return -gamepad1.left_stick_x;
    }
    public double turn(){
        return -gamepad1.right_stick_x;
    }

    public boolean intake(){
        return gamepad1.right_bumper;
    }
    public boolean outtake(){
        return gamepad1.left_bumper;
    }
    public boolean shoot(){
        return gamepad1.right_trigger > 0.5;
    }
    public boolean polycordIntake(){
        return gamepad1.dpad_up;
    }
    public boolean spitOut(){
        return gamepad1.dpad_down;
    }
    public boolean shooterToggle(){
        return gamepad1.left_trigger > 0.5;
    }
    public boolean slowMode(){
        return gamepad1.y;
    }
    public boolean shooterStop(){
        return gamepad1.b;
    }
    public boolean upperTransitionIntake(){ return gamepad1.dpad_left;}
    public boolean upperTransitionOuttake(){ return gamepad1.dpad_right;}
    public boolean increaseShooterSpeed(){ return gamepad1.start;}
    public boolean decreaseShooterSpeed(){ return gamepad1.back;}
    public boolean resetMinAndMax(){ return gamepad1.a;}


}
