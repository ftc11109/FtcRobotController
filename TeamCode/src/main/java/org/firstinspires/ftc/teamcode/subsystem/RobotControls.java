package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;

public class RobotControls {
    Gamepad gamepad1;
    Gamepad gamepad2;
    boolean autoShootState = false;
    boolean autoShootLastTrigger = false;

    boolean shootState = false;
    boolean shootLastTrigger = false;


    boolean spinUpState = false;
    boolean spinUpStateLast = false;

    boolean lastY = false;

    public RobotControls(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public double forward() {
        return gamepad1.left_stick_y;
    }

    public double strafe() {
        return -gamepad1.left_stick_x;
    }

    public double turn() {
        return -gamepad1.right_stick_x;
    }

    public boolean intakeToggle() {
        return gamepad1.right_bumper;
    }

    public boolean outtake() {
        return gamepad1.left_bumper;
    }

    public boolean shootToggle() {
        if (gamepad2.right_trigger > 0.5 && !shootLastTrigger) {
            shootState = !shootState;
        }
        shootLastTrigger = gamepad2.right_trigger > 0.5;
        return shootState;
    }

    public boolean polycordIntake() {
        return gamepad2.dpad_up;
    }

    public boolean lowerPolycordIntake(){
        return gamepad2.right_bumper;
    }

    public boolean spitOut() {
        return gamepad2.dpad_down;
    }

    public boolean shooterSpinUp() {
        if (gamepad1.left_trigger > 0.5) {
            shootState = false;
        }
        return gamepad1.left_trigger > 0.5;
    }

    public boolean shooterSpinUp2() {
        if (gamepad2.left_trigger > 0.5){
            shootState = false;
        }
        return gamepad2.left_trigger > 0.5;
    }

    public boolean slowMode() {
        return gamepad1.y;
    }

    public boolean speedMode() {
        return gamepad1.b;
    }

    public boolean autoShootToggle() {
        if (gamepad1.x && !autoShootLastTrigger) {
            autoShootState = !autoShootState;
        }
        autoShootLastTrigger = gamepad1.x;
        return autoShootState;
    }

    //    public boolean shooterStop(){
//        return gamepad1.b;
//    }
    public boolean upperTransitionIntake() {
        return gamepad1.dpad_left;
    }

    public boolean upperTransitionOuttake() {
        return gamepad1.dpad_right;
    }

    public boolean increaseShooterSpeed() {
        return gamepad1.start;
    }

    public boolean decreaseShooterSpeed() {
        return gamepad1.back;
    }

    public boolean autoLineUp() {
        return gamepad1.a;
    }


}
