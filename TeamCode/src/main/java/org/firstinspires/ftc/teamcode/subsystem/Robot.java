package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Navigation nav = new Navigation();

    public void init(HardwareMap hwMap) {
        nav.init(hwMap);
    }
}
