package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Main {
    DcMotor lt, rt, lb, rb;

    public LinearOpMode opMode;
    public HardwareMap hardwareMap;

    public Main(LinearOpMode opMode) {
        this.opMode = opMode;
        getHardware();
    }

    public void getHardware() {

        hardwareMap = opMode.hardwareMap;

        lt = hardwareMap.get(DcMotor.class, "left_top");
        rt = hardwareMap.get(DcMotor.class, "right_top");
        lb = hardwareMap.get(DcMotor.class, "left_bottom");
        rb = hardwareMap.get(DcMotor.class, "right_bottom");

    }

    public DcMotor getLeftTop()
    {
        return lt;

    }


    public DcMotor getRightTop()
    {
        return rt;
    }

    public DcMotor getLeftBottom()
    {
        return lb;

    }


    public DcMotor getRightBottom()
    {
        return rb;
    }
}