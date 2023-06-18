package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Main {
    DcMotor lf, rf, lb, rb;

    public LinearOpMode opMode;
    public HardwareMap hardwareMap;

    public Main(LinearOpMode opMode) {
        this.opMode = opMode;
        getHardware();
    }

    public void getHardware() {

        hardwareMap = opMode.hardwareMap;


        lf = hardwareMap.get(DcMotor.class, "left_front");
        rf = hardwareMap.get(DcMotor.class, "right_front");
        lb = hardwareMap.get(DcMotor.class, "left_back");
        rb = hardwareMap.get(DcMotor.class, "right_back");

    }

    public DcMotor getLeftFront()
    {
        return lf;

    }


    public DcMotor getRightFront()
    {
        return rf;
    }

    public DcMotor getLeftBack()
    {
        return lb;

    }


    public DcMotor getRightBack()
    {
        return rb;
    }
}