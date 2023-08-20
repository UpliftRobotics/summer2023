package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        lt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lt.setDirection(DcMotorSimple.Direction.REVERSE);
//        rt.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
//        rb.setDirection(DcMotorSimple.Direction.REVERSE);




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