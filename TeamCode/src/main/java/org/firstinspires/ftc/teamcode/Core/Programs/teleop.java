package org.firstinspires.ftc.teamcode.Core.Programs;

import static com.google.blocks.ftcrobotcontroller.util.Identifier.RANGE;

import android.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Main;
import org.firstinspires.ftc.teamcode.Core.UpliftAuto;


@TeleOp(name = "test", group = "Opmodes")
public class teleop extends UpliftTele {

    Main robot;
    @Override
    public void initHardware() {

    }

    @Override
    public void initAction() {

    }

    @Override
    public void body() throws InterruptedException {
        robot.getLeftTop().setPower(gamepad1.left_stick_y/2);
        robot.getLeftBottom().setPower(gamepad1.right_stick_y/2);
        robot.getRightTop().setPower(gamepad2.left_stick_y/2);
        robot.getRightBottom().setPower(gamepad2.right_stick_y/2);
    }

    @Override
    public void exit() throws InterruptedException {

    }
}
