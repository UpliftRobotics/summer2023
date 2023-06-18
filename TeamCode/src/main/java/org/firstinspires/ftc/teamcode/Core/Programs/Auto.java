package org.firstinspires.ftc.teamcode.Core.Programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Core.Main;
import org.firstinspires.ftc.teamcode.Core.UpliftAuto;

@Autonomous(name = "test", group = "Opmodes")
public class Auto extends UpliftAuto {

    Main robot;

    @Override
    public void initHardware() {

    }

    @Override
    public void initAction() {

    }

    @Override
    public void body() throws InterruptedException {
        robot.getLeftFront().setPower(0.5);
        robot.getLeftBack().setPower(0.5);
        robot.getRightFront().setPower(0.5);
        robot.getRightBack().setPower(0.5);
        Thread.sleep(1000);
        robot.getLeftFront().setPower(0);
        robot.getLeftBack().setPower(0);
        robot.getRightFront().setPower(0);
        robot.getRightBack().setPower(0);


    }

    @Override
    public void exit() throws InterruptedException {

    }
}
