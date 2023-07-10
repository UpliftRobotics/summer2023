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
       setRobotPowerForward(0.5);


    }

    @Override
    public void exit() throws InterruptedException {

    }

    public void setRobotPowerForward(double power){
        robot.getLeftFront().setPower(power);
        robot.getLeftBack().setPower(power);
        robot.getRightFront().setPower(power);
        robot.getRightBack().setPower(power);
    }
    public void setRobotPowerBackward(double power){
        robot.getLeftFront().setPower(-power);
        robot.getLeftBack().setPower(-power);
        robot.getRightFront().setPower(-power);
        robot.getRightBack().setPower(-power);
    }
    public void setRobotPowerRight(double power){
        robot.getLeftFront().setPower(-power);
        robot.getLeftBack().setPower(power);
        robot.getRightFront().setPower(-power);
        robot.getRightBack().setPower(power);
    }
    public void setRobotPowerLeft(double power){
        robot.getLeftFront().setPower(power);
        robot.getLeftBack().setPower(-power);
        robot.getRightFront().setPower(power);
        robot.getRightBack().setPower(-power);
    }


}
