package org.firstinspires.ftc.teamcode.Core.Programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Core.Main;
import org.firstinspires.ftc.teamcode.Core.UpliftAuto;

@Autonomous(name = "test", group = "Opmodes")
public class Auto extends UpliftAuto {

    Main robot;
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0;





    @Override
    public void initHardware() {

    }

    @Override
    public void initAction() {

    }

    @Override
    public void body() throws InterruptedException {
//       setRobotPowerForward(0.5);


    }

    @Override
    public void exit() throws InterruptedException {

    }

    public void stopMotors()
    {
        robot.getLeftTop().setPower(0);
        robot.getRightTop().setPower(0);
        robot.getLeftBottom().setPower(0);
        robot.getRightBottom().setPower(0);
    }

    public double getAngle()
    {

        Orientation orientation = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        if(deltaAngle > 180)
        {
            deltaAngle -= 360;
        }
        else if(deltaAngle <= 180)
        {
            deltaAngle += 360;
        }

        currAngle += deltaAngle;
        lastAngles = orientation;

        return currAngle;
    }

    public void resetAngle()
    {
        lastAngles = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public void turn(double degrees)
    {
        resetAngle();

        double error = degrees;

        while(opModeIsActive() && Math.abs(error) > 2)
        {
            double motorPower = (error < 0 ? -0.3: 0.3);
            robot.getLeftTop().setPower(motorPower);
            robot.getRightTop().setPower(motorPower);
            robot.getLeftBottom().setPower(-motorPower);
            robot.getRightBottom().setPower(-motorPower);
            error = degrees - getAngle();

        }

        stopMotors();
    }


//    public void setRobotPowerForward(double power){
//        robot.getLeftFront().setPower(power);
//        robot.getLeftBack().setPower(power);
//        robot.getRightFront().setPower(power);
//        robot.getRightBack().setPower(power);
//    }
//    public void setRobotPowerBackward(double power){
//        robot.getLeftFront().setPower(-power);
//        robot.getLeftBack().setPower(-power);
//        robot.getRightFront().setPower(-power);
//        robot.getRightBack().setPower(-power);
//    }
//    public void setRobotPowerRight(double power){
//        robot.getLeftFront().setPower(-power);
//        robot.getLeftBack().setPower(power);
//        robot.getRightFront().setPower(-power);
//        robot.getRightBack().setPower(power);
//    }
//    public void setRobotPowerLeft(double power){
//        robot.getLeftFront().setPower(power);
//        robot.getLeftBack().setPower(-power);
//        robot.getRightFront().setPower(power);
//        robot.getRightBack().setPower(-power);
//    }


}
