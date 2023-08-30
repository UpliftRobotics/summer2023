package org.firstinspires.ftc.teamcode.Core.Programs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.Main;
import org.firstinspires.ftc.teamcode.Core.UpliftTele;


@TeleOp(name = "test4", group = "Opmodes")
public class teleop4 extends UpliftTele {

    Main robot;
    @Override
    public void initHardware()
    {
        robot = new Main(this);
    }


    @Override
    public void initAction()
    {
        robot.getLeftTop().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getLeftBottom().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getRightTop().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getRightBottom().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getLeftTop().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getLeftBottom().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getRightTop().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getRightBottom().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void bodyLoop() throws InterruptedException
    {
//joystick angle
        float angle = getJoystickAngle();   // Calculate joystick angle in degrees
        float joystickAngle = (angle + 90) % 360;   // Adjust angle to have 0 degrees as north and clockwise rotation
        double magnitude = Range.clip(Math.sqrt( Math.pow (gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2 )), 0 , .8); // get magnitude of joystick from center
        if (magnitude == 0)
            joystickAngle = 0;
        double brake = Range.clip( gamepad1.left_trigger , 0 , magnitude);
        double slowMode = 1 - Range.clip(gamepad1.right_trigger , 0 , .5);

// wheel angle
        double wheelPosRight = (((robot.getRightTop().getCurrentPosition() + -robot.getRightBottom().getCurrentPosition()) / 2) / 2.641111 ) % 360; // get wheels angle, assuming starting from forward pos
        double wheelPosLeft =  -((((robot.getLeftTop().getCurrentPosition() + -robot.getLeftBottom().getCurrentPosition()) / 2) / 2.641111 ) % 360);
        if(wheelPosRight < 0)
            wheelPosRight =+ 360;
        if(wheelPosLeft < 0)
            wheelPosLeft =+ 360;
        if(wheelPosRight == 360)
            wheelPosRight = 0;
        if(wheelPosLeft == 360)
            wheelPosLeft = 0;

//wheel turning
        double wheelTurnPowerLeft = 0;
        double wheelTurnPowerRight = 0;

//turning
        double turn = -Range.clip(gamepad1.right_stick_x , -.8 , .8);
        if ((((wheelPosRight + wheelPosLeft) / 2) < 270) && ((wheelPosRight + wheelPosLeft) / 2) > 90)
            turn = -turn;

        if ((wheelPosLeft > (joystickAngle + 2) || wheelPosLeft < (joystickAngle - 2)) && magnitude != 0)
            wheelTurnPowerLeft = (wheelPosLeft - joystickAngle)/ 180;
        if ((wheelPosRight > (joystickAngle + 2) || wheelPosRight < (joystickAngle - 2)) && magnitude != 0)
            wheelTurnPowerRight = (wheelPosRight - joystickAngle)/ 180;









        robot.getLeftTop().setPower(((magnitude - brake) * slowMode) +  (wheelTurnPowerLeft) + turn);
        robot.getLeftBottom().setPower(((magnitude - brake) * slowMode) - (wheelTurnPowerLeft) + turn);
        robot.getRightTop().setPower(((magnitude - brake) * slowMode) -  (wheelTurnPowerRight) - turn);
        robot.getRightBottom().setPower(((magnitude - brake) * slowMode) +  (wheelTurnPowerRight) - turn);

        telemetry.addData("magnitude", magnitude);
        telemetry.addData("wheelPosRight", wheelPosRight);
        telemetry.addData("wheelPosLeft", wheelPosLeft);
        telemetry.addData("JoystickAngle", joystickAngle);
        telemetry.update();
    }

    @Override
    public void exit()
    {

    }


    public float getJoystickAngle()
    {
        //chat gpt code
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;

        // Calculate angle in radians
        double angleRad = Math.atan2(-y, x);

        // Convert angle to degrees
        double angleDeg = Math.toDegrees(angleRad);

        // Adjust angle to be in the range of 0 to 360 degrees
        if (angleDeg < 0) {
            angleDeg += 360;
        }

        return (float) angleDeg;
    }


}
