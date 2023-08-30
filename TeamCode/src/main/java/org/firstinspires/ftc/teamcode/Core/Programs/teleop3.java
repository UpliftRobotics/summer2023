package org.firstinspires.ftc.teamcode.Core.Programs;

import static com.google.blocks.ftcrobotcontroller.util.Identifier.RANGE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.Main;
import org.firstinspires.ftc.teamcode.Core.UpliftTele;
import org.firstinspires.ftc.teamcode.Core.UpliftAuto;


@TeleOp(name = "test3", group = "Opmodes")
public class teleop3 extends UpliftTele {

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
    public void bodyLoop() throws InterruptedException {

// manual correction
        robot.getLeftTop().setPower(gamepad2.left_stick_y);
        robot.getRightTop().setPower(gamepad2.right_stick_y);


                //2nd test
                // This is going to a system where the wheel is basically going to just point in the direcion that the
                // left analog stick is pointing, using the endocers and angle of the joystick as the main 2 inputs.
                // This assumes no skipping in, the system at all.

//chat gpt code start
        float angle = getJoystickAngle();   // Calculate joystick angle in degrees
        float adjustedAngle = (angle + 90) % 360;   // Adjust angle to have 0 degrees as north and clockwise rotation
//chat gpt code end

        double magnitude = Range.clip(Math.sqrt( Math.pow (gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2 )) , 0 , .7); // get magnitude of joystick from center
        double wheelPosRight = (((robot.getRightTop().getCurrentPosition() + -robot.getRightBottom().getCurrentPosition()) / 2) / 2.641111 ) % 360; // get wheels angle, assuming starting from forward pos
        double wheelPosLeft =  -((((robot.getLeftTop().getCurrentPosition() + -robot.getLeftBottom().getCurrentPosition()) / 2) / 2.641111 ) % 360);

        if (wheelPosLeft < 0 )
        {
            wheelPosLeft = wheelPosLeft + 360;
        }
        if (wheelPosRight < 0 )
        {
            wheelPosRight = wheelPosRight + 360;
        }
        double wheelDiff = Math.abs(adjustedAngle - wheelPosRight);
        teleDrive(adjustedAngle ,magnitude, wheelPosLeft, wheelPosRight , gamepad1.left_trigger, gamepad1.right_stick_x ,robot );
        turn(robot);

        double turnVal = 0;
        if (magnitude > .2 && wheelPosLeft > (adjustedAngle + 2) )
            turnVal  = .2;
        if (magnitude > .2 && wheelPosLeft < (adjustedAngle - 2) )
            turnVal = -.2;
        if (magnitude <= .2 || (wheelPosLeft <= (adjustedAngle + 2) && wheelPosLeft >= (adjustedAngle - 2)))
            turnVal = 0;

        double turnDirection = 1;
        if ((wheelPosRight > 270 && adjustedAngle < 90) || (wheelPosRight < 90 && adjustedAngle > 270))
        {
            turnDirection= -1;
        }
        if ((wheelPosRight > adjustedAngle && wheelPosRight <= 90) || ((wheelPosRight < adjustedAngle && wheelPosRight >= 270)))
        {
            turnDirection = 1;
        }

        telemetry.addData("joystick magnitude" , magnitude);
        telemetry.addData("wheel right degree" , wheelPosRight);
        telemetry.addData("wheel left degree" , wheelPosLeft);
        telemetry.addData("joystick degree" , adjustedAngle);
        telemetry.addData("joystick - Wheel Difference" , wheelDiff);
        telemetry.addData("turn val", turnVal);
        telemetry.addData("turnDirection" , turnDirection);
        telemetry.update();




    }

    @Override
    public void exit()
    {

    }

    public static void teleDrive(double joystickAngle, double speedVal,
                                 double wheelPosLeft,  double wheelPosRight, double brake, double turn, Main robot)
    {
        brake = Range.clip(brake , 0 , speedVal);
        boolean canReverse = true;
        int turnDirection = 1;
        double turnValRight = 0;
        double turnValLeft = 0;
        double difference = Math.abs(wheelPosRight - joystickAngle);

        //right shit
       if (speedVal > .2 && wheelPosRight > (wheelPosLeft + 5) && turn < .1)
           turnValRight  = .2;
       if (speedVal > .2 && wheelPosRight < (wheelPosLeft - 5 )&& turn < .1)
           turnValRight = -.2;
       if (speedVal <= .2 || (wheelPosRight <= (wheelPosLeft + 5) && wheelPosRight >= (wheelPosLeft - 5)) && turn >= .1)
           turnValRight = 0;

       //left shit
        if (speedVal > .2 && wheelPosLeft > (joystickAngle + 5) && turn < .1)
            turnValLeft  = .2;
        if (speedVal > .2 && wheelPosLeft < (joystickAngle - 5) && turn < .1)
            turnValLeft = -.2;
        if (speedVal <= .2 || (wheelPosLeft <= (joystickAngle + 5) && wheelPosLeft >= (joystickAngle - 5)) && turn >= .1)
            turnValLeft = 0;

        if ((wheelPosRight > 270 && joystickAngle < 90) || (wheelPosRight < 90 && joystickAngle > 270))
        {
            turnDirection = -1;
        }
        if ((wheelPosRight > joystickAngle && wheelPosRight <= 90) || ((wheelPosRight < joystickAngle && wheelPosRight >= 270)))
        {
            turnDirection = 1;
        }
        robot.getLeftTop().setPower((speedVal - brake) +  (turnDirection * turnValLeft) + turn);
        robot.getLeftBottom().setPower((speedVal - brake) - (turnDirection * turnValLeft) + turn);
        robot.getRightTop().setPower((speedVal - brake) - (turnDirection * turnValLeft) - turn);
        robot.getRightBottom().setPower((speedVal - brake) + (turnDirection * turnValLeft) - turn);




    }

    public float getJoystickAngle() {
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
    public void turn(Main robot)
    {
    }
}
