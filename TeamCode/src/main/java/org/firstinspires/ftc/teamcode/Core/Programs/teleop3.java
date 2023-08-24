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

        //        Manual Control
//        robot.getLeftTop().setPower(.7 *  Range.clip(gamepad1.left_stick_y,-1,1));
//        robot.getLeftBottom().setPower(.7 * Range.clip(gamepad1.right_stick_y,-1,1));
//        robot.getRightTop().setPower(.7 *  Range.clip(gamepad2.left_stick_y,-1,1));
//        robot.getRightBottom().setPower(.7 * Range.clip(gamepad2.right_stick_y,-1,1));

//         1st test
//                double ltpower = (Range.clip(gamepad1.left_stick_y , -.8, .8)) + (Range.clip(gamepad1.right_stick_x , -.2 , .2));
//                double lbpower = (Range.clip(gamepad1.left_stick_y , -.8, .8)) - (Range.clip(gamepad1.right_stick_x , -.2 , .2));;
//
//                double rtpower = (Range.clip(gamepad1.left_stick_y , -.8, .8)) - (Range.clip(gamepad1.right_stick_x , -.2 , .2));;
//                double rbpower = (Range.clip(gamepad1.left_stick_y , -.8, .8)) + (Range.clip(gamepad1.right_stick_x , -.2 , .2));;
//
//
//
//                robot.getLeftTop().setPower(ltpower);
//                robot.getLeftBottom().setPower(-lbpower);
//                robot.getRightTop().setPower(-rtpower);
//                robot.getRightBottom().setPower(rbpower);

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
        double wheelDiff = Math.abs(adjustedAngle - wheelPosRight);
        teleDrive(adjustedAngle ,magnitude, wheelPosLeft, wheelPosRight , gamepad1.right_trigger ,robot );

        double turnVal = 0;
        if (magnitude > .2 && wheelPosLeft > (adjustedAngle + 2) )
            turnVal  = .2;
        if (magnitude > .2 && wheelPosLeft < (adjustedAngle - 2) )
            turnVal = -.2;
        if (magnitude <= .2 || (wheelPosLeft <= (adjustedAngle + 2) && wheelPosLeft >= (adjustedAngle - 2)))
            turnVal = 0;

        telemetry.addData("joystick magnitude" , magnitude);
        telemetry.addData("wheel right degree" , wheelPosRight);
        telemetry.addData("wheel left degree" , wheelPosLeft);
        telemetry.addData("joystick degree" , adjustedAngle);
        telemetry.addData("joystick - Wheel Difference" , wheelDiff);
        telemetry.addData("turn val", turnVal);
        telemetry.update();




    }

    @Override
    public void exit()
    {

    }

    public static void teleDrive(double joystickAngle, double speedVal,
                                 double wheelPosLeft,  double wheelPosRight, float slowModeInput,  Main robot)
    {
        double turnValRight = 0;
        double turnValLeft = 0;
        double difference = Math.abs(wheelPosRight - joystickAngle);

        //right shit
       if (speedVal > .2 && wheelPosRight > (joystickAngle + 5) )
           turnValRight  = .2;
       if (speedVal > .2 && wheelPosRight < (joystickAngle - 5) )
           turnValRight = -.2;
       if (speedVal <= .2 || (wheelPosRight <= (joystickAngle + 5) && wheelPosRight >= (joystickAngle - 5)))
           turnValRight = 0;

       //left shit
        if (speedVal > .2 && wheelPosLeft > (joystickAngle + 5) )
            turnValLeft  = .2;
        if (speedVal > .2 && wheelPosLeft < (joystickAngle - 5) )
            turnValLeft = -.2;
        if (speedVal <= .2 || (wheelPosLeft <= (joystickAngle + 5) && wheelPosLeft >= (joystickAngle - 5)))
            turnValLeft = 0;

        robot.getLeftTop().setPower(speedVal +  turnValLeft);
        robot.getLeftBottom().setPower(speedVal - turnValLeft);
        robot.getRightTop().setPower(speedVal - turnValRight);
        robot.getRightBottom().setPower(speedVal + turnValRight);




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
}
