package org.firstinspires.ftc.teamcode.Core.Programs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.Main;
import org.firstinspires.ftc.teamcode.Core.UpliftTele;


@TeleOp(name = "TeleOp", group = "Opmodes")
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

//resets encoders
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
        float joystickAngle = ((angle+90) % 360) ;   // Adjust angle to have 0 degrees as north and clockwise rotation
        double magnitude = Range.clip(Math.sqrt( Math.pow (gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2 )), 0 , .8); // get magnitude of joystick from center
        if (magnitude == 0)
            joystickAngle = 0;
        double brake = Range.clip( gamepad1.left_trigger , 0 , magnitude);
        double slowMode = 1 - Range.clip(gamepad1.right_trigger , 0 , .5);

// Estimates front and back position of the wheel
        double wheelPosRightf = (((robot.getRightTop().getCurrentPosition() + -robot.getRightBottom().getCurrentPosition()) / 2) / 2.641111 ) % 360; // get wheels angle, assuming starting from forward pos
        double wheelPosLeftf =  -((((robot.getLeftTop().getCurrentPosition() + -robot.getLeftBottom().getCurrentPosition()) / 2) / 2.641111 ) % 360);
        double wheelPosRightb = (wheelPosRightf + 180) % 360;
        double wheelPosLeftb = (wheelPosLeftf + 180) % 360;

//Intializing wheel turning variables
        double wheelTurnPowerLeft = 0;
        double wheelTurnPowerRight = 0;

// Getting the vector angels of error
        double rErr1 = calcAngleDelta(wheelPosRightf, joystickAngle); // angle diff from front of wheel to target angle
        double rErr2 = calcAngleDelta(wheelPosRightb, joystickAngle); // angle diff from back of wheel to target angle
        double lErr1 = calcAngleDelta(wheelPosLeftf, joystickAngle); // angle diff from front of wheel to target angle
        double lErr2 = calcAngleDelta(wheelPosLeftb, joystickAngle); // angle diff from back of wheel to target angle

// Intializing reverse vairable
        int magnitudeDir = 0;

// turning the entire robot
        double turn = Range.clip(gamepad1.right_stick_x, -.8, .8);
        if ((((wheelPosRightf + wheelPosLeftf) / 2) < 270) && ((wheelPosRightf + wheelPosLeftf) / 2) > 90)
            turn = -turn;

// Checks if front error of either wheel is less than 90, then will turn the wheel until both wheels have an error within 2 degs of target angle
// given magnitdue is greater that .1
// else, it flips the direction of the motors, then has the back of the wheel move towards target angle.
        if (Math.abs(lErr1) <= 90 || Math.abs(rErr1) <= 90)
        {
            magnitudeDir = 1;
            if (Math.abs(rErr1) > 2 && magnitude >.1)
                wheelTurnPowerRight = -rErr1/75;
            if (Math.abs(lErr1) > 2 && magnitude >.1)
                wheelTurnPowerLeft = lErr1/75;

        }
        else
        {
            magnitudeDir = -1;
            if (Math.abs(rErr2) > 2 && magnitude > .1)
                wheelTurnPowerRight = -rErr2/75;
            if (Math.abs(lErr2) > 2 && magnitude > .1)
                wheelTurnPowerLeft = lErr2/75;
        }

//        if (Math.abs(rErr1) <= 90)
//        {
//            magnitudeDir = 1;
//            if (Math.abs(rErr1) > 2 && magnitude >.1)
//                wheelTurnPowerRight = -rErr1/90;
//        }
//        else
//        {
//            magnitudeDir = -1;
//            if (Math.abs(rErr2) > 2 && magnitude > .1)
//                wheelTurnPowerRight = -rErr2/90;
//        }
//
//        if (Math.abs(lErr1) <= 90)
//        {
//            if (Math.abs(lErr1) > 2 && magnitude >.1)
//                wheelTurnPowerLeft = lErr1/90;
//        }
//        else
//        {
//            if (Math.abs(lErr2) > 2 && magnitude > .1)
//                wheelTurnPowerLeft = lErr2/90;
//        }



        if(gamepad1.a)
        {
            while((wheelPosLeftf != 180) || (wheelPosRightf != 180))
            {
//                if((wheelPosLeftf > 181 && wheelPosLeftf < 179))
//                {
                    robot.getLeftTop().setPower((wheelPosLeftf-180)/720);
                    robot.getLeftBottom().setPower(-(wheelPosLeftf-180)/720);
//                }

                telemetry.addData("wheelPosRight", wheelPosRightf);
                telemetry.addData("wheelPosLeftf", wheelPosLeftf);
                telemetry.update();

            }
            robot.getLeftTop().setPower(0);
            robot.getLeftBottom().setPower(0);
        }














        robot.getLeftTop().setPower(((magnitude - brake) * slowMode) * magnitudeDir -  (wheelTurnPowerLeft) + turn);
        robot.getLeftBottom().setPower(((magnitude - brake) * slowMode) * magnitudeDir + (wheelTurnPowerLeft) + turn);
        robot.getRightTop().setPower(((magnitude - brake) * slowMode * magnitudeDir) -  (wheelTurnPowerRight) - turn);
        robot.getRightBottom().setPower(((magnitude - brake) * slowMode * magnitudeDir) +  (wheelTurnPowerRight) - turn);

        telemetry.addData("right error 1", rErr1);
        telemetry.addData("left error 1", lErr1);

        telemetry.addData("magnitude", magnitude);
        telemetry.addData("wheelPosRight", wheelPosRightf);
        telemetry.addData("wheelPosLeftf", wheelPosLeftf);
        telemetry.addData("wheelPosRightb", wheelPosRightb);
        telemetry.addData("wheelPosLeftb", wheelPosLeftb);
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

    public double calcAngleDelta(double start, double end)
    {
        // converting start and end vectors to unit vectors u and v
        double ui = Math.cos(Math.toRadians(start));
        double uj = Math.sin(Math.toRadians(start));
        double vi = Math.cos(Math.toRadians(end));
        double vj = Math.sin(Math.toRadians(end));

        // return u.v * sign(uxv)
        return Math.toDegrees(Math.acos(ui*vi + uj*vj)) * Math.signum(ui*vj-uj*vi);
    }


}
