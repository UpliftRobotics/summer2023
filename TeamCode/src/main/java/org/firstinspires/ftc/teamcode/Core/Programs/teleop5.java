package org.firstinspires.ftc.teamcode.Core.Programs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.Main;
import org.firstinspires.ftc.teamcode.Core.UpliftTele;


@TeleOp(name = "test5", group = "Opmodes")
public class teleop5 extends UpliftTele {

    Main robot;
    double prevTime;
    double lPrevError;
    double rPrevError;

    double kP = 0.0;
    double kD = 0.0;

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

        robot.getRightTop().setDirection(DcMotorSimple.Direction.REVERSE);
        robot.getRightBottom().setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void bodyLoop() throws InterruptedException
    {
        if (gamepad1.dpad_down)
           kP-=0.005;
        if (gamepad1.dpad_up)
            kP+=0.005;
        if (gamepad1.dpad_left)
            kD-=0.005;
        if (gamepad1.dpad_right)
            kD+=0.005;

        kP = Range.clip(kP, -1.0, 1.0);
        kD = Range.clip(kD, -1.0, 1.0);

//joystick stuff
        double magnitude = Range.clip(Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2)), 0 , .8); // get magnitude of joystick from center
        float joystickAngle = getJoystickAngle(); //((getJoystickAngle()) % 360) ;   // Adjust angle to have 0 degrees as east and clockwise rotation

//triggers
        double brake = Range.clip( gamepad1.left_trigger , 0 , magnitude);
//        double slowMode = 1 - Range.clip(gamepad1.right_trigger , 0 , .5);

        // get a bunch of angles
        double backJoystick = (joystickAngle + 180) % 360;
        double wheelPosRight = ((((robot.getRightTop().getCurrentPosition() + -robot.getRightBottom().getCurrentPosition()) / 2) / 2.641111 )+90) % 360; // get wheels angle, assuming starting from forward pos
        double wheelPosLeft =  (((((robot.getLeftTop().getCurrentPosition() + -robot.getLeftBottom().getCurrentPosition()) / 2) / 2.641111 )+90) % 360);

        // calculate errors
        double rErr1 = calcAngleDelta(wheelPosRight, joystickAngle);
        double rErr2 = calcAngleDelta(wheelPosRight, backJoystick);
        double lErr1 = calcAngleDelta(wheelPosLeft, joystickAngle);
        double lErr2 = calcAngleDelta(wheelPosLeft, backJoystick);

        // right pod error selection
        boolean rErrSwitch = Math.abs(rErr2) < Math.abs(rErr1);
        double rError = rErrSwitch ? rErr2 : rErr1;
        int rDir = rErrSwitch ? -1 : 1;

        // left pod error selection
        boolean lErrSwitch = Math.abs(lErr2) < Math.abs(lErr1);
        double lError = rErrSwitch ? lErr2 : lErr1;
        int lDir = lErrSwitch ? -1 : 1;

        // woohoo PD error correction. errors are normalized to 0-1 based on 0-180 angle error. No PID cuz its hard to program
        double lErrorCorrected = kP * lError/180 + kD * (lError/180 - lPrevError) / (getRuntime() - prevTime);
        double rErrorCorrected = kP * rError/180 + kD * (rError/180 - rPrevError) / (getRuntime() - prevTime);

        rPrevError = rError;
        lPrevError = lError;
        prevTime = getRuntime();

        robot.getRightTop().setPower(((magnitude-brake) + rErrorCorrected) * rDir);
        robot.getRightBottom().setPower(((magnitude-brake) - rErrorCorrected) * rDir);
//        robot.getLeftTop().setPower(((magnitude-brake) - lErrorCorrected) * lDir);
//        robot.getLeftBottom().setPower(((magnitude-brake) + lErrorCorrected) * lDir);

        telemetry.addData("kP: ", kP);
        telemetry.addData("kD: ", kD);
        telemetry.addData("Target Angle: ", joystickAngle);
        telemetry.addData("Right pos: ", wheelPosRight);
        telemetry.addData("Right err: ", rError);
        telemetry.addData("Right PID: ", rErrorCorrected);
        telemetry.addData("Right dir: ", rDir);
        telemetry.addData("Left pos: ", wheelPosLeft);
        telemetry.addData("Left err: ", lError);
        telemetry.addData("Left PID: ", lErrorCorrected);
        telemetry.addData("left dir: ", lDir);
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
        double y = gamepad1.left_stick_y;

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

    private double calcAngleDelta(double start, double end)
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
