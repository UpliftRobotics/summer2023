    package org.firstinspires.ftc.teamcode.Core.Programs;

    import com.qualcomm.robotcore.util.Range;

    import org.firstinspires.ftc.teamcode.Core.Main;
    import org.firstinspires.ftc.teamcode.Core.UpliftTele;


    @com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "manual Teleop", group = "Opmodes")
    public class teleop1 extends UpliftTele {

        Main robot;
        @Override
        public void initHardware()
        {
            robot = new Main(this);
        }


        @Override
        public void initAction() {

        }

             @Override
        public void bodyLoop() throws InterruptedException {

    //        Manual Control
            robot.getLeftTop().setPower(.7 *  Range.clip(gamepad1.left_stick_y,-1,1));
            robot.getLeftBottom().setPower(.7 * Range.clip(gamepad1.right_stick_y,-1,1));
            robot.getRightTop().setPower(.7 *  Range.clip(gamepad2.left_stick_y,-1,1));
            robot.getRightBottom().setPower(.7 * Range.clip(gamepad2.right_stick_y,-1,1));

            telemetry.addData("left stick 1 power" ,gamepad1.left_stick_y );
                 telemetry.addData("right stick 1 power" ,gamepad1.right_stick_y );
                 telemetry.addData("left stick 2 power" ,gamepad2.left_stick_y );
                 telemetry.addData("right stick 2 power" ,gamepad2.right_stick_y );
                 telemetry.update();





        }

        @Override
        public void exit()
        {

        }
    }
