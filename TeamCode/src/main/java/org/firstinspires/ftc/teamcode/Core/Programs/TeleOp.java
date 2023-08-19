    package org.firstinspires.ftc.teamcode.Core.Programs;

    import static com.google.blocks.ftcrobotcontroller.util.Identifier.RANGE;

    import com.qualcomm.robotcore.util.Range;

    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

    import org.firstinspires.ftc.teamcode.Core.Main;
    import org.firstinspires.ftc.teamcode.Core.UpliftTele;
    import org.firstinspires.ftc.teamcode.Core.UpliftAuto;


    @TeleOp(name = "test1", group = "Opmodes")
    public class teleop extends UpliftTele {

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

            // 1st test
    //        double ltpower = (Range.clip(gamepad1.left_stick_y , -.8, .8)) + (Range.clip(gamepad1.right_stick_x , -.2 , .2));
    //        double lbpower = (Range.clip(gamepad1.left_stick_y , -.8, .8)) - (Range.clip(gamepad1.right_stick_x , -.2 , .2));;
    //
    //        double rtpower = (Range.clip(gamepad1.left_stick_y , -.8, .8)) - (Range.clip(gamepad1.right_stick_x , -.2 , .2));;
    //        double rbpower = (Range.clip(gamepad1.left_stick_y , -.8, .8)) + (Range.clip(gamepad1.right_stick_x , -.2 , .2));;
    //
    //
    //
    //        robot.getLeftTop().setPower(ltpower);
    //        robot.getLeftBottom().setPower(lbpower);
    //        robot.getRightTop().setPower(rtpower);
    //        robot.getRightBottom().setPower(rbpower);

    //        //2nd test
    //        // This is going to a system where the wheel is basically going to just point in the direcion that the
    //        // left analog stick is pointing, using the endocers and angle of the joystick as the main 2 inputs.
    //        // This assumes no skipping in, the system at all.
    //
    //        double wheelGoal= Math.atan(gamepad1.left_stick_y + gamepad1.left_stick_x); // get angle of joystick
    //        double total_power = Math.sqrt( Math.pow (gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2 )); // get magnitude of joystick from center
    //        double wheelReal = (robot.getLeftBottom().getCurrentPosition() + robot.getLeftTop().getCurrentPosition()) / 2; // get wheels angle, assuming starting from forward pos
    //
    //        wheelReal = wheelReal/100; // Change this int to basically convert the total avg encoder ticks to a degree.






        }

        @Override
        public void exit()
        {

        }
    }
