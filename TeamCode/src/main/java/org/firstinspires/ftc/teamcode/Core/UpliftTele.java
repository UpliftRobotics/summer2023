package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class UpliftTele extends LinearOpMode {

    public boolean isStarted, isLooping, isFinished;

    public abstract void initHardware();

    public abstract void initAction();

    public abstract void bodyLoop() throws InterruptedException;

    public abstract void exit();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Initializing", "Started");
        telemetry.update();

        initHardware();
        initAction();


        telemetry.addData("Initializing", "Finished");
        telemetry.update();

        waitForStart();
        isStarted = true;


        telemetry.addData("Body", "Started");
        telemetry.update();


        while (!isStopRequested()) {
            isLooping = true;
            bodyLoop();
        }

        telemetry.addData("Body", "Finished");

        telemetry.update();


        exit();

        telemetry.update();
    }

}