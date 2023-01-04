package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="motor", group="Linear Opmode")

public class motor extends LinearOpMode {

    private DcMotor m;


    @Override
    public void runOpMode() throws InterruptedException {
        m = hardwareMap.get(DcMotor.class, "m");

        waitForStart();

        while (opModeIsActive()) {

            m.setPower(gamepad1.left_stick_x);
        }
    }
}