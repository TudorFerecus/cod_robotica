package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class Test extends LinearOpMode {

    private Encoder parallelEncoder;
    private Encoder perpendicularEncoder;

    //Hardware class instance
    private Hardware hardware;


    private void armMovement(boolean upArm, boolean downArm)
    {
        if(upArm)
        {
            hardware.mArmRight.setPower(Specifications.arm_moving_speed_up);
            hardware.mArmLeft.setPower(Specifications.arm_moving_speed_up);
        }
        else if(downArm)
        {
            hardware.mArmRight.setPower(-Specifications.arm_moving_speed_down);
            hardware.mArmLeft.setPower(-Specifications.arm_moving_speed_down);
        }
        else
        {

            hardware.mArmRight.setPower(0.1);
            hardware.mArmLeft.setPower(0.1);
        }
    }

    private void coneGrab(boolean grab, boolean release)
    {
        if(grab)
        {
            hardware.servoCone.setPower(Specifications.cone_grab_speed);
        }
        else if(release)
        {
            hardware.servoCone.setPower(-Specifications.cone_grab_speed);
        }
        else
        {
            hardware.servoCone.setPower(0);
        }
    }


    private void handMovement(float upHand, float downHand)
    {
        if(upHand != 0)
        {
                hardware.servoHand.setPower(upHand * Specifications.servo_multiplier);
        }
        else if(downHand != 0)
        {
            hardware.servoHand.setPower(-downHand * Specifications.servo_multiplier);
        }
        else
        {
            hardware.servoHand.setPower(0);
        }
    }


    private void movement(float forward, float strafe, float rotation)
    {
        // power applied to the robot wheel by wheel
        double[] power = new double[4];
        rotation *= -1;
        power[0] = (-forward + strafe + rotation) * Specifications.moving_speed_teleop;   //+
        power[1] = (+forward + strafe + rotation) * Specifications.moving_speed_teleop;   //-
        power[2] = (-forward - strafe + rotation) * Specifications.moving_speed_teleop;   //-
        power[3] = (+forward - strafe + rotation) * Specifications.moving_speed_teleop;   //+

        // applying the power
        for(int i=0; i<4; i++)
        {
            hardware.motor[i].setPower(power[i]);
        }
    }

    private void armRotation(boolean rotateLeft, boolean rotateRight)
    {
        if(rotateLeft)
        {
            hardware.mRotate.setPower(Specifications.arm_rotation_speed);
        }
        else if(rotateRight)
        {
            hardware.mRotate.setPower(-Specifications.arm_rotation_speed);
        }
        else
        {
            hardware.mRotate.setPower(0);
        }
    }

    @Override
    public void runOpMode()
    {
        hardware = new Hardware(hardwareMap, false);

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "m3"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "m2"));

        waitForStart();

        while(opModeIsActive())
        {
            boolean upArm = gamepad1.dpad_up;
            boolean downArm = gamepad1.dpad_down;

            boolean grabCone = gamepad1.right_bumper;
            boolean releaseCone = gamepad1.left_bumper;

            float upHand = gamepad1.right_trigger;
            float downHand = gamepad1.left_trigger;

            float fata = gamepad1.left_stick_y; // forward movement
            float lateral = gamepad1.left_stick_x; // strafe movement
            float rotire = gamepad1.right_stick_x; // rotate
            boolean rotateLeft = gamepad1.dpad_left;
            boolean rotateRight = gamepad1.dpad_right;

            armRotation(rotateLeft, rotateRight);
            armMovement(upArm, downArm);
            coneGrab(grabCone, releaseCone);
            //handMovement(upHand, downHand);

            movement(fata, lateral, rotire);

            telemetry.addData("paralel: ", parallelEncoder.getCurrentPosition());
            telemetry.addData("perpendicular: ", perpendicularEncoder.getCurrentPosition());

            telemetry.update();

        }

    }
}