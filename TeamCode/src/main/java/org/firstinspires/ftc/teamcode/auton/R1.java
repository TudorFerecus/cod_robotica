package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EEPROM_232H;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.Specifications;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomy: Rosu 1", group="Autonomy")
public class R1 extends ScheleteAutonomie {
    ElapsedTime elapsedTime;


    @Override
    public void runOpMode()
    {
        init_auto();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPos = new Pose2d(63, -36f , Math.toRadians(180));


        drive.setPoseEstimate(startPos);

        Trajectory traj0 = drive.trajectoryBuilder(startPos)
                .forward(1)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(62, -11f, Math.toRadians(180)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(36, -11f, Math.toRadians(180)))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(16, -11f, Math.toRadians(180)))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(16, -37, Math.toRadians(180)))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(62, -57f, Math.toRadians(180)))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(39, -57f, Math.toRadians(180)))
                .build();




//        Trajectory traj0 = drive.trajectoryBuilder(startPos)
//                .forward()
//                .build();

        elapsedTime= new ElapsedTime();
        elapsedTime.reset();
        AprilTagInit();

        while (!isStarted() && !isStopRequested())
        {
            AprilTagInitLoop();
        }

        if(tagOfInterest != null)
        {
            telemetry.addData("Autonomy Case", tagOfInterest.id);
            telemetry.update();
        }

        coneRaise(50);

        if(tagOfInterest.id == 3) {
            //hardware.servoCone.setPower(Specifications.cone_grab_speed);
            hardware.mArmLeft.setPower(Specifications.arm_moving_speed_up_auto);
            hardware.mArmRight.setPower(Specifications.arm_moving_speed_up_auto);
            sleep(500);

            drive.followTrajectory(traj0);

            sleep(200);

            drive.followTrajectory(traj1);

            sleep(200);

            drive.followTrajectory(traj2);

            // sleep(300);

            //coneRaise(Specifications.coneTickMedium);

            //drive.turn(Math.toRadians(-45));

            //  drive.followTrajectory(traj3);

            // sleep(500);
            //hardware.servoCone.setPower(-Specifications.cone_grab_speed);

            // sleep(1000);

            //hardware.servoCone.setPower(0);

            //sleep(1000);

            //coneRaise(200);
        }




        //hardware.mArmLeft.setPower(0.3f);
        //hardware.mArmRight.setPower(0.3f);


        if(tagOfInterest.id == 2)
        {
            hardware.mArmLeft.setPower(Specifications.arm_moving_speed_up_auto);
            hardware.mArmRight.setPower(Specifications.arm_moving_speed_up_auto);
            sleep(500);
            drive.followTrajectory(traj0);
            sleep(500);
            drive.followTrajectory(traj1);
            sleep(500);
            drive.followTrajectory(traj3);
            sleep(500);
            drive.followTrajectory(traj4);
        }
        if(tagOfInterest.id == 1)
        {
            hardware.mArmLeft.setPower(Specifications.arm_moving_speed_up_auto);
            hardware.mArmRight.setPower(Specifications.arm_moving_speed_up_auto);
            sleep(500);
            drive.followTrajectory(traj0);
            sleep(500);
            drive.followTrajectory(traj5);
            sleep(500);
            drive.followTrajectory(traj6);

        }



        if(isStopRequested()) return;

    }

}
