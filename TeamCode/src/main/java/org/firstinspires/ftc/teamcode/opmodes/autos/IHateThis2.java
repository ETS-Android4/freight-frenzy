package org.firstinspires.ftc.teamcode.opmodes.autos;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.motion.TrajectorySequence;
import org.firstinspires.ftc.teamcode.motion.TrajectorySequenceBuilder;


@Config
@Autonomous(group = "drive")
@Disabled //edited by Randall Delafuente FTC 12014 to be disabled =D
public class IHateThis2 extends RobotAuto {
    public static double DISTANCE = 48; // in

    @Override
    public void runOpMode() throws InterruptedException {
        RRMecanumDrive RRdrive = new RRMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0);

        RRdrive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            TrajectorySequence trajSeq = RRdrive.trajectorySequenceBuilder(startPose)
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .build();
            RRdrive.followTrajectorySequence(trajSeq);
        }
    }
}