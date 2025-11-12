package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Syborgs Auton")
public class Auton extends LinearOpMode {

	@Override
	public void runOpMode() throws InterruptedException {
		waitForStart();
		TrajectoryActionBuilder tab = new TrajectoryActionBuilder();
		Pose2d initialPose = new Pose2d(0, 0, 0);
	}
}
