package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.utils.TimedPoseAverage;
import java.util.List;
import java.util.Locale;

@Autonomous(name = "Syborgs Auton")
public class Auton extends LinearOpMode {
	AprilTagProcessor aprilTag;
	MecanumDrive drive;
	Shooter shooter;
	TimedPoseAverage poseAverage = new TimedPoseAverage(0.5);
	@Override
	public void runOpMode() throws InterruptedException {
		Pose2d initialPose = new Pose2d(60, 12, Math.toRadians(180));
		drive = new MecanumDrive(hardwareMap, initialPose);
		shooter = new Shooter(hardwareMap);
		Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0,0, System.nanoTime()); // TODO: set correct camera position
		YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);


		aprilTag = new AprilTagProcessor.Builder()
				.setCameraPose(cameraPosition, cameraOrientation)
				.build();

		VisionPortal visionPortal = new VisionPortal.Builder()
				.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
				.addProcessor(aprilTag)
				.build();

		TrajectoryActionBuilder tab = drive.actionBuilder(initialPose)
				.splineTo(new Vector2d(-48, 41), Math.toRadians(128));
		shooter.reset();

		while (opModeInInit()) {
			relocalizeFromAprilTag();
			telemetryAprilTag();

			telemetry.addData("Current Pose", drive.localizer.getPose());
			telemetry.update();
		}
		waitForStart();
		Actions.runBlocking(new RaceAction(
				new SequentialAction(
						tab.build(),
						shooter.shoot(),
						shooter.shoot(),
						shooter.shoot()
				),
				t -> {
					shooter.updateVelocity().run(t);
					return true; // this action will never complete, so the race ends when the trajectory ends
				}
		));

	}
	private void relocalizeFromAprilTag() {
		List<AprilTagDetection> detections = aprilTag.getFreshDetections();
		if (detections == null || detections.isEmpty()) {
			return;
		}
		for (AprilTagDetection detection : detections) {
			telemetry.addData("AprilTag", "ID: %d", detection.id);
			double robotPoseX = detection.robotPose.getPosition().x;
			double robotPoseY = detection.robotPose.getPosition().y;
			double yaw = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

			poseAverage.add(new Pose2d(robotPoseX, robotPoseY, yaw));
		}

		poseAverage.getAverage().ifPresent(pose -> {
			drive.localizer.setPose(pose);
			telemetry.addData("Relocalized Pose", pose);
		});

	}
	private void telemetryAprilTag() {
		List<AprilTagDetection> currentDetections = aprilTag.getDetections();
		telemetry.addData("# AprilTags Detected", currentDetections.size());

		for (AprilTagDetection detection : currentDetections) {
			if (detection.metadata != null) {
				telemetry.addLine(String.format(Locale.US, "\n==== (ID %d) %s", detection.id, detection.metadata.name));
				if (!detection.metadata.name.contains("Obelisk")) {
					telemetry.addLine(String.format(Locale.US, "XYZ %6.1f %6.1f %6.1f  (inch)",
							detection.robotPose.getPosition().x,
							detection.robotPose.getPosition().y,
							detection.robotPose.getPosition().z));
					telemetry.addLine(String.format(Locale.US, "PRY %6.1f %6.1f %6.1f  (deg)",
							detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
							detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
							detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
				}
			} else {
				telemetry.addLine(String.format(Locale.US, "\n==== (ID %d) Unknown", detection.id));
				telemetry.addLine(String.format(Locale.US, "Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
			}
		}

		telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
		telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

	}


}
