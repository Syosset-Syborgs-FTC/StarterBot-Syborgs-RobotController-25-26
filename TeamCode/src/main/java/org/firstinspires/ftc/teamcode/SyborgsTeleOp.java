package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Syborgs TeleOp", group = "Robot")
public class SyborgsTeleOp extends LinearOpMode {

	private final FtcDashboard dash = FtcDashboard.getInstance();
	private final List<Action> runningActions = new ArrayList<>();

	private DcMotor fl, fr, bl, br;
	private IMU imu;
	private boolean isShooting = false;

	@Override
	public void runOpMode() throws InterruptedException {
		// --- HARDWARE MAP ---
		fl = hardwareMap.get(DcMotor.class, "fl");
		fr = hardwareMap.get(DcMotor.class, "fr");
		bl = hardwareMap.get(DcMotor.class, "bl");
		br = hardwareMap.get(DcMotor.class, "br");
		imu = hardwareMap.get(IMU.class, "imu");

		Shooting shooting = new Shooting(hardwareMap);

		// --- DRIVE CONFIG ---
		fl.setDirection(DcMotorSimple.Direction.REVERSE);
		bl.setDirection(DcMotorSimple.Direction.REVERSE);
		fr.setDirection(DcMotorSimple.Direction.FORWARD);
		br.setDirection(DcMotorSimple.Direction.FORWARD);

		fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		imu.initialize(new IMU.Parameters(
				new RevHubOrientationOnRobot(
						RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
						RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
				)
		));

		telemetry.addData("Status", "Initialized");
		telemetry.update();
		waitForStart();

		imu.resetYaw();

		while (opModeIsActive()) {
			TelemetryPacket packet = new TelemetryPacket();

			driveRobotFieldCentric();

			if (gamepad1.y) {
				shooting.setTargetVelocity(640);
			}
			if (gamepad1.a) {
				shooting.setTargetVelocity(2100);
			}
			if (gamepad1.b) {
				shooting.setTargetVelocity(1150);
			}

			runningActions.add(shooting.updateVelocityAction());
			if (gamepad1.left_bumper) {
				if (shooting.TARGET_VELOCITY == Shooting.OUTTAKE_HOLD_POWER) {
					shooting.setTargetVelocity(1150);
				} else {
					shooting.setTargetVelocity(Shooting.OUTTAKE_HOLD_POWER);
				}
			}
			if (gamepad1.right_bumper) {
				runningActions.add(shooting.shoot());
			}

			// update running actions
			List<Action> newActions = new ArrayList<>();
			for (Action action : runningActions) {
				action.preview(packet.fieldOverlay());
				if (action.run(packet)) {
					newActions.add(action);
				}
			}
			runningActions.clear();
			runningActions.addAll(newActions);

			dash.sendTelemetryPacket(packet);
			telemetry.addData("1. Outtake Power", "%.2f", shooting.getPower());
			telemetry.addData("2. Velocity", "%.2f", shooting.getCurrentVelocity());
			telemetry.addData("3. Error", "%.2f", shooting.TARGET_VELOCITY - shooting.getCurrentVelocity());
			telemetry.addData("Heading (deg)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
			telemetry.update();
		}
	}

	public void driveRobotFieldCentric() {
		double y = -gamepad1.left_stick_y; // forward/back
		double x = gamepad1.left_stick_x;  // strafe
		double rx = gamepad1.right_stick_x; // rotation
		double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
		if (gamepad1.left_trigger > 0.4 && gamepad1.right_trigger < 0.4) {
			rx = -0.1;
		} else if (gamepad1.right_trigger > 0.4 && gamepad1.left_trigger < 0.4) {
			rx = 0.1;
		}
		if (gamepad1.dpad_down) {
			y = -0.1;
		} else if (gamepad1.dpad_up) {
			y = 0.1;
		}
		if (gamepad1.dpad_right) {
			x = 0.1;
		} else if (gamepad1.dpad_left) {
			x = -0.1;
		}

		// Field-centric transformation
		double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
		double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

		// Mecanum wheel power equations
		double flPower = rotY + rotX + rx;
		double blPower = rotY - rotX + rx;
		double frPower = rotY - rotX - rx;
		double brPower = rotY + rotX - rx;

		double max = Math.max(1.0, Math.abs(flPower));
		max = Math.max(max, Math.abs(frPower));
		max = Math.max(max, Math.abs(blPower));
		max = Math.max(max, Math.abs(brPower));

		fl.setPower(flPower / max);
		fr.setPower(frPower / max);
		bl.setPower(blPower / max);
		br.setPower(brPower / max);
	}
}