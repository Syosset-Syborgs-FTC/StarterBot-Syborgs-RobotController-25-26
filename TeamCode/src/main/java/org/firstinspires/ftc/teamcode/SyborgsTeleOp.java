package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.TimeoutAction;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("unused")
@TeleOp(name = "Syborgs TeleOp", group = "Robot")
public class SyborgsTeleOp extends LinearOpMode {

	private final FtcDashboard dash = FtcDashboard.getInstance();
	private final List<Action> runningActions = new ArrayList<>();

	private DcMotor fl, fr, bl, br;
	private IMU imu;
	private Shooter shooter;
	private Shooter.AutoFire autoFire;

	@Override
	public void runOpMode() {

		runSetup();

		waitForStart();
		imu.resetYaw();

		autoFire = shooter.autoFireAction();

		while (opModeIsActive()) {
			TelemetryPacket packet = new TelemetryPacket();

			driveRobotFieldCentric();

			handleShooting();

			runActions(packet);
			autoFire.run(packet);

			dash.sendTelemetryPacket(packet);
			telemetry.addData("1. Outtake Power", "%.2f", shooter.getPower());
			telemetry.addData("2. Velocity", "%.2f", shooter.getCurrentVelocity());
			telemetry.addData("3. Error", "%.2f", shooter.TARGET_VELOCITY - shooter.getCurrentVelocity());
			telemetry.addData("Heading (deg)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
			telemetry.update();
		}
	}

	private void runActions(TelemetryPacket packet) {
		List<Action> newActions = new ArrayList<>();
		for (Action action : runningActions) {
			action.preview(packet.fieldOverlay());
			if (action.run(packet)) {
				newActions.add(action);
			}
		}
		runningActions.clear();
		runningActions.addAll(newActions);
	}

	private void handleShooting() {
		if (gamepad1.y) {
			shooter.setTargetVelocity(640);
		}
		if (gamepad1.a) {
			shooter.setTargetVelocity(2100);
		}
		if (gamepad1.b) {
			shooter.setTargetVelocity(1150);
		}

		runningActions.add(shooter.updateVelocity());
		if (gamepad1.left_bumper) {
			if (shooter.TARGET_VELOCITY == Shooter.OUTTAKE_HOLD_POWER) {
				shooter.setTargetVelocity(1150);
			} else {
				shooter.setTargetVelocity(Shooter.OUTTAKE_HOLD_POWER);
			}
		}
		if (gamepad1.x) {
			autoFire.enabled = !autoFire.enabled;
		}
		if (gamepad1.right_bumper && !autoFire.enabled) {
			runningActions.add(new TimeoutAction(shooter.shoot(), 4));
		}
	}

	private void runSetup() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		setupHardwareMap();

		shooter.reset();

		fl.setDirection(DcMotorSimple.Direction.REVERSE);
		bl.setDirection(DcMotorSimple.Direction.REVERSE);

		runForAllMotors(m -> m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));

		runForAllMotors(m -> m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));

		imu.initialize(new IMU.Parameters(
				new RevHubOrientationOnRobot(
						RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
						RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
				)
		));

		telemetry.addData("Status", "Initialized");
		telemetry.update();
	}

	private void setupHardwareMap() {
		fl = hardwareMap.get(DcMotor.class, "fl");
		fr = hardwareMap.get(DcMotor.class, "fr");
		bl = hardwareMap.get(DcMotor.class, "bl");
		br = hardwareMap.get(DcMotor.class, "br");
		imu = hardwareMap.get(IMU.class, "imu");

		shooter = new Shooter(hardwareMap);
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
	public void runForAllMotors(Consumer<DcMotor> c) {
		c.accept(fl);
		c.accept(fr);
		c.accept(bl);
		c.accept(br);
	}
}