package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
	private final DcMotorEx ot;
	private final Servo lr, rr;

	public static final double OUTTAKE_HOLD_POWER = 0;
	public static final double FAR_SHOT_POWER = 2100;
	public static final double NORMAL_SHOT_POWER = 1150;
	public static final double NEAR_SHOT_POWER = 640;
	public double TARGET_VELOCITY = OUTTAKE_HOLD_POWER;
	private static final double LEFT_SERVO_HOME_POS = 0.1;
	private static final double LEFT_SERVO_SET_POS = 0.6;
	private static final double RIGHT_SERVO_HOME_POS = LEFT_SERVO_SET_POS;
	private static final double RIGHT_SERVO_SET_POS = LEFT_SERVO_HOME_POS;

	public Shooter(HardwareMap hardwareMap) {
		ot = hardwareMap.get(DcMotorEx.class, "ot");
		lr = hardwareMap.get(Servo.class, "lr");
		rr = hardwareMap.get(Servo.class, "rr");
		ot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(100, 7, 5, 0));
	}

	public void reset() {
		lr.setPosition(LEFT_SERVO_HOME_POS);
		rr.setPosition(RIGHT_SERVO_HOME_POS);
		ot.setVelocity(OUTTAKE_HOLD_POWER);
	}

	public void setTargetVelocity(double velocity) {
		this.TARGET_VELOCITY = velocity;
	}

	public double getCurrentVelocity() {
		return ot.getVelocity();
	}

	public double getPower() {
		return ot.getPower();
	}

	public Action updateVelocity() {
		return t -> {
			ot.setVelocity(TARGET_VELOCITY);
			if (ot.isOverCurrent()) {
				t.addLine("WARNING: Motor is over current! Please reduce power to prevent overheating.");
			}
			return false;
		};
	}

	public Action shoot() {
		return new SequentialAction(
				t -> {
					if (ot.getVelocity() > TARGET_VELOCITY - 20) {
						lr.setPosition(LEFT_SERVO_SET_POS);
						rr.setPosition(RIGHT_SERVO_SET_POS);
						return false;
					}
					return true;
				},
				new SleepAction(0.5),
				new InstantAction(() -> {
					lr.setPosition(LEFT_SERVO_HOME_POS);
					rr.setPosition(RIGHT_SERVO_HOME_POS);
				}),
				new SleepAction(0.5)
		);
	}

	public class AutoFire implements Action {
		Action shootAction = shoot();
		public boolean enabled = false;

		@Override
		public boolean run(@NonNull TelemetryPacket t) {
			if (!enabled) {
				return true;
			}
			if (!shootAction.run(t)) {
				shootAction = shoot(); // keep firing
			}
			return true;
		}
	}

	public AutoFire autoFireAction() {
		return new AutoFire();
	}
}
