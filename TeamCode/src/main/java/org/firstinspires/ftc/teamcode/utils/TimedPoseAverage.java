package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Rotation2d;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Optional;

public class TimedPoseAverage {

	private final double windowSeconds;

	private static class TimedPose {
		final long timeMillis;
		final Pose2d pose;

		TimedPose(long timeMillis, Pose2d pose) {
			this.timeMillis = timeMillis;
			this.pose = pose;
		}
	}

	private final Deque<TimedPose> buffer = new ArrayDeque<>();

	public TimedPoseAverage(double windowSeconds) {
		this.windowSeconds = windowSeconds;
	}

	public void add(Pose2d pose) {
		long now = System.currentTimeMillis();
		buffer.addLast(new TimedPose(now, pose));

		// Remove old entries
		double cutoff = now - (windowSeconds * 1000.0); // to ms
		while (!buffer.isEmpty() && buffer.getFirst().timeMillis < cutoff) {
			buffer.removeFirst();
		}
	}

	public Optional<Pose2d> getAverage() {
		if (buffer.isEmpty()) {
			return Optional.empty();
		}

		// Average translation
		double sumX = 0.0;
		double sumY = 0.0;

		// Average heading using vector addition to avoid wrap-around issues
		double sumCos = 0.0;
		double sumSin = 0.0;

		for (TimedPose entry : buffer) {
			Pose2d p = entry.pose;
			sumX += p.position.x;
			sumY += p.position.y;

			double angle = p.heading.log(); // returns theta in radians
			sumCos += Math.cos(angle);
			sumSin += Math.sin(angle);
		}

		double n = buffer.size();
		double avgX = sumX / n;
		double avgY = sumY / n;
		double avgHeading = Math.atan2(sumSin / n, sumCos / n);

		return Optional.of(new Pose2d(new Vector2d(avgX, avgY), Rotation2d.exp(avgHeading)));
	}

	public boolean isReady() {
		return !buffer.isEmpty();
	}
}
