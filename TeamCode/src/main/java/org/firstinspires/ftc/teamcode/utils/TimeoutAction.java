package org.firstinspires.ftc.teamcode.utils;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class TimeoutAction implements Action {
	private final Action inner;
	private final long timeoutNanos;
	private long startNanos = -1;
	private boolean firstRun = true;

	public TimeoutAction(Action inner, double timeoutSeconds) {
		this.inner = inner;
		this.timeoutNanos = (long) (timeoutSeconds * 1e9);
	}

	@Override
	public boolean run(TelemetryPacket p) {
		if (firstRun) {
			startNanos = System.nanoTime();
			firstRun = false;
		}

		boolean innerShouldContinue = inner.run(p);
		if (!innerShouldContinue) {
			return false;
		}

		// Timeout check
		long elapsed = System.nanoTime() - startNanos;
		return elapsed < timeoutNanos; // stop if past time
	}
}
