package frc.robot;

import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

/**
 * The one simulated Robot shared by all tests that need the real robot. Its subsystems are
 * singletons, so it can only be constructed once per test JVM.
 */
final class SimRobot {
	private static Robot s_robot;

	private SimRobot() {
	}

	/** @return the shared robot, created on first use */
	static synchronized Robot get() {
		if (s_robot == null) {
			HAL.initialize();
			// On the robot, RobotBase does this; until then the robot mode always reads UNKNOWN
			DriverStationBackend.observeUserProgramStarting();
			s_robot = new Robot();
		}
		return s_robot;
	}

	static void setMode(RobotMode mode, boolean enabled) {
		DriverStationSim.setRobotMode(mode);
		DriverStationSim.setEnabled(enabled);
		DriverStationSim.notifyNewData();
		DriverStationBackend.refreshData();
	}

	/** Runs robot loops of 20 ms simulated time each, as fast as possible. */
	static void runLoops(int count) {
		runLoops(count, false);
	}

	private static final long kLoopNanos = 20_000_000;
	// Real-time schedule: when the next paced loop is due. Kept across calls, because tests often run
	// one paced loop at a time.
	private static long s_nextDeadlineNanos;
	private static int s_lateLoops;

	/**
	 * Runs robot loops of 20 ms simulated time each.
	 *
	 * @param count the number of loops
	 * @param realTime also keep to 20 ms of real time per loop. CTRE's TalonFX simulation runs on real
	 *        time, so drive tests pace themselves to keep its motor outputs in step with the loop. Loops
	 *        follow a fixed schedule, so a slow loop is made up by shorter waits afterwards.
	 */
	static void runLoops(int count, boolean realTime) {
		SimHooks.pauseTiming();
		for (int i = 0; i < count; i++) {
			if (realTime) {
				long now = System.nanoTime();
				if (s_nextDeadlineNanos == 0 || now - s_nextDeadlineNanos > 5 * kLoopNanos) {
					// First paced loop, or the test paused (e.g. between scenarios): start a new schedule
					s_nextDeadlineNanos = now;
				} else if (now - s_nextDeadlineNanos > kLoopNanos) {
					s_lateLoops++; // started more than a full loop behind schedule
				}
			}
			SimHooks.stepTiming(0.02);
			DriverStationSim.notifyNewData();
			DriverStationBackend.refreshData();
			get().robotPeriodic();
			if (realTime) {
				s_nextDeadlineNanos += kLoopNanos;
				long waitNanos = s_nextDeadlineNanos - System.nanoTime();
				if (waitNanos > 0) {
					try {
						Thread.sleep(waitNanos / 1_000_000, (int) (waitNanos % 1_000_000));
					} catch (InterruptedException e) {
						Thread.currentThread().interrupt();
						return;
					}
				}
			}
		}
	}

	/**
	 * @return how many paced loops have started more than a full loop behind the real-time schedule
	 *         (a sign the machine is too slow for the drive tests' timing to be meaningful)
	 */
	static int lateLoops() {
		return s_lateLoops;
	}

	/** Puts the driver station back the way other tests expect it. */
	static void reset() {
		setMode(RobotMode.UNKNOWN, false);
		SimHooks.resumeTiming();
	}
}
