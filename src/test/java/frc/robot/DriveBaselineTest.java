package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.DriverStationSim;

import frc.robot.subsystems.Drive;

/**
 * Baseline for how the drive responds to the driver's sticks, run on the real Robot in simulation.
 * Each scenario starts from rest, holds the sticks for one second, and checks where the robot ended
 * up and how the wheels are pointed.
 *
 * <p>The expected values were measured on 2026-11-14 with the Commands v3 compat package (before
 * the native-v3 rewrites), over 6 repetitions. CTRE's TalonFX simulation runs on real time, so the
 * loops are paced at real time and results still vary a little between runs; each tolerance is
 * comfortably wider than the variation seen. A failure means the drive behaves differently, not
 * noise: check the change before widening a tolerance.
 *
 * <p>Sticks: left stick = drive (field-relative), left trigger minus right trigger = rotate.
 */
class DriveBaselineTest {
	private static final int kDriver = Constants.ControllerConstants.kDriverControllerPort;
	private static final int kLoopsPerSecond = 50;

	// Baseline values (see class comment)
	private static final double kFullForwardMeters = 3.28; // observed 3.268 - 3.286
	private static final double kHalfForwardMeters = 1.11; // observed 1.090 - 1.120
	private static final double kFullStrafeMeters = 2.95; // observed 2.811 - 3.022
	// Rotation is about 3.5% faster counter-clockwise than clockwise in every run (the clockwise wheel
	// pattern settles at 137 instead of 135 degrees; see the simulated steering in SwerveModule). That
	// is current behavior, so each direction has its own baseline rather than one wide tolerance that
	// would hide a change.
	// Re-recorded 2026-11-14 after the simulated gyro was fixed to turn by the wheels' actual motion
	// (it used to integrate drive motor voltages and read about 600 degrees per second).
	private static final double kFullRotationLeftDegrees = 218; // observed 218.3 - 218.5
	private static final double kFullRotationRightDegrees = -211; // observed -210.4 - -212.5
	// After turning, the first second of forward driving veers a few degrees while the wheels
	// re-steer out of the rotation pattern
	private static final double kFieldRelativeVeerDegrees = -2.8; // observed -2.80 - -2.84

	record Measurement(double x, double y, double headingChangeDeg, double[] moduleAnglesDeg) {
		@Override
		public String toString() {
			return String.format("x=%.3f y=%.3f dHeading=%.1f angles=%s", x, y, headingChangeDeg,
					Arrays.toString(Arrays.stream(moduleAnglesDeg).map(Math::round).toArray()));
		}
	}

	@AfterAll
	static void reset() {
		setSticks(0, 0, 0, 0);
		SimRobot.reset();
	}

	@BeforeEach
	void startTeleop() {
		Robot robot = SimRobot.get();
		DriverStationSim.setJoystickIsGamepad(kDriver, true);
		DriverStationSim.setJoystickAxesMaximumIndex(kDriver, 5);
		DriverStationSim.setJoystickAxesAvailable(kDriver, 0b111111); // a bitmask of axes, not a count
		setSticks(0, 0, 0, 0);
		SimRobot.setMode(RobotMode.TELEOPERATED, true);
		robot.teleopInit(); // binds the competition controls, including the JoystickDrive default
		s_lateLoopsAtStart = SimRobot.lateLoops();
	}

	private static int s_lateLoopsAtStart;

	/**
	 * The baselines only mean something if the loops kept to real time (see SimRobot.runLoops). On a
	 * machine too slow for that, say so plainly, before any distance is checked, instead of reporting
	 * odd distances.
	 */
	private static void assertLoopsKeptToRealTime() {
		int late = SimRobot.lateLoops() - s_lateLoopsAtStart;
		assertTrue(late <= 5, late + " robot loops ran more than 20 ms behind real time; this machine is too slow"
				+ " for the drive baselines (CTRE's sim runs on real time), so their numbers aren't comparable");
	}

	// ---- scenarios ----

	@Test
	void fullForward() {
		Measurement m = drive(0, -1, 0, 0); // stick pushed away from the driver reads -1
		assertNear(kFullForwardMeters, m.x(), 0.05 * kFullForwardMeters, "distance forward", m);
		assertNear(0, m.y(), 0.05, "sideways drift", m);
		assertNear(0, m.headingChangeDeg(), 1.0, "heading change", m);
		assertWheelsPointed(m, 0, "all wheels straight");
	}

	@Test
	void halfForwardIsAboutAThirdOfFullBecauseOfTheStickCurve() {
		// JoystickDrive maps the stick through 2*asin(x)/pi: half stick = one third speed
		Measurement m = drive(0, -0.5, 0, 0);
		assertNear(kHalfForwardMeters, m.x(), 0.07 * kHalfForwardMeters, "distance forward", m);
		assertNear(0, m.y(), 0.05, "sideways drift", m);
		assertWheelsPointed(m, 0, "all wheels straight");
	}

	@Test
	void fullStrafeLeft() {
		Measurement m = drive(-1, 0, 0, 0); // stick left reads -1
		assertNear(kFullStrafeMeters, m.y(), 0.10 * kFullStrafeMeters, "distance sideways", m);
		// The wheels drive while still turning toward 90 degrees, so some forward/back drift is normal
		assertNear(0, m.x(), 0.75, "forward drift while the wheels turn", m);
		assertNear(0, m.headingChangeDeg(), 1.5, "heading change", m);
		assertWheelsPointed(m, 90, "all wheels sideways");
	}

	@Test
	void leftTriggerRotatesCounterClockwise() {
		Measurement m = drive(0, 0, 1, 0);
		assertNear(kFullRotationLeftDegrees, m.headingChangeDeg(), 0.03 * kFullRotationLeftDegrees, "rotation", m);
		assertNear(0, m.x(), 0.05, "drift while rotating", m);
		assertNear(0, m.y(), 0.05, "drift while rotating", m);
		assertWheelsInRotationPattern(m);
	}

	@Test
	void rightTriggerRotatesClockwise() {
		Measurement m = drive(0, 0, 0, 1);
		assertNear(kFullRotationRightDegrees, m.headingChangeDeg(), -0.03 * kFullRotationRightDegrees, "rotation", m);
		assertNear(0, m.x(), 0.05, "drift while rotating", m);
		assertNear(0, m.y(), 0.05, "drift while rotating", m);
		assertWheelsInRotationPattern(m);
	}

	@Test
	void releasedSticksDoNotMove() {
		Measurement m = drive(0, 0, 0, 0);
		assertNear(0, m.x(), 0.001, "x", m);
		assertNear(0, m.y(), 0.001, "y", m);
		assertNear(0, m.headingChangeDeg(), 0.1, "heading change", m);
	}

	@Test
	void forwardStaysFieldForwardAfterTheRobotTurns() {
		// Driving is field-relative: after turning, "forward" still moves the same way on the field
		startFromRest();
		Rotation2d start = Drive.getHeading();
		setSticks(0, 0, 1, 0);
		SimRobot.runLoops(22, true); // steer into the rotation pattern, then turn part way
		setSticks(0, 0, 0, 0);
		SimRobot.runLoops(20, true);
		double turned = Drive.getHeading().minus(start).getDegrees();
		Pose2d before = Drive.getPose();

		setSticks(0, -1, 0, 0);
		SimRobot.runLoops(kLoopsPerSecond, true);
		Pose2d after = Drive.getPose();
		double dx = after.getX() - before.getX();
		double dy = after.getY() - before.getY();
		assertLoopsKeptToRealTime();
		String detail = String.format("turned %.1f deg, then moved dx=%.3f dy=%.3f", turned, dx, dy);

		// The robot turns about 110 degrees first, so robot-relative driving would go off at a large
		// angle; field-relative driving keeps going along the field's x axis
		assertTrue(Math.abs(turned) > 30, "the robot turned first: " + detail);
		assertNear(kFieldRelativeVeerDegrees, Math.toDegrees(Math.atan2(dy, dx)), 3, "direction of travel on the field",
				detail);
		assertTrue(Math.hypot(dx, dy) > 0.7 * kFullForwardMeters, "moved most of a full-speed second: " + detail);
	}

	// ---- helpers ----

	private static void setSticks(double leftX, double leftY, double leftTrigger, double rightTrigger) {
		DriverStationSim.setJoystickAxis(kDriver, Gamepad.Axis.LEFT_X.value, leftX);
		DriverStationSim.setJoystickAxis(kDriver, Gamepad.Axis.LEFT_Y.value, leftY);
		DriverStationSim.setJoystickAxis(kDriver, Gamepad.Axis.LEFT_TRIGGER.value, leftTrigger);
		DriverStationSim.setJoystickAxis(kDriver, Gamepad.Axis.RIGHT_TRIGGER.value, rightTrigger);
		DriverStationSim.notifyNewData();
	}

	/**
	 * Puts the robot in the same state before every scenario: gyro heading reset (as teleopInit
	 * does), wheels straightened with a short nudge forward, robot stopped, odometry at the origin
	 * facing the gyro heading.
	 */
	private static void startFromRest() {
		Drive.resetHeading();
		setSticks(0, -0.3, 0, 0);
		SimRobot.runLoops(15, true);
		setSticks(0, 0, 0, 0);
		SimRobot.runLoops(40, true);
		Drive.resetOdometry(new Pose2d(0, 0, Drive.getHeading()));
	}

	/** Holds the sticks for one second from rest. Heading change is summed per loop, so it can exceed 180. */
	private static Measurement drive(double leftX, double leftY, double leftTrigger, double rightTrigger) {
		startFromRest();
		setSticks(leftX, leftY, leftTrigger, rightTrigger);
		double headingChange = 0;
		Rotation2d previous = Drive.getHeading();
		for (int i = 0; i < kLoopsPerSecond; i++) {
			SimRobot.runLoops(1, true);
			Rotation2d now = Drive.getHeading();
			headingChange += now.minus(previous).getDegrees();
			previous = now;
		}
		assertLoopsKeptToRealTime();
		Pose2d pose = Drive.getPose();
		double[] angles = Arrays.stream(Drive.getModulePositions()).mapToDouble(p -> p.angle.getDegrees()).toArray();
		return new Measurement(pose.getX(), pose.getY(), headingChange, angles);
	}

	private static void assertNear(double expected, double actual, double tolerance, String what, Object detail) {
		assertEquals(expected, actual, tolerance, what + " (" + detail + ")");
	}

	/**
	 * How far an angle is from a wheel direction. 0 and 180 degrees are the same direction (the wheel
	 * just drives the other way), which swerve "optimize" uses freely.
	 */
	private static double offsetFromDirection(double angleDeg, double directionDeg) {
		double d = ((angleDeg - directionDeg) % 180 + 180) % 180;
		return Math.min(d, 180 - d);
	}

	private static void assertWheelsPointed(Measurement m, double directionDeg, String what) {
		for (double angle : m.moduleAnglesDeg()) {
			assertTrue(offsetFromDirection(angle, directionDeg) < 3, what + " (" + m + ")");
		}
	}

	/** Rotating in place points each wheel along the circle around the robot center: 45 or 135 degrees. */
	private static void assertWheelsInRotationPattern(Measurement m) {
		for (double angle : m.moduleAnglesDeg()) {
			assertTrue(offsetFromDirection(angle, 45) < 3 || offsetFromDirection(angle, 135) < 3,
					"wheels in the rotate-in-place X pattern (" + m + ")");
		}
		double[] a = m.moduleAnglesDeg();
		assertTrue(offsetFromDirection(a[0], a[1]) > 80, "not all wheels parallel (" + m + ")");
	}
}
