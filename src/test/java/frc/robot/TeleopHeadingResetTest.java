package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.util.Arrays;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Test;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.DriverStationSim;

import frc.robot.commands.DriveCommands;
import frc.robot.commands.compat.CommandScheduler;
import frc.robot.subsystems.Drive;

/**
 * How the gyro heading is set when modes start. The robot is placed facing the driver before a
 * match, so autonomousInit() sets the heading to "facing the driver" (180 degrees). teleopInit()
 * used to do the same, wherever auto had left the robot: if an auto ended with the robot turned,
 * field-relative driving was off by that angle for all of teleop. Now teleop only sets the heading
 * if nothing has since robot code started (teleop-only practice).
 *
 * <p>These tests play match-like sequences in simulation. Between the end of auto and the start of
 * teleop nothing touches the robot, so its heading should not change.
 */
class TeleopHeadingResetTest {
	private static final double kFacingDriverDeg = 180; // what Drive.resetHeading() sets

	@AfterAll
	static void reset() {
		SimRobot.reset();
	}

	/** Heading change from a to b in degrees, -180 to 180. */
	private static double change(Rotation2d from, Rotation2d to) {
		return to.minus(from).getDegrees();
	}

	private static double[] wheelDistances() {
		return Arrays.stream(Drive.getModulePositions()).mapToDouble(p -> p.distance).toArray();
	}

	/** Turns the robot for {@code loops} loops, then lets it stop, as an auto routine might. */
	private static void turn(int loops) {
		for (int i = 0; i < loops; i++) {
			Drive.drive(0, 0, 1, true);
			SimRobot.runLoops(1, true);
		}
		for (int i = 0; i < 40; i++) {
			Drive.drive(0, 0, 0, true);
			SimRobot.runLoops(1, true);
		}
	}

	/**
	 * Turns the robot in teleop the way the driver would: holding the left trigger, which
	 * JoystickDrive (the drive's default command) reads. Direct Drive.drive calls would fight it.
	 */
	private static void turnWithLeftTrigger(int loops) {
		int driver = Constants.ControllerConstants.kDriverControllerPort;
		DriverStationSim.setJoystickIsGamepad(driver, true);
		DriverStationSim.setJoystickAxesMaximumIndex(driver, 5);
		DriverStationSim.setJoystickAxesAvailable(driver, 0b111111); // a bitmask of axes, not a count
		DriverStationSim.setJoystickAxis(driver, Gamepad.Axis.LEFT_TRIGGER.value, 1);
		DriverStationSim.notifyNewData();
		SimRobot.runLoops(loops, true);
		DriverStationSim.setJoystickAxis(driver, Gamepad.Axis.LEFT_TRIGGER.value, 0);
		DriverStationSim.notifyNewData();
		SimRobot.runLoops(40, true);
	}

	/** Makes the shared robot behave as if robot code had just started (no mode has set the heading). */
	private static void simulateFreshBoot() throws ReflectiveOperationException {
		Field headingSet = Robot.class.getDeclaredField("m_headingSet");
		headingSet.setAccessible(true);
		headingSet.setBoolean(SimRobot.get(), false);
	}

	/** The disabled pause, then teleop starts. Checks that nothing moved the robot meanwhile. */
	private static void disabledPauseThenStartTeleop() {
		Robot robot = SimRobot.get();
		SimRobot.setMode(RobotMode.TELEOPERATED, false);
		robot.disabledInit();
		SimRobot.runLoops(10, true);
		double[] before = wheelDistances();

		SimRobot.setMode(RobotMode.TELEOPERATED, true);
		robot.teleopInit();
		SimRobot.runLoops(5, true);

		double[] after = wheelDistances();
		for (int i = 0; i < before.length; i++) {
			assertEquals(before[i], after[i], 0.01, "the robot didn't move before teleop started");
		}
	}

	/** Auto: the robot starts facing the driver, then turns for {@code turnLoops} loops. */
	private static Rotation2d playAuto(int turnLoops) {
		Robot robot = SimRobot.get();
		SimRobot.setMode(RobotMode.AUTONOMOUS, true);
		robot.autonomousInit();
		CommandScheduler.getInstance().cancelAll(); // take over from the selected auto
		SimRobot.runLoops(10, true);
		assertEquals(0, change(Rotation2d.fromDegrees(kFacingDriverDeg), Drive.getHeading()), 1.0,
				"auto starts facing the driver");
		turn(turnLoops);
		return Drive.getHeading();
	}

	@Test
	void teleopKeepsTheHeadingAutoLeftTheRobotAt() {
		Rotation2d endOfAuto = playAuto(25); // about half a second of turning
		double turnedInAuto = change(Rotation2d.fromDegrees(kFacingDriverDeg), endOfAuto);
		assertTrue(Math.abs(turnedInAuto) > 45, "the robot turned during auto: " + turnedInAuto);

		disabledPauseThenStartTeleop();

		Rotation2d startOfTeleop = Drive.getHeading();
		String detail = String.format("turned %.1f deg in auto; heading %.1f at end of auto, %.1f at start of teleop",
				turnedInAuto, endOfAuto.getDegrees(), startOfTeleop.getDegrees());
		assertEquals(0, change(endOfAuto, startOfTeleop), 1.0, "teleop keeps the heading from auto: " + detail);
		assertEquals(0, change(startOfTeleop, Drive.getPose().getRotation()), 1.0,
				"the field pose doesn't jump either: " + detail);
	}

	@Test
	void teleopWithoutAutoSinceBootStillStartsFacingTheDriver() throws ReflectiveOperationException {
		// Practice: robot code starts, the robot is put down facing the driver, teleop is enabled
		// straight away. The heading at boot is not "facing the driver", so teleop must set it.
		simulateFreshBoot();
		SimRobot.setMode(RobotMode.TELEOPERATED, true);
		SimRobot.get().teleopInit();
		SimRobot.runLoops(5, true);
		assertEquals(0, change(Rotation2d.fromDegrees(kFacingDriverDeg), Drive.getHeading()), 1.0,
				"teleop-only practice starts facing the driver");
	}

	@Test
	void reEnablingTeleopDoesNotResetTheHeadingAgain() throws ReflectiveOperationException {
		simulateFreshBoot();
		SimRobot.setMode(RobotMode.TELEOPERATED, true);
		SimRobot.get().teleopInit(); // first teleop: sets "facing the driver"
		turnWithLeftTrigger(25); // the driver turns the robot (JoystickDrive is running)
		Rotation2d beforeDisable = Drive.getHeading();
		double turned = change(Rotation2d.fromDegrees(kFacingDriverDeg), beforeDisable);
		assertTrue(Math.abs(turned) > 45, "the driver turned the robot first: " + turned);

		disabledPauseThenStartTeleop(); // disable and re-enable teleop
		assertEquals(0, change(beforeDisable, Drive.getHeading()), 1.0,
				"re-enabling keeps the heading; use the start button to reset it by hand");
	}

	@Test
	void driverResetButtonStillSetsFacingTheDriver() {
		playAuto(25);
		disabledPauseThenStartTeleop();
		// The driver's start button runs DriveCommands.ResetHeading
		CommandScheduler.getInstance().schedule(new DriveCommands.ResetHeading());
		SimRobot.runLoops(5, true);
		assertEquals(0, change(Rotation2d.fromDegrees(kFacingDriverDeg), Drive.getHeading()), 1.0,
				"the manual reset still works");
	}

	@Test
	void autoEndingFacingTheDriverIsUnaffected() {
		Rotation2d endOfAuto = playAuto(0); // auto doesn't turn
		disabledPauseThenStartTeleop();
		assertEquals(0, change(endOfAuto, Drive.getHeading()), 1.0);
		assertEquals(0, change(Rotation2d.fromDegrees(kFacingDriverDeg), Drive.getHeading()), 1.0);
	}
}
