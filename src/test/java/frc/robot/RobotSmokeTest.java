package frc.robot;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertFalse;

import java.lang.reflect.Method;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.RobotMode;

import frc.robot.commands.compat.CommandScheduler;

/**
 * Builds the real Robot in simulation and runs teleop and the default auto. Catches commands,
 * groups, bindings, and default commands that Commands v3 rejects when they are created or run
 * (for example a default command requiring two subsystems, or parallel children sharing one),
 * which would otherwise only show up when the robot is enabled.
 */
class RobotSmokeTest {
	@AfterAll
	static void reset() {
		SimRobot.reset();
	}

	@Test
	void teleopWithCompetitionAndTestBindingsRunsWithoutErrors() throws Exception {
		Robot robot = SimRobot.get();
		SimRobot.setMode(RobotMode.TELEOPERATED, true);
		assertDoesNotThrow(() -> robot.teleopInit(), "creating the competition bindings");

		// Also create the test-controls bindings (normally chosen by Robot.compControls)
		Method bindTestControls = Robot.class.getDeclaredMethod("bindTestControls");
		bindTestControls.setAccessible(true);
		assertDoesNotThrow(() -> bindTestControls.invoke(robot), "creating the test bindings");

		assertDoesNotThrow(() -> SimRobot.runLoops(100), "two seconds of teleop");
		assertFalse(CommandScheduler.getInstance().getScheduler().getRunningCommands().isEmpty(),
				"default commands (drive, turret) and the LED sequence are running");
	}

	@Test
	void defaultAutoRunsWithoutErrors() {
		Robot robot = SimRobot.get();
		SimRobot.setMode(RobotMode.AUTONOMOUS, true);
		assertDoesNotThrow(() -> robot.autonomousInit());
		assertDoesNotThrow(() -> SimRobot.runLoops(250), "five seconds of auto");
	}

	@Test
	void disablingCancelsEverythingWithoutErrors() {
		Robot robot = SimRobot.get();
		SimRobot.setMode(RobotMode.TELEOPERATED, true);
		robot.teleopInit();
		SimRobot.runLoops(10);
		SimRobot.setMode(RobotMode.TELEOPERATED, false);
		assertDoesNotThrow(() -> robot.disabledInit());
		assertDoesNotThrow(() -> SimRobot.runLoops(10));
	}
}
