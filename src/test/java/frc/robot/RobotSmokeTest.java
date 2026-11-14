package frc.robot;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertFalse;

import java.lang.reflect.Method;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

import frc.robot.commands.compat.CommandScheduler;

/**
 * Builds the real Robot in simulation and runs teleop and the default auto. Catches commands,
 * groups, bindings, and default commands that Commands v3 rejects when they are created or run
 * (for example a default command requiring two subsystems, or parallel children sharing one),
 * which would otherwise only show up when the robot is enabled.
 */
class RobotSmokeTest {
	private static Robot s_robot;

	@BeforeAll
	static void createRobot() {
		HAL.initialize();
		DriverStationBackend.observeUserProgramStarting();
		s_robot = new Robot();
	}

	@AfterAll
	static void reset() {
		setMode(RobotMode.UNKNOWN, false);
		DriverStationBackend.clearUserProgramStarted();
		SimHooks.resumeTiming();
	}

	private static void setMode(RobotMode mode, boolean enabled) {
		DriverStationSim.setRobotMode(mode);
		DriverStationSim.setEnabled(enabled);
		DriverStationSim.notifyNewData();
		DriverStationBackend.refreshData();
	}

	private static void runLoops(int count) {
		SimHooks.pauseTiming();
		for (int i = 0; i < count; i++) {
			SimHooks.stepTiming(0.02);
			DriverStationBackend.refreshData();
			s_robot.robotPeriodic();
		}
	}

	@Test
	void teleopWithCompetitionAndTestBindingsRunsWithoutErrors() throws Exception {
		setMode(RobotMode.TELEOPERATED, true);
		assertDoesNotThrow(() -> s_robot.teleopInit(), "creating the competition bindings");

		// Also create the test-controls bindings (normally chosen by Robot.compControls)
		Method bindTestControls = Robot.class.getDeclaredMethod("bindTestControls");
		bindTestControls.setAccessible(true);
		assertDoesNotThrow(() -> bindTestControls.invoke(s_robot), "creating the test bindings");

		assertDoesNotThrow(() -> runLoops(100), "two seconds of teleop");
		assertFalse(CommandScheduler.getInstance().getScheduler().getRunningCommands().isEmpty(),
				"default commands (drive, turret) and the LED sequence are running");
	}

	@Test
	void defaultAutoRunsWithoutErrors() {
		setMode(RobotMode.AUTONOMOUS, true);
		assertDoesNotThrow(() -> s_robot.autonomousInit());
		assertDoesNotThrow(() -> runLoops(250), "five seconds of auto");
	}

	@Test
	void disablingCancelsEverythingWithoutErrors() {
		setMode(RobotMode.TELEOPERATED, true);
		s_robot.teleopInit();
		runLoops(10);
		setMode(RobotMode.TELEOPERATED, false);
		assertDoesNotThrow(() -> s_robot.disabledInit());
		assertDoesNotThrow(() -> runLoops(10));
	}
}
