package frc.robot.subsystems;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;

import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.SerialPort;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import frc.robot.commands.compat.Command;

public class USBSerialSubsystem implements Mechanism {
	private SerialPort usbSerial;
	private static USBSerialSubsystem s_SerialSubsystem;
	public AtomicBoolean m_isHubShift = new AtomicBoolean(false);

	public static USBSerialSubsystem get() {
		return s_SerialSubsystem;
	}

	public USBSerialSubsystem() {
		if (s_SerialSubsystem == null) {
			s_SerialSubsystem = this;
		} else {
			throw new Error("USB Serial already instantiated");
		}

		try {
			// Initialize USB port (usually USB)
			usbSerial = new SerialPort(9600, SerialPort.Port.USB);
		} catch (Exception e) {
			System.out.println("USB Serial Port Init Failed: " + e.getMessage());
		}
		// Publish starting values so the Lights topics exist (e.g. in AdvantageScope) before anything is sent
		Telemetry.log("Lights/Sends", 0L);
		Telemetry.log("Lights/Pattern", "None sent yet");
	}

	// Bytes sent to the LED controller since robot code started (see sendData)
	private long m_sendCount = 0;

	public void sendData(byte[] data) {
		System.out.println("TRY SEND DATA: " + Arrays.toString(data));
		if (usbSerial != null) {
			usbSerial.write(data, data.length);
		}
		// Show what the lights were told and how often, so it's visible in AdvantageScope (also in
		// simulation, where there is no LED controller). A steep "Lights/Sends" slope means the same
		// pattern is being re-sent every loop.
		m_sendCount += data.length;
		Telemetry.log("Lights/Sends", m_sendCount);
		if (data.length > 0) {
			int pattern = data[0];
			Telemetry.log("Lights/Pattern", pattern >= 0 && pattern < LightPattern.values().length
					? LightPattern.values()[pattern].name()
					: "Unknown (" + pattern + ")");
		}
	}

	public static enum LightPattern {
		DEFAULT_GREEN,
		VISION_AIM_NOT_LOCKED,
		VISION_AIM_LOCKED,
		TURRET_MANUAL,

		HUB_SHIFT,
		ENDGAME_SHIFT,

		STATE_COUNT
	}

	public static class SetLightAutoAimStatus extends Command {

		public SetLightAutoAimStatus() {
			addRequirements(USBSerialSubsystem.get(), Vision.getVision());
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void initialize() {
			if (!USBSerialSubsystem.s_SerialSubsystem.m_isHubShift.get()) {
				USBSerialSubsystem.s_SerialSubsystem.sendData(
						new byte[] { (byte) (Vision.getVision().getFieldPoseEstimator().hasTargets()
								? LightPattern.VISION_AIM_LOCKED
								: LightPattern.VISION_AIM_NOT_LOCKED).ordinal() });
			}
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public void setLightPattern(LightPattern pattern) {
		if (!USBSerialSubsystem.s_SerialSubsystem.m_isHubShift.get())
			USBSerialSubsystem.s_SerialSubsystem.sendData(new byte[] { (byte) pattern.ordinal() });
	}

	public static class SetLightCommand extends Command {
		public final LightPattern m_pattern;

		public SetLightCommand(LightPattern pattern) {
			m_pattern = pattern;
			addRequirements(USBSerialSubsystem.s_SerialSubsystem);
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void initialize() {
			if (!USBSerialSubsystem.s_SerialSubsystem.m_isHubShift.get())
				USBSerialSubsystem.s_SerialSubsystem.sendData(new byte[] { (byte) m_pattern.ordinal() });
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return true;
		}
	}

	public static class SetHubShiftCommand extends Command {
		Timer m_timer = new Timer();

		public SetHubShiftCommand() {
			addRequirements(USBSerialSubsystem.s_SerialSubsystem);
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void initialize() {
			m_timer.reset();
			m_timer.start();
			USBSerialSubsystem.s_SerialSubsystem.sendData(new byte[] { (byte) LightPattern.HUB_SHIFT.ordinal() });
			USBSerialSubsystem.s_SerialSubsystem.m_isHubShift.set(true);
		}

		@Override
		public void end(boolean interrupted) {
			USBSerialSubsystem.s_SerialSubsystem.m_isHubShift.set(false);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return m_timer.hasElapsed(3);
		}
	}

}
