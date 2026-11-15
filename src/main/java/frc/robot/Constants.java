package frc.robot;

import java.util.Set;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;

/**
 * Robot-wide constants.
 *
 * <p>Each value is tagged with where it comes from, so it's clear which ones need checking or
 * re-measuring on the robot:
 * <ul>
 * <li><b>[measured]</b> physically measured on the robot (re-measure if the hardware changes or
 * wears)
 * <li><b>[tuned]</b> adjusted by testing on the robot until it behaved well
 * <li><b>[calibrated]</b> read off the robot's sensors at a known position (hard stop, straight
 * ahead, stowed)
 * <li><b>[spec]</b> from a datasheet, CAD, or the game manual; only changes if the hardware does
 * <li><b>[wiring]</b> must match how the robot is wired (CAN IDs, buses, motor and sensor
 * directions)
 * <li><b>[design]</b> a team choice (safety caps, current limits, driver feel)
 * <li><b>[derived]</b> computed from other constants; don't edit directly
 * <li><b>[unused]</b> not referenced anywhere; kept for reference or candidates for deletion
 * </ul>
 *
 * <p>Units are given after the tag. "duty" means motor output as a fraction, -1 to 1.
 *
 * <p>Tuned values that live outside this file: the intake move gains in IntakeCommands
 * (outInMinPower etc.), the drive-to-pose tolerances in DriveCommands, the shot table in Aim, and
 * the default airtime in AimCommands. CANcoder zero offsets are stored on each CANcoder (set in
 * Phoenix Tuner X), not in code.
 */
public class Constants {
	public static final boolean kCompBot = true; // [design] competition robot vs practice robot (steer inversion)
	public static final boolean kLogging = true; // [design] publish extra telemetry

	public static final class CANConstants {
		// TODO: confirm bus assignments against the final SystemCore wiring
		/** [wiring] CAN port for the swerve drive (TalonFX / CANcoder). */
		public static final CANPort kDrivePort = CANPort.CAN_S0;
		/** [wiring] CAN port for mechanisms (SPARKs, PDH, shooter). */
		public static final CANPort kMechanismPort = CANPort.CAN_S0;
		/** [derived] Phoenix 6 CANBus objects for the ports above. */
		public static final CANBus kDriveBus = new CANBus(kDrivePort);
		public static final CANBus kMechanismBus = new CANBus(kMechanismPort);
	}

	public static final class Subsystems {
		public static final class VisionConstants {

			// [spec] AprilTag IDs used for pose estimation: 3 red, 3 blue
			public static final Set<Integer> kTrackableTags = Set.of(10, 2, 11, 9, 8, 5, 18, 21, 24, 25, 26, 27);
			public static final Pose2d kBlueHub = new Pose2d(4.63, 4.04, Rotation2d.ZERO); // [spec] meters, field drawing
			public static final Pose2d kRedHub = new Pose2d(11.92, 4.04, Rotation2d.ZERO); // [spec] meters, field drawing

			public static final double kRedNeutralZoneStart = 11.1; // [spec] meters (field x)
			public static final double kBlueNeutralZoneStart = 5.2; // [spec] meters (field x)
		}

		public static final class TurretConstants {
			// Angles are in turret-encoder degrees (see kAngleToTicksFactor for real turret degrees)
			public static final boolean kMotorInvert = false; // [wiring] positive power must increase the angle
			public static final boolean kEncoderInvert = true; // [wiring]
			public static final int kTurretPort = 50; // [wiring] CAN ID
			public static final double kMaxDutyCycle = .09; // [design] duty; extra limit for test safety
			public static final double kMinPower = 0.04; // [tuned] duty; clamped-P minimum to overcome friction
			public static final double kMaxPower = kMaxDutyCycle; // [derived] duty; clamped-P maximum
			public static final double kMaxErr = 40; // [tuned] degrees; clamped-P slows down within this (was 25)
			public static final double kTolerance = 1.5; // [tuned] degrees; "settled" band
			public static final double kP = 0.02; // [tuned] duty per degree of error (SPARK position control)
			public static final double kI = 0.0000; // [tuned] SPARK integral gain, same scaling as kP
			// [tuned] volts (SPARK feedforward kS). NOTE: set to kMinPower, which is a duty fraction:
			// 0.04 here means 0.04 V, not 0.04 duty (0.48 V). Check which was intended.
			public static final double kS = kMinPower;
			public static final double kLargeDeadzone = 0.5; // [unused] stick fraction; for X/Y joystick control
			public static final double kSmallDeadzone = 0.05; // [unused] stick fraction
			public static final int kSmartCurrent = 25; // [design] amps (SPARK smart current limit)
			public static final int kCurrent = 30; // [design] amps (SPARK secondary current limit)
			public static final double kMinAngle = 22; // [calibrated] degrees; soft limit at hard stop B
			public static final double kMaxAngle = 180; // [calibrated] degrees; soft limit at hard stop A (was 183)

			// [spec] turret-encoder degrees per real turret degree
			public static final double kAngleToTicksFactor = 6.0 / 7.0;

			// [spec] scale for the turret encoder: reported degrees per encoder rotation = 360 x this.
			// Was 7.0 / 6
			public static final double kPositionConversionFactor = 1;
			public static final double kStraightAheadAngle = 90; // [calibrated] degrees with the turret facing forward (was 105)
		}

		public static final class HoodConstants {
			// Angles are in hood-encoder degrees
			public static final boolean kMotorInvert = true; // [wiring]
			public static final boolean kEncoderInvert = false; // [wiring]
			public static final int kHoodPort = 52; // [wiring] CAN ID
			public static final double kMaxDutyCycle = 0.5; // [design] duty; safety cap
			public static final double kMinPower = 0.025; // [tuned] duty; clamped-P minimum to overcome friction
			public static final double kMaxPower = 0.25; // [tuned] duty; clamped-P maximum
			public static final double kMaxErr = 25; // [tuned] degrees; clamped-P slows down within this
			public static final double kTolerance = 1; // [tuned] degrees; "settled" band
			public static final double kP = 0.0525; // [tuned] duty per degree of error (SPARK position control)
			public static final double kI = 0.0000; // [tuned] SPARK integral gain, same scaling as kP
			public static final double kS = 0.00; // [tuned] volts (SPARK feedforward kS)
			public static final double kDeadzone = 0.05; // [unused] stick fraction
			public static final int kSmartCurrent = 20; // [design] amps (SPARK smart current limit)
			public static final int kCurrent = 25; // [design] amps (SPARK secondary current limit)
			public static final double kMinAngle = 99; // [calibrated] degrees; soft limit, hood fully down
			public static final double kMaxAngle = 137; // [calibrated] degrees; soft limit, hood fully up
			public static final double kDownPosition = kMinAngle + 1; // [derived] degrees; "hood down" target
		}

		public static final class ShooterConstants {
			public static final int kFlywheelPort = 51; // [wiring] CAN ID
			public static final double kCurrentLimit = 60; // [design] amps (TalonFX supply current limit)
			// [tuned] RPM per volt (feedforward = rpm / kV; 480 x 12 V = 5760 RPM, about a Kraken's free
			// speed). NOTE: the inverse of DriveConstants.kV, which is in volts per rotation per second.
			public static final double kV = 480;
			public static final int kDefaultDistance = 10; // [tuned] feet; distance assumed before aiming (see Aim)
			public static final int kDefaultRPM = 2000; // [tuned] RPM; d-pad shooter starting speed
			public static final double kRampRate = 10; // [unused]
			public static final double kRPMRamp = 250; // [design] RPM added or removed per d-pad press
			public static final double kTolerance = 200; // [tuned] RPM; "settled" band
		}

		public static final class IntakeConstants {
			public static final int kIntakeWheelsPort = 54; // [wiring] CAN ID
			public static final int kIntakeArmPort = 53; // [wiring] CAN ID
			public static final int kIntakeExtraArmPort = 56; // [wiring] CAN ID
			public static final double kArmConversionFactor = 0.01; // [unused] since the 2027 migration (see IntakeArm)

			public static final int kWheelSmartCurrentLimit = 20; // [design] amps
			public static final int kWheelSecondaryCurrentLimit = 30; // [design] amps
			public static final boolean kWheelInvert = true; // [wiring]

			public static final int kArmSmartCurrentLimit = 10; // [design] amps
			public static final int kArmSecondaryCurrentLimit = 20; // [design] amps
			public static final boolean kArmInvert = true; // [wiring]

			public static final int kExtraArmSmartCurrentLimit = 7; // [design] amps
			public static final int kExtraArmSecondaryCurrentLimit = 15; // [design] amps
			public static final boolean kExtraArmInvert = false; // [wiring]

			public static final double kArmMaxPower = 1.0; // [design] duty; arm power cap

			public static final double kOutPosition = 230; // [calibrated] arm motor rotations, deployed (was 145)
			public static final double kInPosition = 10; // [calibrated] arm motor rotations, stowed
			// [tuned] duty per motor rotation of error (SPARK position control). NOTE: also used by
			// Climber (scaled by its gear ratio) instead of ClimberConstants.kP.
			public static final double kP = 0.1;
			public static final double kWheelPower = 1; // [design] duty; intake wheel power
		}

		public static final class AgitatorConstants {
			public static final double kTeleopPower = .75; // [design] duty
			public static final int kAgitatorPort = 57; // [wiring] CAN ID
			public static final int kAgitatorSmartCurrentLimit = 40; // [design] amps
			public static final int kAgitatorSecondaryCurrentLimit = 50; // [design] amps
			public static final boolean kAgitatorInvert = true; // [wiring]
		}

		public static final class ClimberConstants {
			public static final int kClimberPort = 55; // [wiring] CAN ID
			public static final int kSmartCurrentLimit = 40; // [design] amps
			public static final int kSecondaryCurrentLimit = 60; // [design] amps
			public static final boolean kInvert = false; // [wiring]
			// [spec] gearbox ratio. NOTE: Climber reports position as motor rotations x 81 (carried over
			// from 2026's positionConversionFactor), not output rotations (motor rotations / 81). The
			// positions below are in those units.
			public static final double kGearRatio = 81.0 / 1.0;
			public static final double kClimbPosition = 31053; // [calibrated] motor rotations x 81 -- TODO: Find actual positions
			public static final double kRetractPosition = 0; // [calibrated] motor rotations x 81
			public static final double kP = 0.2; // [unused] Climber uses IntakeConstants.kP instead
		}
	}

	public static final class ControllerConstants {
		public static final int kDriverControllerPort = 0; // [design] driver station USB slot
		public static final int kOperatorControllerPort = 1; // [design] driver station USB slot
	}

	public static final class DriveConstants {
		public static final double kDeadzone = 0.05; // [design] stick fraction; JoystickDrive deadband
		// [wiring] CAN IDs (updated)
		public static final int kFrontRightDrivePort = 10;
		public static final int kFrontRightSteerPort = 11;
		public static final int kFrontLeftDrivePort = 40;
		public static final int kFrontLeftSteerPort = 41;
		public static final int kBackRightDrivePort = 20;
		public static final int kBackRightSteerPort = 21;
		public static final int kBackLeftDrivePort = 30;
		public static final int kBackLeftSteerPort = 31;
		public static final int kFrontRightCANCoderPort = 12;
		public static final int kFrontLeftCANCoderPort = 42;
		public static final int kBackRightCANCoderPort = 22;
		public static final int kBackLeftCANCoderPort = 32;

		// TODO: Make sure these are tuned (can do with SysId)
		// NOTE: kP/kI/kD are the wheel steering gains (SwerveModule), but Drive also builds its unused
		// robot-heading controller (m_orientationController) from them.
		public static final double kP = 0.01; // [tuned] duty per degree of wheel angle error
		public static final double kI = 0; // [tuned]
		public static final double kD = 0; // [tuned]
		// [tuned] volts per motor rotation per second (SysId drive characterization). Also used by the
		// drive and steer simulation models and kMaxSpeedMetersPerSecond. If SysId reports per meter
		// per second instead (about 2.5), convert before pasting it here.
		public static final double kV = 0.12;
		public static final double kA = 0.009; // [tuned] volts per motor rotation per second squared (SysId)

		public static final double kDriveGearRatio = 6.75; // [spec] motor rotations per wheel rotation
		public static final double kSteerGearRatio = 150.0 / 7; // [spec] motor rotations per module rotation -- TODO: Change value for 5i's
		public static final double kWheelDiameter = Units.inchesToMeters(4); // [measured] meters (nominal 4 in; wheels wear)
		public static final double kWheelCircumference = Math.PI * kWheelDiameter; // [derived] meters

		public static final double kMetersPerMotorRotation = kWheelCircumference / kDriveGearRatio; // [derived] meters
		// [derived] meters per second. Top speed at full power: 12 V / kV (volts per motor rotation per
		// second) = motor rotations per second, times meters per rotation. About 4.7 m/s. Used to show
		// the target module states (which are power fractions, -1 to 1) in meters per second next to
		// the measured ones.
		public static final double kMaxSpeedMetersPerSecond = 12.0 / kV * kMetersPerMotorRotation;

		// https://docs.wpilib.org/en/latest/docs/software/basic-programming/coordinate-system.html
		public static final double kModuleDistFromCenter = Units.inchesToMeters(14.5); // [spec] meters; half the module spacing (CAD)
		// [derived] module positions relative to the robot center, meters
		public static final Translation2d kFrontLeftLocation = new Translation2d(kModuleDistFromCenter,
				kModuleDistFromCenter);
		public static final Translation2d kFrontRightLocation = new Translation2d(kModuleDistFromCenter,
				-kModuleDistFromCenter);
		public static final Translation2d kBackLeftLocation = new Translation2d(-kModuleDistFromCenter,
				kModuleDistFromCenter);
		public static final Translation2d kBackRightLocation = new Translation2d(-kModuleDistFromCenter,
				-kModuleDistFromCenter);

		public static final TalonFXConfiguration kDriveConfig = new TalonFXConfiguration();
		static {
			kDriveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // [design]
			kDriveConfig.CurrentLimits.SupplyCurrentLimit = 45; // [design] amps; for avoiding brownout
			kDriveConfig.CurrentLimits.SupplyCurrentLowerLimit = 45; // [design] amps
			kDriveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
			kDriveConfig.CurrentLimits.StatorCurrentLimit = 80; // [design] amps; output current (proportional to acceleration)
			kDriveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
			kDriveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // [wiring] overridden per module
		}

		public static final TalonFXConfiguration kSteerConfig = new TalonFXConfiguration();
		static {
			kSteerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // [design]
			kSteerConfig.CurrentLimits.StatorCurrentLimit = 60; // [design] amps
			kSteerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
			kSteerConfig.CurrentLimits.SupplyCurrentLimit = 75; // [design] amps
			kSteerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
			kSteerConfig.MotorOutput.Inverted = (kCompBot) ? InvertedValue.CounterClockwise_Positive // [wiring]
					: InvertedValue.Clockwise_Positive;
		}
	}
}
