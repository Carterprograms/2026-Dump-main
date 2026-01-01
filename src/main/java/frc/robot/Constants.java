package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

public class Constants {

    public static final class CANDevices {
        // Set these CAN ID values to the those of your robot, or change your CAN ID's to match this convention.

        //Pigeon
        public static final int imuId = 9;

        //FL Mod
        public static final int frontLeftCanCoderId = 11;
        public static final int frontLeftSteerMtrId = 3;
        public static final int frontLeftDriveMtrId = 4;

        //FR Mod
        public static final int frontRightCanCoderId = 10;
        public static final int frontRightSteerMtrId = 5;
        public static final int frontRightDriveMtrId = 6;

        //BL Mod
        public static final int backLeftCanCoderId = 12;
        public static final int backLeftSteerMtrId = 1;
        public static final int backLeftDriveMtrId = 2;

        //BR Mod
        public static final int backRightCanCoderId = 13;
        public static final int backRightSteerMtrId = 7;
        public static final int backRightDriveMtrId = 8;
       
    }

    public static final class ControllerConstants {

        //Controllers
        public static final int driverGamepadPort = 0;
        public static final int operatorGamepadPort = 1;
        
        public static final double joystickDeadband = 0.15;

        public static final double triggerPressedThreshhold = 0.25;

        public static final int driverRightJoystick = 1;

    }
    
    public static final class DriveConstants {
        /**
         * The track width from wheel center to wheel center.
         */
        // Make sure to measure from the center of each wheel
        public static final double trackWidth = Units.inchesToMeters(23.75);

        /**
         * The track length from wheel center to wheel center.
         */
        // mature sure to measure from the center of each wheel
        public static final double wheelBase = Units.inchesToMeters(23.75);

        /**
         * The SwerveDriveKinematics used for control and odometry.
         */
        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0),  // front left
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // back left
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // back right
            );

        /**
         * The gear reduction from the drive motor to the wheel.
         * 
         * The drive gear ratios for the different levels can be found from the chart at
         * swervedrivespecialties.com/products/mk41-swerve-module.
         */
        // This is the gear ratio for L3 modules.
        public static final double driveMtrGearReduction = (16.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);

        /**
         * The gear reduction from the steer motor to the wheel.
         */
        public static final double steerMtrGearReduction = (14.0 / 50.0) * (10.0 / 60.0);

        public static final double wheelRadiusMeters = Units.inchesToMeters(2);
        public static final double wheelCircumferenceMeters = 2.0 * wheelRadiusMeters * Math.PI;

        public static final double driveMetersPerEncRev = wheelCircumferenceMeters * driveMtrGearReduction;
        public static final double driveMetersPerSecPerMtrRPM = driveMetersPerEncRev / 60.0;

        public static final double steerRadiansPerEncRev = 2 * Math.PI * DriveConstants.steerMtrGearReduction;

        public static final double freeMetersPerSecond = 6784 * driveMetersPerSecPerMtrRPM;

        /**
         * The maximum possible speed a module can be driven. Used for desaturation.
         */
        public static final double maxModuleSpeedMetersPerSec = 10;

        public static final double maxDriveSpeedMetersPerSec = 10;

        /**
         * The rate the robot will spin with full Rot command.
         */
        public static final double maxTurnSpeedRadPerSec = 3.0 * Math.PI;

        // Set line up the swerve modules and set these values.

        // The bolt heads should be pointing to the right. These values are subtracted from the CANCoder reading,
        // so they should be the raw CANCoder value when set straight. These values should be between 0 and 360
        // degrees.
        public static final Rotation2d frontLeftModOffset = Rotation2d.fromDegrees(45.87876); // 122.43, 318.164, 135.97, 124.09, -120.23
        public static final Rotation2d frontRightModOffset = Rotation2d.fromDegrees(175.69332); // 184.12, 234.756, 134.5, 18.72, 87.28
        public static final Rotation2d backLeftModOffset = Rotation2d.fromDegrees(47.191744); // 62, 252.15804, -164.5, -150.01
        public static final Rotation2d backRightModOffset = Rotation2d.fromDegrees(74.88288); // 82.7, 252.15804, 40.07, 14.5

        // You may want to change this value.
        public static final int driveCurrentLimitAmps = 70;
        public static final double brownoutVoltage = 6.25;
        
        // These values should be fine, but if the modules start to rattle you may want to play with the steer PID values.
        public static final double drivekP = 0.13;//0.005;
        public static final double drivekD = 0.0;

        public static final double steerkP = 0.37431;
        public static final double steerkD = 0.27186;

        public static final double ksVolts = 0.667;
        public static final double kvVoltSecsPerMeter = 2.44;
        public static final double kaVoltSecsPerMeterSq = 0.0;

        public static final SimpleMotorFeedforward driveFF =
            new SimpleMotorFeedforward(ksVolts, kvVoltSecsPerMeter, kaVoltSecsPerMeterSq);

    }

    public static final class AutoConstants {

        // These drive and rotation PID constants most likely need to be tuned for better accuracy.
        public static final double drivekP = 5.0;
        public static final double drivekD = 0;

        public static final double rotkP = 5.0;
        public static final double rotkD = 0;

        public static final double maxAccelMetersPerSec = 4.7;
        public static final double maxAccelMetersPerSecSq = 3.7;

        public static final double maxRotAccelMetersPerSecSq = 5.88;

        // Auto aim PID values should ideally be the same as the PathPlanner rotation ones. They are separate for safe measure.

        public static final double autoAimkP = 10.9;
        public static final double autoAimkD = 0.5;

        public static final double autoAimToleranceDeg = 0.2;

        public static final double autoAimTurnSpeedRadPerSec = 2.0 * Math.PI;
        public static final double autoAumTurnAccelRadPerSecSq = 3.0 * Math.PI;
    }

    public class LightsConstants {
        public static final Color blueAllianceColor = new Color(0, 0, 255);
        public static final Color redAllianceColor = new Color(255, 0, 0);
        public static final Color noAllianceColor = new Color(200, 255, 200);

        public static final double partyModeHueIncrement = 5;
        public static final double partyModeTranslationTimeSec = 0.05;

        public static final double brightnessPercentage = 0.5;
    }

    public class VisionConstants {
        public static final String limeLightName = "limelight";

        public static final double targetAreaPercentThreshold = 0.15;
    }

    public class FieldConstants {
        // These Constants are for the 2025 FRC Field and are bound to change
        public static final double fieldLengthMeters = Units.feetToMeters(54.25);
        public static final double fieldWidthMeters = Units.feetToMeters(27.25);
    }
}