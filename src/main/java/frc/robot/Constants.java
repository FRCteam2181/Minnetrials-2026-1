// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_LEADER_ID = 3;
    public static final int LEFT_FOLLOWER_ID = 1;
    public static final int RIGHT_LEADER_ID = 7;
    public static final int RIGHT_FOLLOWER_ID = 5;

    public static final int PIGEON_CAN_ID = 20; // set later

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

    public static final double DRIVE_GEAR_RATIO = 0.1182;

    public static final double kTrackWidth = 0.657;
    public static final double kWheelRadius = 0.0762; // 3 inches

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);

    // public static final RobotConfig kRobotConfig = RobotConfig.fromGUISettings();

    // public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(RIGHT_FOLLOWER_ID, PIGEON_CAN_ID, LEFT_LEADER_ID, null, LEFT_FOLLOWER_ID, DRIVE_MOTOR_CURRENT_LIMIT);

    // public static final RobotConfig kRobotConfigManual = new RobotConfig();
    // public static final RobotConfig kRobotConfig = RobotConfig.fromGUISettings(), ;

    // public static final PathFollowingController kAutoController = new PPLTVController(TimedRobot.kDefaultPeriod,
    //         kMaxSpeedMetersPerSecond);
  }

  public static final class RollerConstants {
    public static final int BOTTOM_ROLLER_MOTOR_ID = 5;
    public static final int BOTTOM_ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double BOTTOM_ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double SPITTER_ROLLER_EJECT_VALUE = 0.6;
    public static final double SPITTER_ROLLER_TOP_EJECT_VALUE = 0.3;

    public static final int TOP_ROLLER_MOTOR_ID = 6;
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double SUCKER_ROLLER_EJECT_VALUE = 0.8;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }
}
