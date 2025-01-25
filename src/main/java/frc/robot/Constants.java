// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;


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
    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 2;
    public static final int RIGHT_LEADER_ID = 3;
    public static final int RIGHT_FOLLOWER_ID = 4;


    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;




    public static final double maxSpeed = 5;
    public static final double maxAngularSpeed = Math.PI;
  }


  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 5;
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double ROLLER_EJECT_VALUE = 0.44;
  }


  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }
 public static final class ClimbConstants {
    public static final int ClimbGearRatio=100;
    public static final int EncoaderConstant=42; 
    public static final double kS=0;
    public static final double kA=0;
    public static final double kV=0;
    public static final double kG=9.92;
    public static final float kForwardSoftLimit=0;
    public static final float kReverseSoftLimit=0;
    public static final boolean kRightMotorInverted=true;
    public static final double kAbsPositionConversionFactor=0;
    public static final double kAbsVelocityConversionFactor=0;
    public static final double kRelativePositionConversionFactor=0;
    public static final double kRelativeVelocityConversionFactor=0;
    public static final int kMotorCurrentLimit=0;
    public static final boolean kLeftMotorInverted=true;
    public static final double kP=0;
    public static final double kI=0;
    public static final double kD=0;
    public static final double kTravelPosition=0;
    public static final int kLimitSwitchId=0;
    public static final int kArmMotorLeftId=0;
    public static final double kTolerance=0;
    public static final double kGroundPosition=0;
    


    


    
 }
}
