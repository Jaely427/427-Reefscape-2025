// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RollerConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Class to run the rollers over the wire standard the yellow and green serial connection (CAN) */
public class AlgaeSubsystem extends SubsystemBase {
  
  private final SparkMax IntakeMotor;
  private AlgaeSubsystem() {
    IntakeMotor = new SparkMax(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushed);
    SparkMax OuttakeMotor = new SparkMax(Constants.RollerConstants, MotorType.kBrushless);
   
   // This funtions will set up the motors and encoders
   setupmotors();  
   RelativeEncoder IntakeMotorEncoder = IntakeMotor.getEncoder ();
   RelativeEncoder OutakeMotorEncoder = OuttakeMotor

   // Add PID controller
   // Add feedforward
   // Add limit switch
   // Add velocity
   // Add target position
   // Add control type 
  }

  public void setupmotors(){

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    IntakeMotor.setCANTimeout(250);

    // Create and apply configuration for roller motor. Voltage compensation helps
    // the roller behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the roller stalls.
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.voltageCompensation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP);
    rollerConfig.smartCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);
    IntakeMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  @Override
  public void periodic() {
  }

  /** This is a method that makes the roller spin */
  public void runRoller(double forward, double reverse) {
    IntakeMotor.set(forward - reverse);
  }
}
