// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    private CANSparkMax motor = new CANSparkMax(DriveConstants.MOTOR_ID, MotorType.kBrushless);
    private TalonSRX encoder = new TalonSRX(DriveConstants.ENCODER_ID);

    private NetworkTableEntry manualTargetEntry;
    private NetworkTableEntry maxSpeedEntry;

    private XboxController controller;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem(XboxController controller) {
        this.controller = controller;

        var tab = Shuffleboard.getTab("Debug");

        tab.addNumber("Current", this::getCurrent);

        manualTargetEntry = tab.add("Manual Target", 0).getEntry();
        tab.addNumber("Controller Target", this::degreesFromController);

        maxSpeedEntry = tab.add("Max Speed", 0).getEntry();

        tab.addNumber("Absolute Encoder", encoder.getSensorCollection()::getAnalogIn);

        resetEncoder();

        setDefaultCommand(new RunCommand(this::talonIndicator, this));
    }

    public void resetEncoder() {
        motor.getEncoder().setPosition(0);
    }

    /**
     *
     * @return Controller joystick rotation in the range of -180 to +180 degrees
     */
    private double degreesFromController() {
        var x = controller.getRawAxis(0);
        var y = controller.getRawAxis(1);
        return Math.toDegrees(Math.atan2(y, x));
    }

    /**
     *
     * @return Target rotation in the range of -180 to +180 degrees
     */
    public double getTarget() {
        // return targetEntry.getDouble(0) / 360 % 1;
        return degreesFromController();
    }

    /**
     *
     * @return Current rotation in the range of -180 to +180 degrees
     */
    public double getCurrent() {
        var rotations = motor.getEncoder().getPosition() / DriveConstants.GEAR_RATIO;
        // Get portion after decimal point
        var fractional = rotations % 1;
        var degrees = fractional * 360;

        if (degrees > 180) {
            return degrees - 360;
        }

        if (degrees < -180) {
            return degrees + 360;
        }

        return degrees;
    }

    private double getMaxSpeed() {
        return maxSpeedEntry.getDouble(0);
    }

    public void setMotor(final double speed) {
        var output = MathUtil.clamp(speed, -getMaxSpeed(), getMaxSpeed());
        motor.set(output);
    }

    public void stopMotor() {
        motor.set(0);
    }

    // Temporary Communication Test for the Talon SRX
    public void talonIndicator() {
        encoder.set(ControlMode.PercentOutput, controller.getY(Hand.kLeft));
    }
}
