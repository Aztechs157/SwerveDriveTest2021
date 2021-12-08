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
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    private CANSparkMax motor = new CANSparkMax(DriveConstants.MOTOR_ID, MotorType.kBrushless);
    private TalonSRX encoder = new TalonSRX(DriveConstants.ENCODER_ID);

    private NetworkTableEntry targetEntry;
    private NetworkTableEntry tolaranceEntry;
    private NetworkTableEntry speedEntry;
    private XboxController controller;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem(XboxController controller) {
        this.controller = controller;

        var tab = Shuffleboard.getTab("Debug");

        tab.addNumber("Current Ticks", this::getCurrent);
        tab.addNumber("Current Degrees", () -> getCurrent() % 1 * 360);

        // targetEntry = tab.add("Target Degrees", 0).getEntry();
        tab.addNumber("Target Degrees", this::degreesFromController);
        tab.addNumber("Target Ticks", this::getTarget);

        tolaranceEntry = tab.add("Tolarance Degrees", 0).getEntry();
        speedEntry = tab.add("Speed", 0).getEntry();

        tab.addNumber("Absolute Encoder", encoder.getSensorCollection()::getAnalogIn);

        resetEncoder();

        setDefaultCommand(new RunCommand(this::talonIndicator, this));
    }

    public void resetEncoder() {
        motor.getEncoder().setPosition(0);
    }

    private double degreesFromController() {
        var x = controller.getRawAxis(0);
        var y = controller.getRawAxis(1);
        return Math.toDegrees(Math.atan2(y, x)) + 180;
    }

    public double getTarget() {
        // return targetEntry.getDouble(0) / 360 % 1;
        return degreesFromController() / 360 % 1;
    }

    public double getCurrent() {
        return (motor.getEncoder().getPosition() / 69.33) % 1;
    }

    public double getTolorance() {
        return tolaranceEntry.getDouble(0) / 360 % 1;
    }

    public double getSpeed() {
        return speedEntry.getDouble(0);
    }

    public void setMotor(final double speed) {
        motor.set(speed);
    }

    public void startMotorPositive() {
        motor.set(getSpeed());
    }

    public void startMotorNegative() {
        motor.set(-getSpeed());
    }

    public void stopMotor() {
        motor.set(0);
    }

    // Temporary Communication Test for the Talon SRX
    public void talonIndicator() {
        encoder.set(ControlMode.PercentOutput, controller.getY(Hand.kLeft));
    }
}
