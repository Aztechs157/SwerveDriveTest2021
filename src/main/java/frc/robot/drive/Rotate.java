// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;

public class Rotate extends CommandBase {
    private final DriveSubsystem drive;
    private final PIDController pid = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

    /** Creates a new Rotate. */
    public Rotate(final DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);

        pid.enableContinuousInput(-180, 180);

        final var tab = Shuffleboard.getTab("Debug");
        tab.add(pid);
        tab.addNumber("Speed", this::getSpeed);
    }

    private double getSpeed() {
        return pid.calculate(drive.getCurrent(), drive.getTarget());
    }

    @Override
    public void execute() {
        drive.setMotor(getSpeed());
    }

    @Override
    public void end(final boolean interrupted) {
        drive.stopMotor();
    }
}
