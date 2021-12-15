// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Rotate extends CommandBase {
    private DriveSubsystem drive;
    private PIDController pid = new PIDController(0, 0, 0);

    /** Creates a new Rotate. */
    public Rotate(final DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);

        pid.enableContinuousInput(-180, 180);

        var tab = Shuffleboard.getTab("Debug");
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
    public void end(boolean interrupted) {
        drive.stopMotor();
    }
}
