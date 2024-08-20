package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Swerve.SwerveSubsystem;

public class DriverControl extends CommandBase {
    private SwerveSubsystem swerve;
    Gamepad gamepad;
    public DriverControl(SwerveSubsystem swerve, Gamepad gamepad){
        this.swerve=swerve;
        this.gamepad=gamepad;
    }
    @Override
    public void initialize(){
        swerve.runFeildCentric(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x);
        if(gamepad.start)swerve.imu.resetYaw();
    }
    @Override
    public void execute(){
        swerve.runFeildCentric(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x);
        if(gamepad.start)swerve.imu.resetYaw();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
