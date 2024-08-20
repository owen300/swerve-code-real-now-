package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Swerve.SwerveSubsystem;
import org.firstinspires.ftc.teamcode.SwerveAuto.Localizing.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.SwerveAuto.PidToPoint.PidToPointContinous;

public class AutoCommandContinous extends CommandBase {
    private final PidToPointContinous p2p;
    private final TwoWheelTrackingLocalizer localizer;
    private final SwerveSubsystem swerve;
    public AutoCommandContinous(Pose2d pose, TwoWheelTrackingLocalizer localizer, SwerveSubsystem swerve){
        this.localizer=localizer;
        this.p2p=new PidToPointContinous(localizer.getPoseEstimate());
        p2p.setTarget(pose);
        this.swerve=swerve;
    }

    @Override
    public void execute(){
        p2p.updatePose(localizer.getPoseEstimate());
        double[] powers=p2p.calculate();
        swerve.runRobotCentric(powers[0],powers[1],powers[2]);
    }

    @Override
    public boolean isFinished(){
        return p2p.isAtTarget();
    }
}
