package org.firstinspires.ftc.teamcode.SwerveAuto;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseEsitmator {
    private Pose2d pose=null;
    public PoseEsitmator(){
        pose= new Pose2d(0,0,0);
    }
    public PoseEsitmator(Pose2d initPose){
        pose= initPose;
    }
    public void updateFromOdo(double xChange, double yChange, double heading){
        pose=new Pose2d(pose.position.x+xChange,pose.position.y+yChange, heading);
    }
    public Pose2d getPose(){
        return pose;
    }
}
