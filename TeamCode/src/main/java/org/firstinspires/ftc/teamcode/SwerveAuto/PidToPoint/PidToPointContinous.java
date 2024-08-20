package org.firstinspires.ftc.teamcode.SwerveAuto.PidToPoint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class PidToPointContinous {
    //public PoseEsitmator poseE=null;
    public Pose2d current= null;
    public Pose2d target=null;
    private PIDFCoefficients xPidc = new PIDFCoefficients(0.1,0,0.1,0);
    private PIDFCoefficients yPidc = new PIDFCoefficients(0.1,0,0.1,0);
    private PIDFCoefficients hPidc = new PIDFCoefficients(0.1,0,0.1,0);
    private PIDController xPid= new PIDController(xPidc.p, xPidc.i, xPidc.d);
    private PIDController yPid= new PIDController(yPidc.p, yPidc.i, yPidc.d);
    private PIDController hPid= new PIDController(hPidc.p, hPidc.i, hPidc.d);
    public PidToPointContinous(Pose2d initPose){
        current=initPose;
        xPid.setTolerance(3);
        yPid.setTolerance(3);
        hPid.setTolerance(3);
    }
    public void updatePose(Pose2d pose){
        current=pose;
    }
    public void setTarget(Pose2d t){
        target=t;
    }
    public double[] calculate(){
        double[] powers= new double[3];
        powers[0]= xPid.calculate(current.getX(),target.getX());
        powers[1]= yPid.calculate(current.getY(),target.getY());
        powers[2]= hPid.calculate(current.getHeading(),target.getHeading());
        return powers;
    }
    public boolean isAtTarget(){
        return xPid.atSetPoint()&& yPid.atSetPoint()&& hPid.atSetPoint();
    }

}
