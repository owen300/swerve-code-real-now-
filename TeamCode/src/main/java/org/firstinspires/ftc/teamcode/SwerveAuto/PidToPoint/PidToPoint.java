package org.firstinspires.ftc.teamcode.SwerveAuto.PidToPoint;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.SwerveAuto.Localizing.Localizer;
import org.firstinspires.ftc.teamcode.SwerveAuto.Localizing.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.SwerveAuto.PoseEsitmator;

public class PidToPoint {
    public PoseEsitmator poseE=null;
    public Pose2d target=null;
    private PIDFCoefficients xPidc = new PIDFCoefficients(0.1,0,0.1,0);
    private PIDFCoefficients yPidc = new PIDFCoefficients(0.1,0,0.1,0);
    private PIDFCoefficients hPidc = new PIDFCoefficients(0.1,0,0.1,0);
    private PIDController xPid= new PIDController(xPidc.p, xPidc.i, xPidc.d);
    private PIDController yPid= new PIDController(yPidc.p, yPidc.i, yPidc.d);
    private PIDController hPid= new PIDController(hPidc.p, hPidc.i, hPidc.d);
    public PidToPoint(Pose2d initPose){
        poseE =new PoseEsitmator(initPose);
    }
    public void updatePose(double X, double Y, double heading){

        poseE.updateFromOdo(X-poseE.getPose().position.x,Y-poseE.getPose().position.y,heading);
    }
    public void setTarget(Pose2d t){
        target=t;
    }
    public double[] calculate(){
        double[] powers= new double[3];
        powers[0]= xPid.calculate(poseE.getPose().position.x,target.position.x);
        powers[1]= yPid.calculate(poseE.getPose().position.y,target.position.y);
        powers[2]= hPid.calculate(poseE.getPose().heading.toDouble(),target.heading.toDouble());
        return powers;
    }
    public boolean isAtTarget(){
        return xPid.atSetPoint()&& yPid.atSetPoint()&& hPid.atSetPoint();
    }

}
