package org.firstinspires.ftc.teamcode.Swerve;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CodeUtil.ArrayPrintToTelem;
import org.firstinspires.ftc.teamcode.SwerveAuto.Localizing.TwoWheelTrackingLocalizer;

import java.util.List;
public class SwerveSubsystem extends SubsystemBase {
    public IMU imu;
    public static TwoWheelTrackingLocalizer localizer;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    private SwerveKinmatics S=new SwerveKinmatics(14,14);
    DcMotorEx rfm;
    DcMotorEx lfm;
    DcMotorEx lbm;
    DcMotorEx rbm;
    CRServo rf;
    CRServo lf;
    CRServo lb;
    CRServo rb;
    SwerveModule Rf=null;//modules
    SwerveModule Rb=null;
    SwerveModule Lf=null;
    SwerveModule Lb=null;
    public SwerveSubsystem(HardwareMap hardwareMap, Telemetry telemetry, TwoWheelTrackingLocalizer localizer){
        this.localizer=localizer;
        this.hardwareMap=hardwareMap;
        this.telemetry=telemetry;
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);//bulk reads
        }
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        AbsoluteAnalogEncoder rfe = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class,"rfe"));
        AbsoluteAnalogEncoder lfe = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class,"lfe"));
        AbsoluteAnalogEncoder rbe = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class,"rbe"));
        AbsoluteAnalogEncoder lbe = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class,"lbe"));
        rfm=hardwareMap.get(DcMotorEx.class,"rightFront");//hardware mappings
        rbm=hardwareMap.get(DcMotorEx.class,"rightRear");
        lbm=hardwareMap.get(DcMotorEx.class,"leftRear");
        lfm=hardwareMap.get(DcMotorEx.class,"leftFront");
        rf=hardwareMap.get(CRServo.class,"RFservo");
        rb=hardwareMap.get(CRServo.class,"RBservo");
        lf=hardwareMap.get(CRServo.class,"LFservo");
        lb=hardwareMap.get(CRServo.class,"LBservo");

        SwerveModule Rf=new SwerveModule(rfm,rf,rfe,telemetry);//modules
        SwerveModule Rb=new SwerveModule(rbm,rb,rbe,telemetry);
        SwerveModule Lf=new SwerveModule(lfm,lf,lfe,telemetry);
        SwerveModule Lb=new SwerveModule(lbm,lb,lbe,telemetry);
    }
    public void runFeildCentric(double x, double y, double rx){
        localizer.update();
        double[] angle= S.calculateAngle(x,y,rx,imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        Rf.setTargetRotation(angle[0]);
        Lf.setTargetRotation(angle[1]);
        Lb.setTargetRotation(angle[2]);
        Rb.setTargetRotation(angle[3]);//angles
        ArrayPrintToTelem.arrayPrintToTelem("angles",angle,telemetry);

        double[] power=S.calculatePower(x,y,rx,imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        Rf.setMotorPower(power[0]);
        Lf.setMotorPower(power[1]);
        Lb.setMotorPower(power[2]);//wheel power
        Rb.setMotorPower(power[3]);
        ArrayPrintToTelem.arrayPrintToTelem("powers",power,telemetry);

        Rf.update();
        Rb.update();//update modules
        Lf.update();
        Lb.update();
    }
    public void runRobotCentric(double x, double y, double rx){
        localizer.update();
        double[] angle= S.calculateAngle(x,y,rx,0);
        Rf.setTargetRotation(angle[0]);
        Lf.setTargetRotation(angle[1]);
        Lb.setTargetRotation(angle[2]);
        Rb.setTargetRotation(angle[3]);//angles
        ArrayPrintToTelem.arrayPrintToTelem("angles",angle,telemetry);

        double[] power=S.calculatePower(x,y,rx,0);
        Rf.setMotorPower(power[0]);
        Lf.setMotorPower(power[1]);
        Lb.setMotorPower(power[2]);//wheel power
        Rb.setMotorPower(power[3]);
        ArrayPrintToTelem.arrayPrintToTelem("powers",power,telemetry);

        Rf.update();
        Rb.update();//update modules
        Lf.update();
        Lb.update();
    }
    @Override
    public void periodic() {
        localizer.update();
    }
}

