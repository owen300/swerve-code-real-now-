/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;



/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="swerve test", group="Linear OpMode")

public class SwerveTestt extends LinearOpMode {
    private SwerveKinmatics S=new SwerveKinmatics(14,14);
    DcMotorEx rfm;
    DcMotorEx lfm;
    DcMotorEx lbm;
    DcMotorEx rbm;
    CRServo rf;
    CRServo lf;
    CRServo lb;
    CRServo rb;


public void runOpMode(){
    List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule module : allHubs) {
        module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);//bulk reads
    }

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

    SwerveModule Rf=new SwerveModule(rfm,rf,rfe);//modules
    SwerveModule Rb=new SwerveModule(rbm,rb,rbe);
    SwerveModule Lf=new SwerveModule(lfm,lf,lfe);
    SwerveModule Lb=new SwerveModule(lbm,lb,lbe);
        waitForStart();



        while (opModeIsActive()) {
            double[] angle= S.calculateAngle(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,0);
            Rf.setTargetRotation(angle[0]);
            Lf.setTargetRotation(angle[1]);
            Lb.setTargetRotation(angle[2]);
            Rb.setTargetRotation(angle[3]);//angles

            double[] power=S.calculatePower(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,0);
            Rf.setMotorPower(power[0]);
            Lf.setMotorPower(power[1]);
            Lb.setMotorPower(power[2]);//wheel power
            Rb.setMotorPower(power[3]);
            Rf.update();
            Rb.update();//update modules
            Lf.update();
            Lb.update();
        }
    }}
