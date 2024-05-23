package org.firstinspires.ftc.teamcode.Swerve;


import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import java.util.Arrays;
import java.util.Collections;

public class SwerveKinmatics {
    public double[] lastAngles={0,0,0,0};
    public double x=0;
    public double y=0;
    public double rx=0;
    public double a,b,c,d=0;
    public double L,W,R=0;
    public double temp=0;
    public SwerveKinmatics(double L,double W){
        this.L=L;
        this.W=W;
        R=sqrt((L*L)+(W*W));
    }
    public double[] calculateAngle(double x,double y,double rx,double heading){
        if(x==0&&y==0&&rx==0){
            return lastAngles;
        }else if(x==0&&y==0){
            double[] out={135,45,-45,-135};
            return out;
        }else {
            this.x = x;
            this.y = y;
            this.rx = rx;
            temp = this.y * cos(heading) + this.x * sin(heading);
            this.x = this.y * sin(heading) + this.x * cos(heading);
            this.y = temp;
            a = this.x - this.rx * (L / R);
            b = this.x + this.rx * (L / R);
            c = this.y - this.rx * (W / R);
            d = this.y + this.rx * (W / R);
            double[] angles = {atan2(b, c), atan2(b, d), atan2(a, d), atan2(a, c)};
            lastAngles = angles;
            return angles;
        }
    }
    public double[] calculatePower(double x,double y,double rx,double heading){
        if(x==0&&y==0&&rx==0){
            double[] out={0,0,0,0};
            return out;
        }else if(x==0&&y==0){
            double[] out={rx,rx,rx,rx};
            return out;
        }else {
            this.x = x;
            this.y = y;
            this.rx = rx;
            temp = this.y * cos(heading) + this.x * sin(heading);
            this.x = -this.y * sin(heading) + this.x * cos(heading);
            this.y = temp;
            a = this.x - this.rx * (L / R);
            b = this.x + this.rx * (L / R);
            c = this.y - this.rx * (W / R);
            d = this.y + this.rx * (W / R);
            double[] Power = {sqrt((b * b) + (c * c)), sqrt((b * b) + (d * d)), sqrt((a * a) + (d * d)), sqrt((a * a) + (c * c))};
            double max = Arrays.stream(Power).max().getAsDouble();
            if (max > 1) {
                for (int i = 0; i <= Power.length - 1; i++) {
                    Power[i] = Power[i] / max;
                }
            }

            return Power;
        }
    }

}
