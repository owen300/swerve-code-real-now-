package org.firstinspires.ftc.teamcode.CodeUtil;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArrayPrintToTelem {
    public static void arrayPrintToTelem(String caption, double[] a, Telemetry telemetry){
        String out="";
        for(int i=0;i<a.length;i++){
            out+=a[i]+" ";
        }
        telemetry.addData(caption, out);
        telemetry.update();
    }
}
