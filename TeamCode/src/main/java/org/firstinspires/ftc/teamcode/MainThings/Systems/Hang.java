package org.firstinspires.ftc.teamcode.MainThings.Systems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.MainThings.utils.GlobalVars;

public class Hang {
    //TODO: TBD
    ServoImplEx ptoLeft,ptoRight;
    DcMotorEx hangMotor;

    public void startPTO(){
        ptoRight.setPosition(GlobalVars.ptoRightStartPos);
        ptoLeft.setPosition(GlobalVars.ptoLeftStartPos);
    }
    public void releasePTO(){
        ptoRight.setPosition(GlobalVars.ptoRightReleasePos);
        ptoLeft.setPosition(GlobalVars.ptoLeftReleasePos);
    }

    public void hangLevel2(double pow){
        hangMotor.setPower(pow);
    }
    public void LiftoffLevel2(double pow){
        hangMotor.setPower(-pow);
    }

    public Hang(HardwareMap hardwareMap){
        ptoLeft= hardwareMap.get(ServoImplEx.class,"ptoLeft");
        ptoRight= hardwareMap.get(ServoImplEx.class, "ptoRight");
        hangMotor= hardwareMap.get(DcMotorEx.class, "hang");
    }
}
