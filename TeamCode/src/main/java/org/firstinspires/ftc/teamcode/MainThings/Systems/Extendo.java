package org.firstinspires.ftc.teamcode.MainThings.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.ServoHubConfiguration;

import org.firstinspires.ftc.teamcode.MainThings.utils.GlobalVars;

public class Extendo {
    public ServoImplEx extendoLeft,extendoRight;
    public ServoHub
    public void moveExtendoToPos(double pos){
        extendoLeft.setPosition(pos);
        extendoRight.setPosition(pos);
    }
    public void moveExtendo(GlobalVars.extendoState state){
        if(state == GlobalVars.extendoState.EXTEND) {
            extendoLeft.setPosition(GlobalVars.extendoExtendPos);
            extendoRight.setPosition(GlobalVars.extendoExtendPos);
        } else if(state == GlobalVars.extendoState.RETRACT) {
            extendoLeft.setPosition(GlobalVars.extendoRetractPos);
            extendoRight.setPosition(GlobalVars.extendoRetractPos);
        }
        else if(state == GlobalVars.extendoState.EXTEND_AUTO) {
            extendoLeft.setPosition(GlobalVars.extendoExtendPosAuto);
            extendoRight.setPosition(GlobalVars.extendoExtendPosAuto);
        }

    }

    public Extendo(HardwareMap hardwareMap){
        extendoLeft = hardwareMap.get(ServoImplEx.class, "extendoLeft");
        extendoRight = hardwareMap.get(ServoImplEx.class, "extendoRight");
    }
}