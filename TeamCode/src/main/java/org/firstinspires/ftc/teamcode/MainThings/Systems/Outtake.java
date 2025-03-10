package org.firstinspires.ftc.teamcode.MainThings.Systems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MainThings.utils.GlobalVars;

public class Outtake {
    public ServoImplEx armLeft, armRight, outtakeClaw, extendoOuttake;
    public RevColorSensorV3 sensor;
    public void moveArm(double armPos, double extendoPos) {armLeft.setPosition(armPos); armRight.setPosition(armPos);extendoOuttake.setPosition(extendoPos);}
    public void clawOuttake(GlobalVars.ClawState state) {
        if(state == GlobalVars.ClawState.OPEN) outtakeClaw.setPosition(GlobalVars.outtakeClawOpenPos);
        else if(state == GlobalVars.ClawState.CLOSE) outtakeClaw.setPosition(GlobalVars.outtakeClawClosePos);
        else if(state == GlobalVars.ClawState.MORE_CLOSED) outtakeClaw.setPosition(GlobalVars.outtakeClawMoreClosePos);
    }
    public boolean claw_has_detected(){
        if(sensor.getDistance(DistanceUnit.MM) <= 21) return true;
        return false;
    }
    public Outtake(HardwareMap hardwareMap){
        armLeft = hardwareMap.get(ServoImplEx.class, "bratLeft");
        armRight = hardwareMap.get(ServoImplEx.class,"bratRight");
        outtakeClaw = hardwareMap.get(ServoImplEx.class, "outtakeClaw");
        extendoOuttake = hardwareMap.get(ServoImplEx.class, "extendoOuttake");
        sensor = hardwareMap.get(RevColorSensorV3.class, "sensor");
    }
}