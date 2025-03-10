package org.firstinspires.ftc.teamcode.MainThings.Systems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MainThings.utils.GlobalVars;

public class Intake{
    public ServoImplEx intakeLeft, intakeRight, intakeClaw, coaxial, angular;
    public RevColorSensorV3 intakeSensor;
    public void clawIntakeSetPos(double pos)
    {
        intakeClaw.setPwmEnable();
        intakeClaw.setPosition(pos);
    }

    public void clawIntake(GlobalVars.ClawState state){
        if(state == GlobalVars.ClawState.OPEN) {
            intakeClaw.setPwmEnable();
            intakeClaw.setPosition(GlobalVars.intakeClawOpenPos);
        }
        else if(state == GlobalVars.ClawState.CLOSE){
            intakeClaw.setPwmEnable();
            intakeClaw.setPosition(GlobalVars.intakeClawClosePos);
        }
        else if(state == GlobalVars.ClawState.MORE_CLOSED){
            intakeClaw.setPwmEnable();
            intakeClaw.setPosition(GlobalVars.intakeClawMoreClosedPos);
        }
        else if(state == GlobalVars.ClawState.OPEN_AUTO)
            intakeClaw.setPosition(GlobalVars.intakeClawOpenPosAuto);
        else if(state == GlobalVars.ClawState.CLOSE_AUTO)
            intakeClaw.setPosition(GlobalVars.intakeClawClosePosAuto);
        else if(state == GlobalVars.ClawState.IDLE)
            intakeClaw.setPwmDisable();
    }
    public void goCoaxialToPos(double pos) {coaxial.setPosition(pos);}
    public void moveAngular(GlobalVars.angularState state) {
        if(state == GlobalVars.angularState.HORIZONTAL) angular.setPosition(GlobalVars.angularHorizontalPos);
        else if(state == GlobalVars.angularState.VERTICAL) angular.setPosition(GlobalVars.angularVerticalPos);
    }

    public void moveAngularToPos(double pos) {
        angular.setPosition(pos);
    }

    public void pivot()
    {
        angular.setPosition(GlobalVars.angularDreaptaPos);
    }

    public void pivotauto()
    {
        angular.setPosition(GlobalVars.angularDreaptaPos-0.3);
    }
    public void goIntakeToPos(double pos){
        intakeLeft.setPosition(pos);
        intakeRight.setPosition(pos);
    }
    public boolean detectedSample(){
        if(intakeSensor.getDistance(DistanceUnit.MM) <= 30) return true;
        return false;
    }
    public Intake(HardwareMap hardwareMap){
        intakeLeft = hardwareMap.get(ServoImplEx.class, "intakeArmLeft");
        intakeRight = hardwareMap.get(ServoImplEx.class, "intakeArmRight");
        intakeClaw = hardwareMap.get(ServoImplEx.class, "intakeClaw");
        coaxial = hardwareMap.get(ServoImplEx.class, "coaxial");
        angular = hardwareMap.get(ServoImplEx.class, "angular");
        intakeSensor=hardwareMap.get(RevColorSensorV3.class, "intakeSensor");
    }
}