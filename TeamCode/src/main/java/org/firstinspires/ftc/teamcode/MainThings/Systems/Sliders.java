package org.firstinspires.ftc.teamcode.MainThings.Systems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MainThings.utils.GlobalVars;

public class Sliders {
    public DcMotorEx sliderLeft, sliderRight;
    public void kill_sliders(){
        if(sliderLeft.getCurrentPosition()<=15){
            sliderLeft.setPower(0);
            sliderRight.setPower(0);
            sliderLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            sliderRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void moveSliders(GlobalVars.sliderState state){
        if(state == GlobalVars.sliderState.UP) {
            sliderRight.setTargetPosition(GlobalVars.sliderUpPos);
            sliderLeft.setTargetPosition(GlobalVars.sliderUpPos);
            sliderRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            sliderLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            sliderRight.setPower(1);
            sliderLeft.setPower(1);
        } else if(state == GlobalVars.sliderState.HIGH){
            sliderRight.setTargetPosition(GlobalVars.sliderHighPos);
            sliderLeft.setTargetPosition(GlobalVars.sliderHighPos);
            sliderRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            sliderLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            sliderRight.setPower(1);
            sliderLeft.setPower(1);
        }
        else if(state == GlobalVars.sliderState.BASKET_LOW){
            sliderRight.setTargetPosition(GlobalVars.sliderBasketLowPos);
            sliderLeft.setTargetPosition(GlobalVars.sliderBasketLowPos);
            sliderRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            sliderLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            sliderRight.setPower(1);
            sliderLeft.setPower(1);
        }
        else if(state == GlobalVars.sliderState.DOWN){
            sliderRight.setTargetPosition(GlobalVars.sliderDownPos);
            sliderLeft.setTargetPosition(GlobalVars.sliderDownPos);
            sliderRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            sliderLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            sliderRight.setPower(1);
            sliderLeft.setPower(1);
        }
        else if(state == GlobalVars.sliderState.PARCARE) {
            sliderRight.setTargetPosition(GlobalVars.slider_parcare);
            sliderLeft.setTargetPosition(GlobalVars.slider_parcare);
            sliderRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            sliderLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            sliderRight.setPower(1);
            sliderLeft.setPower(-1);
        }
    }
    public void extend(double pow){
        sliderLeft.setPower(pow);
        sliderRight.setPower(pow);
    }
    public void retract(double pow){
        sliderLeft.setPower(-pow);
        sliderRight.setPower(-pow);
    }

    public void moveSlidersToPos(int pos,double pow){
        sliderLeft.setTargetPosition(pos); //inlocuit cu variabile
        sliderRight.setTargetPosition(pos); //inlocuit cu variabile
        sliderLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sliderRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sliderLeft.setPower(pow);
        sliderRight.setPower(pow);
    }


    public Sliders(HardwareMap hardwareMap){
        sliderLeft=hardwareMap.get(DcMotorEx.class,"sliderLeft");
        sliderRight=hardwareMap.get(DcMotorEx.class, "sliderRight");

        sliderLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sliderRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        sliderLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sliderRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
}