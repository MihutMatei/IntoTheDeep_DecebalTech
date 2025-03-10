package org.firstinspires.ftc.teamcode.MainThings.utils;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

@Config
public class GlobalVars {

    public enum Modedrive {DRIVER_CONTROL, TURBO, PRECISION}
    public enum ClawState {OPEN, CLOSE, MORE_CLOSED, OPEN_AUTO , CLOSE_AUTO, IDLE}
    public enum extendoState {EXTEND, RETRACT, EXTEND_AUTO}
    public enum angularState {VERTICAL, HORIZONTAL}
    public enum sliderState {UP, DOWN, HIGH,BASKET_LOW, PARCARE}

    //TODO: find slider positions
    public static final double TURBO_SCALE = 1.0, PRECISION_SCALE = 4.3, DRIVE_SCALE = 1.7;
    public static double extendoRetractPos = 0.67  , extendoExtendPos = 0.44 , extendoExtendPosAuto = 0.55 ;
    public static double intakeDownPos = 0.59, intakeUpPos = 0.43, intakeInterPos = 0.525,intakeSlidePos=0.35,intakeParallelPos=0.52;
    public static double armTransferPos = 0.27, armOuttakePos = 0.75,armBarPos=0.45,armBarUpperPos=0.5, armIntermediarPos=0.3, armIntakePos = 0.97, armParcarePos = 0.65;
    public static double coaxialTransferPos =0.18, coaxialDetectiePos=0.55 ,coaxialIntakePos = 0.85, coaxialSlideForTransferPos=0.1;
    public static double angularVerticalPos = 0.12, angularHorizontalPos = 0.4,angularDreaptaPos=0.21,angularStangaPos=0.01;
    public static double outtakeClawOpenPos = 0.9, outtakeClawClosePos = 0.73,outtakeClawMoreClosePos = 0.68;
    public static double intakeClawOpenPos=0.85, intakeClawMoreOpenPos = 0.6, intakeClawClosePos=0.69,intakeClawMoreClosedPos=0.6 ,intakeClawOpenPosAuto = 0.91, intakeClawClosePosAuto = 0.39;
    public static double ptoLeftStartPos=0.535, ptoLeftReleasePos=0.425;
    public static double ptoRightStartPos=0.58, ptoRightReleasePos=0.49;
    public static double extendoOuttakeRetractPos=0.471, extendoOuttakeExtendedPos=0.64;
    public static double motorAmps=6;

    public static double sliderPower = 0.8, sliberPowerBreak = 0.001;
    public static int sliderUpPos = 400,sliderBasketLowPos=250, sliderDownPos = 3, sliderHighPos = 770,sliderParcarePos=240;
    public static int slider_parcare =190;


}