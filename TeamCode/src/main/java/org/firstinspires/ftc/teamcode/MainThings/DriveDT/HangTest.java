package org.firstinspires.ftc.teamcode.MainThings.DriveDT;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MainThings.Systems.Extendo;
import org.firstinspires.ftc.teamcode.MainThings.Systems.Hang;
import org.firstinspires.ftc.teamcode.MainThings.Systems.Intake;
import org.firstinspires.ftc.teamcode.MainThings.Systems.Outtake;
import org.firstinspires.ftc.teamcode.MainThings.Systems.Sliders;
import org.firstinspires.ftc.teamcode.MainThings.utils.GlobalVars;
import org.firstinspires.ftc.teamcode.MainThings.utils.SampleMecanumDrive;

@TeleOp(name="hang", group = "Main")
@Config

public class HangTest extends LinearOpMode {


    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Hang hang = new Hang(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y ,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    ));
            if(gamepad1.left_trigger>0.3) hang.hangLevel2(gamepad1.left_trigger);
            else if(gamepad1.right_trigger>0.3) hang.LiftoffLevel2(gamepad1.right_trigger);
            else hang.hangLevel2(0);
            if(gamepad1.square)hang.startPTO();
            if(gamepad1.circle)hang.releasePTO();
        }
    }
}