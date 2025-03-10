/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.MainThings.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MainThings.Systems.Extendo;
import org.firstinspires.ftc.teamcode.MainThings.Systems.Intake;
import org.firstinspires.ftc.teamcode.MainThings.Systems.LL3A;
import org.firstinspires.ftc.teamcode.MainThings.Systems.Outtake;

import java.util.List;

/*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */
@TeleOp(name = "Sensor: Limelight3A", group = "Sensor")
public class LimeLight extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        LL3A lime = new LL3A(hardwareMap,0);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        telemetry.setMsTransmissionInterval(100);

        limelight.pipelineSwitch(0);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(35, 60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Intake intake = new Intake(hardwareMap);
        Extendo extendo = new Extendo(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        intake.goIntakeToPos(GlobalVars.intakeUpPos);
        intake.clawIntake(GlobalVars.ClawState.OPEN);
        intake.clawIntakeSetPos(0.92);
        intake.goCoaxialToPos(GlobalVars.coaxialTransferPos);
        intake.moveAngular(GlobalVars.angularState.VERTICAL);
        extendo.moveExtendo(GlobalVars.extendoState.RETRACT);
        int cnt=0;
        waitForStart();
        int lastValue = 0;
        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s", status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());


            if (gamepad1.square) {
                intake.goIntakeToPos(GlobalVars.intakeUpPos);
                intake.clawIntake(GlobalVars.ClawState.OPEN);
                intake.clawIntakeSetPos(0.92);
                intake.goCoaxialToPos(GlobalVars.coaxialTransferPos);
                intake.moveAngular(GlobalVars.angularState.VERTICAL);
                extendo.moveExtendo(GlobalVars.extendoState.RETRACT);
            }

            if(gamepad1.triangle) {
                lime.alignLimeLight(drive); // are sleep in el
                continue;
            }
            if(gamepad1.circle) {
                lime.takeSample(); // are sleeps
                continue;
            }

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
                telemetry.addData("Is_Valid", result.isValid());
                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("tync", result.getTyNC());
                    telemetry.addData("Botpose", botpose.toString());
                    telemetry.addData("Botpose", botpose.toString());

                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();

                    // practic daca ii prea la stanga sau prea la dreapta
                    // ne miscam pana ne aliniem
                    // logica asta in auto ar trb sa vina intr-un while
                    if(result.getTx() > -1 && gamepad1.triangle)
                    {
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        0,
                                        0.2,
                                        0
                                )
                        );
                    }
                    else if(result.getTx() < -4 && gamepad1.triangle) {
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        0,
                                        -0.2,
                                        0
                                )
                        );
                    }
                    else
                    {
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        0,
                                        0,
                                        0
                                )
                        );
                    }

                    if(!gamepad1.triangle)
                    {

                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        0,
                                        0,
                                        0
                                )
                        );
                    }

                    for (LLResultTypes.ColorResult cr : colorResults) {
                        // Retrieve target corners

                        // Retrieve the target corners
                        // Retrieve the target corners
                        telemetry.addData("Target Area: ", cr.getTargetArea());
                        List<List<Double>> targetCorners = cr.getTargetCorners();
                        if (targetCorners != null && !targetCorners.isEmpty()) {
                            // Initialize variables to track the maximum differences
                            double maxXDelta = 0;
                            double maxYDelta = 0;

                            double maxSignedXDelta = 0;
                            double maxSignedYDelta = 0;

                            // Iterate over all pairs of corners to find the maximum differences
                            for (int i = 0; i < targetCorners.size(); i++) {
                                for (int j = i + 1; j < targetCorners.size(); j++) {
                                    List<Double> corner1 = targetCorners.get(i);
                                    List<Double> corner2 = targetCorners.get(j);

                                    // Calculate the differences in x and y coordinates
                                    double deltaX = Math.abs(corner2.get(0) - corner1.get(0));
                                    double deltaY = Math.abs(corner2.get(1) - corner1.get(1));
                                    // Update the maximum differences
                                    if(deltaX > maxXDelta)
                                        maxSignedXDelta = corner2.get(0) - corner1.get(0);

                                    if(deltaY > maxYDelta)
                                        maxSignedYDelta = corner2.get(1) - corner1.get(1);

                                    maxXDelta = Math.max(maxXDelta, deltaX);
                                    maxYDelta = Math.max(maxYDelta, deltaY);

                                }
                            }
                            telemetry.addData("CR Y Degrees:", cr.getTargetYDegrees());
                            if (cr.getTargetYDegrees() > -7.5) // daca e prea departe targetu poate trb schimbat daca nu vr sa ia
                                continue;

                            double val = (cr.getTargetYDegrees() + 20) / 14; // cea mai mica valoarea sa zicem ca ii - 20
                            // trb adaugat + 20 ca sa ajungem la 0.
                            // apoi lua cea mai mare valoare care sa zicem ca ii gen -6
                            // -6 + 20 = 14 si impartim la 14


                            // -8 // -15
                            // safety check sa nu futem robotu
                            if (val < 0)
                                continue;

                            telemetry.addData("MaxXDelta Target:", maxXDelta);
                            telemetry.addData("MaxYDelta Target:", maxYDelta);
                            telemetry.addData("MaxXDeltaSigned Target:", maxSignedXDelta);
                            telemetry.addData("MaxYDeltaSigned Target:", maxYDelta);

                            // horizontal => x = 2 * y
                            // vertical => y = 1.5 * y
                            double ratio = maxXDelta / maxYDelta;
                            boolean is_diagonal = Math.abs(1 - ratio) < 0.2;

                            if (maxXDelta > maxYDelta && !is_diagonal) {

                                if (gamepad1.circle)
                                    intake.moveAngular(GlobalVars.angularState.HORIZONTAL);


                                telemetry.addData("Orientation Target:", "Horizontal");
                            } else {

                                if (gamepad1.circle)
                                    intake.moveAngular(GlobalVars.angularState.VERTICAL);


                                telemetry.addData("Orientation Target:", "Vertical");
                            }



                            if (gamepad1.circle) {
                                // Determine the orientation based on the maximum differences

                                extendo.moveExtendoToPos(Math.max(0.4, 0.575 - val * 0.14)); // calculu magic explicat sus
                                intake.goCoaxialToPos(GlobalVars.coaxialIntakePos);

                                if(is_diagonal)
                                    intake.clawIntakeSetPos(GlobalVars.intakeClawMoreOpenPos);
                                else
                                    intake.clawIntakeSetPos(GlobalVars.intakeClawOpenPos);

                                intake.goIntakeToPos(GlobalVars.intakeInterPos);
                                sleep(600);
                                intake.goIntakeToPos(GlobalVars.intakeDownPos);
                                sleep(250);
                                intake.clawIntakeSetPos(GlobalVars.intakeClawClosePos);
                                outtake.moveArm(GlobalVars.armTransferPos,GlobalVars.extendoOuttakeRetractPos);
                                outtake.clawOuttake(GlobalVars.ClawState.OPEN);
                                sleep(500);
                                intake.goCoaxialToPos(GlobalVars.coaxialSlideForTransferPos);

                                intake.moveAngular(GlobalVars.angularState.VERTICAL);
                                intake.goIntakeToPos(GlobalVars.intakeUpPos);
                                outtake.clawOuttake(GlobalVars.ClawState.OPEN);
                                outtake.moveArm(GlobalVars.armTransferPos, GlobalVars.extendoOuttakeRetractPos);

                                sleep(500);

                                extendo.moveExtendo(GlobalVars.extendoState.RETRACT);
                                intake.clawIntake(GlobalVars.ClawState.MORE_CLOSED);
                                intake.goCoaxialToPos(GlobalVars.coaxialTransferPos);

                                sleep(700);

                                outtake.clawOuttake(GlobalVars.ClawState.CLOSE);
                                sleep(600);
                                intake.clawIntake(GlobalVars.ClawState.OPEN);
                                sleep(300);
                                outtake.moveArm(GlobalVars.armOuttakePos, GlobalVars.extendoOuttakeRetractPos);
                                sleep(500);
                                outtake.clawOuttake(GlobalVars.ClawState.OPEN);
                            }
                            break;

                        }
                        else {
                            telemetry.addData("Color Target", "Invalid or empty corner data");
                        }
                    }
                    telemetry.update();
                }

                telemetry.update();
            }


        }
    }
}


/*

   // Access color results
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    for (LLResultTypes.ColorResult cr : colorResults) {
                        // Retrieve target corners

                        // Retrieve the target corners
                        // Retrieve the target corners
                        telemetry.addData("Target Area: " , cr.getTargetArea());
                        List<List<Double>> targetCorners = cr.getTargetCorners();
                        if (targetCorners != null && !targetCorners.isEmpty()) {
                            // Initialize variables to track the maximum differences
                            double maxXDelta = 0;
                            double maxYDelta = 0;

                            // Iterate over all pairs of corners to find the maximum differences
                            for (int i = 0; i < targetCorners.size(); i++) {
                                for (int j = i + 1; j < targetCorners.size(); j++) {
                                    List<Double> corner1 = targetCorners.get(i);
                                    List<Double> corner2 = targetCorners.get(j);

                                    // Calculate the differences in x and y coordinates
                                    double deltaX = Math.abs(corner2.get(0) - corner1.get(0));
                                    double deltaY = Math.abs(corner2.get(1) - corner1.get(1));

                                    // Update the maximum differences
                                    maxXDelta = Math.max(maxXDelta, deltaX);
                                    maxYDelta = Math.max(maxYDelta, deltaY);
                                }
                            }

                            if(cr.getTargetYDegrees() > -7.5)
                                continue;

                            double val = (cr.getTargetYDegrees() + 20) / 14; // 0, 13

                            // -8 // -15
                            if(val < 0)
                                continue;

                            if(gamepad1.circle) {
                                // Determine the orientation based on the maximum differences
                                if (maxXDelta > maxYDelta) {

                                    intake.moveAngular(GlobalVars.angularState.HORIZONTAL);

                                    telemetry.addData("Orientation Target:", "Horizontal");
                                } else {

                                    intake.moveAngular(GlobalVars.angularState.VERTICAL);


                                    telemetry.addData("Orientation Target:", "Vertical");
                                }
                                extendo.moveExtendoToPos(Math.max(0.33, 0.48 - val * 0.15));
                                intake.goCoaxialToPos(GlobalVars.coaxialIntakePos);
                                intake.clawIntakeSetPos(GlobalVars.intakeClawOpenPos);
                                intake.goIntakeToPos(GlobalVars.intakeInterPos);
                                sleep(700);
                                intake.goCoaxialToPos(GlobalVars.intakeDownPos);
                                sleep(200);
                                intake.clawIntakeSetPos(GlobalVars.intakeClawClosePos);
                                outtake.moveArm(GlobalVars.armTransferPos);
                                outtake.clawOuttake(GlobalVars.ClawState.OPEN);
                                sleep(300);
                                intake.goCoaxialToPos(GlobalVars.coaxialSlideForTransferPos);
                                sleep(500);

                                extendo.moveExtendo(GlobalVars.extendoState.RETRACT);

                            } else {
                            telemetry.addData("Color Target", "Invalid or empty corner data");
                        }
 */