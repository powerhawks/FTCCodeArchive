package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Pierce on 12/10/17.
 */

public class TestAuto3583 extends MBaseAutoMode3583 {

    enum ALLIANCE {
        RED,
        BLUE
    }

    enum ROBOT_STATE {
        SENSE_VUMARK,
        SENSE_JEWEL,
        DEPLOYING_ARM,
        RETRACT_ARM,
        GRAB_GLYPH,
        KNOCK_OFF_JEWEL,
        END
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Set this to whatever your alliance is
        ALLIANCE alliance = ALLIANCE.RED;
        JewelColor jewelColor = JewelColor.UNKNOWN;

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

        ROBOT_STATE state = ROBOT_STATE.SENSE_VUMARK;

        initRobot();

        waitForStart();

        while (opModeIsActive()) {

            switch (state) {
                case SENSE_VUMARK: {
                    vuMark = determineVumarkPosition();
                    if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                        // next state
                        state = ROBOT_STATE.GRAB_GLYPH;
                    }
                }
                break;

                case GRAB_GLYPH: {
                    grabGlyph();
                    state = ROBOT_STATE.DEPLOYING_ARM;
                }
                break;

                case DEPLOYING_ARM:  {

                    // next state
                    state = ROBOT_STATE.SENSE_JEWEL;

                }
                break;

                case SENSE_JEWEL: {
                    jewelColor = determineJewlelColor();
                    if (jewelColor == JewelColor.RED || jewelColor == JewelColor.BLUE) {

                        // next state
                        state = ROBOT_STATE.KNOCK_OFF_JEWEL;
                    }

                }

                case KNOCK_OFF_JEWEL: {
                    if (jewelColor == JewelColor.RED) {
                        if (alliance == ALLIANCE.RED) {
                            // go forwardPos
                        } else {
                            // go backward
                        }
                    } else if (jewelColor == JewelColor.BLUE) {
                        if (alliance == ALLIANCE.RED) {
                            // go backward
                        } else {
                            // go forwardPos
                        }
                    }
                    // next state
                    state = ROBOT_STATE.RETRACT_ARM;
                }

            }

            idle();
        }
    }
}
