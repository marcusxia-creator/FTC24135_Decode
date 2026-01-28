package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueSideFarAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(15.5,16)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(64, -8, Math.toRadians(0)))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(36,-32),Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(36,-48),Math.toRadians(-90))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(12,-12),Math.toRadians(22.5))
                .strafeToLinearHeading(new Vector2d(-12,-12),Math.toRadians(45))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(12,-32),Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(12,-48),Math.toRadians(-90))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(-12,-12),Math.toRadians(45))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
