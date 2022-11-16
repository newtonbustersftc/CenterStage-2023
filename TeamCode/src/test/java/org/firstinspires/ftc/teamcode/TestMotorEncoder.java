package org.firstinspires.ftc.teamcode;

import org.junit.Test;

public class TestMotorEncoder {
    @Test
    public void createLocalizer(){
        System.out.println("Hello Scratch\n");
        FourStandardTrackingWheelLocalizer l = new FourStandardTrackingWheelLocalizer();
        TestEncoder leftEncoder = new TestEncoder(0);
        TestEncoder rightEncoder = new TestEncoder(0);
        TestEncoder frontEncoder = new TestEncoder(0);
        TestEncoder backEncoder = new TestEncoder(0);
        l.leftEncoder = leftEncoder;
        l.frontEncoder = frontEncoder;
        l.rightEncoder = rightEncoder;
        l.backEncoder = backEncoder;
        l.update();
        System.out.println(l.getPoseEstimate());
    }
}