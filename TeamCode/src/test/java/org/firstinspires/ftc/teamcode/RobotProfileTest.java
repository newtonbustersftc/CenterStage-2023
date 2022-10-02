package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.assertTrue;

public class RobotProfileTest {
    @Test
    public void createProfile() {
        System.out.println("Hello Scratch\n");
        RobotProfile p = new RobotProfile();
        p.createSampleProfile();
        try {
            p.saveToFile(new java.io.File("/Users/haifeng/StudioProjects/FtcRobotController2022/config/test-profile.json"));
        }
        catch (Exception ex) {
            ex.printStackTrace();
        }
        assertTrue(true);
    }
}