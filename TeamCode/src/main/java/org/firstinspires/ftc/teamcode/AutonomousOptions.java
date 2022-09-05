package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;

/**
 * Emily 11-6-21 09:00-11:00
 * Updated the current year specifications
 */
@TeleOp(name="Autonomous Options", group="Main")

public class AutonomousOptions extends OpMode {

    // ADD preference names here
    public static final String START_POS_MODES_PREF = "starting position";
    public static final String START_DELAY_PREF = "start_delay";
    public static final String PARK_ONLY_PREF = "park only";
    public static final String DELIVER_TO_HUB_USING_OPENCV_PREF ="Deliver to hub using openCV";
    public static final String DUCK_DELIVER_TO_HUB_BY_SIDE_PREF = "Duck deliver to hub by side route";
    public static final String FREIGHT_DELIVERY_COUNT_PREF = "freight delivery count";
    public static final String DUCK_PARKING_DIRECTION_PREF = "duck parking direction";
    public static final String DELAY_PARKING_STORAGE_PREF = "delay parking by storage";
    public static final String DELAY_PARKING_WAREHOUSE_PREF = "delay parking by warehouse";
    public static final String PARKING_PREF = "parking";
    // ADD preference values here
    public static final String[] START_POS_MODES = {"RED_DUCK", "RED_DEPOT","BLUE_DUCK", "BLUE_DEPOT"};
    public static final String[] START_DELAY = {"0 " + "sec", "1 sec", "2 sec", "3 sec", "4 sec", "5 sec", "25 sec"};
    public static final String[] PARK_ONLY = {"PARK_ONLY", "NONE"};
    public static final String[] DELIVER_TO_HUB_USING_OPENCV ={"YES", "NO"};
    public static final String[] DUCK_DELIVER_TO_HUB_BY_SIDE ={"YES", "NO"};
    public static final String[] FREIGHT_DELIVERY_COUNT = {"0", "1","2","3"};
    public static final String[] DUCK_PARKING_DIRECTION = {"NONE", "CW","CCW"};
    public static final String[] DELAY_TIME_PARKING_STORAGE_FROM_ENDING = {"0 sec", "5 sec", "8 sec", "10 sec"};
    public static final String[] DELAY_TIME_PARKING_WAREHOUSE_FROM_ENDING = {"0 sec", "5 sec", "8 sec", "10 sec"};
    public static final String[] PARKING_LOCATION = {"STORAGE", "WALL", "CENTRAL"};
    private static final String NONE = "none";

    public static Map<String, String[]> prefMap = new HashMap<>();
    private static String[] prefKeys = {START_POS_MODES_PREF, START_DELAY_PREF, PARK_ONLY_PREF,DELIVER_TO_HUB_USING_OPENCV_PREF, DUCK_DELIVER_TO_HUB_BY_SIDE_PREF, FREIGHT_DELIVERY_COUNT_PREF,DUCK_PARKING_DIRECTION_PREF, DELAY_PARKING_STORAGE_PREF, DELAY_PARKING_WAREHOUSE_PREF, PARKING_PREF};
    private static int keyIdx = 0;

    //private static String[] prefKeys = prefMap.keySet().toArray(new String[prefMap.keySet().size()]);

    static {
        // ADD entries to preference map here
        prefMap.put(START_DELAY_PREF, START_DELAY);
        prefMap.put(START_POS_MODES_PREF, START_POS_MODES);
        prefMap.put(PARKING_PREF, PARKING_LOCATION);
        prefMap.put(DELAY_PARKING_STORAGE_PREF, DELAY_TIME_PARKING_STORAGE_FROM_ENDING);
        prefMap.put(DELAY_PARKING_WAREHOUSE_PREF, DELAY_TIME_PARKING_WAREHOUSE_FROM_ENDING);
        prefMap.put(PARK_ONLY_PREF, PARK_ONLY);
        prefMap.put(FREIGHT_DELIVERY_COUNT_PREF,FREIGHT_DELIVERY_COUNT);
        prefMap.put(DUCK_PARKING_DIRECTION_PREF, DUCK_PARKING_DIRECTION);
        prefMap.put(DELIVER_TO_HUB_USING_OPENCV_PREF, DELIVER_TO_HUB_USING_OPENCV);
        prefMap.put(DUCK_DELIVER_TO_HUB_BY_SIDE_PREF, DUCK_DELIVER_TO_HUB_BY_SIDE);
    }

//    static {
//        Arrays.sort(prefKeys);
//    }

    public boolean isUpPressed;
    public boolean isDownPressed;
    public boolean isRightPressed;
    public boolean isLeftPressed;
    private SharedPreferences prefs;
    private SharedPreferences.Editor editor;
    private int selectionIdx = 0;

    public static SharedPreferences getSharedPrefs(HardwareMap hardwareMap) {
        return hardwareMap.appContext.getSharedPreferences("autonomous", 0);
    }

    private int getIndex(String val, String[] array) {
        if (array!=null) {
            for (int i = 0; i < array.length; i++) {
                if (array[i].equals(val)) {
                    return i;
                }
            }
        }
        return -1;
    }

    @Override
    public void init() {
        prefs = getSharedPrefs(hardwareMap);
        editor = prefs.edit();
        editor.apply();
        for (int i = 0; i < prefKeys.length; i++ ) {
            telemetry.addData(prefKeys[i], prefs.getString(prefKeys[i], NONE));
        }
    }

    @Override
    public void loop() {
        displayAll();
    }

    private void displayAll () {
        telemetry.clear();
//        telemetry.addData("Choose", "X - accept | Y - change");
//        for (String key : prefs.getAll().keySet()) {
//            telemetry.addData(key, prefs.getString(key, NONE));
//            if(keyIdx == )
//        }

        for (int i = 0; i < prefKeys.length; i++ ) {
            if (keyIdx == i) {
                String cap = prefKeys[i].toUpperCase();
                telemetry.addData("*" + cap + "*", prefs.getString(prefKeys[i], NONE));
            } else {
                telemetry.addData(prefKeys[i], prefs.getString(prefKeys[i], NONE));
            }
        }

        String key = prefKeys[keyIdx];
        String[] array = prefMap.get(key);

        if (key != null) {
            String prefValue = prefs.getString(key, NONE);
            selectionIdx = getIndex(prefValue, array);
            updateTelemetry(telemetry);
        }
        //accept and no change in current key, move to next key
        if (gamepad1.dpad_down && !isDownPressed) {
            int nextKeyIdx = keyIdx+1;

            if (nextKeyIdx >= prefKeys.length && gamepad1.dpad_down) {
                keyIdx = 0;
            } else {
                keyIdx = nextKeyIdx;
            }

            isDownPressed = true;
        }

        if (!gamepad1.dpad_down) {
            isDownPressed = false;
        }

        if (gamepad1.dpad_up && !isUpPressed) {
            int nextKeyIdx = keyIdx - 1;

            if (nextKeyIdx < 0 && gamepad1.dpad_up) {
                keyIdx = prefKeys.length - 1;
            } else {
                keyIdx = nextKeyIdx;
            }

            isUpPressed = true;
        }

        if (!gamepad1.dpad_up) {
            isUpPressed = false;
        }

        //change to next idx in array
        if (gamepad1.dpad_right && !isRightPressed) {
            selectionIdx++;  //value[] idx

            if (selectionIdx >= array.length && gamepad1.dpad_right) {
                selectionIdx = 0;
            }

            editor.putString(key, array[selectionIdx]);
            updateAutoPref(key, array[selectionIdx]);
            editor.apply();
            isRightPressed = true;
        }

        if (!gamepad1.dpad_right) {
            isRightPressed = false;
        }

        if (gamepad1.dpad_left && !isLeftPressed) {
            selectionIdx--;  //value[] idx

            if (selectionIdx < 0 && gamepad1.dpad_left) {
                selectionIdx = array.length-1;
            }

            editor.putString(key, array[selectionIdx]);
            updateAutoPref(key, array[selectionIdx]);
            editor.apply();
            isLeftPressed = true;
        }

        if (!gamepad1.dpad_left) {
            isLeftPressed = false;
        }
//        if (gamepad1.y && !isYPressed) {
//            isYPressed = true;
//            menuState = State.DisplaySingle;
//        }
//        if (!gamepad1.y) {
//            isYPressed = false;
//        }
    }
    void updateAutoPref(String key, String value) {
        /*
        if (key.equals(START_POS_MODES_PREF)) {
            if (value.equals("RED_5") || value.equals("BLUE_5")) {
                editor.putString(STONE_PREF, "no");
            }
            if(value.equals("RED_3") || value.equals("BLUE_3")){
                editor.putString(STONE_PREF, "group2");
            }
            if (value.equals("RED_2") || value.equals("BLUE_2")) {
                editor.putString(STONE_PREF, "group1"); // default
            }
        }
        if(key.equals(PARKING_ONLY_PREF) && value.equals("yes")) {
            editor.putString(STONE_PREF, "no");
            editor.putString(FOUNDATION_PREF, "no move");
        }
        if(key.equals(FOUNDATION_PREF) && value.equals("move only")) {
            editor.putString(STONE_PREF, "no");
        }
        */
    }
}


