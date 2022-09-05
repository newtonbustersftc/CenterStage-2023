package org.firstinspires.ftc.teamcode;

public class DriverOptions {
    private int startDelay;
    private String startingPositionModes;
    private String parking;
    private int delayParkingByStorageFromEnding;
    private int delayParkingByWarehouseFromEnding;
    private int alliance;
    private String parkingOnly;
    private int freightDeliveryCount;
    private boolean isDuckParkingCCW;
    private boolean deliverToHubUsingOpencv;
    private boolean duckDeliverToHubBySideRoute;


    public boolean isDuckDeliverToHubBySideRoute() {
        return duckDeliverToHubBySideRoute;
    }

    public void setDuckDeliverToHubBySideRoute(boolean deliverToHubBySideRoute) {
        this.duckDeliverToHubBySideRoute = deliverToHubBySideRoute;
    }

    public boolean isDeliverToHubUsingOpencv() {
        return deliverToHubUsingOpencv;
    }

    public void setDeliverToHubUsingOpencv(boolean deliverToHubUsingOpencv) {
        this.deliverToHubUsingOpencv = deliverToHubUsingOpencv;
    }


    public boolean isDuckParkingCCW() {
        return isDuckParkingCCW;
    }

    public void setDuckParkingDirection(boolean duckParkingCCW) {
        this.isDuckParkingCCW = duckParkingCCW;
    }



    public String getParkingOnly() {
        return parkingOnly;
    }

    public void setParkingOnly(String parkingOnly) {
        this.parkingOnly = parkingOnly;
    }

    public int getFreightDeliveryCount() { return freightDeliveryCount; }

    public void setFreightDeliveryCount(int freightDeliveryCount) {
        this.freightDeliveryCount = freightDeliveryCount;
    }

    public String getStartingPositionModes() {
        return startingPositionModes;
    }

    public void setStartingPositionModes(String startingPositionModes) {
        this.startingPositionModes = startingPositionModes;
        if(getStartingPositionModes().contains("RED"))
            this.alliance = 1;
        else
            this.alliance = -1;
    }

    public String getParking() {
        return parking;
    }

    public void setParking(String parking) {
        this.parking = parking;
    }

    public int getStartDelay() {
        return startDelay;
    }

    public void setStartDelay(int delay) {
        this.startDelay = delay;
    }

    public int getAlliance(){
        return alliance;
    }

    public int getDelayParkingByStorageFromEnding() {
        return delayParkingByStorageFromEnding;
    }

    public void setDelayParkingByStorageFromEnding(int delayParkingByStorageFromEnding) {
        this.delayParkingByStorageFromEnding = delayParkingByStorageFromEnding;
    }

    public int getDelayParkingByWarehouseFromEnding() {
        return delayParkingByWarehouseFromEnding;
    }

    public void setDelayParkingByWarehouseFromEnding(int delayParkingByWarehouseFromEnding) {
        this.delayParkingByWarehouseFromEnding = delayParkingByWarehouseFromEnding;
    }
}
