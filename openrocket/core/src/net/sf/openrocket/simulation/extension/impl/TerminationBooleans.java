package net.sf.openrocket.simulation.extension.impl;

import java.util.ArrayList;

public class TerminationBooleans {
    private ArrayList<Boolean> successBooleans;
    public Boolean verticalSuccess;
    public Boolean angleSuccess;
    public TerminationBooleans(ArrayList<Boolean> successBooleans) {
        this.successBooleans = successBooleans;
        if ((successBooleans == null) || (this.successBooleans.size() == 0)) {
            verticalSuccess = true;
            angleSuccess = true;
        } else if (this.successBooleans.size() == 1) {
            verticalSuccess = this.successBooleans.get(0);
            angleSuccess = this.successBooleans.get(0);
        } else {
            verticalSuccess = this.successBooleans.get(0);
            angleSuccess = true;
            for (int i = 1; i < successBooleans.size(); i++) {
                angleSuccess = angleSuccess && successBooleans.get(i);
            }
        }
    }
    public boolean simulationFailed() {
        return !this.verticalSuccess || !this.angleSuccess;
    }
    public boolean totalSuccess() {
        boolean success = true;
        for (boolean bool: successBooleans) success = success && bool;
        return success;
    }
    public boolean landerSucceeded() {
        return this.verticalSuccess;
    }
    public boolean stabilizerSucceeded() {
        return this.angleSuccess;
    }
}
