package net.sf.openrocket.simulation.extension.impl;

public class TerminationBooleanTuple {
    public final Boolean verticalSuccess;
    public final Boolean angleSuccess;
    public TerminationBooleanTuple(Boolean verticalSuccess, Boolean angleSuccess) {
        this.verticalSuccess = verticalSuccess;
        this.angleSuccess = angleSuccess;
    }
    public boolean simulationFailed() {
        return !this.verticalSuccess || !this.angleSuccess;
    }
    public boolean landerSucceeded() {
        return this.verticalSuccess && this.angleSuccess;
    }
    public boolean stabilizerSucceeded() {
        return this.angleSuccess;
    }
}
