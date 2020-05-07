package net.sf.openrocket.simulation.extension.impl.visualize3d;

import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;

public abstract class AbstractSimulationListenerSupportsVisualize3DListener extends AbstractSimulationListener {
    public abstract double getMaxMotorPower();
    public abstract double getLastThrust();
    public abstract double getLastGimbalX();
    public abstract double getLastGimbalY();
    public abstract double getLastLateralThrustX();
    public abstract double getLastLateralThrustY();
}
