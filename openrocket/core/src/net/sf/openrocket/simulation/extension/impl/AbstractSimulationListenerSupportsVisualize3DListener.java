package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;

public abstract class AbstractSimulationListenerSupportsVisualize3DListener extends AbstractSimulationListener {
    abstract double getMaxMotorPower();
    abstract double getLastThrust();
    abstract double getLastGimbalX();
    abstract double getLastGimbalY();
    abstract double getLastLateralThrustX();
    abstract double getLastLateralThrustY();
}
