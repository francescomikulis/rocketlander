package net.sf.openrocket.simulation.extension.impl.visualize3d;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.simulation.listeners.SimulationListener;

import java.util.List;

public abstract class AbstractSimulationListenerSupportsVisualize3DListener extends AbstractSimulationListener {
    public abstract double getMaxMotorPower();
    public abstract double getLastThrust();
    public abstract double getLastGimbalX();
    public abstract double getLastGimbalY();
    public abstract double getLastLateralThrustX();
    public abstract double getLastLateralThrustY();

    public Visualize3DListener findVisualize3DListener(SimulationStatus status) {
        Visualize3DListener visualize3DListener = null;
        List<SimulationListener> listeners = status.getSimulationConditions().getSimulationListenerList();
        for (SimulationListener listener: listeners) {
            if (listener.getClass().toString().contains("Visualize3DListener")) {
                visualize3DListener = (Visualize3DListener) listener;
            }
        }
        return visualize3DListener;
    }

    public void disableVisualization(SimulationStatus status) {
        Visualize3DListener visualize3DListener = findVisualize3DListener(status);
        if (visualize3DListener == null)  return;
        visualize3DListener.setVisualizeDuringPostStep(false);
    }

    public Visualize3DListener activateVisualize3DListener(SimulationStatus status) {
        Visualize3DListener visualize3DListener = findVisualize3DListener(status);
        if (visualize3DListener == null)  return null;
        visualize3DListener.setListener(this);
        visualize3DListener.setVisualizeDuringPostStep(true);
        return visualize3DListener;
    }
}
