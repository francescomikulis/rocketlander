package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.l10n.L10N;
import net.sf.openrocket.simulation.SimulationConditions;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtension;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.unit.UnitGroup;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.util.Quaternion;
import net.sf.openrocket.simulation.SimulationOptions.*;

public class Visualize3D extends AbstractSimulationExtension {
	
	@Override
	public void initialize(SimulationConditions conditions) throws SimulationException {
		conditions.getSimulationListenerList().add(new Visualize3DListener(this));
	}

	@Override
	public String getName() {
		String name;
		//if (getRate() != 0.0) {
		//name = trans.get("SimulationExtension.visualize3d.name.rate");
		//}
		name="Visualize3D"+"({rate})";
		name = L10N.replace(name, "{rate}", UnitGroup.UNITS_TIME_STEP.toStringUnit(getTimeRate()));
		//name = UnitGroup.UNITS_TIME_STEP.toStringUnit(getRate());
		return name;
	}

	public double getTimeRate() {
		return config.getDouble("rate", 0.0);
	}

	public void setTimeRate(double rate) {
		config.put("rate", rate);
		fireChangeEvent();
	}
}
