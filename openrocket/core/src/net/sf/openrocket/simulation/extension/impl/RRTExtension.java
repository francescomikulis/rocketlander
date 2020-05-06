package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.l10n.L10N;
import net.sf.openrocket.simulation.SimulationConditions;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtension;
import net.sf.openrocket.unit.UnitGroup;

import static net.sf.openrocket.startup.Preferences.WIND_AVERAGE;

public class RRTExtension extends AbstractSimulationExtension {
	@Override
	public void initialize(SimulationConditions conditions) throws SimulationException {
		conditions.getSimulationListenerList().add(new RRTListener(this));
	}

	@Override
	public String getName() {
		String name;
		name="RRT"+"(UsingLateralObjective: {UsingLateralObjective})";
		name = L10N.replace(name, "{UsingLateralObjective}", String.valueOf(getUsingLateralObjective()));
		return name;
	}

	public boolean getUsingLateralObjective() {
		return config.getBoolean("UsingLateralObjective", false);
	}

	public void setUsingLateralObjective(boolean usingLateralObjectiveDouble) {
		config.put("UsingLateralObjective", usingLateralObjectiveDouble);
		fireChangeEvent();
	}
}
