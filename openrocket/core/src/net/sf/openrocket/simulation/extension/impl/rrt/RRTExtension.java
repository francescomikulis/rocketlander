package net.sf.openrocket.simulation.extension.impl.rrt;

import net.sf.openrocket.l10n.L10N;
import net.sf.openrocket.simulation.SimulationConditions;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.impl.initialconditions.AbstractGenericInitialConditionsExtension;

public class RRTExtension extends AbstractGenericInitialConditionsExtension {
	public String UNIQUE_PREFIX() { return "RRT"; }

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
