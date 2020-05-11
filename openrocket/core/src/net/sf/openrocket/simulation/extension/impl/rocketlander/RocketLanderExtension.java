package net.sf.openrocket.simulation.extension.impl.rocketlander;

import net.sf.openrocket.simulation.SimulationConditions;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.impl.initialconditions.AbstractGenericInitialConditionsExtension;

public class RocketLanderExtension extends AbstractGenericInitialConditionsExtension {
	public String UNIQUE_PREFIX() { return "RL"; }

	@Override
	public void initialize(SimulationConditions conditions) throws SimulationException {
		conditions.getSimulationListenerList().add(new RocketLanderListener(this));
	}
	
	@Override
	public String getName() {
		String name;
		name="Rocket Lander (open to view conditions)";
		return name;
	}
}
