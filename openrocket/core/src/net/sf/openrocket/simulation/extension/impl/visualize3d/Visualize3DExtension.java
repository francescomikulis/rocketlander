package net.sf.openrocket.simulation.extension.impl.visualize3d;

import net.sf.openrocket.l10n.L10N;
import net.sf.openrocket.simulation.SimulationConditions;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtension;
import net.sf.openrocket.unit.UnitGroup;

public class Visualize3DExtension extends AbstractSimulationExtension {
	@Override
	public void initialize(SimulationConditions conditions) throws SimulationException {
		conditions.getSimulationListenerList().add(new Visualize3DListener(this));
	}

	@Override
	public String getName() {
		String name;
		name="Visualize3DExtension"+"(rate={Visualize3DTimeRate}, IP={Visualize3DIP})";
		name = L10N.replace(name, "{Visualize3DTimeRate}", UnitGroup.UNITS_TIME_STEP.toStringUnit(getVisualize3DTimeRate()));
		name = L10N.replace(name, "{Visualize3DIP}", getVisualize3DIP());
		return name;
	}

	public double getVisualize3DTimeRate() {
		return config.getDouble("Visualize3DTimeRate", 1.0);
	}
	public void setVisualize3DTimeRate(double value) {
		config.put("Visualize3DTimeRate", value);
		fireChangeEvent();
	}

	public String getVisualize3DIP() { return config.getString("Visualize3DIP", "127.0.0.1:8080");}
	public void setVisualize3DIP(String ipString) {
		config.put("Visualize3DIP", ipString);
		fireChangeEvent();
	}
}
