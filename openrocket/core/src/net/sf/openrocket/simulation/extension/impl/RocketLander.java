package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.l10n.L10N;
import net.sf.openrocket.simulation.SimulationConditions;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtension;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.unit.UnitGroup;
import net.sf.openrocket.util.Coordinate;

import java.util.HashMap;

import static net.sf.openrocket.startup.Preferences.WIND_AVERAGE;

public class RocketLander extends AbstractSimulationExtension {
	
	@Override
	public void initialize(SimulationConditions conditions) throws SimulationException {
		conditions.getSimulationListenerList().add(new RocketLanderListener());
	}
	
	@Override
	public String getName() {
		String name;
		if (getLaunchVelocity() != 0.0) {
			name = trans.get("SimulationExtension.rocketlander.name.altvel");
		} else {
			name = trans.get("SimulationExtension.rocketlander.name.alt");
		}
		name = L10N.replace(name, "{alt}", UnitGroup.UNITS_DISTANCE.toStringUnit(getLaunchAltitude()));
		name = L10N.replace(name, "{vel}", UnitGroup.UNITS_VELOCITY.toStringUnit(getLaunchVelocity()));
		return name;
	}
	
	public double getLaunchAltitude() {
		return config.getDouble("launchAltitude", 0.0);
	}
	
	public void setLaunchAltitude(double launchAltitude) {
		config.put("launchAltitude", launchAltitude);
		fireChangeEvent();
	}
	
	public double getLaunchVelocity() {
		return config.getDouble("launchVelocity", 0.0);
	}

	public void setLaunchVelocity(double launchVelocity) {
		config.put("launchVelocity", launchVelocity);
		fireChangeEvent();
	}
	
	public void setWindAverage(double windAverage) {
		config.put(WIND_AVERAGE, windAverage);
		fireChangeEvent();
	}

	public double getWindSpeed() {
		return config.getDouble(WIND_AVERAGE, 0.0);
	}
	
	
	private class RocketLanderListener extends AbstractSimulationListener {
		@Override
		public void startSimulation(SimulationStatus status) throws SimulationException {
			status.setRocketPosition(new Coordinate(0, 0, getLaunchAltitude()));
			status.setRocketVelocity(status.getRocketOrientationQuaternion().rotate(new Coordinate(0, 0, getLaunchVelocity())));
		}
		@Override
		public void postStep(SimulationStatus status) throws SimulationException {
			HashMap<String, String> data = new HashMap<>();
			data.put("position", status.getRocketPosition().toString());
			data.put("velocity", status.getRocketVelocity().toString());
			data.put("quaternion", status.getRocketOrientationQuaternion().toString());
			data.put("rotationVelocity", status.getRocketRotationVelocity().toString());
			// System.out.println(data);
			status.getFlightData();
		}
	}
}
