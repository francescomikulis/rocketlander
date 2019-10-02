package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.l10n.L10N;
import net.sf.openrocket.simulation.SimulationConditions;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtension;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.unit.UnitGroup;
import java.util.ArrayList;
import net.sf.openrocket.util.Coordinate;

import java.util.HashMap;

import static net.sf.openrocket.startup.Preferences.WIND_AVERAGE;

public class RocketLander extends AbstractSimulationExtension {
	private static ArrayList<HashMap<String, ArrayList<Double>>> episodes = null;
	MyObjectFileStore mof = new MyObjectFileStore();
	
	@Override
	public void initialize(SimulationConditions conditions) throws SimulationException {
		if (episodes == null) {
			try {
				ArrayList<HashMap<String, ArrayList<Double>>> fromFile = mof.readObjects();
				episodes = fromFile;
			} catch (Exception e) {
				episodes = new ArrayList<>();
			}
		}

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
		HashMap<String, ArrayList<Double>> episode;

		@Override
		public void startSimulation(SimulationStatus status) throws SimulationException {
			episode = new HashMap<>();
			episode.put("position_x", new ArrayList<Double>());
			episode.put("position_y", new ArrayList<Double>());
			episode.put("position_z", new ArrayList<Double>());
			episode.put("velocity_x", new ArrayList<Double>());
			episode.put("velocity_y", new ArrayList<Double>());
			episode.put("velocity_z", new ArrayList<Double>());
			episode.put("quat_x", new ArrayList<Double>());
			episode.put("quat_y", new ArrayList<Double>());
			episode.put("quat_z", new ArrayList<Double>());
			episode.put("quat_w", new ArrayList<Double>());
			episode.put("rotationV_x", new ArrayList<Double>());
			episode.put("rotationV_y", new ArrayList<Double>());
			episode.put("rotationV_z", new ArrayList<Double>());


			status.setRocketPosition(new Coordinate(0, 0, getLaunchAltitude()));
			status.setRocketVelocity(status.getRocketOrientationQuaternion().rotate(new Coordinate(0, 0, getLaunchVelocity())));

			System.out.println("CALLED START SIMULATION");
		}
		@Override
		public void postStep(SimulationStatus status) throws SimulationException {
			/*
			data.put("position", status.getRocketPosition().toString());
			data.put("velocity", status.getRocketVelocity().toString());
			data.put("quaternion", status.getRocketOrientationQuaternion().toString());
			data.put("rotationVelocity", status.getRocketRotationVelocity().toString());
			System.out.println(data);
			//status.getFlightData();
			*/
			episode.get("position_x").add(status.getRocketPosition().x);
			episode.get("position_y").add(status.getRocketPosition().y);
			episode.get("position_z").add(status.getRocketPosition().z);
			episode.get("velocity_x").add(status.getRocketVelocity().x);
			episode.get("velocity_y").add(status.getRocketVelocity().y);
			episode.get("velocity_z").add(status.getRocketVelocity().z);
			episode.get("quat_x").add(status.getRocketOrientationQuaternion().getX());
			episode.get("quat_y").add(status.getRocketOrientationQuaternion().getY());
			episode.get("quat_z").add(status.getRocketOrientationQuaternion().getZ());
			episode.get("quat_w").add(status.getRocketOrientationQuaternion().getW());
			episode.get("rotationV_x").add(status.getRocketRotationVelocity().x);
			episode.get("rotationV_y").add(status.getRocketRotationVelocity().y);
			episode.get("rotationV_z").add(status.getRocketRotationVelocity().z);

		}
		@Override
		public void endSimulation(SimulationStatus status, SimulationException exception) {
			episodes.add(episode);

			if (episodes.size() % 5 == 0) {
				mof.storeObject(episodes);
			}

			System.out.println("Sim number: " + episodes.size() + " " + status.getRocketVelocity().z);
		}
	}
}
