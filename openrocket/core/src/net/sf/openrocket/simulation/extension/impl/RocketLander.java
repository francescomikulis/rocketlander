package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.document.Simulation;
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
import java.util.concurrent.Callable;
import java.util.function.BiFunction;
import java.util.function.Function;
import net.sf.openrocket.simulation.extension.impl.RLModel.*;

import static net.sf.openrocket.startup.Preferences.WIND_AVERAGE;

public class RocketLander extends AbstractSimulationExtension {
	private static RLEpisodeManagment episodeManagment = new RLEpisodeManagment();
	private static RLModel model = new RLModel(episodeManagment);
	
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
		HashMap<String, ArrayList<Double>> episode;

		@Override
		public void startSimulation(SimulationStatus status) throws SimulationException {
			episode = episodeManagment.initializeEmptyEpisode();
			episodeManagment.setupParameters(status);

			status.setRocketPosition(new Coordinate(0, 0, getLaunchAltitude()));
			status.setRocketVelocity(status.getRocketOrientationQuaternion().rotate(new Coordinate(0, 0, getLaunchVelocity())));

			System.out.println("CALLED START SIMULATION");
		}

		@Override
		public double preSimpleThrustCalculation(SimulationStatus status) throws SimulationException {
			// note we would want to also fix the fuel.  This ignores the fuel level of the rocket.

			//status.getRocketVelocity();
			//return 0.0;
			return Double.NaN;
		}

		@Override
		public boolean preStep(SimulationStatus status) throws SimulationException {
			Action action = model.run_policy(status);
			if (action.thrust == 0.0) {
				Coordinate currentVel = status.getRocketVelocity();

				// NOTE THIS IS THE THRUST ACCESS
				//status.getActiveMotors().iterator().next().getThrust(status.getSimulationTime());

				// Reduce the rocket velocity - THIS IS A HACK.

				//Coordinate reducedVel = new Coordinate(currentVel.x * 0.99, currentVel.y * 0.99, currentVel.z * 0.99);
				//status.setRocketVelocity(reducedVel);
			} else {
				// leave motor on.
			}
			return true;
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

			episodeManagment.addData(status, episode);
		}
		@Override
		public void endSimulation(SimulationStatus status, SimulationException exception) {
			episodeManagment.addEpisode(episode);
			System.out.println(episodeManagment.readLastTimeStep(episode));
		}
	}
}
