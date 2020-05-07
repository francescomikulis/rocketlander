package net.sf.openrocket.simulation.extension.impl.rocketlander;

import com.google.gson.reflect.TypeToken;
import net.sf.openrocket.l10n.L10N;
import net.sf.openrocket.simulation.SimulationConditions;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtension;
import net.sf.openrocket.unit.UnitGroup;


import java.lang.reflect.Type;
import java.util.LinkedHashMap;

import static net.sf.openrocket.simulation.extension.impl.rocketlander.MDPDefinition.cleanJsonStringByRemovingArraySpaces;
import static net.sf.openrocket.startup.Preferences.WIND_AVERAGE;

public class RocketLander extends AbstractSimulationExtension {
	private Type initialConditionsType = new TypeToken<LinkedHashMap<String, double[]>>(){}.getType();

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







	public void setRLInitialConditions(String RLInitialConditions) {
		Object hashMapObject = MDPDefinition.genericObjectBuildFromJsonString(RLInitialConditions, initialConditionsType);
		LinkedHashMap<String, double[]> initialConditions = (LinkedHashMap<String, double[]>) hashMapObject;
		convertInitialConditionAnglesToRadians(initialConditions);
		String convertedStringConditions = MDPDefinition.genericObjectToJsonString(initialConditions, initialConditionsType);
		convertedStringConditions = cleanJsonStringByRemovingArraySpaces(convertedStringConditions);
		config.put("RLInitialConditions", convertedStringConditions);
		fireChangeEvent();
	}

	public String getRLInitialConditions() {
		convertInitialConditionAnglesToRadians(defaultInitialConditions);
		String result = config.getString("RLInitialConditions", MDPDefinition.genericObjectToJsonString(defaultInitialConditions, initialConditionsType));
		return cleanJsonStringByRemovingArraySpaces(result);
	}

	private static final LinkedHashMap<String, double[]> defaultInitialConditions = new LinkedHashMap<String, double[]>(){{
		put("positionX", new double[]{-6, 6});
		put("positionY", new double[]{-6, 6});
		put("positionZ", new double[]{28, 32});
		put("velocityX", new double[]{-6, 6});
		put("velocityY", new double[]{-6, 6});
		put("velocityZ", new double[]{-12, -8});
		put("angleX", new double[]{-16, 16});
		put("angleY", new double[]{-16, 16});
		put("angleVelocityX", new double[]{-26, 26});
		put("angleVelocityY", new double[]{-26, 26});
	}};

	private static void convertInitialConditionAnglesToRadians(LinkedHashMap<String, double[]> initialConditions) {
		if (initialConditions == null) return;
		_convertInitialConditionAnglesToRadians(initialConditions, "angleX");
		_convertInitialConditionAnglesToRadians(initialConditions, "angleY");
		_convertInitialConditionAnglesToRadians(initialConditions, "angleVelocityX");
		_convertInitialConditionAnglesToRadians(initialConditions, "angleVelocityY");
	}

	private static void _convertInitialConditionAnglesToRadians(LinkedHashMap<String, double[]> initialConditions, String field) {
		double[] minMax = initialConditions.get(field);
		if (Math.abs(minMax[0]) > 1.0) {
			minMax[0] *= Math.PI / 180;
			minMax[1] *= Math.PI / 180;
		}
	}
}
