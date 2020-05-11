package net.sf.openrocket.simulation.extension.impl.initialconditions;

import net.sf.openrocket.simulation.extension.AbstractSimulationExtension;

import static net.sf.openrocket.simulation.extension.impl.rocketlander.MDPDefinition.cleanJsonStringByRemovingArraySpaces;

public abstract class AbstractGenericInitialConditionsExtension extends AbstractSimulationExtension {
	public abstract String UNIQUE_PREFIX();
	private static final InitialConditions defaultInitialConditions = getDefaultInitialConditions();
	private static final String defaultInitialConditionsString = InitialConditions.toJsonString(defaultInitialConditions);

	public void resetInitialConditionsToDefault() {
		setInitialConditions(defaultInitialConditionsString);
	}

	public void setInitialConditions(String initialConditionsString) {
		InitialConditions initialConditions = InitialConditions.buildFromJsonString(initialConditionsString);
		String convertedStringConditions = InitialConditions.toJsonString(initialConditions);
		convertedStringConditions = cleanJsonStringByRemovingArraySpaces(convertedStringConditions);
		config.put(UNIQUE_PREFIX() + "InitialConditions", convertedStringConditions);
		fireChangeEvent();
	}

	public String getInitialConditions() {
		String result = config.getString(UNIQUE_PREFIX() + "InitialConditions", defaultInitialConditionsString);
		return cleanJsonStringByRemovingArraySpaces(result);
	}

	public InitialConditions getInitialConditionsObject() {
		return InitialConditions.buildFromJsonString(
				(String)config.get(UNIQUE_PREFIX() + "InitialConditions", defaultInitialConditions)
		);
	}

	/** Default InitialConditions ***/

	public static InitialConditions getDefaultInitialConditions() {
		InitialConditions initialConditions = new InitialConditions();
		initialConditions.numDimensions = 3;
		initialConditions.symmetryAxis2D = "X";
		initialConditions.positionX = new double[]{-6, 6};
		initialConditions.positionY = new double[]{-6, 6};
		initialConditions.positionZ = new double[]{28, 32};
		initialConditions.velocityX = new double[]{-6, 6};
		initialConditions.velocityY = new double[]{-6, 6};
		initialConditions.velocityZ = new double[]{-10, -8};
		initialConditions.angleX = new double[]{-16, 16};
		initialConditions.angleY = new double[]{-16, 16};
		initialConditions.angleVelocityX = new double[]{-26, 26};
		initialConditions.angleVelocityY = new double[]{-26, 26};
		initialConditions.postConstructor();
		return initialConditions;
	}
}
