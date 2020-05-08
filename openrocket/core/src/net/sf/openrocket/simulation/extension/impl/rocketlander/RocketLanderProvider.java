package net.sf.openrocket.simulation.extension.impl.rocketlander;

import net.sf.openrocket.plugin.Plugin;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtensionProvider;

@Plugin
public class RocketLanderProvider extends AbstractSimulationExtensionProvider {

	public RocketLanderProvider() {
		super(RocketLanderExtension.class, "Rocket Lander Extension", "Rocket-lander");
	}
	
}
