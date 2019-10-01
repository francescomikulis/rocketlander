package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.plugin.Plugin;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtensionProvider;

@Plugin
public class RocketLanderProvider extends AbstractSimulationExtensionProvider {

	public RocketLanderProvider() {
		super(RocketLander.class, "Launch conditions", "Rocket-lander");
	}
	
}
