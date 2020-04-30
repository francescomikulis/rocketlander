package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.plugin.Plugin;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtensionProvider;

@Plugin
public class RRTProvider extends AbstractSimulationExtensionProvider {

	public RRTProvider() {
		super(RRTExtension.class, "Launch conditions", "RRT");
	}
	
}
