package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.plugin.Plugin;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtensionProvider;

@Plugin
public class Visualize3DProvider extends AbstractSimulationExtensionProvider {

	public Visualize3DProvider() {
		super(Visualize3D.class, "Launch conditions", "Visualize-3D");
	}
	
}
