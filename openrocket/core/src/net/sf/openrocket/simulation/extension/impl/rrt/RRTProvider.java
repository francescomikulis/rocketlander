package net.sf.openrocket.simulation.extension.impl.rrt;

import net.sf.openrocket.plugin.Plugin;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtensionProvider;

@Plugin
public class RRTProvider extends AbstractSimulationExtensionProvider {

	public RRTProvider() {
		super(RRTExtension.class, "RRT Extension", "Rapidly Expanding Random Trees");
	}
	
}
