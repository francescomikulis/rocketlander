package net.sf.openrocket.simulation.extension.impl.visualize3d;

import net.sf.openrocket.plugin.Plugin;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtensionProvider;

@Plugin
public class Visualize3DProvider extends AbstractSimulationExtensionProvider {

	public Visualize3DProvider() {
		super(Visualize3DExtension.class, "Visualize-3D Blender Extension", "Visualize-3D");
	}
	
}
