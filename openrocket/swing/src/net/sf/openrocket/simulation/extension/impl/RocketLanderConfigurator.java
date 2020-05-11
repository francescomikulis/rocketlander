package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.document.Simulation;
import net.sf.openrocket.plugin.Plugin;
import net.sf.openrocket.simulation.extension.impl.rocketlander.RocketLanderExtension;
import org.fife.ui.rsyntaxtextarea.RSyntaxTextArea;

import javax.swing.*;

@Plugin
public class RocketLanderConfigurator extends GenericInitialConditionsConfigurator<RocketLanderExtension> {
	private RSyntaxTextArea text;

	public RocketLanderConfigurator() {
		super(RocketLanderExtension.class);
	}

	@Override
	protected JComponent getConfigurationComponent(RocketLanderExtension extension, Simulation simulation, JPanel panel) {
		super.addInitialConditionsToComponent(extension, simulation, panel);
		return panel;
	}
}
