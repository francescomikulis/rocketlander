package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.document.Simulation;
import net.sf.openrocket.gui.SpinnerEditor;
import net.sf.openrocket.gui.adaptors.DoubleModel;
import net.sf.openrocket.plugin.Plugin;
import net.sf.openrocket.simulation.extension.AbstractSwingSimulationExtensionConfigurator;
import net.sf.openrocket.simulation.extension.impl.visualize3d.Visualize3D;
import net.sf.openrocket.unit.UnitGroup;
import org.fife.ui.rsyntaxtextarea.RSyntaxTextArea;
import org.fife.ui.rsyntaxtextarea.SyntaxConstants;
import org.fife.ui.rtextarea.RTextScrollPane;

import javax.swing.*;
import java.awt.event.FocusEvent;
import java.awt.event.FocusListener;

@Plugin
public class Visualize3DConfigurator extends AbstractSwingSimulationExtensionConfigurator<Visualize3D> {
	private RSyntaxTextArea text;

	public Visualize3DConfigurator() {
		super(Visualize3D.class);
	}
	
	@Override
	protected JComponent getConfigurationComponent(Visualize3D extension, Simulation simulation, JPanel panel) {
		panel.add(new JLabel("Real time rate:"));
		DoubleModel m = new DoubleModel(extension, "Visualize3DTimeRate", UnitGroup.UNITS_NONE);
		JSpinner spin = new JSpinner(m.getSpinnerModel());
		spin.setEditor(new SpinnerEditor(spin));
		panel.add(spin, "w 100lp, wrap");

		panel.add(new JLabel("IP Address (XXX.XXX.XXX.XXX:PORT)"), "w 100, wrap");
		text = new RSyntaxTextArea("", 1, 40);
		text.setSyntaxEditingStyle(SyntaxConstants.SYNTAX_STYLE_JAVASCRIPT);
		text.setCodeFoldingEnabled(true);
		text.setLineWrap(true);
		text.setWrapStyleWord(true);
		text.setEditable(true);
		text.setText(extension.getVisualize3DIP());
		text.addFocusListener(new FocusListener() {
			@Override
			public void focusGained(FocusEvent event) { }
			@Override
			public void focusLost(FocusEvent event) {
				String str = text.getText();
				if (!extension.getVisualize3DIP().equals(str)) {
					extension.setVisualize3DIP(str);
				}
			}
		});
		JScrollPane scroll = new JScrollPane(text);
		panel.add(scroll, "spanx, grow, wrap para");
		return panel;
	}
	
}
