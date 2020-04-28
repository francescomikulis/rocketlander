package net.sf.openrocket.file.openrocket.importt;

import com.google.inject.Key;
import net.sf.openrocket.aerodynamics.WarningSet;
import net.sf.openrocket.document.Definition;
import net.sf.openrocket.document.OpenRocketDocument;
import net.sf.openrocket.document.Simulation;
import net.sf.openrocket.document.Simulation.Status;
import net.sf.openrocket.file.DocumentLoadingContext;
import net.sf.openrocket.file.simplesax.AbstractElementHandler;
import net.sf.openrocket.file.simplesax.ElementHandler;
import net.sf.openrocket.file.simplesax.PlainTextHandler;
import net.sf.openrocket.rocketcomponent.FlightConfigurationId;
import net.sf.openrocket.simulation.FlightData;
import net.sf.openrocket.simulation.SimulationOptions;
import net.sf.openrocket.simulation.extension.SimulationExtension;
import net.sf.openrocket.simulation.extension.SimulationExtensionProvider;
import net.sf.openrocket.simulation.extension.impl.JavaCode;
import net.sf.openrocket.startup.Application;
import net.sf.openrocket.util.StringUtil;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;

class SingleDefinitionHandler extends AbstractElementHandler {

	private final DocumentLoadingContext context;

	private final OpenRocketDocument doc;

	private String name;
	private String data;
	private boolean ignore = false;

	public SingleDefinitionHandler(OpenRocketDocument doc, DocumentLoadingContext context) {
		this.doc = doc;
		this.context = context;
	}

	public OpenRocketDocument getDocument() {
		return doc;
	}

	@Override
	public ElementHandler openElement(String element, HashMap<String, String> attributes,
			WarningSet warnings) {

		if (element.equals("name") || element.equals("data") || element.equals("ignore")) {
			return PlainTextHandler.INSTANCE;
		} else {
			warnings.add("Unknown element '" + element + "', ignoring.");
			return null;
		}
	}

	@Override
	public void closeElement(String element, HashMap<String, String> attributes,
			String content, WarningSet warnings) {

		if (element.equals("name")) {
			name = content;
		} else if (element.equals("data")) {
			data = content;
		} else if (element.equals("ignore")) {
			ignore = !content.equals("0");
		} else {
			warnings.add("Unable to parse the definition '" + content + "'.");
		}

	}

	@Override
	public void endHandler(String element, HashMap<String, String> attributes,
			String content, WarningSet warnings) {
		
		if (name == null)
			name = "Definition";
		if (data == null)
			data = "";

		
		Definition definition = new Definition(name, data);
		definition.setIgnore(ignore);
		
		doc.definitions.add(definition);
	}
	
	
	private SimulationExtension compatibilityExtension(String className) {
		JavaCode extension = Application.getInjector().getInstance(JavaCode.class);
		extension.setClassName(className);
		return extension;
	}
	
}