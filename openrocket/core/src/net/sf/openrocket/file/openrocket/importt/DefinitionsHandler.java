package net.sf.openrocket.file.openrocket.importt;

import net.sf.openrocket.aerodynamics.WarningSet;
import net.sf.openrocket.document.OpenRocketDocument;
import net.sf.openrocket.file.DocumentLoadingContext;
import net.sf.openrocket.file.simplesax.AbstractElementHandler;
import net.sf.openrocket.file.simplesax.ElementHandler;
import net.sf.openrocket.simulation.customexpression.CustomExpression;
import org.xml.sax.SAXException;

import java.util.HashMap;

class DefinitionsHandler extends AbstractElementHandler {
	private final DocumentLoadingContext context;
	private final OpenRocketDocument doc;
	private SingleDefinitionHandler handler;

	public DefinitionsHandler(OpenRocketDocument doc, DocumentLoadingContext context) {
		this.doc = doc;
		this.context = context;
	}
	
	@Override
	public ElementHandler openElement(String element, HashMap<String, String> attributes,
			WarningSet warnings) {
		
		if (!element.equals("definition")) {
			warnings.add("Unknown element '" + element + "', ignoring.");
			return null;
		}
		
		handler = new SingleDefinitionHandler(doc, context);
		return handler;
	}
	
	@Override
	public void closeElement(String element, HashMap<String, String> attributes,
			String content, WarningSet warnings) throws SAXException {
		super.closeElement(element, attributes, content, warnings);
	}
}