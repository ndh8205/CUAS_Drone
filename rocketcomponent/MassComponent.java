package info.openrocket.core.rocketcomponent;

import info.openrocket.core.l10n.Translator;
import info.openrocket.core.startup.Application;
import info.openrocket.core.util.MathUtil;

/**
 * This class represents a generic component that has a specific mass and an
 * approximate shape.
 * The mass is accessed via get/setComponentMass.
 * 
 * @author Sampo Niskanen <sampo.niskanen@iki.fi>
 */
public class MassComponent extends MassObject {
	private static final Translator trans = Application.getTranslator();

	private double mass = 0;

	public static enum MassComponentType {
		MASSCOMPONENT(Application.getTranslator().get("MassComponent.MassComponent")),
		ALTIMETER(Application.getTranslator().get("MassComponent.Altimeter")),
		FLIGHTCOMPUTER(Application.getTranslator().get("MassComponent.FlightComputer")),
		DEPLOYMENTCHARGE(Application.getTranslator().get("MassComponent.DeploymentCharge")),
		TRACKER(Application.getTranslator().get("MassComponent.Tracker")),
		PAYLOAD(Application.getTranslator().get("MassComponent.Payload")),
		RECOVERYHARDWARE(Application.getTranslator().get("MassComponent.RecoveryHardware")),
		BATTERY(Application.getTranslator().get("MassComponent.Battery"));

		private final String title;

		MassComponentType(String title) {
			this.title = title;
		}

		@Override
		public String toString() {
			return title;
		}
	}

	private MassComponentType massComponentType = MassComponentType.MASSCOMPONENT;

	public MassComponent() {
		super();
		super.displayOrder_side = 13; // Order for displaying the component in the 2D side view
		super.displayOrder_back = 10; // Order for displaying the component in the 2D back view
	}

	public MassComponent(double length, double radius, double mass) {
		super(length, radius);
		this.mass = mass;
		super.displayOrder_side = 13; // Order for displaying the component in the 2D side view
		super.displayOrder_back = 10; // Order for displaying the component in the 2D back view
	}

	@Override
	public double getComponentMass() {
		return mass;
	}

	public void setComponentMass(double mass) {
		for (RocketComponent listener : configListeners) {
			if (listener instanceof MassComponent) {
				((MassComponent) listener).setComponentMass(mass);
			}
		}

		mass = Math.max(mass, 0);
		if (MathUtil.equals(this.mass, mass))
			return;
		this.mass = mass;
		fireComponentChangeEvent(ComponentChangeEvent.MASS_CHANGE);
	}

	public double getDensity() {
		double d = getComponentMass() / getVolume();
		if (Double.isNaN(d))
			d = 0;
		return d;
	}

	public void setDensity(double density) {
		for (RocketComponent listener : configListeners) {
			if (listener instanceof MassComponent) {
				((MassComponent) listener).setDensity(density);
			}
		}

		double m = density * getVolume();
		m = MathUtil.clamp(m, 0, 1000000);
		if (Double.isNaN(m))
			m = 0;
		setComponentMass(m);
	}

	private double getVolume() {
		return Math.PI * MathUtil.pow2(getRadius()) * getLength();
	}

	@Override
	public String getComponentName() {
		//// Mass component
		return trans.get("MassComponent.MassComponent");
	}

	public final MassComponent.MassComponentType getMassComponentType() {
		mutex.verify();
		return this.massComponentType;
	}

	public void setMassComponentType(MassComponent.MassComponentType compType) {
		for (RocketComponent listener : configListeners) {
			if (listener instanceof MassComponent) {
				((MassComponent) listener).setMassComponentType(compType);
			}
		}

		mutex.verify();
		if (this.massComponentType == compType) {
			return;
		}
		checkState();
		this.massComponentType = compType;
		fireComponentChangeEvent(ComponentChangeEvent.NONFUNCTIONAL_CHANGE);
	}

	@Override
	public boolean allowsChildren() {
		return false;
	}

	@Override
	public boolean isCompatible(Class<? extends RocketComponent> type) {
		// Allow no components to be attached to a MassComponent
		return false;
	}
}
