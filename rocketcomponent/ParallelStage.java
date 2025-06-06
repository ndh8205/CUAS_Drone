package info.openrocket.core.rocketcomponent;

import java.util.ArrayList;
import java.util.Collection;

import info.openrocket.core.l10n.Translator;
import info.openrocket.core.rocketcomponent.position.AngleMethod;
import info.openrocket.core.rocketcomponent.position.AnglePositionable;
import info.openrocket.core.rocketcomponent.position.AxialMethod;
import info.openrocket.core.rocketcomponent.position.RadiusMethod;
import info.openrocket.core.rocketcomponent.position.RadiusPositionable;
import info.openrocket.core.startup.Application;
import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.MathUtil;

public class ParallelStage extends AxialStage implements FlightConfigurableComponent, RingInstanceable {
	
	private static final Translator trans = Application.getTranslator();
	//private static final Logger log = LoggerFactory.getLogger(BoosterSet.class);
	
	protected int instanceCount = 1;

	protected AngleMethod angleMethod = AngleMethod.RELATIVE;
	protected double angleSeparation = Math.PI;
	protected double angleOffset_rad = 0;
	
	protected RadiusMethod radiusMethod = RadiusMethod.RELATIVE;
	protected double radiusOffset_m = 0;
	
	public ParallelStage() {
		this.instanceCount = 2;
		this.axialMethod = AxialMethod.BOTTOM;
		this.angleSeparation = Math.PI * 2 / this.instanceCount;
	}

	public ParallelStage(final int _count) {
		this();
		
		this.instanceCount = _count;
		this.angleSeparation = Math.PI * 2 / this.instanceCount;
	}
	
	@Override
	public String getComponentName() {
		//// Stage
		return trans.get("BoosterSet.BoosterSet");
	}
	
	// not strictly accurate, but this should provide an acceptable estimate for total vehicle size 
	@Override
	public Collection<Coordinate> getComponentBounds() {
		Collection<Coordinate> bounds = new ArrayList<>(8);
		double x_min = Double.MAX_VALUE;
		double x_max = Double.MIN_VALUE;
		double r_max = 0;
		
		Coordinate[] instanceLocations = this.getComponentLocations();
		
		for (Coordinate currentInstanceLocation : instanceLocations) {
			if (x_min > (currentInstanceLocation.x)) {
				x_min = currentInstanceLocation.x;
			}
			if (x_max < (currentInstanceLocation.x + this.length)) {
				x_max = currentInstanceLocation.x + this.length;
			}
			if (r_max < (this.getRadiusOffset())) {
				r_max = this.getRadiusOffset();
			}
		}
		addBound(bounds, x_min, r_max);
		addBound(bounds, x_max, r_max);
		
		return bounds;
	}
	
	/**
	 * Check whether the given type can be added to this component.  A Stage allows
	 * only BodyComponents to be added.
	 *
	 * @param type The RocketComponent class type to add.
	 *
	 * @return Whether such a component can be added.
	 */
	@Override
	public boolean isCompatible(Class<? extends RocketComponent> type) {
		return BodyComponent.class.isAssignableFrom(type);
	}
	
	@Override
	public void copyFlightConfiguration(FlightConfigurationId oldConfigId, FlightConfigurationId newConfigId) {
		this.separations.copyFlightConfiguration(oldConfigId, newConfigId);
	}
	
	@Override
	protected RocketComponent copyWithOriginalID() {
		ParallelStage copy = (ParallelStage) (super.copyWithOriginalID());
		return copy;
	}

	@Override
	public double getAngleOffset() {
		return this.angleOffset_rad;
	}

	@Override
	public int getInstanceCount() {
		return this.instanceCount;
	}
	
	@Override
	public boolean isAfter() {
		return false;
	}
	
	@Override
	public boolean isLaunchStage(FlightConfiguration config) {
		return config.isStageActive(this.stageNumber);
	}

	@Override
	public void setInstanceCount(final int newCount) {
		for (RocketComponent listener : configListeners) {
			listener.setInstanceCount(newCount);
		}

		mutex.verify();
		if ( newCount < 1) {
			// there must be at least one instance....   
			return;
		}
		
        this.instanceCount = newCount;
        this.angleSeparation = Math.PI * 2 / this.instanceCount;
        fireComponentChangeEvent(ComponentChangeEvent.BOTH_CHANGE);
	}
	
	@Override
	public double getRadiusOffset() {
		return this.radiusOffset_m;
	}

	@Override
	public double[] getInstanceAngles() {
		final double baseAngle = getAngleOffset();
		final double incrAngle = getInstanceAngleIncrement();

		double[] result = new double[getInstanceCount()];
		for (int i = 0; i < getInstanceCount(); ++i) {
			result[i] = baseAngle + incrAngle * i;
		}
		
		return result;
	}

	@Override
	public double getInstanceAngleIncrement() {
		return this.angleSeparation;
	}

	@Override
	public Coordinate[] getInstanceOffsets() {
		checkState();

		final double radius = this.radiusMethod.getRadius(this.parent, this, radiusOffset_m);

		Coordinate[] toReturn = new Coordinate[this.instanceCount];
		final double[] angles = getInstanceAngles();
		for (int instanceNumber = 0; instanceNumber < this.instanceCount; instanceNumber++) {
			final double curY = radius * Math.cos(angles[instanceNumber]);
			final double curZ = radius * Math.sin(angles[instanceNumber]);
			toReturn[instanceNumber] = new Coordinate(0, curY, curZ);
		}
		
		return toReturn;
	}
	
	@Override
	public String getPatternName() {
		return (this.getInstanceCount() + "-ring");
	}
	
	@Override
	public void setAxialMethod(final AxialMethod _newPosition) {
		for (RocketComponent listener : configListeners) {
			listener.setAxialMethod(_newPosition);
		}

		if (null == this.parent) {
			throw new NullPointerException(" a Stage requires a parent before any positioning! ");
		}
		
		super.setAxialMethod(_newPosition);
		
		fireComponentChangeEvent(ComponentChangeEvent.NONFUNCTIONAL_CHANGE);
	}
	
	@Override
	public void setRadiusOffset(final double radius_m) {
		for (RocketComponent listener : configListeners) {
			if (listener instanceof RadiusPositionable) {
				((RadiusPositionable) listener).setRadiusOffset(radius_m);
			}
		}

		if (radius_m == this.radiusOffset_m)
			return;

		if (this.radiusMethod.clampToZero()) {
			this.radiusOffset_m = 0.0;
		} else {
			this.radiusOffset_m = radius_m;
		}
		fireComponentChangeEvent(ComponentChangeEvent.BOTH_CHANGE);
	}

	@Override
	public void setAngleOffset(final double angle_rad) {
		for (RocketComponent listener : configListeners) {
			if (listener instanceof AnglePositionable) {
				((AnglePositionable) listener).setAngleOffset(angle_rad);
			}
		}

		mutex.verify();
		this.angleOffset_rad = MathUtil.reducePi(angle_rad);
		fireComponentChangeEvent(ComponentChangeEvent.BOTH_CHANGE);
	}
		
	@Override
	public void setRadius(RadiusMethod requestedMethod, double requestedRadius) {
		for (RocketComponent listener : configListeners) {
			if (listener instanceof RadiusPositionable) {
				((RadiusPositionable) listener).setRadius(requestedMethod, requestedRadius);
			}
		}

		mutex.verify();

		double newRadius = requestedRadius;

		if (requestedMethod.clampToZero()) {
			newRadius = 0.0;
		}

		this.radiusMethod = requestedMethod;
		this.radiusOffset_m = this.radiusMethod.getAsOffset(getParent(), this, newRadius);

		fireComponentChangeEvent(ComponentChangeEvent.BOTH_CHANGE);
	}

	@Override
	public AngleMethod getAngleMethod() {
		return this.angleMethod;
	}

	@Override
	public void setAngleMethod(AngleMethod newAngleMethod) {
		for (RocketComponent listener : configListeners) {
			if (listener instanceof AnglePositionable) {
				((AnglePositionable) listener).setAngleMethod(newAngleMethod);
			}
		}

		mutex.verify();
		this.angleMethod = newAngleMethod;
		fireComponentChangeEvent(ComponentChangeEvent.BOTH_CHANGE);
	}
	
	@Override
	public RadiusMethod getRadiusMethod() {
		return this.radiusMethod;
	}

	@Override
	public void setRadiusMethod(RadiusMethod newMethod) {
		for (RocketComponent listener : configListeners) {
			if (listener instanceof RadiusPositionable) {
				((RadiusPositionable) listener).setRadiusMethod(newMethod);
			}
		}

		if (newMethod == this.radiusMethod)
			return;

		mutex.verify();
		double radius = this.radiusMethod.getRadius(getParent(), this, this.radiusOffset_m); // Radius from the parent's
																								// center
		setRadius(newMethod, radius);
	}

}
