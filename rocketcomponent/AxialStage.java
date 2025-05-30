package info.openrocket.core.rocketcomponent;

import info.openrocket.core.l10n.Translator;
import info.openrocket.core.rocketcomponent.position.AxialMethod;
import info.openrocket.core.startup.Application;
import info.openrocket.core.util.Coordinate;

public class AxialStage extends ComponentAssembly implements FlightConfigurableComponent {
	
	private static final Translator trans = Application.getTranslator();

	/** list of separations to be happening */
	protected FlightConfigurableParameterSet<StageSeparationConfiguration> separations;
	/** number of stages */
	protected int stageNumber;
	
	/**
	 * default constructor, builds a rocket with zero stages
	 */
	public AxialStage() {
		this.separations = new FlightConfigurableParameterSet<>(
				new StageSeparationConfiguration());
		this.axialMethod = AxialMethod.AFTER;
		this.stageNumber = 0;
	}
	
	/**
	 * {@inheritDoc}
	 * AxialStage will always accept children
	 */
	@Override
	public boolean allowsChildren() {
		return true;
	}
	
	@Override
	public String getComponentName() {
		//// Stage
		return trans.get("Stage.Stage");
	}
	
	/**
	 * gets the separation configuration of the rocket
	 *
	 * @return the separation configuration of the rocket
	 */
	public FlightConfigurableParameterSet<StageSeparationConfiguration> getSeparationConfigurations() {
		return separations;
	}
	
	@Override
	public void reset(final FlightConfigurationId fcid) {
		separations.reset(fcid);
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

	/**
	 * Returns whether the current stage is active in the currently selected
	 * configuration.
	 *
	 * @return true if the stage is active, false if not
	 */
	public boolean isStageActive() {
		return getRocket().getSelectedConfiguration().isStageActive(getStageNumber());
	}

	/**
	 * Returns whether the current stage is active in the flight configuration.
	 * @param fc the flight configuration to check
	 * @return true if the stage is active, false if not
	 */
	public boolean isStageActive(FlightConfiguration fc) {
		return fc.isStageActive(getStageNumber());
	}
	
	@Override
	public void copyFlightConfiguration(FlightConfigurationId oldConfigId, FlightConfigurationId newConfigId) {
		separations.copyFlightConfiguration(oldConfigId, newConfigId);
	}
	
	@Override
	protected RocketComponent copyWithOriginalID() {
		AxialStage copy = (AxialStage) super.copyWithOriginalID();
		copy.separations = new FlightConfigurableParameterSet<>(separations);
		return copy;
	}

	/**
	 * Stages may be positioned relative to other stages. In that case, this will
	 * set the stage number
	 * against which this stage is positioned.
	 * 
	 * @return the stage number which this stage is positioned relative to
	 */
	public int getRelativeToStage() {
		if (null == this.parent) {
			return -1;
		} else if (1 == this.getInstanceCount()) {
			return --this.stageNumber;
		} else {
			return this.parent.getStageNumber();
		}
	}

	@Override
	public int getStageNumber() {
		return this.stageNumber;
	}
	
	/**
	 * {@inheritDoc}
	 * axialStage is always after
	 */
	@Override
	public boolean isAfter() {
		return true;
	}

	/**
	 * returns if the object is a launch stage
	 *
	 * @param config the flight configuration which will check which stages are
	 *               active
	 * @return if the object is a launch stage
	 */
	public boolean isLaunchStage(FlightConfiguration config) {
		return (getRocket().getBottomCoreStage(config).equals(this));
	}

	/**
	 * sets the stage number
	 *
	 * @param newStageNumber
	 */
	public void setStageNumber(final int newStageNumber) {
		this.stageNumber = newStageNumber;
	}

	@Override
	protected StringBuilder toDebugDetail() {
		StringBuilder buf = super.toDebugDetail();
		// if (-1 == this.getRelativeToStage()) {
		// System.err.println(" >>refStageName: " + null + "\n");
		// } else {
		// Stage refStage = (Stage) this.parent;
		// System.err.println(" >>refStageName: " + refStage.getName() + "\n");
		// System.err.println(" ..refCenterX: " + refStage.position.x + "\n");
		// System.err.println(" ..refLength: " + refStage.getLengthAerodynamic() +
		// "\n");
		// }
		return buf;
	}

	/**
	 * method used for debugging separation
	 *
	 * @return a string that represents the debug message of separation
	 */
	public String toDebugSeparation() {
		StringBuilder buff = new StringBuilder();
		buff.append(this.separations.toDebug());
		return buff.toString();
	}

	/**
	 * gets the previous stage installed in the rockets
	 * returns null if this is the first stage
	 *
	 * @return the previous stage in the rocket
	 */
	public AxialStage getUpperStage() {
		if (this.parent == null) {
			return null;
		} else if (Rocket.class.isAssignableFrom(this.parent.getClass())) {
			final int thisIndex = parent.getChildPosition(this);
			if (thisIndex > 0) {
				return (AxialStage) parent.getChild(thisIndex - 1);
			}
		} else {
			return this.parent.getStage();
		}
		return null;
	}
	
	@Override
	public void toDebugTreeNode(final StringBuilder buffer, final String indent) {
		
		Coordinate[] relCoords = this.getInstanceOffsets();
		Coordinate[] absCoords = this.getComponentLocations();
		if( 1 == getInstanceCount()){
			buffer.append(String.format("%-40s|  %5.3f; %24s; %24s;", indent+this.getName()+" (# "+this.getStageNumber()+")", 
					this.getLength(), this.getPosition(), this.getComponentLocations()[0]));
			buffer.append(String.format("len: %6.4f )(offset: %4.1f  via: %s )\n", this.getLength(), this.getAxialOffset(), this.axialMethod.name() ));
		}else{
			buffer.append(String.format("%-40s|(len: %6.4f )(offset: %4.1f via: %s)\n", (indent+this.getName()+"(# "+this.getStageNumber()+")"), this.getLength(), this.getAxialOffset(), this.axialMethod.name() ));
			for (int instanceNumber = 0; instanceNumber < this.getInstanceCount(); instanceNumber++) {
				Coordinate instanceRelativePosition = relCoords[instanceNumber];
				Coordinate instanceAbsolutePosition = absCoords[instanceNumber];
				final String prefix = String.format("%s    [%2d/%2d]", indent, instanceNumber+1, getInstanceCount()); 
				buffer.append(String.format("%-40s|  %5.3f; %24s; %24s;\n", prefix, this.getLength(), instanceRelativePosition, instanceAbsolutePosition));
			}
		}
		
	}

	public StageSeparationConfiguration getSeparationConfiguration() {
		FlightConfiguration flConfig = getRocket().getSelectedConfiguration();
		StageSeparationConfiguration sepConfig = getSeparationConfigurations().get(flConfig.getId());
		// To ensure the configuration is distinct, and we're not modifying the default
		if ((sepConfig == getSeparationConfigurations().getDefault())
				&& (flConfig.getId() != FlightConfigurationId.DEFAULT_VALUE_FCID)) {
			sepConfig = sepConfig.copy(flConfig.getId());
			getSeparationConfigurations().set(flConfig.getId(), sepConfig);
		}
		return sepConfig;
	}

	@Override
	public boolean addConfigListener(RocketComponent listener) {
		boolean success = super.addConfigListener(listener);
		if (listener instanceof AxialStage) {
			StageSeparationConfiguration thisConfig = getSeparationConfiguration();
			StageSeparationConfiguration listenerConfig = ((AxialStage) listener).getSeparationConfiguration();
			success = success && thisConfig.addConfigListener(listenerConfig);
			return success;
		}
		return false;
	}

	@Override
	public void removeConfigListener(RocketComponent listener) {
		super.removeConfigListener(listener);
		if (listener instanceof AxialStage) {
			StageSeparationConfiguration thisConfig = getSeparationConfiguration();
			StageSeparationConfiguration listenerConfig = ((AxialStage) listener).getSeparationConfiguration();
			thisConfig.removeConfigListener(listenerConfig);
		}
	}

	@Override
	public void clearConfigListeners() {
		super.clearConfigListeners();
		// StageSeparationConfiguration also has config listeners, so clear them as well
		if (getRoot() instanceof Rocket) {		// Root can be different from the rocket if this stage (or its parent) has been removed from the rocket
			StageSeparationConfiguration thisConfig = getSeparationConfiguration();
			thisConfig.clearConfigListeners();
		}
	}
}
