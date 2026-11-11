## ADDED Requirements

### Requirement: The feet group
The Aecert source SHALL include the feet group: Foot Left, Foot Right, Foot Shaft and Foot Tip, each built from named dimensions in its STL's coordinates, passing the gates and exported as STEP. The serpentine flexure in Foot Left and Foot Right SHALL be modelled from named dimensions of its path and beam widths, shared by both feet, so that changing one and rebuilding changes the spring of both. Foot Right SHALL be built from Foot Left's model, not modelled separately.

#### Scenario: Feet group built
- **WHEN** the build is run
- **THEN** it checks and exports Foot Left, Foot Right, Foot Shaft and Foot Tip, each against its STL in `hardware/aecert-hexapod-v1/stl/`, and each STEP file imports as one closed solid in FreeCAD and solidcraft

#### Scenario: The flexure is retuned
- **WHEN** a named dimension of the flexure's path or beam widths is changed and the build is run in check mode
- **THEN** Foot Left and Foot Right are both rebuilt with the changed flexure, and the gates report each one's deviation from its STL

#### Scenario: A foot feature cannot be matched
- **WHEN** a foot part's STL chord error exceeds 0.15 mm, or a foot part has a freeform feature that feature-based modelling cannot match
- **THEN** that part declares a per-part exception naming the feature and its size, the source README records it with the amount over the global limit, and every other part keeps the global limits
