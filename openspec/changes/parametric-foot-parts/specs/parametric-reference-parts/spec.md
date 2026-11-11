## ADDED Requirements

### Requirement: The feet group
The Aecert source SHALL include the feet group: Foot Left, Foot Right, Foot Shaft and Foot Tip, each built from named dimensions in its STL's coordinates, passing the gates (the global limits, or a documented per-part exception) and exported as STEP. The serpentine flexure in Foot Left and Foot Right SHALL be modelled from named dimensions of its path and beam widths, shared by both feet, so that changing one and rebuilding changes the spring of both. Foot Right SHALL be built from Foot Left's model, not modelled separately: since its STL is Foot Left's mirror image moved 1.069 mm along y, which a mirror plane cannot express, Foot Right SHALL be a part file of its own whose build mirrors Foot Left's solid and moves it, rather than the second side of a mirrored pair.

#### Scenario: Feet group built
- **WHEN** the build is run
- **THEN** it checks and exports Foot Left, Foot Right, Foot Shaft and Foot Tip, each against its STL in `hardware/aecert-hexapod-v1/stl/`, and each STEP file imports as one closed solid in FreeCAD and solidcraft

#### Scenario: A foot part is missing
- **WHEN** any of the four foot parts is not declared in the Aecert source's parts folder
- **THEN** the unit tests fail naming the missing part

#### Scenario: The flexure is retuned
- **WHEN** a flexure beam width in the shared foot model is raised by 0.3 mm and both feet are rebuilt
- **THEN** Foot Left and Foot Right both differ from their unchanged builds, and each fails its volume or surface deviation gate against its STL

#### Scenario: A foot STL's chord error is too coarse
- **WHEN** a foot part's STL chord error, as `chord_error.measure` gives it, exceeds 0.15 mm
- **THEN** that part declares a `deviation_max` exception naming the feature, with the limit at the chord error plus 0.05 mm rounded up to 0.01 mm, and the source README's row for it gives the feature, the limit and the amount over the global limit

#### Scenario: A foot feature is freeform
- **WHEN** a foot part has a freeform feature that feature-based modelling cannot match
- **THEN** that part declares an exception for the failing gate only, whose reason starts with "freeform" and whose limit exceeds the global limit, and the source README's row for it gives the feature, the limit and the amount over the global limit

#### Scenario: Other foot parts keep the global limits
- **WHEN** a foot part's chord error is at most 0.15 mm and it has no freeform feature
- **THEN** it declares no exception
