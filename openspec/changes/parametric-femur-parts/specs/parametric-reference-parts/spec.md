## ADDED Requirements

### Requirement: The femur group
The Aecert source SHALL include the femur group: Femur Horn Side, Femur Bearing Side and Femur Support, each built from features in its STL's own coordinates, passing the gates (with only documented per-part exceptions) and exported as STEP. The femur's axis-to-axis length, the distance between the two bore axes of each femur side plate, SHALL be one named constant from which both side plates place their bores, and the unit tests SHALL check it against the bore axes found on the built solids.

#### Scenario: Femur group built
- **WHEN** the build is run
- **THEN** it checks and exports Femur Horn Side, Femur Bearing Side and Femur Support, each against its STL in `hardware/aecert-hexapod-v1/stl/`, and each STEP file passes the FreeCAD and solidcraft import checks

#### Scenario: A femur part is missing
- **WHEN** any of the three femur parts is not declared in the Aecert source's parts folder
- **THEN** the unit tests fail naming the missing part

#### Scenario: Axis-to-axis length checked
- **WHEN** the unit tests build Femur Horn Side and Femur Bearing Side
- **THEN** each side's two bore axes, found from the solid's cylindrical faces, lie the named axis-to-axis length apart, and a bore placed elsewhere fails the test
