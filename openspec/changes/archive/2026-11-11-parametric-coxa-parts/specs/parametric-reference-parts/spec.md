## ADDED Requirements

### Requirement: The coxa group
The Aecert source SHALL include the coxa group: Coxa Horn Connector, Coxa Horn Side, Coxa Bearing Connector and Coxa Bearing Side, each modelled from features with named, sourced dimensions in its STL's own coordinates, each passing the gates under the global limits (or a documented per-part exception where the STL's own chord error exceeds 0.15 mm, or for a named feature that feature-based modelling cannot match, with its size), and each exported as STEP that passes the FreeCAD and solidcraft import checks where the tool is installed. The source README SHALL compare the built parts with the coxa link geometry that `hardware/aecert-hexapod-v1/README.md` records (the tongues' insert holes 21.5 mm from the coxa axis, the 8.05 mm ring, the ears 30.10 mm either side of the 28.5 mm hole pair's middle, the 68.25 × 48 mm side plates), giving for each value the recorded value and the model's, and state each disagreement. The unit tests SHALL check that the four coxa parts are declared and that the model's values in that comparison match the built solids.

#### Scenario: Coxa group built
- **WHEN** the build is run
- **THEN** it checks and exports the four coxa parts, each against its STL in `hardware/aecert-hexapod-v1/stl/`, and each STEP file passes the FreeCAD and solidcraft import checks where the tool is installed, and reports `SKIPPED` where it is not

#### Scenario: A coxa part is missing
- **WHEN** any of the four coxa parts is not declared in the Aecert source's parts folder
- **THEN** the unit tests fail naming the missing part

#### Scenario: Coxa comparison matches the built parts
- **WHEN** the unit tests build the four coxa parts
- **THEN** each value of the recorded coxa link geometry, measured on the built solids, matches the model's value that the source README's coxa comparison states to 0.01 mm, the comparison's recorded values match those in `hardware/aecert-hexapod-v1/README.md`, each difference above 0.05 mm is marked as a disagreement and no other is, and a part changed so that a compared value moves fails the test

#### Scenario: Recorded coxa geometry compared
- **WHEN** a value of the coxa link geometry recorded in `hardware/aecert-hexapod-v1/README.md` differs from the built part's by more than 0.05 mm
- **THEN** the source README marks it as a disagreement, with the recorded value, the model's value and the part

## MODIFIED Requirements

### Requirement: Documentation of the parametric parts
The parametric parts folder SHALL carry a README stating how a part is added, the part file's contract, the gates and their tolerances, the import checks with the tool versions checked, and how to run the build. Each source SHALL carry a README stating its source repository, the reference commit, its licence (for Aecert's parts: that upstream states no licence, said plainly, and beside it that the author knows about Spooder's derived models and has raised no concerns, stated as informal and not a licence), and a parts table per group, each part with its STL, its STEP file and any gate exception.

#### Scenario: A part is listed
- **WHEN** a part's file exists in a source's parts folder
- **THEN** that source's README lists the part with its STL and STEP file

#### Scenario: An exception is documented
- **WHEN** a part declares a gate exception
- **THEN** the source README names the feature responsible and the amount by which it exceeds the global limit

#### Scenario: Author's awareness stated
- **WHEN** the Aecert source README states that upstream gives no licence
- **THEN** it states beside it that the author knows about Spooder's derived models and has raised no concerns, and that this is informal and grants no licence
