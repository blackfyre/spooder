## ADDED Requirements

### Requirement: The coxa group
The Aecert source SHALL include the coxa group: Coxa Horn Connector, Coxa Horn Side, Coxa Bearing Connector and Coxa Bearing Side, each modelled from features with named, sourced dimensions in its STL's own coordinates, each passing the gates under the global limits (or a documented per-part exception where the STL's own chord error exceeds 0.15 mm, or for a named feature that feature-based modelling cannot match, with its size), and each exported as STEP that passes the FreeCAD and solidcraft import checks. The source README SHALL compare the built parts with the coxa link geometry that `hardware/aecert-hexapod-v1/README.md` records (the tongues' insert holes 21.5 mm from the coxa axis, the 8.05 mm ring, the ears 30.10 mm either side of the 28.5 mm hole pair's middle, the 68.25 × 48 mm side plates) and state each disagreement with its measured value.

#### Scenario: Coxa group built
- **WHEN** the build is run
- **THEN** it checks and exports the four coxa parts, each against its STL in `hardware/aecert-hexapod-v1/stl/`, and every FreeCAD and solidcraft import check of them passes

#### Scenario: Recorded coxa geometry compared
- **WHEN** a value of the coxa link geometry recorded in `hardware/aecert-hexapod-v1/README.md` differs from the built part's by more than 0.05 mm
- **THEN** the source README states the recorded value, the model's value and the part

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
