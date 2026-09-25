# YAMS Examples

This folder contains example robot projects demonstrating YAMS usage. Runnable projects are
assembled by the Makefile from source fragments and a shared skeleton.

## Usage

```bash
# Generate all examples
make

# Generate and build all examples
make build

# Clean generated output
make clean

# Generate a single example
make advantage_kit
```

Generated projects are written to `generated/<example_name>/` and are fully self-contained
WPILib Gradle projects ready to open in VS Code or IntelliJ.

## How it works

### Directory layout

```
examples/
├── commands2/              # Commands v2 examples
│   ├── simple_robot-2027/  # Skeleton: base WPILib project (build.gradle, vendordeps, Gradle wrapper)
│   └── <example_name>/     # Source-only folders, one per example
│       ├── java/           # Java source tree (copied to src/main/java/)
│       ├── deploy/         # Optional deploy files (replace src/main/deploy/)
│       ├── vendordeps/     # Optional extra vendordeps (added to vendordeps/)
│       └── build.gradle    # Optional replacement for the skeleton's build.gradle
├── cpptest/                # C++ example, built on its own
├── generated/              # Output, created by `make` and gitignored
└── Makefile
```

### What the Makefile does

1. **Discovers examples**: finds every folder in `commands2/` that contains a `java/` folder. The
   skeleton has no `java/` folder, so it is skipped.

2. **Copies the skeleton**: each discovered example gets a copy of `commands2/simple_robot-2027/`
   at `generated/<example_name>/`, without the skeleton's own source or build output.

3. **Injects source**: the example's `java/` and `deploy/` folders become
   `generated/<example_name>/src/main/java/` and `src/main/deploy/`, its `vendordeps/` files are
   added to the skeleton's, and its `build.gradle`, if present, replaces the skeleton's.

Generated projects must stay in `examples/generated/`: the skeleton's `build.gradle` compiles YAMS
from `../../../yams/java`, which only resolves from there.

The result looks like:

```
generated/advantage_kit/
├── build.gradle
├── vendordeps/
├── gradle/
├── src/
│   ├── main/
│   │   ├── java/       # from examples/commands2/advantage_kit/java/
│   │   └── deploy/     # from examples/commands2/advantage_kit/deploy/
│   └── test/
└── ...
```

### Adding a new example

1. Create a new folder under `examples/commands2/` with a descriptive name (no `cpp` in the name).
2. Add a `java/` subfolder containing your source tree: robot code in the `first.robot` package
   (`java/first/robot/`) and the entry point at `java/first/Main.java`, which the skeleton's
   `build.gradle` launches as `first.Main`.
3. Optionally add a `deploy/` subfolder for any files that should be deployed to the robot.
4. Optionally add a `vendordeps/` subfolder for vendor libraries only this example needs (for
   example ChoreoLib or LimelightLib); its files are added to the skeleton's vendordeps.
5. Optionally add a `build.gradle` to replace the skeleton's, for plain Maven dependencies that
   have no vendordep.

Running `make` will automatically pick up the new folder and generate its project.

### Skeleton

`commands2/simple_robot-2027/` is a WPILib project with the YAMS vendordeps pre-configured. It
only compiles YAMS core and the commands2 layer. Do not add robot-specific source to it; it is
shared by all examples.
