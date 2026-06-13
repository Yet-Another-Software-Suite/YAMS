# YAMS Examples

This folder contains example robot projects demonstrating YAMS usage. Runnable projects are
assembled by the Makefile from source fragments and a shared skeleton.

## Usage

```bash
# Build all examples
make

# Clean generated output
make clean

# Build a single example
make advantage_kit
```

Generated projects are written to `generated/<example_name>/` and are fully self-contained
WPILib Gradle projects ready to open in VS Code or IntelliJ.

## How it works

### Directory layout

```
examples/
├── skeleton/           # Base WPILib project (build.gradle, vendordeps, Gradle wrapper, etc.)
├── <example_name>/     # Source-only folders, one per example
│   ├── java/           # Java source tree (copied to src/main/java/)
│   └── deploy/         # Deploy files  (copied to src/main/deploy/)
├── generated/          # Output — created by `make`, gitignored
└── Makefile
```

### What the Makefile does

1. **Discovers examples** — finds every subdirectory that contains a `java/` folder, excluding
   `skeleton`, `generated`, and anything with `cpp` in its name.

2. **Copies the skeleton** — each discovered example gets a full copy of `skeleton/` placed at
   `generated/<example_name>/`.

3. **Injects source** — the example's `java/` and `deploy/` folders are copied into
   `generated/<example_name>/src/main/` to produce the standard WPILib source layout.

The result looks like:

```
generated/advantage_kit/
├── build.gradle
├── vendordeps/
├── gradle/
├── src/
│   ├── main/
│   │   ├── java/       # from examples/advantage_kit/java/
│   │   └── deploy/     # from examples/advantage_kit/deploy/
│   └── test/
└── ...
```

### Adding a new example

1. Create a new folder under `examples/` with a descriptive name (no `cpp` in the name).
2. Add a `java/` subfolder containing your source tree rooted at `frc/robot/`.
3. Optionally add a `deploy/` subfolder for any files that should be deployed to the robot.

Running `make` will automatically pick up the new folder and generate its project.

### Skeleton

The `skeleton/` directory is a vanilla WPILib project with the full set of YAMS vendordeps
pre-configured. Do not add robot-specific source to it — it is shared by all examples.
