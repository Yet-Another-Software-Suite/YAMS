# C++ groups

The C++ unit tests use Catch2. Each test case has tags, such as `[ArmTest]`, and CI runs each group with a Catch2 filter of its tags. A comma in a filter means "or".

| Group                                       | Test files | Test cases |
| ------------------------------------------- | ---------- | ---------- |
| [arm](arm/)                                 | 1          | 4          |
| [elevator](elevator/)                       | 1          | 4          |
| [pivot](pivot/)                             | 1          | 8          |
| [shooter](shooter/)                         | 1          | 5          |
| [swerve-drive](swerve-drive/)               | 1          | 65         |
| [motor-controllers](motor-controllers/)     | 6          | 87         |
| [talon-parity](talon-parity/)               | 1          | 21         |
| [follower-simulation](follower-simulation/) | 1          | 4          |
| [other](other/)                             | 3          | 59         |

`mechs/SwerveDriveTest2.cpp` holds only a license header and has no tests.
