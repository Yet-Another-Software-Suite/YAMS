# Java groups

The Java unit tests use JUnit 5. CI runs each group in its own JVM, with Gradle's `--tests` filters. A parameterized test counts once here, however many cases it runs.

| Group                                               | Test classes | Tests |
| --------------------------------------------------- | ------------ | ----- |
| [arm](arm/)                                         | 1            | 5     |
| [elevator](elevator/)                               | 1            | 5     |
| [pivot](pivot/)                                     | 1            | 4     |
| [shooter](shooter/)                                 | 1            | 4     |
| [mechanism-limits](mechanism-limits/)               | 1            | 13    |
| [loose-followers](loose-followers/)                 | 1            | 2     |
| [follower-simulation](follower-simulation/)         | 1            | 10    |
| [vendor-simulation](vendor-simulation/)             | 1            | 4     |
| [hard-stops](hard-stops/)                           | 1            | 4     |
| [swerve-drive](swerve-drive/)                       | 5            | 89    |
| [continuous-wrapping](continuous-wrapping/)         | 1            | 9     |
| [absolute-encoders](absolute-encoders/)             | 5            | 17    |
| [absolute-encoder-ranges](absolute-encoder-ranges/) | 1            | 1     |
| [zero-offsets](zero-offsets/)                       | 2            | 4     |
| [motor-controllers](motor-controllers/)             | 10           | 29    |
| [live-tuning](live-tuning/)                         | 3            | 4     |
| [closed-loop-gains](closed-loop-gains/)             | 3            | 16    |
| [closed-loop-profiles](closed-loop-profiles/)       | 1            | 3     |
| [closed-loop-gain-units](closed-loop-gain-units/)   | 2            | 19    |
| [other](other/)                                     | 8            | 62    |
