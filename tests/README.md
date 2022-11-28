# Non-regression tests

Geogram uses extensive testing. More details [here](https://github.com/BrunoLevy/geogram/wiki/DevOps#tests) on how to run the
tests manually. Tests are run after each commit (`smoke`), and on a regular basis (`nightly` and `weekly`), using github actions.
Test results are published both as artifacts attached to the corresponding actions, and as HTML pages (links below).

# Dashboard

## Robot Framework reports

|                 | Smoke | Nightly | Weekly |
| :---            | :---: |  :---:  |  :---: |
| Linux Debug     | [![robot](robot.png)](https://brunolevy.github.io/geogram/reports/smoke-Linux-Debug/report.html)     | [![robot](robot.png)](https://brunolevy.github.io/geogram/reports/nightly-Linux-Debug/report.html)     | [![robot](robot.png)](https://brunolevy.github.io/geogram/reports/weekly-Linux-Debug/report.html) |
| Linux Release   | [![robot](robot.png)](https://brunolevy.github.io/geogram/reports/smoke-Linux-Release/report.html)   | [![robot](robot.png)](https://brunolevy.github.io/geogram/reports/nightly-Linux-Release/report.html)   | [![robot](robot.png)](https://brunolevy.github.io/geogram/reports/weekly-Linux-Release/report.html) |
| macOS Debug     | [![robot](robot.png)](https://brunolevy.github.io/geogram/reports/smoke-macOS-Debug/report.html)     | [![robot](robot.png)](https://brunolevy.github.io/geogram/reports/nightly-macOS-Debug/report.html)     | [![robot](robot.png)](https://brunolevy.github.io/geogram/reports/weekly-macOS-Debug/report.html) |
| macOS Release   | [![robot](robot.png)](https://brunolevy.github.io/geogram/reports/smoke-macOS-Release/report.html)   | [![robot](robot.png)](https://brunolevy.github.io/geogram/reports/nightly-macOS-Release/report.html)   | [![robot](robot.png)](https://brunolevy.github.io/geogram/reports/weekly-macOS-Release/report.html) |
| Windows Debug   | [![robot](robot.png)](https://brunolevy.github.io/geogram/reports/smoke-Windows-Debug/report.html)   | [![robot](robot.png)](https://brunolevy.github.io/geogram/reports/nightly-Windows-Debug/report.html)   | [![robot](robot.png)](https://brunolevy.github.io/geogram/reports/weekly-Windows-Debug/report.html) |
| Windows Release | [![robot](robot.png)](https://brunolevy.github.io/geogram/reports/smoke-Windows-Release/report.html) | [![robot](robot.png)](https://brunolevy.github.io/geogram/reports/nightly-Windows-Release/report.html) | [![robot](robot.png)](https://brunolevy.github.io/geogram/reports/weekly-Windows-Release/report.html) |


