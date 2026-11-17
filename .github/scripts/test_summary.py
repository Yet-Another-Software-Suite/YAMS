"""Write a job summary of JUnit XML test results.

Usage: test_summary.py <title> <results directory>

Reads every *.xml JUnit report under the results directory, as Gradle writes for the Java tests and
Catch2's JUnit reporter writes for the C++ tests, and appends a Markdown summary to the file named
by GITHUB_STEP_SUMMARY (or prints it, when that is not set): the totals, each failure with its
message, and every test's result, grouped by class in collapsible sections.
"""

import os
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

# GitHub shows at most 1 MiB of a step's summary, so the list of every test stops short of that.
MAX_SUMMARY_BYTES = 900_000
MAX_MESSAGE_CHARS = 400


def cell(text):
    """Text for one line of Markdown: no newlines or pipes, and not too long."""
    text = " ".join(text.split()).replace("|", "\\|")
    return text if len(text) <= MAX_MESSAGE_CHARS else text[:MAX_MESSAGE_CHARS] + "..."


def failure_message(failure):
    """A failure's message. Catch2 puts only the expression in the message and its file, line and
    expansion in the text; Gradle's text is the stack trace, whose first line repeats the message."""
    message = (failure.get("message") or "").strip()
    text = [line.strip() for line in (failure.text or "").splitlines() if line.strip()]
    if text and (not message or message not in text[0]):
        message = " / ".join(([message] if message else []) + text[:3])
    return message


def read_results(directory):
    """Every test case in the reports: (class, name, status, seconds, message)."""
    results = []
    for report in sorted(Path(directory).rglob("*.xml")):
        try:
            root = ET.parse(report).getroot()
        except ET.ParseError as error:
            print(f"Skipping {report}: {error}", file=sys.stderr)
            continue
        for case in root.iter("testcase"):
            failure = case.find("failure")
            if failure is None:
                failure = case.find("error")
            if failure is not None:
                status = "failed"
                message = failure_message(failure)
            elif case.find("skipped") is not None:
                status = "skipped"
                message = case.find("skipped").get("message", "")
            else:
                status = "passed"
                message = ""
            results.append((case.get("classname", ""), case.get("name", ""), status,
                            float(case.get("time") or 0), message))
    return results


def summarize(title, results):
    counts = defaultdict(int)
    for result in results:
        counts[result[2]] += 1
    seconds = sum(result[3] for result in results)
    icon = {"passed": "✅", "failed": "❌", "skipped": "⏭️"}

    lines = [f"## {title}", ""]
    if not results:
        lines += ["No test results were found. The tests may not have compiled or started.", ""]
        return "\n".join(lines)
    lines += [f"{icon['failed'] if counts['failed'] else icon['passed']} "
              f"**{counts['passed']} passed, {counts['failed']} failed, {counts['skipped']} skipped** "
              f"of {len(results)} tests, in {seconds:.1f} s", ""]

    failures = [r for r in results if r[2] == "failed"]
    if failures:
        lines += ["### Failures", "", "| Test | Message |", "| --- | --- |"]
        lines += [f"| `{cell(cls)}` {cell(name)} | {cell(message)} |" for cls, name, _, _, message in failures]
        lines.append("")

    by_class = defaultdict(list)
    for result in results:
        by_class[result[0]].append(result)
    lines += ["### Every test", ""]
    size = len("\n".join(lines).encode())
    for cls in sorted(by_class):
        cases = by_class[cls]
        failed = sum(1 for c in cases if c[2] == "failed")
        passed = sum(1 for c in cases if c[2] == "passed")
        header = (f"<details{' open' if failed else ''}><summary>{icon['failed'] if failed else icon['passed']} "
                  f"<code>{cell(cls) or 'tests'}</code>: {passed}/{len(cases)} passed</summary>")
        block = [header, ""] + [f"- {icon[status]} {cell(name)} ({secs:.2f} s)" for _, name, status, secs, _ in cases]
        block += ["", "</details>", ""]
        block_size = len("\n".join(block).encode())
        if size + block_size > MAX_SUMMARY_BYTES:
            lines.append("The rest of the tests are left out, to keep the summary within GitHub's size limit.")
            break
        lines += block
        size += block_size
    return "\n".join(lines)


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    summary = summarize(sys.argv[1], read_results(sys.argv[2]))
    path = os.environ.get("GITHUB_STEP_SUMMARY")
    if path:
        with open(path, "a", encoding="utf-8") as out:
            out.write(summary + "\n")
    else:
        sys.stdout.reconfigure(encoding="utf-8")
        print(summary)


if __name__ == "__main__":
    main()
