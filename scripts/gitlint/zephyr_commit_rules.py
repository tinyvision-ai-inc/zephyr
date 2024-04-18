# SPDX-License-Identifier: Apache-2.0

"""
The classes below are examples of user-defined CommitRules. Commit rules are gitlint rules that
act on the entire commit at once. Once the rules are discovered, gitlint will automatically take care of applying them
to the entire commit. This happens exactly once per commit.

A CommitRule contrasts with a LineRule (see examples/my_line_rules.py) in that a commit rule is only applied once on
an entire commit. This allows commit rules to implement more complex checks that span multiple lines and/or checks
that should only be done once per gitlint run.

While every LineRule can be implemented as a CommitRule, it's usually easier and more concise to go with a LineRule if
that fits your needs.
"""

from gitlint.rules import CommitRule, RuleViolation, LineRule, CommitMessageBody
from gitlint.options import IntOption
import re

class BodyMaxLineCount(CommitRule):
    # A rule MUST have a human friendly name
    name = "body-max-line-count"

    # A rule MUST have an *unique* id, we recommend starting with UC (for User-defined Commit-rule).
    id = "UC1"

    # A rule MAY have an options_spec if its behavior should be configurable.
    options_spec = [IntOption('max-line-count', 200, "Maximum body line count")]

    def validate(self, commit):
        line_count = len(commit.message.body)
        max_line_count = self.options['max-line-count'].value
        if line_count > max_line_count:
            message = "Commit message body contains too many lines ({0} > {1})".format(line_count, max_line_count)
            return [RuleViolation(self.id, message, line_nr=1)]


class MaxLineLengthExceptions(LineRule):
    name = "max-line-length-with-exceptions"
    id = "UC4"
    target = CommitMessageBody
    options_spec = [IntOption('line-length', 75, "Max line length")]
    violation_message = "Commit message body line exceeds max length ({0}>{1})"

    def validate(self, line, _commit):
        max_length = self.options['line-length'].value
        urls = re.findall(r'http[s]?://(?:[a-zA-Z]|[0-9]|[$-_@.&+]|[!*\(\),]|(?:%[0-9a-fA-F][0-9a-fA-F]))+', line)
        if line.lower().startswith('signed-off-by') or line.lower().startswith('co-authored-by'):
            return

        if urls:
            return

        if len(line) > max_length:
            return [RuleViolation(self.id, self.violation_message.format(len(line), max_length), line)]

class BodyContainsBlockedTags(LineRule):
    name = "body-contains-blocked-tags"
    id = "UC7"
    target = CommitMessageBody
    tags = ["Change-Id"]

    def validate(self, line, _commit):
        flags = re.IGNORECASE
        for tag in self.tags:
            if re.search(rf"^\s*{tag}:", line, flags=flags):
                return [RuleViolation(self.id, f"Commit message contains a blocked tag: {tag}")]
        return
