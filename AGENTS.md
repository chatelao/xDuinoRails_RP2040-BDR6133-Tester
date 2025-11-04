AGENTS.md

# Structure
- Keep sources in “src”, exceptions are possible depending on special technologies

# Documentation
- Create and maintain on every change a README.md, add the usual summary
- Create a LICENSE.md if missing with AGPL 3.0 (Affero GNU)
- Create and maintain on every change a “CONTRIBUTE.md”
- Keep all documentation below in the ““docs” directory
- Create and maintain on every change a “HOW_TO_USE.md”, “USER_MANUAL.md”,
  “CORE_CONCEPTS.md”, “DEVELOPER_REFERENCE.md” and “TECHNICAL_DEBTS.md”

# Code
Follow the most used file & code naming conventions in the project:
- If not yet provided use “snake case”
- Keep ".md" documentation files uppercase snake
- Comment lines if magic numbers or complex operations are used

# Testing
Add a comment to each testcase
- Summarizing the goal of the testcase in a short sentence.
- Summarizing the steps executed in 1-2 lines with longer sentences.
- List the references the underlying specification / documentation to each test case.

# Add the following GitHub actions:
- Build and test the software after every push on every branch
- Provide all firmware examples with every release as assets
