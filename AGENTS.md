# AGENTS.md

Instructions for coding agents in this repository live in **[CLAUDE.md](CLAUDE.md)**.
They are tool-agnostic — read that file and follow it.
Read [Test scope](agent-docs/testing.md#test-scope) for slow tests and consumer models.
Read [Splitting oversized agent files](agent-docs/layout.md#splitting-oversized-agent-files)
when a file size or layout cell budget fails.

The single copy is intentional: two sets of agent instructions drift apart.

## GitHub merge method

Use `gh pr merge <number> --squash` for PRs. GitHub disables merge commits and rebase merges here.
