Streamline the exported API of github.com/lestrrat-3d/decad. Audit every export in the root package, including aliases, functions, options, constants, errors, types, and exported methods. For each one, determine whether an end user needs it to model or inspect CAD geometry. Keep useful public API; move implementation details to internal packages or remove them. This module has no releases, so compatibility is not a constraint.

Judge planned features by whether they are essential to making decad true CAD software. ErrUnsupported may mark a feature that is planned but unfinished; it is not, by itself, evidence that its public API should be removed. An essential feature may warrant keeping its public placeholder. A less essential one may move to internal. Read the owning design docs and code before deciding, and record the reason. PR #1167's removal of WithSweepTwist was a mistake: it treated unsupported nonzero values as proof that a documented placeholder was unnecessary. Do not repeat that reasoning. Do not change a design doc merely to justify a deletion; if you decide to change a design, explain the decision and update the doc as the repository requires.

Apply the same care to option tiers. Merged PR #1174 deliberately restored DocumentOption,
DraftOption, ChainExtrudeOption, ChainRevolveOption, ChainSweepOption, and ChainLoftOption
after PRs #1153 and #1163 removed them. PR #1228 repeated the DocumentOption removal
and was closed; the uncommitted repeat of the other removals was discarded. Treat #1174
as the current design decision. An option tier having no concrete options today is
insufficient reason to remove it. Before proposing any such removal, explain why its
documented extension point is unnecessary for CAD modeling or inspection, including
how the proposed public signature preserves intended option constraints.

Assess aliases on their own merits. If callers need an operation but not its current alias, consider a clearer public signature instead of preserving the alias automatically.

For each alias, make two separate decisions: whether callers need its root-package
name, and where its actual definition belongs. Define types that users need to
name, construct, or inspect in the root package, rather than exposing aliases
of definitions in internal packages. Internal packages should consume the data
from those public types through values passed by the root package. Refactor the
internal boundary when needed to avoid an import cycle; an existing internal
dependency is not by itself a reason to retain a public type's definition
there. Keep definitions in internal packages when the types are implementation
details. Record the concrete reason for each alias decision and definition move.
The corresponding public API rule is in docs/api-design.md §2.

PR #1231 removed DiagUnsupportedPair after confirming that Verify emits only
the specific unsupported-pair diagnostic codes. That broad code had no
documented planned behavior; its deletion does not establish a rule for
removing documented feature placeholders or option tiers.

Another agent is moving code from the root package to internal packages. Reducing root-package line count is not your priority. Check current work before editing, avoid overlapping that agent's changes, and do not undo its mechanical moves just to simplify your audit.

Follow AGENTS.md and CLAUDE.md. Work in a separate worktree, make scoped PRs, run focused local tests, and watch each PR's CI through completion. You have permission to create PRs and merge them when CI is green and the PR is ready.

The goal is complete only when every current root export has been reviewed, every justified change has been merged, and the final report accounts for what stayed public, moved, or was removed and why. Passing tests or merging one PR does not complete the audit. If evidence is insufficient for a decision, report the specific uncertainty instead of treating the export as dead.

The explicit completion condition follows OpenAI’s guidance for Goals.
