# Exact arithmetic implementation task plans

## Status and authority

These plans describe unexecuted implementation work for five proposals. They
are instructions for an implementer, not evidence that a capability has shipped.
Each linked design owns its proposed geometry and proof contract. Existing
current-state owner documents and repository instructions win over these plans.
Resolve a disagreement with the coordinating agent before changing that contract.

The plans were prepared against source revision `5ed2045c6`. The planning
worktree is `.worktrees/docs-exact-arithmetic-designs`. Its designs and plans
may be uncommitted; do not assume a fresh worktree from `main` contains them.

## Choose a plan

| Item | Task plan | First acceptance boundary |
|---|---|---|
| SameSet levels | [Exact level equality](plans/same-set-level-implementation.md) | Public `Extrude` to `Verify`, with an exact overlap oracle. |
| Split spherical meridians | [Wall survey](plans/split-circle-wall-implementation.md) | Real split-circle `Revolve` to `Verify`, with a diameter enclosure. |
| Boolean BReps | [Point containment](plans/brep-point-containment-implementation.md) | An admitted exact construction used by a real point-history query. |
| Whole sphere and ring STEP | [Analytic export](plans/closed-surface-step-implementation.md) | Real source through the candidate writer into an independent solid reader. |
| Gap-shifted Boolean results | [Point and section queries](plans/gap-shifted-query-implementation.md) | Evidence for the unchanged-target relation, then a public point query. |

Start with SameSet: public probes confirmed an incorrect zero-bound overlap
measurement. The split-circle survey also has a concrete bounded construction.
The query and export plans contain prerequisite gates; follow their stop
conditions before enabling new answers or export arms.

The BRep plan's real rectangular A7 starter fixture fails the proposal's
zero-displacement admission. Its plan records that failed boundary and blocks
certificate attachment pending the source-set proof. Do not remove the charge
or substitute a passing fixture to bypass this prerequisite.

## Prepare the implementation

1. Work in the assigned `.worktrees/<branch>` directory. Read its `CLAUDE.md`,
   applicable pre-read documents and the matching design before editing.
   Follow `~/.claude/docs/git-operations.md`, "Worktrees", for setup.
2. Confirm that the design and task plan are available in that worktree.
   If they exist only in the planning worktree, have the coordinating agent
   transfer the reviewed documents before implementation. Preserve other work.
3. Record the implementation base revision and local changes. Recheck the
   named symbols on that revision. A moved symbol is not permission to replace
   its semantics. Report a changed construction or contract before proceeding.
4. Create `.tmp/` for commands and evidence. Read the plan's numbered tasks in
   order; distinguish existing helpers from explicitly proposed additions.
5. Keep the linked design as the proof owner. Plan pseudocode and signatures
   are non-authoritative implementation guidance. Do not copy an observed
   baseline result into a test as the desired new behavior.

## Execute one real case first

Run the plan's smallest real producer-to-consumer case before expanding its
fixture matrix or adding infrastructure for later slices. A handwritten
payload, exported file or certificate does not replace a real producer.

At each numbered task, record its command, exit status and bounded result under
`.tmp/`. A logging probe that passes establishes only the observations it
asserted. An expected baseline failure does not establish a broken test command.
Distinguish that failure from a compiler error, fixture-construction refusal,
timeout or missing dependency.

Stop expansion when the first real case fails. Report the failing boundary,
saved evidence and smallest repair scope. Do not repair another repository or
a prerequisite outside the selected slice. Follow the repository's cross-repo
handoff rule when an external component needs a change.

## Respect unresolved proof gates

Implement only the admissions whose proof obligations the design and plan
settle. A task marked as investigation must produce the named evidence and
proof artifact before its dependent code can be enabled.

Do not replace missing set equality with an operation name, retained tag,
small residual, zero reported bound or successful shell audit. Do not replace
independent STEP solid reconstruction with parsing or entity-string matching.
If the required proof is still unknown after the bounded investigation, leave
the corresponding route disabled and return the evidence to the coordinating
agent. A new assertion that says the route is certified does not supply proof.

The BRep and shifted-query plans share point-query code. Coordinate their
changes to `historyWalk`, dispatch and evidence types. Their certificates have
different meanings; one must not enable the other. Keep Boolean regrouping and
history-section eligibility unchanged unless the relevant design proves more.

## Verify the selected slice

Follow [Test scope](testing.md#test-scope) and the matching plan's focused tests.
Use the existing shard rules for new root and `apitest` names. Keep geometry
assertions and bounded failure diagnostics; do not compare whole topology or
mesh graphs through recursive assertion formatters.

Run focused tests for the completed change. Remote CI owns full suites under
the repository's execution rules. For an authorized PR, use the lint, message,
push-guard and CI-watch requirements in `~/.claude/docs/git-operations.md`.
Do not report a partial local test run as a passing suite.

Update current-state owner documents only for behavior that the completed
implementation actually establishes. Register new design documents and root
source files under [Layout rows](layout.md#layout-rows). Keep blocked future
slices explicitly unimplemented.

## Return implementation evidence

Report the implementation worktree and revision, completed task numbers,
actual source classes admitted, first real acceptance result, focused test
results and evidence paths. Name any route still disabled and the exact
missing prerequisite. Include CI results only when CI actually ran.

An investigation-only result is complete for its investigation task, not for
the capability. Do not mark the whole plan implemented while a required
producer certificate or reader acceptance gate remains unmet.
