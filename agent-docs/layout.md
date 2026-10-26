# decad file layout

Every row names what a file owns and where its detail lives. Read "Layout
rows" below before editing one.
`CLAUDE.md` points here from its "Read before you write" table.

## Layout rows

**Layout rows are pointers, not summaries.** A row states what the file owns
in one or two sentences and names where the detail lives, within 300
characters. NEVER grow a row to record an invariant, a derivation, a sign
convention or a refusal — that belongs in the owning design doc or the
function's own doc comment, and a row that restates it drifts from the code.
`claude_md_layout_test.go` enforces these rules over every line of the part
files below, this file and `CLAUDE.md`, refusing any line it cannot classify.
Its `parseAgentDoc` doc comment owns the complete rule list — heading
spellings, the one `## Layout` heading per part file, declared tables — and
its file-level comment owns what the rules leave to the byte budget.

## Part files

Each file's rows live in exactly one part file under `agent-docs/layout/`. Add a
row to the part that covers the file, never to this index. Every part file
has its own byte budget, so a full part does not break a parallel PR that
touches another. A new part file must be declared in `claude_md_layout_test.go`
first.

- `agent-docs/layout/design-docs.md`: design documents, user guides, seam and
  records, mass properties and free-form curves.
- `agent-docs/layout/features.md`: feature files and the cap-loop chamfer.
- `agent-docs/layout/modify-booleans.md`: modify operations.
- `agent-docs/layout/booleans.md`: Boolean files.
- `agent-docs/layout/verification.md`: verification, survey and check files.
- `agent-docs/layout/output-repository.md`: tessellation and export, then
  examples, test kits, tooling and CI.
- `agent-docs/layout/internal-core.md`: internal packages for arithmetic, records,
  geometry and mesh construction.
- `agent-docs/layout/internal-revolve-sweep.md`: Revolve, Sweep, coil and pair-sweep path internal packages.
- `agent-docs/layout/internal-checks.md`: internal packages for bounds, surveys,
  sweeps and clearance.
- `agent-docs/layout/internal-loft.md`: Loft, point-loft, cone and contact internal packages.
- `agent-docs/layout/internal-prism.md`: prism, stacked, cup and Shell internal packages.
- `agent-docs/layout/internal-placement.md`: body layout and path-spacing internal packages.
- `agent-docs/layout/operation-observability.md`: operation history and diagnostics internals.

## Splitting oversized agent files

- If `CLAUDE.md` exceeds 8192 bytes, move a cohesive topic into `docs/<topic>.md`.
  Replace its text with a short trigger and link in `CLAUDE.md`'s pre-read table.
- If this index exceeds 4096 bytes, move topic detail into its part file or
  owning document. Keep the index's part-file links short.
- If a `agent-docs/layout/` part exceeds 12288 bytes, move a cohesive group of rows
  into a new `agent-docs/layout/<topic>.md` file. Give it one `## Layout` heading and
  its own table, register it with `layoutPartSpec` in `claude_md_layout_test.go`,
  and link it under **Part files** above. Move each row; never copy it.
- If a Layout cell exceeds 300 characters, shorten it to a pointer and move
  the detail into the file or design document that owns it.
- Run `go test . -run '^TestCLAUDEMDLayoutStaysCompact$'` after a split.
