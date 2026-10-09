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

Each file's rows live in exactly one part file under `docs/layout/`. Add a
row to the part that covers the file, never to this index. Every part file
has its own byte budget, so a full part does not break a parallel PR that
touches another. A new part file must be declared in `claude_md_layout_test.go`
first.

- `docs/layout/design-docs.md`: design documents, user guides, seam and
  records, mass properties and free-form curves.
- `docs/layout/features.md`: feature files and the cap-loop chamfer.
- `docs/layout/modify-booleans.md`: modify operations and boolean files.
- `docs/layout/verification.md`: verification, survey and check files.
- `docs/layout/output-repository.md`: tessellation and export, then
  examples, test kits, tooling and CI.
- `docs/layout/internal-core.md`: internal packages for arithmetic, records,
  geometry and mesh construction.
- `docs/layout/internal-checks.md`: internal packages for bounds, surveys,
  sweeps and clearance.
