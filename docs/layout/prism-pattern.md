# decad file layout: prism pattern Union

Circular-prism joins of radial copied sections.
The rules for rows live in `docs/layout.md`.

## Layout

| Path | Responsibility |
|---|---|
| `prism_pattern_union.go` | Certifies sectors and dispatches the structural Union. See prism-pattern-union design. |
| `prism_pattern_shell.go` | Builds and audits the shared wall and caps. See prism-pattern-union design. |
| `prism_pattern_proof.go` | Checks retained source walls, then builds the joined body from certified bounds. See prism-pattern-union design. |
| `prism_pattern_fillet_admission.go` | Matches selected edges to authenticated source roots. See prism-pattern-union design. |
| `prism_pattern_fillet_proof.go` | Charges rounded boundary, area and occupied volume. See prism-pattern-union design. |
