# decad file layout: prism pattern Union

Circular-prism joins of radial copied sections.
The rules for rows live in `docs/layout.md`.

## Layout

| Path | Responsibility |
|---|---|
| `prism_pattern_source.go` | Authenticates the blank and radial section copies. See prism-pattern-union design. |
| `prism_pattern_union.go` | Certifies sectors and dispatches the structural Union. See prism-pattern-union design. |
| `prism_pattern_shell.go` | Builds and audits the shared wall and caps. See prism-pattern-union design. |
| `prism_pattern_cap.go` | Checks omitted root chords inside each joined cap. See prism-pattern-union design. |
| `prism_pattern_proof.go` | Charges seam boundary and occupied-volume bounds. See prism-pattern-union design. |
