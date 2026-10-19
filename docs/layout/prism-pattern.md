# decad file layout: prism pattern Union

Circular-prism joins of radial copied sections.
The rules for rows live in `docs/layout.md`.

## Layout

| Path | Responsibility |
|---|---|
| `prism_pattern_union.go` | Certifies sectors, dispatches the structural Union, checks retained source walls, and builds the joined body from certified bounds. See prism-pattern-union design. |
| `prism_pattern_shell.go` | Builds and audits the shared wall and caps. See prism-pattern-union design. |
| `prism_pattern_fillet.go` | Matches selected edges to authenticated source roots, and charges the rounded boundary, area and occupied volume. See prism-pattern-union design. |
