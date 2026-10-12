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
| `prism_pattern_fillet_admission.go` | Matches source roots and checks local clearance. See prism-pattern-union design. |
| `prism_pattern_fillet_contact.go` | Brackets exact connector/circle fillet contacts. See prism-pattern-union design. |
| `prism_pattern_fillet_ring.go` | Certifies the held and source ring stations consumed by each fillet. See prism-pattern-union design. |
| `prism_pattern_fillet_sector.go` | Bounds rounded contacts and copied connectors by certified polar sectors. See prism-pattern-union design. |
| `prism_pattern_fillet_shell.go` | Builds and audits rounded local bands and caps. See prism-pattern-union design. |
| `prism_pattern_fillet_proof.go` | Charges rounded boundary, area and occupied volume. See prism-pattern-union design. |
