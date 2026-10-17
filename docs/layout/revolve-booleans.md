# decad file layout: Revolve and sector booleans

Revolve and point-Loft Boolean files.
The rules for rows live in `docs/layout.md`.

## Layout

| Path | Responsibility |
|---|---|
| `revolve_loft_source.go` | Authenticates finite Revolve section spans, retained source faces and candidate conical roots. See revolve-loft-sector design. |
| `revolve_loft_sector.go` | Certifies a live trimmed point-Loft sector's Sketch contacts and source sides. See revolve-loft-sector design. |
| `revolve_loft_fit.go` | Certifies fitted flank contacts and side transitions against the finite Revolve root. See revolve-loft-sector design. |
| `revolve_loft_union.go` | Routes one, serial and variadic certified Revolve/point-Loft joins and retains their source records. See revolve-loft-sector design. |
| `revolve_sector_many.go` | Builds and audits one or more certified sectors against one Revolve. See revolve-loft-sector design. |
| `revolve_sector_cylinder_cut.go` | Admits a centered cylinder and rewrites certified Revolve/sector cap fans as annuli. See revolve-loft-sector design. |
| `revolve_sector_cylinder_band.go` | Rewrites one exact straight meridian-band crossing for a centered cylinder Cut. See revolve-loft-sector design. |
