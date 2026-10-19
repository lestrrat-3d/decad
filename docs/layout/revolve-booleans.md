# decad file layout: Revolve and sector booleans

Revolve and point-Loft Boolean files.
The rules for rows live in `docs/layout.md`.

## Layout

| Path | Responsibility |
|---|---|
| `revolve_loft_source.go` | Authenticates finite Revolve section spans, retained source faces and candidate conical roots. See revolve-loft-sector design. |
| `revolve_loft_union.go` | Routes one, serial and variadic certified Revolve/point-Loft joins, retains their source records, and selects the live trimmed sector and Revolve root. See revolve-loft-sector design. |
| `revolve_sector_many.go` | Checks live sector sources, builds and audits the joined mesh, and publishes its measured body. See revolve-loft-sector design. |
| `revolve_sector_cylinder_cut.go` | Admits a centered cylinder and rewrites certified Revolve/sector cap fans as annuli. See revolve-loft-sector design. |
