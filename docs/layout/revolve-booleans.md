# decad file layout: Revolve and sector booleans

Revolve and point-Loft Boolean files.
The rules for rows live in `docs/layout.md`.

## Layout

| Path | Responsibility |
|---|---|
| `revolve_loft_union.go` | Routes one, serial and variadic certified Revolve/point-Loft joins, selects the live trimmed sector and Revolve root, and authenticates finite Revolve section spans and conical roots. See revolve-loft-sector design. |
| `revolve_sector_cylinder_cut.go` | Admits a centered cylinder and rewrites certified Revolve/sector cap fans as annuli. See revolve-loft-sector design. |
