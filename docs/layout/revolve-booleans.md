# decad file layout: Revolve and sector booleans

Revolve and point-Loft Boolean files.
The rules for rows live in `docs/layout.md`.

## Layout

| Path | Responsibility |
|---|---|
| `revolve_loft_union.go` | Routes one, serial and variadic certified Revolve/point-Loft joins and collects the placed sector copies. `internal/revolveshell` authenticates the finite Revolve section, its conical roots and the live trimmed sector. See revolve-loft-sector design. |
