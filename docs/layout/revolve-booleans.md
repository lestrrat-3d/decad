# decad file layout: Revolve and sector booleans

Revolve and point-Loft Boolean files.
The rules for rows live in `docs/layout.md`.

## Layout

| Path | Responsibility |
|---|---|
| `revolve_loft_source.go` | Authenticates finite Revolve section spans, retained source faces and candidate conical roots. See revolve-loft-sector design. |
| `revolve_loft_sector.go` | Certifies a live trimmed point-Loft sector's Sketch contacts and source sides. See revolve-loft-sector design. |
| `revolve_loft_fit.go` | Certifies fitted flank contacts and side transitions against the finite Revolve root. See revolve-loft-sector design. |
| `revolve_loft_seat.go` | Bounds exact Sketch line crossings against the recorded finite Revolve meridian. See revolve-loft-sector design. |
| `revolve_loft_path.go` | Selects the certified exterior point-Loft fan path and root sector. See revolve-loft-sector design. |
| `revolve_loft_union.go` | Routes one, serial and variadic certified Revolve/point-Loft joins and retains their source records. See revolve-loft-sector design. |
| `revolve_section_angles.go` | Chooses bounded angular stations for a finite Revolve section. See revolve-loft-sector design. |
| `revolve_section_ring.go` | Meshes ordered meridian spans into shared section rings. See revolve-loft-sector design. |
| `revolve_section_tool_gap.go` | Bounds cone-tool reach against the recorded meridian. See revolve-loft-sector design. |
| `revolve_sector_cap.go` | Closes a certified exterior path on the root cone. See revolve-loft-sector design. |
| `revolve_sector_mesh.go` | Stitches mapped point-Loft caps to shared section rings. See revolve-loft-sector design. |
| `revolve_sector_many.go` | Builds and audits one or more certified sectors against one Revolve. See revolve-loft-sector design. |
| `revolve_sector_placement.go` | Checks axis placements, complete-source separation and angular gaps. See revolve-loft-sector design. |
| `joined_sector_cap.go` | Triangulates and refines source-labeled sector caps with shared root stations. See revolve-loft-sector design. |
| `joined_sector_cap_refine.go` | Refines cone-mapped sector cap cells with shared edges. See revolve-loft-sector design. |
