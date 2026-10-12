# decad file layout: curved booleans

Curved LoftFaces and Revolve Boolean files.
The rules for rows live in `docs/layout.md`.

## Layout

| Path | Responsibility |
|---|---|
| `revolve_loft_faces_source.go` | Authenticates the live cone-trimmed LoftFaces source and finite Revolve root. See loft-face-sections design. |
| `revolve_loft_faces_fit.go` | Proves fitted-curve sides and moving root crossings from exact mapped Bernstein coefficients. See loft-face-sections design. |
| `revolve_loft_faces_arc.go` | Proves finite meridian sides for live Sketch arc parameter and band-time cells. See loft-face-sections design. |
| `revolve_loft_faces_arc_radial.go` | Proves an entire live Sketch arc has one axial and cone-implicit value at each band time. See loft-face-sections design. |
| `revolve_loft_faces_arc_cells.go` | Classifies every live mapped Arc band as strict, axially missed, or whole-arc time contact. See loft-face-sections design. |
| `revolve_loft_faces_adaptive.go` | Partitions mapped fitted spans into strict-side and unique-crossing cells. See loft-face-sections design. |
| `revolve_loft_faces_span.go` | Proves mapped Bernstein source-span sides against finite Revolve meridians. See loft-face-sections design. |
| `revolve_loft_faces_joint.go` | Authenticates shared fitted and arc source vertices. See loft-face-sections design. |
| `revolve_loft_faces_joint_band.go` | Isolates radial-arc time roots and partitions adjacent time bands. See loft-face-sections design. |
| `revolve_loft_faces_joint_cell.go` | Certifies one-sided fitted source cells beside isolated time roots. See loft-face-sections design. |
| `revolve_loft_faces_fit_knot.go` | Certifies the exact side and adjacency at an interior fitted Bezier knot. See loft-face-sections design. |
| `revolve_loft_faces_knot_cells.go` | Partitions fitted spans around certified interior-knot time roots. See loft-face-sections design. |
| `revolve_loft_faces_contacts.go` | Enumerates authenticated fitted-knot and radial-arc joints on a finite Revolve side. See loft-face-sections design. |
| `revolve_loft_faces_time_merge.go` | Orders fitted-knot and radial-arc time roots with distinct live source ownership. See loft-face-sections design. |
| `revolve_loft_faces_manifest.go` | Binds mapped source cells to current LoftFaces and Revolve producers and face origins. See loft-face-sections design. |
| `revolve_loft_faces_vertical.go` | Classifies exact constant-X source cells and vertical Revolve side contact or strict miss. See loft-face-sections design. |
| `revolve_loft_faces_contact_manifest.go` | Records one source candidate or strict miss for each live Revolve boundary face. See loft-face-sections design. |
| `revolve_loft_faces_tool_cap.go` | Locates possible exact contact circles between live Revolve sides and LoftFaces trim cones. See loft-face-sections design. |
| `revolve_loft_faces_simple_source.go` | Certifies a live Sketch source boundary as one simple, oriented curve walk. See loft-face-sections design. |
| `revolve_loft_faces_simple_arc.go` | Bounds exact short Sketch arcs and their one-sided endpoint contacts for the simple source walk. See loft-face-sections design. |
| `revolve_loft_faces_tool_cap_occupancy.go` | Certifies occupied heel tool-cap arcs from live source cells. See loft-face-sections design. |
| `revolve_loft_faces_tool_cap_sloped.go` | Proves radial fill and occupied heel tool-cap arcs on sloped Revolve sides. See loft-face-sections design. |
| `revolve_loft_faces_tool_cap_current.go` | Rechecks recorded heel tool-cap arcs against live source and circle producers. See loft-face-sections design. |
| `revolve_loft_faces_tool_cap_cells.go` | Compares complete constant-X source cells before cap-arc endpoints are used. See loft-face-sections design. |
| `revolve_loft_faces_pair_census.go` | Classifies every live finite Revolve and trimmed LoftFaces face pair from current source records. See loft-face-sections design. |
| `revolve_loft_faces_cap_side_tie.go` | Binds occupied heel cap arc endpoints to unique live LoftFaces wall contact cells. See loft-face-sections design. |
| `revolve_loft_faces_seam_vertices.go` | Places one shared held vertex row inside a certified radial arc root bracket and bounds each source side. See loft-face-sections design. |
| `revolve_loft_faces_seam_patch.go` | Splits open Revolve and LoftFaces source strips through shared seam vertices. See loft-face-sections design. |
| `revolve_loft_faces_seam_checked.go` | Rechecks live source face pairs and the complete radial arc station run before building open seam patches. See loft-face-sections design. |
| `revolve_loft_faces_blank_shell.go` | Builds and audits a closed Revolve shell with a source-aligned radial seam. See loft-face-sections design. |
| `revolve_loft_faces_shell_manifest.go` | Compares every retained Revolve and LoftFaces source-side cell against current producers. See loft-face-sections design. |
