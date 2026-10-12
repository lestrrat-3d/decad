# decad file layout: curved booleans

Curved LoftFaces and Revolve Boolean files.
The rules for rows live in `docs/layout.md`.

## Layout

| Path | Responsibility |
|---|---|
| `revolve_loft_faces_source.go` | Authenticates the live cone-trimmed LoftFaces source and finite Revolve root. See loft-face-sections design. |
| `revolve_loft_faces_fit.go` | Proves fitted-curve sides and moving root crossings from exact mapped Bernstein coefficients. See loft-face-sections design. |
| `revolve_loft_faces_arc.go` | Proves finite meridian sides for live Sketch arc parameter and band-time cells. See loft-face-sections design. |
| `revolve_loft_faces_arc_derivative.go` | Bounds a mapped Sketch arc implicit derivative over one source and band-time cell. See loft-face-sections design. |
| `revolve_loft_faces_arc_radial.go` | Proves an entire live Sketch arc has one axial and cone-implicit value at each band time. See loft-face-sections design. |
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
| `revolve_loft_faces_held.go` | Audits held face switches for source-certified curved contact. See loft-face-sections design. |
| `revolve_loft_faces_held_path.go` | Checks complete held switch paths without a fixed face-role table. See loft-face-sections design. |
