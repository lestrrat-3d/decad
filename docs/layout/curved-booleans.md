# decad file layout: curved booleans

Curved LoftFaces and Revolve Boolean files.
The rules for rows live in `docs/layout.md`.

## Layout

| Path | Responsibility |
|---|---|
| `revolve_loft_faces_source.go` | Authenticates the live cone-trimmed LoftFaces source and finite Revolve root. See loft-face-sections design. |
| `revolve_loft_faces_fit.go` | Proves fitted-curve sides and moving root crossings from exact mapped Bernstein coefficients. See loft-face-sections design. |
| `revolve_loft_faces_manifest.go` | Binds mapped source cells to current LoftFaces and Revolve producers and face origins. See loft-face-sections design. |
| `revolve_loft_faces_vertical.go` | Classifies exact constant-X source cells and vertical Revolve side contact or strict miss. See loft-face-sections design. |
| `revolve_loft_faces_contact_manifest.go` | Records one source candidate or strict miss for each live Revolve boundary face. See loft-face-sections design. |
| `revolve_loft_faces_tool_cap.go` | Locates possible exact contact circles between live Revolve sides and LoftFaces trim cones. See loft-face-sections design. |
| `revolve_loft_faces_simple_source.go` | Admits the current live Sketch point source before the internal simple curve walk. See loft-face-sections design. |
| `revolve_loft_faces_tool_cap_occupancy.go` | Certifies occupied heel tool-cap arcs from live source cells. See loft-face-sections design. |
| `revolve_loft_faces_tool_cap_sloped.go` | Proves radial fill and occupied heel tool-cap arcs on sloped Revolve sides. See loft-face-sections design. |
| `revolve_loft_faces_tool_cap_current.go` | Rechecks recorded heel tool-cap arcs against live source and circle producers. See loft-face-sections design. |
| `revolve_loft_faces_tool_cap_cells.go` | Compares complete constant-X source cells before cap-arc endpoints are used. See loft-face-sections design. |
| `revolve_loft_faces_pair_census.go` | Classifies every live finite Revolve and trimmed LoftFaces face pair from current source records. See loft-face-sections design. |
| `revolve_loft_faces_contact_graph.go` | Assigns source-event IDs and binds them to checked open patches and cap-ring blank shells. See revolve-loft-contact-graph design. |
| `revolve_loft_faces_cap_side_tie.go` | Binds occupied heel cap arc endpoints to unique live LoftFaces wall contact cells. See loft-face-sections design. |
| `revolve_loft_faces_seam_vertices.go` | Preserves root seam-vertex records and delegates unchecked placement to internal/revolveloftfaces. See loft-face-sections design. |
| `revolve_loft_faces_seam_patch.go` | Preserves root open-patch records and graph-owned station IDs while delegating unchecked strip splitting. See loft-face-sections design. |
| `revolve_loft_faces_seam_checked.go` | Rechecks live source face pairs and the complete radial arc station run before building open seam patches. See loft-face-sections design. |
| `revolve_loft_faces_blank_shell.go` | Builds and audits a closed Revolve shell with source-aligned meridian cuts. See revolve-loft-contact-graph design. |
| `revolve_loft_faces_blank_shell_multi.go` | Binds all contact graph runs and occupied cap arcs into one audited Revolve shell. See revolve-loft-contact-graph design. |
| `revolve_loft_faces_clear_patch.go` | Certifies tool occupancy for a blank-shell triangle whose entire patch clears the trimmed tool. See revolve-loft-contact-graph design. |
| `revolve_loft_faces_blank_shell_caps.go` | Splits the closed Revolve shell at every live occupied heel-cap circle. See loft-face-sections design. |
| `revolve_loft_faces_cap_seam_vertices.go` | Bounds held heel-cap endpoints against live source boxes. See loft-face-sections design. |
| `revolve_loft_faces_shell_manifest.go` | Compares every retained Revolve and LoftFaces source-side cell against current producers. See loft-face-sections design. |
| `revolve_loft_faces_vertical_trace.go` | Links graph-owned vertical source cells into one exact parameter path. See revolve-loft-contact-graph design. |
| `revolve_loft_faces_vertical_trace_proof.go` | Proves the line and centered-arc source path stays simple inside its cap circle. See revolve-loft-contact-graph design. |
| `revolve_loft_faces_vertical_trace_path.go` | Finds every source crossing on shell rays and bounds the held contact path. See revolve-loft-contact-graph design. |
| `revolve_loft_faces_vertical_trace_wedge.go` | Selects shell wedges and inserts bounded contact edges; internal/revolveloftfaces triangulates them. See revolve-loft-contact-graph design. |
| `revolve_loft_faces_vertical_trace_mesh.go` | Inserts the vertical path into a fresh all-run shell and repeats shell audits. See revolve-loft-contact-graph design. |
