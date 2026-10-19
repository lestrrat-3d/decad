# decad file layout: curved booleans

Curved LoftFaces and Revolve Boolean files.
The rules for rows live in `docs/layout.md`.

## Layout

| Path | Responsibility |
|---|---|
| `revolve_loft_faces_source.go` | Authenticates the live cone-trimmed LoftFaces source and finite Revolve root, and admits the live Sketch point source. See loft-face-sections design. |
| `revolve_loft_faces_manifest.go` | Binds mapped source cells to current LoftFaces and Revolve producers and face origins, records one source candidate or strict miss per live Revolve boundary face, and compares retained source-side cells with current producers. See loft-face-sections design. |
| `revolve_loft_faces_vertical.go` | Classifies exact constant-X source cells and vertical Revolve side contact or strict miss. See loft-face-sections design. |
| `revolve_loft_faces_tool_cap.go` | Locates exact contact circles between live Revolve sides and LoftFaces trim cones, compares constant-X source cells, and certifies and rechecks occupied heel tool-cap arcs, including sloped Revolve sides. See loft-face-sections design. |
| `revolve_loft_faces_contact_graph.go` | Assigns source-event IDs and binds them to checked open patches and cap-ring blank shells; classifies every live face pair and rechecks live source pairs, arc stations and root seam records. See revolve-loft-contact-graph design. |
| `revolve_loft_faces_cap.go` | Binds occupied heel cap arc endpoints to live LoftFaces wall cells, bounds held heel-cap endpoints against live source boxes, and splits the closed Revolve shell at every occupied heel-cap circle. See loft-face-sections design. |
| `revolve_loft_faces_blank_shell.go` | Builds and audits a closed Revolve shell with source-aligned meridian cuts, and binds all contact graph runs and occupied cap arcs into one audited shell. See revolve-loft-contact-graph design. |
| `revolve_loft_faces_clear_patch.go` | Certifies tool occupancy for a blank-shell triangle whose entire patch clears the trimmed tool. See revolve-loft-contact-graph design. |
| `revolve_loft_faces_vertical_trace.go` | Links graph-owned vertical source cells into one exact parameter path, finds source crossings on shell rays, bounds the held contact path, and inserts it into a fresh all-run shell. See revolve-loft-contact-graph design. |
