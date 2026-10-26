# decad file layout: internal packages, surveys

Internal packages that answer the wall, undercut and concave-radius questions of Verify.
The rules for rows live in `agent-docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/capblendsurvey/` | Cap-blend and Draft undercut readers, patch normal ranges, and cap-blend concave-radius reading. See modify-reach Table DX and draft DD7. |
| `internal/analyticundercut/` | Maps prism, revolve and cup records to their own-step faces and decides their undercut surveys. See verification §6. |
| `internal/survey2d/` | 2D disks, prism readers, walks, corner loops, Bézier carriers. See verification §6. |
| `internal/revolvesurvey/` | Revolve wall, undercut and concave-radius readers. See verification §6. |
| `internal/wallsurvey/` | Prism and revolve wall record preparation and bounded readings; revolve clearance carrier construction and bounds. See verification §6, clearance §2. |
| `internal/radiussurvey/` | Prism, revolve, cup and BRep concave-radius readings. See verification §6. |
| `internal/meshthick/` | Ray-cast wall thickness and pull undercuts of a closed triangle mesh through a bounding-volume tree. See mesh-wall-survey design. |
