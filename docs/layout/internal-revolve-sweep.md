# decad file layout: internal revolve and sweep packages

Internal packages for Revolve, Sweep, and coil geometry and records.
The rules for rows live in `docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/sweeppath/` | Pair paths, validation, rounded poses, motion travel bounds, and the node's checked rotational radius reader. |
| `internal/revolveaxis/` | Axis frames, walks, side/contact gates, snap and section charges, extent readings, the simple-revolution radial proof, and wall and cap denotations. See evaluator §6. |
| `internal/revolveshell/` | Surveys shell walls and builds section rings, sector caps, sector plans, numeric bounds, sector meshes, contact checks, bore checks, and the straight-band cylinder Cut mesh and proof. See modify-reach §9.3 and revolve-loft-sector design. |
| `internal/revolveloftfaces/` | Certifies LoftFaces source geometry and Revolve contacts; builds seams, strips, shell rays and their triangle labels, and vertical-trace paths: ray crossings, held vertices, displacement bounds and wedge splits. See loft-face-sections and revolve-loft-contact-graph. |
| `internal/revolvemass/` | Revolve moments and centroid bounds. |
| `internal/revolveangle/` | Exact angle denotations and certified sweep extremes. See evaluator §6 and sweep §3. |
| `internal/sweeparc/` | Exact circle carriers, rational vectors, axis lines, plane gates and angle bounds. See sweep §3. |
| `internal/sweepinput/` | Recorded Sweep `Path`, exact path records, tangent gates, path validation, recorded section gates and closed-path class gates. See sweep §3, §5, §15.4. |
| `internal/sweepmitre/` | The mitred sweep payload and its builder over nodes: profile gates, exact sections, placement, rounding, triangle assembly, Table BM topology and measurements. Root `mitredSweepPayload` aliases it. See sweep §16, evaluator §8. |
| `internal/sweeprecord/` | The sweep and chain sweep records, their span payloads and builders over nodes (`BuildComposite`, `BuildArc`, `BuildClosedCyclic`, `ReplayChainWork`, `Placed`, finishing roles). Root `sweepPayload` and `chainSweepPayload` alias it. See sweep §3, §15, evaluator §8. |
| `internal/coil/` | Coil magnitude, profile side and pitch gates, preflight gates, stations, axis coordinates, moments and closed forms. See helix §4, §5, §7. |
| `internal/coilshell/` | The coil payload and its builder over nodes: the held shell, Table CB topology, Table CM readings, mesh proofs and wall-area enclosures. Root `coilPayload` aliases it. See helix §5, §7, §8, evaluator §8. |
| `internal/sweeptransport/` | Bounded rotation-minimizing endpoint frames over recorded line and arc spans, and frames along a recorded path. See sweep §3.2. |
| `internal/revolvemesh/` | Revolve rings, cells, caps, cap frames, construction and area proofs, quarter-turn cap-rim blend stations, faceted mesh and bounds, and sector-bore cap fans, station steps and projected patches. |
| `internal/revolvetess/` | Assembles Revolve and chain Sweep angular meshes over topology nodes, including retries, sheet and solid audits, and proof publication. See tessellation §8–§11 and sweep §15.8–§15.9. |
| `internal/revolveplan/` | Revolve and chain records, common-axis Boolean preparation, node builders, axis-side, Trim and Extend pair gates, and mesh walks. Root `revolvePayload` aliases its record. See evaluator §6, §8; general-boolean §3; surface-intersection §2.1. |
| `internal/revolvesampling/` | Revolve meridian junctions, stations, samples and section readings. |
| `internal/revolveproof/` | Meridian envelopes, facet budgets, circular and fitted-spline meridian volume charges, cell bounds, chain-arc source density, and point-Loft sector source contacts, root, seat and exterior-path proofs. See tessellation §8–§11, sweep §15.9 and revolve-loft-sector design. |
