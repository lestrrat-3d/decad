# decad file layout: internal revolve and sweep packages

Internal packages for Revolve, Sweep, and coil geometry and records, and for pair-sweep paths.
The rules for rows live in `docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/sweeppath/` | Pair paths, validation, rounded poses, motion travel bounds, and the node's checked rotational radius reader. |
| `internal/planarsweep/` | Prepared sweep body paths (ideal pose, point deviation, rounded box, corner spans and their memo), support-plane records, motion, vertex rates and depth bounds, and the §10.2–§10.4 departure, band, track and rolling certificates with their replay checks. See contact-sweep §3, §7 and multibody §10. |
| `internal/sweepdeparture/` | Exact source-box departure proofs. See multibody §10.2. |
| `internal/sweepmemo/` | Sweep path replay coverage, pose bits, path memo tables and corner-span hull gaps. See contact-sweep §6–§7 and multibody §10.4. |
| `internal/revolveaxis/` | Axis frames, walks, side/contact gates, snap and section charges, extent readings, the simple-revolution radial proof, and wall and cap denotations. See evaluator §6. |
| `internal/revolveshell/` | Shells a Revolve; surveys shell walls; certifies Revolve sections, conical roots and trimmed sectors; routes the Revolve/point-Loft sector Unions; builds and audits their joins from rings, caps, plans, bounds and meshes; admits a cylinder Cut. See modify-reach §9.3, revolve-loft-sector design. |
| `internal/revolveloftfaces/` | Builds LoftFaces bodies and authenticates their cap sections (`Collect`); certifies sources and Revolve contacts; builds seams, strips, the audited blank-shell mesh with its rays and labels, and vertical-trace paths. See loft-face-sections and revolve-loft-contact-graph. |
| `internal/revolvecontact/` | Certifies a LoftFaces tooth against a Revolve blank: source and root, cell manifest, tool-cap circles, contact graph and census, cap seams, blank shell, vertical trace and clear-patch occupancy. See loft-face-sections and revolve-loft-contact-graph. |
| `internal/loftfacestrim/` | Certifies the one-use LoftFaces toe Cut to heel Intersect cone continuation: toe record, heel Boolean and the tangent heel's extended tooth operand. See loft-face-sections design. |
| `internal/revolvemass/` | Revolve moments and centroid bounds. |
| `internal/revolveangle/` | Exact angle denotations and certified sweep extremes. See evaluator §6 and sweep §3. |
| `internal/sweeparc/` | Exact circle carriers, rational vectors, axis lines, plane gates and angle bounds. See sweep §3. |
| `internal/sweepinput/` | Recorded Sweep `Path`, exact path records, tangent gates, path validation, recorded section gates and closed-path class gates. See sweep §3, §5, §15.4. |
| `internal/sweepmitre/` | The mitred sweep payload and its builder over nodes: profile gates, exact sections, placement, rounding, triangle assembly, Table BM topology and measurements. See sweep §16, evaluator §8. |
| `internal/sweeprecord/` | The sweep and chain sweep records, their span payloads, builders over nodes (`BuildComposite`, `BuildArc`, `BuildClosedCyclic`, `ReplayChainWork`, `Placed`, finishing roles) and chain reductions. Root `sweepPayload` aliases it. See sweep §3, §15, evaluator §8. |
| `internal/sweepbuild/` | The profile-fed and chain-fed Sweep builds over nodes: twisted, mitred, closed, composite, arc and straight paths. See sweep §3–§17. |
| `internal/coil/` | Coil magnitude, profile side and pitch gates, preflight gates, stations, axis coordinates, moments and closed forms. See helix §4, §5, §7. |
| `internal/coilshell/` | The coil payload and its builder over nodes: the held shell, Table CB topology, Table CM readings, the held-shell mesh, its proofs and wall-area enclosures. Root `coilPayload` aliases it. See helix §5, §7, §8, evaluator §8. |
| `internal/sweeptransport/` | Bounded rotation-minimizing endpoint frames over recorded line and arc spans, and frames along a recorded path. See sweep §3.2. |
| `internal/revolvemesh/` | Revolve rings, cells, caps, cap frames, construction and area proofs, quarter-turn cap-rim blend stations, faceted mesh and bounds, and sector-bore cap fans, station steps and projected patches. |
| `internal/revolvetess/` | Assembles Revolve and chain Sweep angular meshes over topology nodes, including retries, sheet and solid audits, and proof publication. See tessellation §8–§11 and sweep §15.8–§15.9. |
| `internal/revolveplan/` | Revolve and chain records, common-axis Boolean preparation, node builders, axis-side, Trim and Extend pair gates, mesh walks, the point-cone source read and the point-Loft cone trim. Root `revolvePayload` aliases its record. See evaluator §6, §8; general-boolean §3; surface-intersection §2.1. |
| `internal/revolvesampling/` | Revolve meridian junctions, stations, samples and section readings. |
| `internal/revolveproof/` | Meridian envelopes, facet budgets, circular and fitted-spline meridian volume charges, cell bounds, chain-arc source density, and point-Loft sector source contacts, root, seat and exterior-path proofs. See tessellation §8–§11, sweep §15.9 and revolve-loft-sector design. |
