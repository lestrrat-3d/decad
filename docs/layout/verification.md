# decad file layout: verification

Verification, survey and check files.
The rules for rows live in `docs/layout.md`.

## Layout

### Verification and surveys

| Path | Responsibility |
|---|---|
| `verify.go` | Public verification option tiers and `Document.Verify` orchestration. See verification §1–§3. |
| `verify_pairs.go` | `Verify`'s pair proofs and job list. See interference §2. |
| `report.go` | Public report aliases. |
| `verify_tolerance.go` | Adapts `internal/tolerance/` to `Verify` readings and diagnostics. See verification §2-§3. |
| `verify_gate.go` | Verify's payload diameter adapters. See verification §3. |
| `verify_gate_points.go` | Points a gate diameter reads, and the pair diameter. See verification §3. |
| `verify_result.go` | Public verification result aliases. |
| `verify_publish.go` | Adapts private surveys to `internal/reportvocab` publication. See verification §1, §6. |
| `clearance.go` | The pair kernel: `clearancePair` and `sheetSolidPair`. See clearance §1-§3/§6. |
| `clearance_box.go` | Unplaced axis-aligned box pairs: gap from exact planes, ahead of the kernel. |
| `clearance_planar.go` | The exact planar pair arm for mitred, faceted and coil bodies: interference §3.2. |
| `contact_pair.go` | Pair gates and report aliases. See contact-geometry. |
| `contact_box.go` | Source-box admission and public face/measurement mapping. See contact-geometry §4. |
| `contact_faceted_pair.go` | Planar admission, convexity and the bands. See multibody §9–§10. |
| `contact_faceted_manifold.go` / `contact_faceted_patch.go` | Planar manifold faces and witnesses. See multibody §9. |
| `contact_faceted_support.go` | Exact faceted support-face proof and bounded strict separation. See contact-geometry §4. |
| `contact_faceted_sweep.go` | Faceted floor sweeps: exact support face, or clearance by swept boxes. See contact-sweep. |
| `contact_oriented_box.go` | Rotated boxes. See contact-geometry §4. |
| `contact_oriented_patch.go` | Publishes oblique box patches from `internal/pair/box/` geometry. See contact geometry §4. |
| `contact_clipped_patch.go` | Publishes horizontal box patches from exact polygon clips. See contact geometry §4. |
| `contact_sphere.go` / `contact_sphere_sweep.go` | Sphere-box contact and sweep. See contact-sweep §4–§5. |
| `contact_cylinder.go` / `contact_cylinder_sweep.go` | Cylinder contact and sweep adapters over `internal/pair/box/`. See contact-sweep §4–§5. |
| `contact_analytic_manifold.go` | Ruling contact adapters. See contact-geometry §4.5. |
| `contact_sphere_oriented.go` / `contact_sphere_oriented_sweep.go` | Rotated sphere-box report and sweep adapters. See the contact designs. |
| `contact_sphere_pair.go` / `contact_sphere_pair_sweep.go` | Sphere-pair contact and sweep. See contact-sweep §4–§5. |
| `contact_sweep_replay.go` | Replay adapters. See contact-sweep §6. |
| `clearance_cells.go` | Pruned cell walk and face-pair adapter. See clearance §3–§5. |
| `clearance_tiers.go` | Tier adapters and vertex budget. See clearance §3/§6. |
| `clearance_geom.go` | `bodyGeom`: clearance faces, edges and nesting over `internal/clearance/`. See clearance §2–§3. |
| `survey.go` | Adapts analytic wall, undercut and radius readers. See verification §6. |
| `survey_undercut.go` | Folds three-valued undercut readings. |
| `interference.go` | Pairwise overlap behind `Verify`. See interference §4-§8. |
| `motion.go` / `motion_verify.go` | Motion options, reports and pose checks. See motion-check §2–§6. |
| `motion_bound.go` | Reads payload record radii for `internal/motionbound/`. |
| `linkage.go` / `linkage_verify.go` | `VerifyLinkage`. See linkage design. |
| `linkage_box.go` | `VerifyJointBox` and its public joint-space inputs and reports. See `docs/linkage-check-design.md`. |
| `linkage_loop.go` | Closed loops and `Schedule`. See `docs/linkage-check-design.md` §15. |
| `linkage_bound.go` | Linkage reach and projection adapters. See linkage §5.2, §5.8. |
| `contact_sweep.go` | Sweeps, report aliases, and tracks. |
| `contact_sweep_rotation.go` / `contact_sweep_faceted.go` | Rotating and planar sweep adapters. See contact-sweep §4. |
| `contact_sweep_memo.go` | Sweep memo adapter. See contact-sweep §7. |
| `contact_sweep_band.go` / `contact_sweep_rolling.go` | Contact bands and rolling. See multibody §10. |
| `swept_box.go` | Whole-path box. See multibody §4.2. |
