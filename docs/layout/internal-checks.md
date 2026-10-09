# decad file layout: internal packages, checks

Internal packages for records, bounds, surveys, sweeps and clearance.
The rules for rows live in `docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/stackedrecord/` | Derives and audits slab interfaces, wall columns, and exposed patch rings. See stacked-prism §2.2–§3. |
| `internal/proofbound/` | Certified bounds, intervals, work budgets and trig. See file comments. |
| `internal/compositesweep/` | Composite Sweep limits, bounded measurements and separation audit. See sweep §7, §9. |
| `internal/diameter/` | Lower-bound diameter of held witness points. See verification §3. |
| `internal/tolerance/` | Relative tolerance comparisons, body reference formulas, and reading diagnostics. See verification §2-§3. |
| `internal/verifyoption/` | Verify options. See verification §2. |
| `internal/featureoption/` | Profile-fed feature option records and validation. See evaluator §5–§6, sweep §2/§16, loft §2, helix §4. |
| `internal/modifyoption/` | Fillet, Chamfer and Shell option codecs. See modify-reach §2. |
| `internal/orderedwork/` | Runs independent jobs and returns results in input order. |
| `internal/prismextent/` | Prism extremes, bounds and extent readings. See evaluator §5. |
| `internal/prismplacement/` | Exact prism axis shifts, sweep spans and relative placement. See prism-boolean §3. |
| `internal/circularbounds/` | Circular endpoints, lengths, area, and moment bounds over neutral records. |
| `internal/prismcells/` | Prism admission, scene budgets, cells, cuts, charges and trim walks. See prism-boolean §4. |
| `internal/mirrorjoin/` | Exact line admission, reflection and record splice. See mirror-pattern §5. |
| `internal/patternrecord/` | Pattern spec gates, instance motion and record mapping. See mirror-pattern §4.3, §6.2. |
| `internal/massmoment/` | Rational mass moments and inertia. See dynamic-mass §2–§3. |
| `internal/capcontour/` | Cap contour displacement, shell offset intervals, and edge and arc bounds. See modify-reach §8.3-§8.4. |
| `internal/offset2d/` | Offset carriers, proofs. See modify §6–§9, shell-opening §3–§5, draft §2. |
| `internal/capband/` | Cap-band contour, patch, locus, volume, first-moment and centroid bounds. See modify-reach §8.3–§8.4. |
| `internal/filletband/` | Loop-fillet closed forms. See loop-fillet §5. |
| `internal/decaderr/` | The sentinel error values `errors.go` re-exports, so internal packages can return them. |
| `internal/cupwall/` | Cup wall theorem and morphology recheck. See payload verification §4. |
| `internal/shellsurvey/` | Shell inradius, work budget and contained-disk witness. See modify §8. |
| `internal/survey2d/` | 2D disks, prism readers, walks, Bézier carriers. See verification §6. |
| `internal/thickenaxis/` | Certifies offsets, ribbons and interval clearance. See surface §16. |
| `internal/revolvesurvey/` | Revolve wall, undercut and concave-radius readers. See verification §6. |
| `internal/wallsurvey/` | Prism and revolve wall record preparation and bounded readings. See verification §6. |
| `internal/radiussurvey/` | Prism, revolve, cup and BRep concave-radius readings. See verification §6. |
| `internal/motionbound/` | Motion variants, exact parameters, poses, box bounds, pair certificates, record radii, overlap transfer and sweeps. |
| `internal/motionoption/` | Motion options. See motion-check §3. |
| `internal/planarsweep/` | Plane selection, motion, vertex rates, depth and rolling bounds. |
| `internal/sweepdeparture/` | Exact source-box departure proofs. See multibody §10.2. |
| `internal/sweepmemo/` | Sweep replay coverage and memo tables. See contact-sweep §6–§7. |
| `internal/spherepath/` | Sphere path gaps, brackets and pair replay. See contact-sweep §4–§6. |
| `internal/linkagebound/` | Link bounds and projections. See linkage §5, §15. |
| `internal/loopscene/` | Closed-loop sketch scenes. See linkage §15.2. |
| `internal/linkagebound/loopchain/` | Sketch loop enclosures. See linkage §15. |
| `internal/polynomial/` | Exact polynomial arithmetic and root brackets. |
| `internal/freeform/` | Free-form curve proofs. |
| `internal/meshbool/` | Mesh boolean contact, subdivision, stitching, embedding, audits. See evaluator §9. |
| `internal/facetedtopology/` | Chains audited mesh face boundaries and selects loops. See evaluator §9. |
| `internal/clearance/` | Clearance geometry and cell sums. See clearance design. |
| `internal/clearance/facepair/` | Face-pair cells. See clearance §4. |
| `internal/clearance/curvecells/` | Face-edge and edge-edge cells. See clearance §4. |
| `internal/clearance/tier/` | Vertex cells and ruling proofs. See clearance §3/§6. |
| `internal/clearance/spine/` | Point, line and circle spine cell pairs. See clearance §4. |
| `internal/stitchflux/` | Stitch exact revolve tag admission, face flux and mass proofs. See surface §6.4. |
| `internal/stitchweld/` | Stitch welds, topology and bounds. See surface §6.2–6.4. |
| `internal/denotation/` | Level and curve identity certificates. See surface §5.2, §6.2. |
| `internal/surfacenormal/` | Exact enclosures and error bounds for analytic face normals. See `normal_bound.go`. |
