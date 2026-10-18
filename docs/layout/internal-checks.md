# decad file layout: internal packages, checks

Internal packages for records, bounds, surveys, sweeps and clearance.
The rules for rows live in `docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/chainrecord/` | The chain ribbon record and its builder over nodes (`Build`, `Placed`, `Prism` view, walk capture and bounds walks); builds one wall face per recorded segment of every walk. Root `chainPayload` aliases it. See surface §13.4, evaluator §8. |
| `internal/draftrecord/` | The draft and two-sided draft records and their builder over nodes (`Build`, `BuildTwoSided`, `Placed`, band view, offset span, measurements, gate and motion readings). Root `draftPayload` and `twoSidedDraftPayload` alias it. See draft §5–§8, evaluator §8. |
| `internal/stackedrecord/` | Stacked record and node builder (`Build`, `Placed`, `Plan`, extents, runs, bounds); builds blind-cut slabs through `prismcells`; derives and audits interfaces, wall columns, exposed patches and the stacked mesh. Root `stackedPrismPayload` aliases it. See stacked-prism §2.2–§5, evaluator §8. |
| `internal/twistsweep/` | The twisted sweep payloads and their builder over nodes, with polygon admission, stations, Loft cell sewing, wall areas, offset source and cup mesh. Root `twistedSweepPayload` and `compositeTwistedSweepPayload` alias it. See sweep §17, evaluator §8. |
| `internal/sectionclip/` | Exact section records for prism, stack slab and Loft (parallel, smooth rectangle, homothetic); oblique clips, cap polygons and circle-line trim; cardinal sphere and cone sections; rational plane arithmetic. See section design. |
| `internal/sectioncert/` | Private bounded-section Sketch provenance, guarded Extrude recording, and `RecordProfile`, the Sketch-profile recording seam. See section design, sketch-seam §1–§2. |
| `internal/exactrotation/` | Exactly orthogonal rational rotation recorded at Copy time: scale-safe quaternion construction, exact translation, structural X-axis test, and the reject-only gap against r3's float rotation. See file comments. |
| `internal/proofbound/` | Certified bounds, intervals, work budgets and trig. See file comments. |
| `internal/compositemesh/` | Plans shared composite Sweep stations, maps reduced span rings and source roles, and assembles a transient mesh node with its numeric audits. See sweep Table D. |
| `internal/compositesweep/` | Composite Sweep limits, span assembly (cap pairing, rim sewing, cap removal), boundary and vertex-link audits, bounded and body measurements, separation, body-extent and cross-span facet audits, rim-join certificate, side-role and cap-role rewrites. See sweep §7, §9. |
| `internal/diameter/` | Lower-bound diameter of held witness points, including vertex, axis-extreme and prism station witnesses and the free-form prism section arm. See verification §3. |
| `internal/tolerance/` | Relative tolerance comparisons, body reference formulas, and reading diagnostics. See verification §2-§3. |
| `internal/verifyoption/` | Verify options. See verification §2. |
| `internal/featureoption/` | Profile-fed feature option records and validation. See evaluator §5–§6, sweep §2/§16, loft §2. |
| `internal/modifyoption/` | Fillet, Chamfer and Shell option codecs and side-role segment reading. See modify-reach §2. |
| `internal/orderedwork/` | Runs independent jobs and returns results in input order. |
| `internal/prismextent/` | Prism and profile boundary extremes, bounds and extent readings. See evaluator §5. |
| `internal/prismplacement/` | Exact prism axis shifts, sweep spans, relative placement and exact rectangular boxes in a reference frame. See prism-boolean §3. |
| `internal/circularbounds/` | Circular endpoints, lengths, area, moment and arc-box bounds over neutral records, and certified circular bore rings. |
| `internal/prismcells/` | Prism admission, scene budgets and construction, §4.2 union, cut, intersect and crossing resolution and records, merge audits, cells, charges, trim walks, the placed-axis rectangle gate, surface operands, Split pieces and profile record context helpers. See prism-boolean §4, §6, G3. |
| `internal/prismpattern/` | Prism-pattern Union blank and copy authentication, radial source gaps, wall certificates, interval fillet contacts, ring cuts, polar sectors, clearance, rounded shell and caps, root chords and section separation. See prism-pattern-union design. |
| `internal/mirrorjoin/` | Exact line admission, reflection, prism and stacked record rewrites, splice and joined-section audit. See mirror-pattern §5. |
| `internal/patternrecord/` | Pattern spec gates, instance motion and record mapping. See mirror-pattern §4.3, §6.2. |
| `internal/massmoment/` | Rational mass moments and inertia, and X-axis placement map charges. See dynamic-mass §2–§3. |
| `internal/capcontour/` | Cap contour displacement, shell offset intervals, and edge and arc bounds. See modify-reach §8.3-§8.4. |
| `internal/offset2d/` | Offset carriers, proofs, profile offsets and corner loops, rewrite and offset audits, offset displacement, and the draft near and far section stages. See modify §5–§9, shell-opening §3–§5, draft §2, §7. |
| `internal/capband/` | Cap-band loop walks, offset joins, contour, loop boundary, patch, locus, volume, first-moment and centroid bounds, and cap-loop chamfer refusals. See modify-reach §8.3–§8.4. |
| `internal/capblendsurvey/` | Cap-blend and Draft undercut readers, patch normal ranges, and cap-blend concave-radius reading. See modify-reach Table DX and draft DD7. |
| `internal/analyticundercut/` | Maps prism, revolve and cup records to their own-step faces and decides their undercut surveys. See verification §6. |
| `internal/filletband/` | Loop-fillet closed forms and reference-frame band mass. See loop-fillet §5. |
| `internal/decaderr/` | The sentinel error values `errors.go` re-exports, so internal packages can return them. |
| `internal/cupwall/` | The cup record and its two-prism view, its builder over nodes (`Build`, `Placed`, cup roles and face order), and the cup wall theorem and morphology recheck. Root `cupPayload` and `cupView` alias it. See modify-reach §9.1, payload verification §4, evaluator §8. |
| `internal/shellsurvey/` | Shell inradius, section cavity limit, work budget and contained-disk witness. See modify §8. |
| `internal/survey2d/` | 2D disks, prism readers, walks, corner loops, Bézier carriers. See verification §6. |
| `internal/thickenaxis/` | Holds certified section pairs, checks offsets, ribbons and interval clearance, and builds the circle and axis offset sections of a prism sheet. See surface §16, §17. |
| `internal/revolvesurvey/` | Revolve wall, undercut and concave-radius readers. See verification §6. |
| `internal/wallsurvey/` | Prism and revolve wall record preparation and bounded readings, and the revolve clearance carrier input. See verification §6, clearance §2. |
| `internal/radiussurvey/` | Prism, revolve, cup and BRep concave-radius readings. See verification §6. |
| `internal/motionbound/` | Motion variants, exact parameters, poses, box bounds, pair certificates, record radii, overlap transfer and sweeps. |
| `internal/motionrun/` | Static mover/stationary pair validity, swept-box exclusion and sheet classification over topology nodes. See motion-check §2–§6. |
| `internal/motionoption/` | Motion options. See motion-check §3. |
| `internal/planarsweep/` | Support-plane records, motion, vertex rates, depth and rolling bounds. |
| `internal/sweepdeparture/` | Exact source-box departure proofs. See multibody §10.2. |
| `internal/sweepmemo/` | Sweep path replay coverage, pose bits and path memo tables. See contact-sweep §6–§7. |
| `internal/radiusmemo/` | Body rotational sweep-radius keys and bounded memo readings. See contact-sweep §6–§7. |
| `internal/spherepath/` | Sphere path gaps, face corridors, point transfer, track bounds, brackets and pair replay. See contact-sweep §4–§6. |
| `internal/linkagebound/` | Link bounds and projections. See linkage §5, §15. |
| `internal/loopscene/` | Closed-loop sketch scenes. See linkage §15.2. |
| `internal/linkagebound/loopchain/` | Sketch loop enclosures. See linkage §15. |
| `internal/polynomial/` | Exact polynomial arithmetic and root brackets, and exact Bernstein rows: values, signs, splits, restrictions, products and integer-row chains. |
| `internal/loftface/` | LoftFaces exact cap maps, band gates, root isolation, source cells, vertical-trace box bounds, cone-tangent end map gates, and the glued tangent-cap prism and its clearance gate. See loft-face-sections design. |
| `internal/freeform/` | Free-form curve proofs. |
| `internal/meshbool/` | Pair chord inputs, mesh boolean contact, subdivision, stitching, embedding, exact similarity images, audits, and staging and rim refusals. See evaluator §9. |
| `internal/pointquery/` | Exact point-to-mesh distance and the occupied-volume transfer proof. See point-containment design. |
| `internal/facetedtopology/` | Chains audited mesh face boundaries, proves axial chain directions, selects loops, resolves source-group faces and matches cap patches. See evaluator §9. |
| `internal/clearance/` | Clearance geometry, edge carriers, shell witnesses and cell sums. See clearance design. |
| `internal/clearance/facepair/` | Face-pair cells. See clearance §4. |
| `internal/clearance/curvecells/` | Face-edge and edge-edge cells. See clearance §4. |
| `internal/clearance/tier/` | Vertex cells and ruling proofs. See clearance §3/§6. |
| `internal/clearance/spine/` | Point, line and circle spine cell pairs. See clearance §4. |
| `internal/stitchflux/` | Stitch exact revolve tag admission, face flux and mass proofs. See surface §6.4. |
| `internal/stitchrecord/` | Stitch/Unstitch payloads and node builders, weld/topology/flux proofs, one-face copies, and sibling-revolve source identity for tessellation. Root aliases its payloads. See surface §6, §10 and evaluator §8. |
| `internal/stitchweld/` | Stitch welds, topology and bounds. See surface §6.2–6.4. |
| `internal/denotation/` | Level and curve identity certificates. See surface §5.2, §6.2. |
| `internal/surfacenormal/` | Exact enclosures and error bounds for analytic face normals. See `normal_bound.go`. |
| `internal/spiralsector/` | Spiral first-join source gates, heel/toe and root-flank proofs, held switches, short caps, blank meridian, contact/output gates, and exact copy/inverse placement. See faceted-vertex-bounds §3.6–§3.7 and loft-face-sections design. |
| `internal/shellseparation/` | Bounded source-shell and held-copy separation, exact slab projections, and placement error. See faceted-vertex-bounds §3.7. |
| `internal/obliqueboss/` | Recognizes a complete oblique boss junction and builds its fillet mesh and boundary, volume and area proofs. See oblique-boss-fillet design. |
