# decad file layout: design documents

Design documents, user guides, seam and records, mass properties and free-form curves.
The rules for rows live in `agent-docs/layout.md`.

## Layout

### Design documents

| Path | Responsibility |
|---|---|
| `agent-docs/container-design.md` | `Container`: grouping of sketches, planes and bodies, result placement, `AddBody`, `ErrCrossContainer`. |
| `agent-docs/api-design.md` | Public API contract for modeling, features, selectors, and verification. |
| `agent-docs/body-digest-design.md` | `Body.Digest` and `Document.PairDigest`: what the walk covers, id relabeling, refusals and the C1-C5 soundness tests. |
| `docs/api.md` | User-facing API reference by task; `apitest/api_reference_test.go` checks its names exist. |
| `docs/data-structures.md` | User-facing type index and structure guide; `apitest/api_reference_test.go` checks it lists every exported type. |
| `agent-docs/sketch-seam-design.md` | `sketch` recording: `TExact`, `CurveSegment`, and `ErrUnrecordableProfile`. |
| `agent-docs/verification-design.md` | `Verify` reports, statuses, interference costs, deadlines, `WithTolerance`, and noise floor. |
| `agent-docs/payload-verification-design.md` | Per-payload proofs, certificates, bounded validity/clearance/survey algorithms. |
| `agent-docs/evaluator-design.md` | Evaluator topology, payloads, mass properties, feature builds, and mesh booleans. |
| `agent-docs/tessellation-design.md` | Tessellation: curve samples, manifold proofs, source faces, certificates, boolean handoff. |
| `agent-docs/clearance-design.md` | Pair disjointness and bounded gap proofs. |
| `agent-docs/proofs-design.md` | Lean proofs of decad's bounded arithmetic: layout, assumptions, trace manifest, CI, upgrade and recipe. |
| `agent-docs/clearance-band-design.md` | Band readings of the clearance cells for placed carriers the degeneracy oracle cannot decide. |
| `agent-docs/point-containment-design.md` | Public point classification, distance, mesh-to-body proofs, the exact prism-record route and the Boolean-history route. |
| `agent-docs/section-design.md` | Read-only planar section gate and Sketch reconstruction proof. |
| `agent-docs/interference-design.md` | Read-only pair overlap and bounded volume proofs. |
| `agent-docs/coplanar-contact-design.md` | Exact coplanar contact classification in the mesh boolean and exact planar faces of its results. |
| `agent-docs/tiny-bound-contact-design.md` | Exact planar face carriers, the carrier snap of rounded vertices, and rational chamfer feet. |
| `agent-docs/flush-contact-design.md` | Admitting the coplanar contact of two planar faces held within a bound when both planes are exact and the outlines keep their arrangement. |
| `agent-docs/union-face-merge-design.md` | Merging adjacent coplanar same-facing faces of different operands in a mesh Boolean result into one face. |
| `agent-docs/exact-chamfer-design.md` | Exact setback points and bounds for the retained R1 straight-edge `Chamfer` of a mesh Boolean result. |
| `agent-docs/hole-design.md` | `Hole`: face and point placement, size gates, tool shapes and the atomic cut. |
| `agent-docs/rib-design.md` | `WebPath`: line and arc outlines, extent, overrun, draft, touch and volume checks, and the merged join. |
| `agent-docs/emboss-design.md` | `Emboss`: face-derived sweep, plane checks, margin, volume checks and the shared join. |
| `agent-docs/modify-design.md` | `Fillet`/`Chamfer`/`Shell` tables, section rewrite, exact offset, build audit, and as parts every modify reach, blend, shell-opening and draft design. |
| `agent-docs/twist-sweep-cut-design.md` | Stationed refinement of a twisted Sweep so Cut, Union and Intersect admit it, and the fine partner chord that admits chained twisted bore and window cuts. |
| `agent-docs/spline-design.md` | Free-form kinds, exactness tiers, refusals, Tier A moments, work budget, proven brackets, and reach. |
| `agent-docs/loft-design.md` | `Loft` pairing, refusals, results, consumers, chains, mass properties, wall-crossing audit, and `LoftFromPoint` (§20). |
| `agent-docs/loft-sections-design.md` | Smooth-section loft classes, held meshes and proofs. |
| `agent-docs/loft-face-sections-design.md` | Source-cap nine-section loft, cap correspondence, embedding, occupied-volume proof and two-cone continuation. |
| `agent-docs/revolve-axis-design.md` | Revolve/point-Loft sector Union and point-cone trims about a coordinate-plane axis through the origin: exact axis vector, predicates, charges, angle basis, ring, coaxial bore Cut, consumer contract, tests, +X digest. |
| `agent-docs/loft-gear-bounds-design.md` | Loft gear bounds: per-cell residuals, centroid shift, `A/P` target, audit, ceilings. |
| `agent-docs/loft-boolean-design.md` | Loft Booleans with a mirror image and with a cylinder. |
| `agent-docs/sweep-design.md` | `Path`/`Sweep`, its chain section (§15), mitred sweeps (§16) and bounded one-cell, composite and stationed twist (§17). |
| `agent-docs/helix-design.md` | `Coil`: a profile screwed about an in-plane axis. |
| `agent-docs/prism-boolean-design.md` | Analytic `Union`/`Cut`/`Intersect` over co-directional prisms, and as parts the general, regroup, continuation, stacked-prism, carrier-record, pattern and fold Boolean designs. |
| `agent-docs/mirror-pattern-design.md` | Mirror and pattern entry points, the exact mirror join, the prism group. |
| `agent-docs/tessellation-reach-design.md` | Tessellation reach for lofts, free-form prisms, revolves and cap-loop chamfers. |
| `agent-docs/faceted-vertex-bounds-design.md` | Per-vertex displacement bounds on faceted bodies and their boolean composition. |
| `agent-docs/param-binding-design.md` | Reading feature values from a `sketch/param` table, and the ledger of those reads. |
| `agent-docs/gap-tolerance-design.md` | The document's gap tolerance: the reach of the held-mesh embedding repair, its default, its limits and its charge. |
| `agent-docs/operation-history-design.md` | Computational proof lineage, source-derived quantity calculations, operation snapshots, and shared operation kinds. |
| `agent-docs/contained-cut-moments-design.md` | Certified contained-Cut V/P/Q source moments, faceted mass-property selection, bounds, refusals, and their exact-rational transfer through one rotation Copy. |
| `agent-docs/operation-diagnostics-design.md` | Operation failure diagnostics, successful-call warnings, delivery APIs, compatibility rules, and proof fixtures. |
| `agent-docs/faceted-source-retention-design.md` | Analytic source surfaces carried onto mesh Boolean result faces: claim, admission, normal bound, readers, increments. |
| `agent-docs/surface-intersection-design.md` | `Trim`/`Extend`/`Split` over shared-generator sweeps. |
| `agent-docs/surface-design.md` | Sheet bodies, surface operations, verification and export. |
| `agent-docs/linkage-check-design.md` | `VerifyLinkage` and `VerifyJointBox`, and as a part `VerifyMotion`: motions, the per-pose proof, the interval certificate, `MotionReport`. |
| `agent-docs/multibody-dynamics-design.md` | N-body dynamics, islands, `Timeline`, and as parts the system map, rigid response, mass readings, contact geometry and the two-body sweep. |
| `agent-docs/planar-convex-filter-design.md` | Float pre-test of the §9.2 convexity certificate: error bound, range guard, poll count. |
| `agent-docs/step-export-design.md` | Export package entry points and the AP214 faceted writer contract. |

### User guides

| Path | Responsibility |
|---|---|
| `agent-docs/collision-v1-support.md` | Certified collision paths, response limits, and refusal outcomes. |
| `agent-docs/missing-features.md` | Index of what decad refuses or lacks, each row pointing at its owner. |

### Seam and records

| Path | Responsibility |
|---|---|
| `doc.go` | Package scope, support map and layering. |
| `errors.go` | Sentinel errors (from `internal/decaderr/`) and `BooleanError`. See api §12, §8. |
| `measurement.go` | Public bounded reading types and their internal adapters. |
| `record.go` | Root-owned `Point2`, structural record and payload aliases (including the faceted payload and its records), and slice adapters. See sketch-seam §2. |
| `seam.go` | Sketch chain recording adapters, `MeasureProfile` and `SecondMoments`; profile recording lives in `internal/sectioncert/`. See sketch-seam §1–§2. |
| `section_outline.go` | Public `SectionOutline`, `Body.SectionOutline` (the verified mesh cut by a plane via `internal/meshslice/`) and `SectionOutline.Clearance`. See section-outline design. |
| `section_orthogonal.go` | The orthogonal-polyhedron route of `Body.Section`: gates the frame and body, `internal/orthoshell/` traces the cut and `sectionclip` rebuilds the sketch. See orthogonal-section design. |
| `section.go` | Planar section query over certified prism, stack-backed BRep, stacked prism, Loft, sphere, cone, Revolve meridian and mesh Boolean history records; builders live in `internal/sectionclip/` and `internal/sectionhistory/`; also the `Section` constraint. See section design, surface §13.2. |
| `path.go` | The immutable spatial `Path`, sealed segments and its recorded `internal/sweepinput.Path`. See sweep §2–§3. |
| `extent.go` | Public linear and angular extent variants and normalization. See API §8.1. |
| `selector.go` | `EdgeQuery`/`FaceQuery`, which resolve against live bodies through `internal/selectorquery` node views. See API §9. |
| `selection_error.go` | Public selector error types and query rendering adapters. See API §9. |

### Mass properties and free-form curves

| Path | Responsibility |
|---|---|
| `mass_properties.go` | Public mass and inertia types and the `Body.MassProperties` conversion over `internal/massdispatch/`. See `agent-docs/multibody-dynamics-design.md` Part M §2–§3. |
