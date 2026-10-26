# decad file layout: design documents

Design documents, user guides, seam and records, mass properties and free-form curves.
The rules for rows live in `agent-docs/layout.md`.

## Layout

### Design documents

| Path | Responsibility |
|---|---|
| `agent-docs/container-design.md` | `Container`: grouping of sketches, planes and bodies, result placement, `MoveBody`, `ErrCrossContainer`. |
| `agent-docs/api-design.md` | Public API contract for modeling, features, selectors, and verification. |
| `docs/api.md` | User-facing API reference by task; `apitest/api_reference_test.go` checks its names exist. |
| `docs/data-structures.md` | User-facing type index and structure guide; `apitest/api_reference_test.go` checks it lists every exported type. |
| `agent-docs/sketch-seam-design.md` | `sketch` recording: `TExact`, `CurveSegment`, and `ErrUnrecordableProfile`. |
| `agent-docs/verification-design.md` | `Verify` reports, statuses, interference costs, deadlines, `WithTolerance`, and noise floor. |
| `agent-docs/payload-verification-design.md` | Per-payload proofs, certificates, bounded validity/clearance/survey algorithms. |
| `agent-docs/evaluator-design.md` | Evaluator topology, payloads, mass properties, feature builds, and mesh booleans. |
| `agent-docs/tessellation-design.md` | Tessellation: curve samples, manifold proofs, source faces, certificates, boolean handoff. |
| `agent-docs/clearance-design.md` | Pair disjointness and bounded gap proofs. |
| `agent-docs/point-containment-design.md` | Public point classification, distance, and mesh-to-body proofs. |
| `agent-docs/section-design.md` | Read-only planar section gate and Sketch reconstruction proof. |
| `agent-docs/interference-design.md` | Read-only pair overlap and bounded volume proofs. |
| `agent-docs/thread-design.md` | `Thread`: ISO metric thread cut into a cylindrical face through a coil, and its groove profile. |
| `agent-docs/gltf-export-design.md` | The glTF writer's file layout, metre scaling and error contract. |
| `agent-docs/mesh-import-design.md` | `MeshBody`, `ReadSTL` and `ReadOBJ`: the mesh readers, the mesh report and its exact volume. |
| `agent-docs/path-pattern-design.md` | `PathPatternCopies`: copies spaced at equal arc length along a `Path`, and the path sampler. |
| `agent-docs/fillet-chord-design.md` | `FilletChord`: chord-length fillet as a radius, sharing the right-angle gate of `ChamferAngle`. |
| `agent-docs/arrange-design.md` | `Arrange`: shelf packing of body footprints onto a flat bed by translation. |
| `agent-docs/chamfer-angle-design.md` | `ChamferAngle`: distance-and-angle bevel as a second setback, and its right-angle gate. |
| `agent-docs/hole-design.md` | `Hole`: face and point placement, size gates, tool shapes and the atomic cut. |
| `agent-docs/rib-design.md` | `Rib` and `Web`: line outline, extent, overrun, touch and volume checks, and the merged join. |
| `agent-docs/emboss-design.md` | `Emboss`: face-derived sweep, plane checks, margin, volume checks and the shared join. |
| `agent-docs/modify-design.md` | `Fillet`/`Chamfer`/`Shell` tables, section rewrite, exact offset, and build audit. |
| `agent-docs/shell-twisted-sweep-design.md` | Inward top-cap Shell of a one-line twisted rounded Sweep: offset, held cup and bounds. |
| `agent-docs/spline-design.md` | Free-form kinds, exactness tiers, refusals, Tier A moments, work budget, proven brackets, and reach. |
| `agent-docs/modify-reach-design.md` | Modify reach: tangent chains, asymmetric chamfers, cap-loop blends, shell reach and staging. |
| `agent-docs/brep-modify-design.md` | Modify ops on brep and stacked receivers; Tables RB/EB/SB/BB/DB. |
| `agent-docs/modify-general-design.md` | Shell of a through-cut brep (S), complete-loop chamfers (L); Tables TC/SG/LB/SL. |
| `agent-docs/loop-fillet-design.md` | Route L's fillet arm: pipe bands on a planar face's loop. |
| `agent-docs/vertex-blend-design.md` | Three-edge fillets, sphere patches, route V, and exact-radius refusals. |
| `agent-docs/shell-opening-design.md` | Shell side openings on a prism and a revolve: rim rule, brep record, Tables RO/SO/BO/DO. |
| `agent-docs/shell-faceted-boss-design.md` | Source-backed Shell of a nested rectangular boss in a faceted prism Union. |
| `agent-docs/draft-design.md` | `WithTaper` and `Draft`: the sharp offset family, `draftPayload`, Tables RD/SD/BD/DD. |
| `agent-docs/loft-design.md` | `Loft` pairing, refusals, results, consumers, chains, mass properties, and wall-crossing audit. |
| `agent-docs/loft-sections-design.md` | Smooth-section loft classes, held meshes and proofs. |
| `agent-docs/loft-face-sections-design.md` | Source-cap nine-section loft, cap correspondence, embedding, occupied-volume proof and two-cone continuation. |
| `agent-docs/revolve-loft-contact-graph-design.md` | Source-event ownership and held anchors for finite Revolve and trimmed LoftFaces contacts. |
| `agent-docs/loft-point-design.md` | Point-to-profile solid loft, its apex topology and mesh proof. |
| `agent-docs/revolve-loft-sector-design.md` | Finite Revolve and trimmed point-Loft source certificate, first Union class and removal ledger. |
| `agent-docs/revolve-axis-design.md` | Revolve/point-Loft sector Union and point-cone trims about a coordinate-plane axis through the origin: exact axis vector, predicates, charges, angle basis, ring, coaxial bore Cut, consumer contract, tests, +X digest. |
| `agent-docs/loft-gear-bounds-design.md` | Loft gear bounds: per-cell residuals, centroid shift, `A/P` target, audit, ceilings. |
| `agent-docs/loft-cylinder-boolean-design.md` | Certified axial Loft and centered cylinder Union, cap record, seam and volume bounds. |
| `agent-docs/loft-mirror-boolean-design.md` | Certified end-cap reflection of one axial Loft and its sewn three-ring Union. |
| `agent-docs/sweep-design.md` | `Path`/`Sweep`, its chain section (§15), mitred sweeps (§16) and bounded one-cell, composite and stationed twist (§17). |
| `agent-docs/helix-design.md` | `Coil`: a profile screwed about an in-plane axis. |
| `agent-docs/axial-section-boolean-design.md` | Union of axial extrusions as the extrusion of the 2D union of their sections, through sketch loop containment. |
| `agent-docs/axial-pattern-join-design.md` | Union of axial extrusions that carry a rigid placement, such as pattern copies: exact plane map, re-recorded sections and the join displacement. |
| `agent-docs/prism-boolean-design.md` | Analytic `Union`/`Cut`/`Intersect` over co-directional prisms. |
| `agent-docs/prism-pattern-union-design.md` | Certified Union of a circular prism and radial copies of one fitted section. |
| `agent-docs/stacked-prism-design.md` | Stacked slabs, walls, measurements, mesh and consumers. |
| `agent-docs/mirror-pattern-design.md` | Mirror and pattern entry points, the exact mirror join, the prism group. |
| `agent-docs/general-boolean-design.md` | Boolean classes past the prism pair, and `brepPayload`. |
| `agent-docs/oblique-boss-fillet-design.md` | Complete plane-to-oblique-cylinder boss junction and bounded fillet mesh. |
| `agent-docs/cross-cylinder-bore-chamfer-design.md` | Source-backed perpendicular cylinder bore-mouth chamfer and mesh proof. |
| `agent-docs/tessellation-reach-design.md` | Tessellation reach for lofts, free-form prisms, revolves and cap-loop chamfers. |
| `agent-docs/faceted-vertex-bounds-design.md` | Per-vertex displacement bounds on faceted bodies and their boolean composition. |
| `agent-docs/param-binding-design.md` | Reading feature values from a `sketch/param` table, and the ledger of those reads. |
| `agent-docs/gap-tolerance-design.md` | The document's gap tolerance: the reach of the held-mesh embedding repair, its default, its limits and its charge. |
| `agent-docs/operation-history-design.md` | Computational proof lineage, source-derived quantity calculations, operation snapshots, and shared operation kinds. |
| `agent-docs/contained-cut-moments-design.md` | Certified contained-Cut V/P/Q source moments, faceted mass-property selection, bounds, and refusal rules. |
| `agent-docs/exact-copy-moments-design.md` | Exact-rational moment transfer through one direct rotation Copy of an admitted contained Cut. |
| `agent-docs/operation-diagnostics-design.md` | Operation failure diagnostics, successful-call warnings, delivery APIs, compatibility rules, and proof fixtures. |
| `agent-docs/faceted-source-retention-design.md` | Analytic source surfaces carried onto mesh Boolean result faces: claim, admission, normal bound, readers, increments. |
| `agent-docs/retained-edge-blend-design.md` | `Fillet` and `Chamfer` of an edge between two retained faces of a mesh Boolean result: edge classes, exact blend geometry, local mesh rewrite, gates, bounds, increments. |
| `agent-docs/retained-modify-reach-design.md` | Modify and placement reach on retained mesh Boolean results: the per-call probe table, two-distance and tangent-chain blends, exact-copy and scaled tags, parallel-cylinder blends, and the Shell, Draft and PatternJoin refusals. |
| `agent-docs/surface-intersection-design.md` | `Trim`/`Extend`/`Split` over shared-generator sweeps. |
| `agent-docs/surface-design.md` | Sheet bodies, surface operations, verification and export. |
| `agent-docs/motion-check-design.md` | `VerifyMotion`: motions, the per-pose proof, the interval certificate, `MotionReport`. |
| `agent-docs/linkage-check-design.md` | `VerifyLinkage` and `VerifyJointBox`. |
| `agent-docs/collision-dynamics-design.md` | Pair contact/sweep in decad and rigid response in `dynamics`. |
| `agent-docs/contact-geometry-design.md` | Pair relation and contact manifold proofs. |
| `agent-docs/contact-sweep-design.md` | Two-body continuous sweep and first-contact brackets. |
| `agent-docs/dynamic-mass-design.md` | Bounded mass and inertia for rigid dynamics. |
| `agent-docs/rigid-dynamics-design.md` | Rigid-body steps, impulses, and reports. |
| `agent-docs/multibody-dynamics-design.md` | N-body dynamics, islands, `Timeline`. |
| `agent-docs/step-export-design.md` | Export package entry points and the AP214 faceted writer contract. |
| `agent-docs/3mf-export-design.md` | The 3MF writer's package parts, mesh mapping, and error contract. |

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
| `section.go` | Public planar section query over certified prism, stack-backed BRep, direct and smooth Loft, sphere and cone records; exact builders and clips live in `internal/sectionclip/`; also the `Section` profile-or-chain constraint. See section design, surface §13.2. |
| `path.go` | The immutable spatial `Path`, sealed segments and its recorded `internal/sweepinput.Path`. See sweep §2–§3. |
| `extent.go` | Public linear and angular extent variants and normalization. See API §8.1. |
| `selector.go` | `EdgeQuery`/`FaceQuery`, which resolve against live bodies through `internal/selectorquery` node views. See API §9. |
| `selection_error.go` | Public selector error types and query rendering adapters. See API §9. |

### Mass properties and free-form curves

| Path | Responsibility |
|---|---|
| `mass_properties.go` | Public mass and inertia types and the `Body.MassProperties` conversion over `internal/massdispatch/`. See `agent-docs/dynamic-mass-design.md` §2–§3. |
