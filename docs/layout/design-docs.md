# decad file layout: design documents

Design documents, user guides, seam and records, mass properties and free-form curves.
The rules for rows live in `docs/layout.md`.

## Layout

### Design documents

| Path | Responsibility |
|---|---|
| `docs/api-design.md` | Public API contract for modeling, features, selectors, and verification. |
| `docs/sketch-seam-design.md` | `sketch` recording: `TExact`, `CurveSegment`, and `ErrUnrecordableProfile`. |
| `docs/verification-design.md` | `Verify` reports, statuses, interference costs, deadlines, `WithTolerance`, and noise floor. |
| `docs/payload-verification-design.md` | Per-payload proofs, certificates, bounded validity/clearance/survey algorithms. |
| `docs/evaluator-design.md` | Evaluator topology, payloads, mass properties, feature builds, and mesh booleans. |
| `docs/tessellation-design.md` | Tessellation: curve samples, manifold proofs, source faces, certificates, boolean handoff. |
| `docs/clearance-design.md` | Pair disjointness and bounded gap proofs. |
| `docs/point-containment-design.md` | Public point classification, distance, and mesh-to-body proofs. |
| `docs/section-design.md` | Read-only planar section gate and Sketch reconstruction proof. |
| `docs/interference-design.md` | Read-only pair overlap and bounded volume proofs. |
| `docs/modify-design.md` | `Fillet`/`Chamfer`/`Shell` tables, section rewrite, exact offset, and build audit. |
| `docs/shell-twisted-sweep-design.md` | Inward top-cap Shell of a one-line twisted rounded Sweep: offset, held cup and bounds. |
| `docs/spline-design.md` | Free-form kinds, exactness tiers, refusals, Tier A moments, work budget, proven brackets, and reach. |
| `docs/modify-reach-design.md` | Modify reach: tangent chains, asymmetric chamfers, cap-loop blends, shell reach and staging. |
| `docs/brep-modify-design.md` | Modify ops on brep and stacked receivers; Tables RB/EB/SB/BB/DB. |
| `docs/modify-general-design.md` | Shell of a through-cut brep (S), complete-loop chamfers (L); Tables TC/SG/LB/SL. |
| `docs/loop-fillet-design.md` | Route L's fillet arm: pipe bands on a planar face's loop. |
| `docs/vertex-blend-design.md` | Three-edge fillets, sphere patches, route V, and exact-radius refusals. |
| `docs/shell-opening-design.md` | Shell side openings on a prism and a revolve: rim rule, brep record, Tables RO/SO/BO/DO. |
| `docs/shell-faceted-boss-design.md` | Source-backed Shell of a nested rectangular boss in a faceted prism Union. |
| `docs/draft-design.md` | `WithTaper` and `Draft`: the sharp offset family, `draftPayload`, Tables RD/SD/BD/DD. |
| `docs/loft-design.md` | `Loft` pairing, refusals, results, consumers, chains, mass properties, and wall-crossing audit. |
| `docs/loft-sections-design.md` | Smooth-section loft classes, held meshes and proofs. |
| `docs/loft-face-sections-design.md` | Source-cap nine-section loft, cap correspondence, embedding, occupied-volume proof and two-cone continuation. |
| `docs/revolve-loft-contact-graph-design.md` | Source-event ownership and held anchors for finite Revolve and trimmed LoftFaces contacts. |
| `docs/loft-point-design.md` | Point-to-profile solid loft, its apex topology and mesh proof. |
| `docs/revolve-loft-sector-design.md` | Finite Revolve and trimmed point-Loft source certificate, first Union class and removal ledger. |
| `docs/loft-gear-bounds-design.md` | Loft gear bounds: per-cell residuals, centroid shift, `A/P` target, audit, ceilings. |
| `docs/loft-cylinder-boolean-design.md` | Certified axial Loft and centered cylinder Union, cap record, seam and volume bounds. |
| `docs/loft-mirror-boolean-design.md` | Certified end-cap reflection of one axial Loft and its sewn three-ring Union. |
| `docs/sweep-design.md` | `Path`/`Sweep`, its chain section (§15), mitred sweeps (§16) and bounded one-cell, composite and stationed twist (§17). |
| `docs/helix-design.md` | `Coil`: a profile screwed about an in-plane axis. |
| `docs/prism-boolean-design.md` | Analytic `Union`/`Cut`/`Intersect` over co-directional prisms. |
| `docs/prism-pattern-union-design.md` | Certified Union of a circular prism and radial copies of one fitted section. |
| `docs/stacked-prism-design.md` | Stacked slabs, walls, measurements, mesh and consumers. |
| `docs/mirror-pattern-design.md` | Mirror and pattern entry points, the exact mirror join, the prism group. |
| `docs/general-boolean-design.md` | Boolean classes past the prism pair, and `brepPayload`. |
| `docs/oblique-boss-fillet-design.md` | Complete plane-to-oblique-cylinder boss junction and bounded fillet mesh. |
| `docs/cross-cylinder-bore-chamfer-design.md` | Source-backed perpendicular cylinder bore-mouth chamfer and mesh proof. |
| `docs/tessellation-reach-design.md` | Tessellation reach for lofts, free-form prisms, revolves and cap-loop chamfers. |
| `docs/faceted-vertex-bounds-design.md` | Per-vertex displacement bounds on faceted bodies and their boolean composition. |
| `docs/surface-intersection-design.md` | `Trim`/`Extend`/`Split` over shared-generator sweeps. |
| `docs/surface-design.md` | Sheet bodies, surface operations, verification and export. |
| `docs/motion-check-design.md` | `VerifyMotion`: motions, the per-pose proof, the interval certificate, `MotionReport`. |
| `docs/linkage-check-design.md` | `VerifyLinkage` and `VerifyJointBox`. |
| `docs/collision-dynamics-design.md` | Pair contact/sweep in decad and rigid response in `dynamics`. |
| `docs/contact-geometry-design.md` | Pair relation and contact manifold proofs. |
| `docs/contact-sweep-design.md` | Two-body continuous sweep and first-contact brackets. |
| `docs/dynamic-mass-design.md` | Bounded mass and inertia for rigid dynamics. |
| `docs/rigid-dynamics-design.md` | Rigid-body steps, impulses, and reports. |
| `docs/multibody-dynamics-design.md` | N-body dynamics, islands, `Timeline`. |
| `docs/step-export-design.md` | Export package entry points and the AP214 faceted writer contract. |
| `docs/3mf-export-design.md` | The 3MF writer's package parts, mesh mapping, and error contract. |

### User guides

| Path | Responsibility |
|---|---|
| `docs/collision-v1-support.md` | Certified collision paths, response limits, and refusal outcomes. |
| `docs/missing-features.md` | Index of what decad refuses or lacks, each row pointing at its owner. |

### Seam and records

| Path | Responsibility |
|---|---|
| `doc.go` | Package scope, support map and layering. |
| `errors.go` | Sentinel errors (from `internal/decaderr/`) and `BooleanError`. See api §12, §8. |
| `measurement.go` | Public bounded reading types, internal adapters and analytic result gate. |
| `identity.go` | The root spelling of the provenance producer identity and the shared zero-vector predicate. |
| `record.go` | Root-owned `Point2`, structural record aliases, and slice adapters. See sketch-seam §2. |
| `seam.go` | Sketch chain recording adapters and `MeasureProfile`; profile recording lives in `internal/sectioncert/`. See sketch-seam §1–§2. |
| `section.go` | Public planar section query over certified prism, stack-backed BRep, direct and rectangular smooth Loft, sphere and cone records. See section design. |
| `section_loft_circle.go` | Exact parallel section of authenticated circular-boundary and circular-hole Lofts, with shared exact scalar interpolation. See section design. |
| `section_loft_matching.go` | Exact parallel section of an authenticated Loft with matching profiles. See section design. |
| `section_loft_ruled_lines.go` | Exact parallel section of an authenticated Loft with full line boundaries. See section design. |
| `section_circle_axis.go` | Exact whole-circle prism section on a cardinal plane with an exact chord. See section design. |
| `section_cone_axis.go` | Full right-cone section on a cardinal plane with an exact circle or bounded polygon. See section design. |
| `section_sphere_axis.go` | Authenticated source-sphere section on a cardinal plane with an exact circle or bounded polygon. See section design. |
| `section_circle_oblique.go` | Oblique full-ellipse clip of an exact whole-circle prism; the rational polygon helpers live in `internal/sectionclip/`. See section design. |
| `section_circle_cap.go` | One-cap oblique circle clip entry; the certified circle-line trim and cap polygon live in `internal/sectionclip/`. See section design. |
| `section_rectangle_axis.go` | Exact rectangular-prism section on an interior cardinal vertical plane. See section design. |
| `section_oblique_clip.go` | Exact rational clipping of a rectangular prism by an oblique plane; the rational point type and arithmetic live in `internal/sectionclip/`. See section design. |
| `path.go` | The immutable spatial `Path`, sealed segments and its recorded `internal/sweepinput.Path`. See sweep §2–§3. |
| `extent.go` | Public linear and angular extent variants and normalization. See API §8.1. |
| `selector.go` | `EdgeQuery`/`FaceQuery` and live-topology adapters for `internal/selectorquery`. See API §9. |
| `selection_error.go` | Public selector error types and query rendering adapters. See API §9. |

### Mass properties and free-form curves

| Path | Responsibility |
|---|---|
| `moments.go` / `moments_validate.go` | Public second moments and moment adapters. See evaluator §4. |
| `mass_properties.go` | Public mass and inertia types, mass dispatch and payload adapters. See `docs/dynamic-mass-design.md` §2–§3. |
| `mass_properties_faceted.go` / `mass_properties_mesh.go` | Mass read off verified meshes. See dynamic-mass. |
