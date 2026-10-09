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
| `docs/interference-design.md` | Read-only pair overlap and bounded volume proofs. |
| `docs/modify-design.md` | `Fillet`/`Chamfer`/`Shell` tables, section rewrite, exact offset, and build audit. |
| `docs/spline-design.md` | Free-form kinds, exactness tiers, refusals, Tier A moments, work budget, proven brackets, and reach. |
| `docs/modify-reach-design.md` | Modify reach: tangent chains, asymmetric chamfers, cap-loop blends, shell reach and staging. |
| `docs/brep-modify-design.md` | Modify ops on brep and stacked receivers; Tables RB/EB/SB/BB/DB. |
| `docs/modify-general-design.md` | Shell of a through-cut brep (S), complete-loop chamfers (L); Tables TC/SG/LB/SL. |
| `docs/loop-fillet-design.md` | Route L's fillet arm: pipe bands on a planar face's loop. |
| `docs/vertex-blend-design.md` | Three-edge fillets, sphere patches, route V, and exact-radius refusals. |
| `docs/shell-opening-design.md` | Shell side openings on a prism and a revolve: rim rule, brep record, Tables RO/SO/BO/DO. |
| `docs/draft-design.md` | `WithTaper` and `Body.Draft`: the sharp offset family, `draftPayload`, Tables RD/SD/BD/DD. |
| `docs/loft-design.md` | `Loft` pairing, refusals, results, consumers, chains, mass properties, and wall-crossing audit. |
| `docs/loft-gear-bounds-design.md` | Loft gear bounds: per-cell residuals, centroid shift, `A/P` target, audit, ceilings. |
| `docs/sweep-design.md` | `Path`/`Sweep`, `SweepChain` and mitred sweeps (§16). |
| `docs/helix-design.md` | `Document.Coil`: a profile screwed about an in-plane axis. |
| `docs/prism-boolean-design.md` | Analytic `Union`/`Cut`/`Intersect` over co-directional prisms. |
| `docs/stacked-prism-design.md` | Stacked slabs, walls, measurements, mesh and consumers. |
| `docs/mirror-pattern-design.md` | Mirror and pattern entry points, the exact mirror join, the prism group. |
| `docs/general-boolean-design.md` | Boolean classes past the prism pair, and `brepPayload`. |
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
| `measurement.go` | Public reading aliases and analytic result gate. |
| `identity.go` | Private document-local producer identities and the shared zero-vector predicate. |
| `record.go` | Private aliases for structural records and public `Point2`. See sketch-seam §2. |
| `seam.go` | Sketch recording adapters and `MeasureProfile`. See sketch-seam §1–§2. |
| `path.go` | The immutable spatial `Path` and its sealed segment vocabulary. See sweep §2–§3. |
| `extent.go` | Public extent aliases and normalization. See API §8.1. |
| `selector.go` | `EdgeQuery`/`FaceQuery` and live-topology adapters for `internal/selectorquery`. See API §9. |
| `selection_error.go` | Public selector error types and query rendering adapters. See API §9. |

### Mass properties and free-form curves

| Path | Responsibility |
|---|---|
| `moments.go` / `moments_validate.go` | Public second moments and moment adapters. See evaluator §4. |
| `mass_properties.go` | Mass dispatch and payload adapters. See `docs/dynamic-mass-design.md` §2–§3. |
| `mass_properties_faceted.go` / `mass_properties_mesh.go` | Mass read off verified meshes. See dynamic-mass. |
