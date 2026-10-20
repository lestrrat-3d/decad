# decad file layout: booleans

Boolean files.
The rules for rows live in `docs/layout.md`.

## Layout

| Path | Responsibility |
|---|---|
| `boolean.go` | Public `Union`/`Cut`/`Intersect`, `BooleanError` mapping and scale-source marking; hands the route order to `internal/booleandispatch/` with root-only hooks. See evaluator §9. |
| `boolean_many.go` | Validates and commits a multi-body Union, dispatching the certified pattern, one-scene prism and straight-source Loft routes and transient prism pair chains. See API §8 and prism-boolean §4.6. |
| `spiral_combine.go` | Builds a multi-tool spiral-bevel join and its cap proxy, and admits the first curved join by requiring every copy to carry an exact X-axis rotation (`WithCopyRotation`). `internal/spiralsector` binds copy roles and validates each copy. See faceted-vertex-bounds §3.7. |
| `loft_contact_source_spans.go` | Reads authenticated line, circle, arc and fit-spline cap spans, ordered Sketch cap cells and Loft wall-cell source roles for the private curved-contact lane; it publishes no output proof. |
| `loft_contact_quarter.go` | Relates certified quarter-arc cap crossings to original Loft wall sources, pairs witnesses with held seams, and checks a paired wall's source order, under private source and output budgets. |
| `loft_contact_direct_shell.go` | Builds a bounded public Union shell from the certified Loft/prism output boundary and its `internal/loftcontact` volume proof. See its proof comments. |
| `loft_contact_output_proof.go` | Reads the payloads of two live Loft/prism bodies in either order and runs the source partition and output shell in `internal/loftcontact`, which certifies the partition and both Sketch caps, then builds, tiles and audits the transient held shell. |
| `prism_overlap.go` | Wraps `prismcells.OverlapVolume`, prism-boolean §4.5's overlap-area reading, as a `Verify` measurement. See its doc comment. |
| `brep_measure.go` | The brep surveys, including fillet-band pull readings and the undercut verdict fold; `internal/breprecord` states prisms and stacked prisms as face views, then builds and measures the record. See general-boolean §4. |
| `surface_trim.go` | Public `Trim`, `Extend` and `Split` entry checks, result builds and commits; gates and revolve `Split` in `internal/surfacetrim/`. See surface-intersection §2–§3. |
| `point_plane_split.go` | Routes a point-loft Split through facetedrecord's source and sheet gates and plane order, and commits its two slabs. See loft-point design. |
| `point_plane_scale.go` | ScaledAt: resolves the similarity source by payload kind and commits the image; facetedrecord.ScaleImage admits and scales it. See loft-point design and evaluator §8. |
| `prism_scale.go` | Admits prism and full-Revolve certified similarity sources and transfers their face roles. See evaluator §8. |
| `stack_brep_scale.go` | Admits exact line-only stacks and stack-backed BReps for certified similarity. See evaluator §8. |
| `point_cap_extremes.go` | Resolves Split cap faces and selects the ends using facetedrecord's centroid-order proof. See loft-point design. |
| `boolean_mesh.go` | Runs `internal/booleaneval`'s mesh boolean over two bodies and states the audited mesh's occupied volume as a public measurement, with the exactness the bound implies. See evaluator §9. |
