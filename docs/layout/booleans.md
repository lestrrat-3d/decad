# decad file layout: booleans

Boolean files.
The rules for rows live in `docs/layout.md`.

## Layout

| Path | Responsibility |
|---|---|
| `boolean.go` | Public `Union`/`Cut`/`Intersect`, `BooleanError` mapping, scale-source marking, and the class B and common-axis Revolve routes. See evaluator §9. |
| `boolean_many.go` | Validates and commits a multi-body Union, including certified straight-source Loft one-scene unions, transient prism pair chains, and identical-interval line prisms arranged in one Sketch scene. See API §8 and prism-boolean §4.6. |
| `point_cone_trim.go` | Routes point-loft Cut/Intersect with an exact cone tool (revolveplan.PointConeSource) to facetedrecord. See loft-point design. |
| `loft_faces_cone_trim.go` | Certifies the one-use LoftFaces toe Cut to heel Intersect cone continuation, including the tangent heel's extended tooth operand. See loft-face-sections design. |
| `spiral_combine.go` | Builds a multi-tool spiral-bevel join and its cap proxy, and admits the first curved join by requiring every copy to carry an exact X-axis rotation (`WithCopyRotation`). `internal/spiralsector` binds copy roles and validates each copy. See faceted-vertex-bounds §3.7. |
| `loft_cut.go` | Preserves a matching axial loft through a certified coaxial circle-bore Cut. See loft §18. |
| `loft_mirror_union.go` | Joins one authenticated axial Loft to its end-cap reflection. See loft-mirror-boolean design. |
| `loft_mirror_cylinder_union.go` | Dispatches repeated reflected Loft pairs and a through-bore Cut, builds the half blank, places the lower teeth and certifies the lower slab. `internal/loftcylinder` checks the blank, levels and pairs, and builds, audits and tags the joined or bored shell. See loft-mirror-boolean design. |
| `loft_mirror_cylinder_chamfer.go` | Recognizes the Chamfer of both exterior cap loops of an audited mirrored bore and checks its setback and reserves. `internal/loftcylinder` recomputes the upper tooth shifts, builds the lower half, and reflects, assembles, proves and tags the mirrored shell. See loft-mirror-boolean design. |
| `loft_cylinder_union.go` | Dispatches a coaxial cylinder and Loft tooth `Union` and a centered bore `Cut`, verifies each proof tooth's mesh and the blank's, and builds the joined result. Proof teeth, tooth checks, cap tags, shell proofs and the bore live in `internal/loftcylinder`. See loft-cylinder-boolean design. |
| `loft_contact_source_spans.go` | Reads authenticated line, circle, arc and fit-spline cap spans, ordered Sketch cap cells and Loft wall-cell source roles for the private curved-contact lane; it publishes no output proof. |
| `loft_contact_quarter.go` | Relates certified quarter-arc cap crossings to original Loft wall sources, pairs witnesses with held seams, and checks a paired wall's source order, under private source and output budgets. |
| `loft_contact_direct_shell.go` | Builds a bounded public Union shell from the certified Loft/prism output boundary and its `internal/loftcontact` volume proof. See its proof comments. |
| `loft_contact_output_proof.go` | Reads the payloads of two live Loft/prism bodies in either order and runs the source partition and output shell in `internal/loftcontact`, which certifies the partition and both Sketch caps, then builds, tiles and audits the transient held shell. |
| `loft_cylinder_placement.go` | Dispatches the common signed-axis cylinder/Loft `Union` and bore `Cut`, rebuilds the local join and tool, and places the audited shell. `internal/loftcylinder` holds the shape and source checks and the local blank and tooth rebuilds. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer.go` | Recognizes the two-cap Chamfer of an audited loft/cylinder result, reads the receiver's cap refs, and maps the result node built in `internal/loftcylinder` to its handle. See loft-cylinder-boolean design. |
| `prism_overlap.go` | Wraps `prismcells.OverlapVolume`, prism-boolean §4.5's overlap-area reading, as a `Verify` measurement. See its doc comment. |
| `brep_measure.go` | The brep surveys, including fillet-band pull readings and the undercut verdict fold; `internal/breprecord` states prisms and stacked prisms as face views, then builds and measures the record. See general-boolean §4. |
| `surface_trim.go` / `surface_split_revolve.go` | Public `Trim`, `Extend` and `Split` entry checks, result builds and commits, and revolve `Split`; gates in `internal/surfacetrim/`. See surface-intersection §2–§3. |
| `point_plane_split.go` | Routes a point-loft Split through facetedrecord's source and sheet gates and plane order, and commits its two slabs. See loft-point design. |
| `point_plane_scale.go` | ScaledAt: resolves the similarity source by payload kind and commits the image; facetedrecord.ScaleImage admits and scales it. See loft-point design and evaluator §8. |
| `prism_scale.go` | Admits prism and full-Revolve certified similarity sources and transfers their face roles. See evaluator §8. |
| `stack_brep_scale.go` | Admits exact line-only stacks and stack-backed BReps for certified similarity. See evaluator §8. |
| `point_cap_extremes.go` | Resolves Split cap faces and selects the ends using facetedrecord's centroid-order proof. See loft-point design. |
| `boolean_mesh.go` | Prepares an operand's mesh for `internal/meshbool/`, runs the cone-cap mesh boolean with the `internal/spiralsector` first curved-tooth join gates, and reads an audited mesh's volume into a measurement. See evaluator §9. |
