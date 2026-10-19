# decad file layout: booleans

Boolean files.
The rules for rows live in `docs/layout.md`.

## Layout

| Path | Responsibility |
|---|---|
| `boolean.go` | Public `Union`/`Cut`/`Intersect`, `BooleanError` mapping, scale-source marking, and the class B and common-axis Revolve routes. See evaluator §9. |
| `boolean_many.go` | Validates and commits a multi-body Union, including certified straight-source Loft one-scene unions, transient prism pair chains, and identical-interval line prisms arranged in one Sketch scene. See API §8 and prism-boolean §4.6. |
| `point_cone_trim.go` | Recognizes exact cone tools and routes point-loft Cut/Intersect to facetedrecord. See loft-point design. |
| `loft_faces_cone_trim.go` | Certifies the one-use LoftFaces toe Cut to heel Intersect cone continuation, including the tangent heel's extended tooth operand. See loft-face-sections design. |
| `spiral_join_source.go` | Adapts an authenticated Revolve section, orchestrates source gates for a cone-trimmed nine-section tooth, and binds blank and tooth source faces and near-face roles to live sources. See loft-face-sections design. |
| `spiral_join_blank.go` | Checks held blank meridian facets form a radial graph and bounds held blank cap radius against its exact meridian. See faceted-vertex-bounds §3.6. |
| `spiral_join_union.go` | Gates the first curved-tooth Union's source pairs, held contacts and output curves, adds its area allowance, builds the temporary cap proxy, and matches held seam switches to source curves. See faceted-vertex-bounds §3.6. |
| `spiral_combine.go` | Builds and validates a multi-tool spiral-bevel join, and admits the first curved join by requiring every copy to carry an exact X-axis rotation (`WithCopyRotation`). See faceted-vertex-bounds §3.7. |
| `loft_cut.go` | Preserves a matching axial loft through a certified coaxial circle-bore Cut. See loft §18. |
| `loft_mirror_union.go` | Joins one authenticated axial Loft to its end-cap reflection. See loft-mirror-boolean design. |
| `loft_mirror_cylinder_union.go` | Authenticates repeated reflected Loft pairs and a through-bore Cut, places the lower teeth, audits the exact blank, and publishes the joined or bored shell. `internal/loftcylinder` assembles the mirrored halves. See loft-mirror-boolean design. |
| `loft_mirror_cylinder_chamfer.go` | Rewrites both exterior cap loops through one audited bored half and a reflected seam, and certifies translated upper source teeth and the reflected cap band. See loft-mirror-boolean design. |
| `loft_cylinder_union.go` | Dispatches a coaxial cylinder and Loft tooth `Union`, builds each tooth's private proof body and verified mesh, and tags and builds the joined result. Tooth admission, separation and the joined shell's seam and volume proofs live in `internal/loftcylinder`. See loft-cylinder-boolean design. |
| `loft_contact_partition.go` | Certifies a private varying-line Loft/prism source partition and both Sketch cap correspondences. `internal/loftcontact` holds the partition record and certifies each cap. See its proof comments. |
| `loft_contact_source_spans.go` | Reads authenticated line, circle, arc and fit-spline cap spans, ordered Sketch cap cells and Loft wall-cell source roles for the private curved-contact lane; it publishes no output proof. |
| `loft_contact_quarter.go` | Relates certified quarter-arc cap crossings to original Loft wall sources, pairs witnesses with held seams, and checks a paired wall's source order, under private source and output budgets. |
| `loft_contact_direct_shell.go` | Builds a bounded public Union shell from certified Loft/prism source and output proofs. See its proof comments. |
| `loft_contact_output_proof.go` | Runs the source partition on two live Loft/prism bodies and passes their payloads to `internal/loftcontact`, which builds, tiles and audits the transient held output shell. |
| `loft_contact_volume_proof.go` | Computes a transient source/shell volume proof consumed by the admitted public varying line-only Loft/prism Union route, and checks the authenticated source against the shell. `internal/loftcontact` checks the shell's exact embedding and outward volume. See its proof comments. |
| `loft_cylinder_placement.go` | Rebuilds a common signed-axis cylinder/Loft join in local coordinates and places its audited shell. See loft-cylinder-boolean design. |
| `loft_cylinder_cap.go` | Tags certified cap faces, and bounds a one-ulp axial placement difference for Union proof copies. See loft-cylinder-boolean design. |
| `loft_cylinder_bore.go` | Cuts a centered through-bore from an audited loft/cylinder join. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer.go` | Recognizes the two-cap Chamfer of an audited loft/cylinder result, reads the receiver's cap refs, and maps the result node built in `internal/loftcylinder` to its handle. See loft-cylinder-boolean design. |
| `prism_boolean.go` | Root holed-Intersect route, axial-Loft and exact face-patch Union adapters, its one-hole nesting admission, body-to-payload adapters for analytic prism booleans, blind and spanning Cut admission, and the exact blind Cut across one through hole. See prism-boolean §3, general-boolean A7. |
| `prism_group.go` | Prism-group `Cut` tools and disjoint `Union` results. See general-boolean A5. |
| `stacked_union.go` | Admits A1 `Union` operands and passes their slabs to `internal/stackedunion`. See general-boolean A1. |
| `prism_overlap.go` | Prism-boolean §4.5's overlap-area reading for `Verify`. See its doc comment. |
| `brep_measure.go` | The brep surveys, including fillet-band pull readings and the undercut verdict fold; `internal/breprecord` states prisms and stacked prisms as face views, then builds and measures the record. See general-boolean §4. |
| `surface_trim.go` / `surface_split_revolve.go` | Surface entry gates, root result construction and revolve `Split`. See surface-intersection §2–§3. |
| `point_plane_split.go` | Authenticates point-loft and sheet operands, orders split planes, and commits facetedrecord slabs. See loft-point design. |
| `point_plane_scale.go` | Validates ScaledAt sources and commits similarity images; facetedrecord carries Split maps. See loft-point design and evaluator §8. |
| `prism_scale.go` | Admits prism and full-Revolve certified similarity sources and transfers their face roles. See evaluator §8. |
| `stack_brep_scale.go` | Admits exact line-only stacks and stack-backed BReps for certified similarity. See evaluator §8. |
| `point_cap_extremes.go` | Resolves Split cap faces and selects the ends using facetedrecord's centroid-order proof. See loft-point design. |
| `boolean_mesh.go` | Prepares an operand's mesh for `internal/meshbool/`, runs the cone-cap mesh boolean, and reads an audited mesh's volume into a measurement. See evaluator §9. |
| `faceted_copy.go` | Binds live faceted copies to source faces, recorded motion and separate source and held shells in a `facetedrecord` copy set, and places a source-certified proxy with its shell separation proof. See faceted-vertex-bounds §3.7. |
