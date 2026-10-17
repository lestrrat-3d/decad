# decad file layout: booleans

Boolean files.
The rules for rows live in `docs/layout.md`.

## Layout

| Path | Responsibility |
|---|---|
| `boolean.go` | Public `Union`/`Cut`/`Intersect`, `BooleanError` mapping, and audited scale-source marking. See evaluator §9. |
| `boolean_many.go` | Validates and commits a multi-body Union, including transient prism pair chains. See API §8. |
| `boolean_many_prism.go` | Arranges identical-interval line prisms in one Sketch scene. See prism-boolean §4.6. |
| `point_cone_trim.go` | Retains point-loft and cone source faces through coaxial Cut/Intersect. See loft-point design. |
| `loft_faces_cone_trim.go` | Certifies the one-use LoftFaces toe Cut to heel Intersect cone continuation, including the tangent heel's extended tooth operand. See loft-face-sections design. |
| `spiral_join_source.go` | Adapts an authenticated Revolve section to the legacy six-side join and certifies a cone-trimmed nine-section tooth's flank crossings. See loft-face-sections design. |
| `spiral_join_heel_seam.go` | Bounds the source root arc's intersection with the blank heel cap. See loft-face-sections design. |
| `spiral_join_toe_seam.go` | Bounds the source root arc's intersection with the blank toe cap. See loft-face-sections design. |
| `spiral_join_short.go` | Bounds fitted-flank and connector intersections with both finite blank end caps. See loft-face-sections design. |
| `spiral_join_short_bound.go` | Compares each finite cap crossing cell with complete held switch edges. See loft-face-sections design. |
| `spiral_join_near_pair.go` | Binds first curved-tooth near-face roles to live sources and exact cone-trim sides. See loft-face-sections design. |
| `spiral_join_root_flank.go` | Bounds a first-band flank/root seam against a held chain. See loft-face-sections design. |
| `spiral_join_faces.go` | Resolves authenticated blank and trimmed-tooth source faces in the first curved join. See faceted-vertex-bounds §3.6. |
| `spiral_join_blank_graph.go` | Checks held blank meridian facets form a radial graph. See faceted-vertex-bounds §3.6. |
| `spiral_join_blank_radial.go` | Bounds held blank cap radius against its exact meridian. See faceted-vertex-bounds §3.6. |
| `spiral_join_offset.go` | Builds and charges the temporary curved-tooth cap proxy. See faceted-vertex-bounds §3.6. |
| `spiral_join_seam.go` | Matches complete held cap and flank switches to certified source curves. See faceted-vertex-bounds §3.6. |
| `spiral_join_area.go` | Adds a conservative area allowance for the first curved Union. See faceted-vertex-bounds §3.6. |
| `spiral_join_union.go` | Gates the first curved-tooth Union's source pairs, held contacts and output curves. See faceted-vertex-bounds §3.6. |
| `spiral_combine.go` | Builds and validates a multi-tool spiral-bevel join. See faceted-vertex-bounds §3.7. |
| `spiral_combine_sector.go` | Admits the first curved join: runs the generic copied-tool proof, then requires every copy to carry an exact X-axis rotation (`WithCopyRotation`). See faceted-vertex-bounds §3.7. |
| `joined_mesh.go` | Checks a joined mesh's closed, connected orientation before geometric audits; `facetedrecord.JoinedMesh` collects its facets. See tessellation §4. |
| `loft_cut.go` | Preserves a matching axial loft through a certified coaxial circle-bore Cut. See loft §18. |
| `loft_mirror_union.go` | Joins one authenticated axial Loft to its end-cap reflection. See loft-mirror-boolean design. |
| `loft_mirror_cylinder_union.go` | Joins repeated reflected Loft pairs to a centered cylinder, including one-pass Union and bounded axial placement. See loft-mirror-boolean design. |
| `loft_mirror_cylinder_bore.go` | Cuts a centered through-bore from a reflected Loft/cylinder join. See loft-mirror-boolean design. |
| `loft_mirror_cylinder_chamfer.go` | Rewrites both exterior cap loops through one audited bored half and a reflected seam. See loft-mirror-boolean design. |
| `loft_mirror_cylinder_chamfer_bridge.go` | Certifies translated upper source teeth and the reflected cap band. See loft-mirror-boolean design. |
| `loft_cylinder_union.go` | Joins a source Loft tooth pattern to a coaxial cylinder with certified seam and volume bounds. See loft-cylinder-boolean design. |
| `loft_prism_union.go` | Reduces authenticated straight-source axial Lofts to prism operands for exact Sketch Union. See general-boolean design §12. |
| `loft_cylinder_placement.go` | Rebuilds a common signed-axis cylinder/Loft join in local coordinates and places its audited shell. See loft-cylinder-boolean design. |
| `loft_cylinder_axial_bridge.go` | Bounds a one-ulp axial placement difference between original Loft sources and private Union proof copies. See loft-cylinder-boolean design. |
| `loft_cylinder_separation.go` | Proves twisted Loft teeth remain separated at every height. See loft-cylinder-boolean design. |
| `loft_cylinder_cap.go` | Reads joined meshes into the exact cap disk and annulus audits and tags certified cap faces. See loft-cylinder-boolean design. |
| `loft_cylinder_source_circle.go` | Builds a source Loft tooth's exact circle record after strict root-collar admission. See loft-cylinder-boolean design. |
| `loft_cylinder_ring_occupancy.go` | Gathers tooth circles and walls for the strict interior split-ring check. See loft-cylinder-boolean design. |
| `loft_cylinder_bore.go` | Cuts a centered through-bore from an audited loft/cylinder join. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer.go` | Admits both outer cap loops and samples their held-polygon insets. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer_bridge.go` | Checks paired original and axial proof teeth for cap Chamfer. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer_mesh.go` | Builds and audits the rational outer-cap cutter and rounded mesh. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer_proof.go` | Builds ideal side vertices, the blank solid witness and the tooth face map. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer_near.go` | Bounds band points through a source-occupied split ring. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer_crossing.go` | Checks cap tiling and localized non-cap crossings for one-cap Chamfer. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer_cap.go` | Checks that cap root slits reach the source root arcs. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer_cap_source.go` | Matches inset cap edges to the original tooth contours. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer_bore.go` | Preserves the centered bore wall and inner loops. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer_side.go` | Certifies retained middle-side reach into the source union. See loft-cylinder-boolean design. |
| `prism_boolean.go` | Analytic booleans of co-directional prisms. See `docs/prism-boolean-design.md`. |
| `revolve_boolean.go` | Arranges authenticated common-axis full-Revolve meridians for public `Union`, `Cut` and `Intersect`. See general-boolean design. |
| `prism_boolean_nesting.go` | Clean-nesting scene adapters. See prism-boolean §4.2. |
| `prism_boolean_blind.go` | Blind and spanning Cuts via the whole-loop match. See `docs/prism-boolean-design.md` §3.2. |
| `prism_boolean_crossing_blind.go` | Exact rectangular blind Cut across one through hole. See general-boolean A7. |
| `prism_group.go` | Prism-group `Cut` tools and disjoint `Union` results. See general-boolean A5. |
| `stacked_union.go` | Admits A1 `Union` operands and passes their slabs to `internal/stackedunion`. See general-boolean A1. |
| `prism_face_patch_union.go` | Re-expresses exact rectangular face-patch Union operands for prism or A1 scenes. See general-boolean A1. |
| `prism_overlap.go` | Prism-boolean §4.5's overlap-area reading for `Verify`. See its doc comment. |
| `brep_measure.go` | The brep surveys; `internal/breprecord` states prisms and stacked prisms as face views, then builds and measures the record. See general-boolean §4. |
| `classb.go` | `tryClassB`: class B over two bodies' payloads through `internal/classbgeom`. See general-boolean §3 B. |
| `oblique_boss.go` | Recognizes the two prism sources a complete oblique cylinder and plate Union retains. See oblique-boss-fillet design. |
| `cross_cylinder_bore_source.go` | Recognizes the two circular prism sources a perpendicular cylinder Cut retains. See cross-cylinder-bore-chamfer design. |
| `surface_trim.go` / `surface_split_revolve.go` | Surface body gates and revolve `Split`. See surface-intersection §2–§3. |
| `point_plane_split.go` | Splits a point loft by a planar chain sheet. See loft-point design. |
| `point_plane_scale.go` | Dispatches certified similar bodies and carries Split-slab maps over `internal/facetedrecord/`'s Boolean source check. See loft-point design and evaluator §8. |
| `prism_scale.go` | Admits prism and full-Revolve certified similarity sources and transfers their face roles. See evaluator §8. |
| `stack_brep_scale.go` | Admits exact line-only stacks and stack-backed BReps for certified similarity. See evaluator §8. |
| `point_cap_centroid.go` | Certifies the far cap centroid; `facetedrecord` owns planar Split cap centroids and rigid placement. See loft-point design. |
| `point_cap_extremes.go` | Bounds side-face centroid projections and selects the extreme Split cap faces. See loft-point design. |
| `boolean_mesh.go` | Prepares an operand's mesh for `internal/meshbool/`. See evaluator §9. |
| `boolean_body.go` | Reads an audited mesh's exact volume into a measurement and keys exactness off a proven bound. See evaluator §9. |
| `faceted_copy_lineage.go` | Binds live faceted copies to source faces, recorded motion, and separate source and held shells in a `facetedrecord` copy set. See faceted-vertex-bounds §3.7. |
| `faceted_copy_proxy.go` | Places a source-certified proxy under generic copied-tool motions and proves proxy shell separation. See faceted-vertex-bounds §3.7. |
