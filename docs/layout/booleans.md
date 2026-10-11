# decad file layout: booleans

Boolean files.
The rules for rows live in `docs/layout.md`.

## Layout

| Path | Responsibility |
|---|---|
| `boolean.go` | Public `Union`/`Cut`/`Intersect` and the `BooleanError` mapping. See evaluator §9. |
| `point_cone_trim.go` | Retains point-loft and cone source faces through coaxial Cut/Intersect. See loft-point design. |
| `loft_faces_cone_trim.go` | Certifies the one-use LoftFaces toe Cut to heel Intersect cone continuation. See loft-face-sections design. |
| `bevel_join_source.go` | Matches the live Sketch tooth source to the finite blank root meridian. See loft-point design. |
| `spiral_join_source.go` | Certifies a cone-trimmed nine-section tooth's continuous flank crossings against a finite bevel blank. See loft-face-sections design. |
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
| `spiral_combine_sector.go` | Proves source and held tooth sectors disjoint before the combined join. See faceted-vertex-bounds §3.7. |
| `bevel_join_combine.go` | Admits an authenticated straight-tooth CombineJoin and proves complete source solids separate. See loft-point design. |
| `bevel_join_cap_refine.go` | Refines straight-tooth caps while keeping shared edges and boundary labels. See loft-point design. |
| `bevel_join.go` | Rebuilds and audits one shared-root bevel blank and tooth Union. See loft-point design. |
| `bevel_join_two.go` | Rebuilds two separate point-loft teeth on one bevel blank root ring. See loft-point design. |
| `bevel_join_pattern.go` | Rebuilds separate point-loft teeth on one bevel blank root ring. See loft-point design. |
| `bevel_join_bore.go` | Cuts a certified centered through-bore from the complete eight-tooth bevel join. See loft-point design. |
| `loft_cut.go` | Preserves a matching axial loft through a certified coaxial circle-bore Cut. See loft §18. |
| `loft_cylinder_union.go` | Joins a source Loft tooth pattern to a coaxial cylinder with certified seam and volume bounds. See loft-cylinder-boolean design. |
| `loft_cylinder_axial_bridge.go` | Bounds a one-ulp axial placement difference between original Loft sources and private Union proof copies. See loft-cylinder-boolean design. |
| `loft_cylinder_separation.go` | Proves twisted Loft teeth remain separated at every height. See loft-cylinder-boolean design. |
| `loft_cylinder_cap.go` | Audits joined disks and bored annuli, then tags certified cap faces. See loft-cylinder-boolean design. |
| `loft_cylinder_seam.go` | Certifies the faceted connector's radial crossing at every height. See loft-cylinder-boolean design. |
| `loft_cylinder_reach.go` | Certifies source and held connector reach and crossing angles. See loft-cylinder-boolean design. |
| `loft_cylinder_contact.go` | Bounds an exact same-height connector contact against its source cylinder crossing. See loft-cylinder-boolean design. |
| `loft_cylinder_source_circle.go` | Bounds a source Loft tooth's exact circle crossings over a rational height interval. See loft-cylinder-boolean design. |
| `loft_cylinder_ring_distance.go` | Measures exact ring-edge clearance from held Loft walls. See loft-cylinder-boolean design. |
| `loft_cylinder_ring_occupancy.go` | Certifies one strict interior split ring inside an authenticated loft/cylinder union. See loft-cylinder-boolean design. |
| `loft_cylinder_bore.go` | Cuts a centered through-bore from an audited loft/cylinder join. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer.go` | Admits both outer cap loops and samples their held-polygon insets. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer_mesh.go` | Builds and audits the rational outer-cap cutter and rounded mesh. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer_proof.go` | Checks cutter containment, source contacts and volume witnesses. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer_near.go` | Bounds band points through a source-occupied split ring. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer_clearance.go` | Supplies exact triangle clearance for source-wall checks. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer_cap.go` | Bounds cap root slits and the retained inset region. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer_cap_source.go` | Matches inset cap edges to the original tooth contours. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer_bore.go` | Preserves the centered bore wall and inner loops. See loft-cylinder-boolean design. |
| `loft_cylinder_chamfer_side.go` | Certifies retained middle-side reach into the source union. See loft-cylinder-boolean design. |
| `prism_boolean.go` | Analytic booleans of co-directional prisms. See `docs/prism-boolean-design.md`. |
| `prism_boolean_nesting.go` | Clean-nesting scene adapters. See prism-boolean §4.2. |
| `prism_boolean_blind.go` | Blind and spanning Cuts via the whole-loop match. See `docs/prism-boolean-design.md` §3.2. |
| `prism_boolean_crossing_blind.go` | Exact rectangular blind Cut across one through hole. See general-boolean A7. |
| `stacked_prism.go` | Stacked slabs, walls and measurements. See `docs/stacked-prism-design.md`. |
| `prism_group.go` | Prism-group `Cut` tools and disjoint `Union` results. See general-boolean A5. |
| `stacked_union.go` / `stacked_union_brep.go` | A1 `Union`: slabs or a brep. See general-boolean A1. |
| `prism_boolean_crossing.go` | Cut/Intersect's crossing resolution. See prism-boolean §4.2. |
| `prism_overlap.go` | Prism-boolean §4.5's overlap-area reading for `Verify`. See its doc comment. |
| `brep_payload.go` / `brep_measure.go` | BRep face views, topology and readings. See general-boolean §4. |
| `classb.go` | Class-B admission and result adapters. See general-boolean §3 B. |
| `classb_crossing.go` | Class-B scene and BRep adapters. See general-boolean §5, §10. |
| `surface_trim.go` / `surface_split_revolve.go` | Surface body gates and revolve `Split`. See surface-intersection §2–§3. |
| `point_plane_split.go` | Splits a point loft by a planar chain sheet. See loft-point design. |
| `point_plane_scale.go` | Scales a certified Split slab about an anchor and carries its source map. See loft-point design. |
| `point_cap_centroid.go` | Certifies far and planar Split cap centroids and maps them through rigid placement. See loft-point design. |
| `point_cap_extremes.go` | Bounds side-face centroid projections and selects the extreme Split cap faces. See loft-point design. |
| `boolean_mesh.go` | Prepares an operand's mesh for `internal/meshbool/`. See evaluator §9. |
| `boolean_body.go` | Builds faceted bodies and readings from an audited mesh. See evaluator §9. |
