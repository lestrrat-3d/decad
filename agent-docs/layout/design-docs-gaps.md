# decad file layout: design documents for added features

Design documents of the features that close rows of `agent-docs/missing-features.md`.
The rules for rows live in `agent-docs/layout.md`.

## Layout

### Design documents

| Path | Responsibility |
|---|---|
| `agent-docs/thread-design.md` | `Thread`: ISO metric thread cut into a cylindrical face through a coil, and its groove profile. |
| `agent-docs/text-design.md` | `TextProfiles`: lettering from font outlines as sketch profiles, its layout and fill rule. |
| `agent-docs/union-fold-design.md` | The two-at-a-time fold of a many-body `Union` that no certified route takes. |
| `agent-docs/subd-design.md` | `SubDMesh`: Catmull-Clark subdivision of a polygon cage into a `MeshBody`. |
| `agent-docs/sheet-metal-design.md` | `SheetMetal`: a bent strip of legs and circular bends as one solid, and its flat pattern. |
| `agent-docs/lattice-design.md` | `GyroidLattice`: the gyroid field, its box cap and the marching tetrahedra that cut it into a closed `MeshBody`. |
| `agent-docs/mesh-solid-design.md` | `SolidFromMesh`: a closed `MeshBody` as a Faceted solid `Body`, its proofs and exact plane groups. |
| `agent-docs/mesh-import-design.md` | `MeshBody`, `ReadSTL` and `ReadOBJ`: the mesh readers, the mesh report and its exact volume. |
| `agent-docs/path-pattern-design.md` | `PathPatternCopies`: copies spaced at equal arc length along a `Path`, and the path sampler. |
| `agent-docs/blend-variants-design.md` | Chord-length fillet, angle chamfer and the oblique boss fillet. |
| `agent-docs/arrange-design.md` | `Arrange`: shelf packing of body footprints onto a flat bed by translation. |
| `agent-docs/parts-design.md` | `parts`: exploded view and schematic of placed bodies; typed joints, part views, BOM, summary, a structural joint check without `Verify`, linkage bridge; `export.ThreeMFItems`. |
| `agent-docs/section-outline-design.md` | `Body.SectionOutline`: the mesh cut by a plane as closed outlines, its on-plane rule and its bound. |
| `agent-docs/boss-design.md` | `Boss`: a round boss and its blind pilot hole in one atomic union-and-cut commit. |
| `agent-docs/mesh-wall-survey-design.md` | The wall and undercut surveys of a faceted body by rays through its held mesh: what they prove and what they cannot. |
| `agent-docs/dxf-export-design.md` | `export.DXF`: feature-edge projection, hidden-line runs and the release-12 file. |
| `agent-docs/png-export-design.md` | `export.PNG` and `export.PNGBodies`: the orthographic flat-shaded renderer, its camera and shading rules. |
| `agent-docs/boolean-chain-regroup-design.md` | Rebuilding a refused two-body Boolean from the source bodies its history names: admitted set identities, plan, fallback and fixtures. |
| `agent-docs/boolean-continuation-design.md` | Certified source-expression continuation after an accepted geometry adjustment; set rewrites, materialization, bounds and staged proofs. |
| `agent-docs/trim-graph-design.md` | General modify reach over retained analytic faces: trim graph, offsets, certified curves, face meshing, gates, increments. |
| `agent-docs/revolve-loft-design.md` | The Revolve and Loft contact graph, the sector Boolean and the operand coarsening retry. |
| `agent-docs/body-refine-design.md` | `Refine`: surface bounds for held Union meshes of convex leaves, the occupancy certificate, records and increments. |
| `agent-docs/orthogonal-design.md` | Axis-square faceted solids on a shared grid: `Shell`, the Booleans, `Body.Section`, `Offset`, `MoveFace` and the wall, undercut and concave-radius surveys. |
| `agent-docs/format-io-design.md` | The small readers and writers: `ReadSTEP`, `ReadIGES`, `export.IGES`, `export.GLTF`, `export.ThreeMF` and `export.GCodePerimeters`. |
