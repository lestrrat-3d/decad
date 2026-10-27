# decad file layout: design documents for added features

Design documents of the features that close rows of `agent-docs/missing-features.md`.
The rules for rows live in `agent-docs/layout.md`.

## Layout

### Design documents

| Path | Responsibility |
|---|---|
| `agent-docs/thread-design.md` | `Thread`: ISO metric thread cut into a cylindrical face through a coil, and its groove profile. |
| `agent-docs/gltf-export-design.md` | The glTF writer's file layout, metre scaling and error contract. |
| `agent-docs/iges-import-design.md` | `ReadIGES`: the IGES 5.3 record parser and the planar B-rep reader into a welded, wound mesh. |
| `agent-docs/step-import-design.md` | `ReadSTEP`: the Part 21 parser, the planar B-rep reader and the polygon triangulation. |
| `agent-docs/text-design.md` | `TextProfiles`: lettering from font outlines as sketch profiles, its layout and fill rule. |
| `agent-docs/union-fold-design.md` | The two-at-a-time fold of a many-body `Union` that no certified route takes. |
| `agent-docs/subd-design.md` | `SubDMesh`: Catmull-Clark subdivision of a polygon cage into a `MeshBody`. |
| `agent-docs/orthogonal-move-design.md` | `MoveFace`: the press-pull of one face of an orthogonal mesh solid on the shell's grid. |
| `agent-docs/sheet-metal-design.md` | `SheetMetal`: a bent strip of legs and circular bends as one solid, and its flat pattern. |
| `agent-docs/orthogonal-boolean-design.md` | `Union`, `Cut` and `Intersect` of two axis-square faceted solids on a shared grid: exact, any contact. |
| `agent-docs/orthogonal-shell-design.md` | `Shell` of a faceted solid whose faces all square to an axis: the grid erosion, the removed-face slab and the wall mesh. |
| `agent-docs/lattice-design.md` | `GyroidLattice`: the gyroid field, its box cap and the marching tetrahedra that cut it into a closed `MeshBody`. |
| `agent-docs/mesh-solid-design.md` | `SolidFromMesh`: a closed `MeshBody` as a Faceted solid `Body`, its proofs and exact plane groups. |
| `agent-docs/mesh-import-design.md` | `MeshBody`, `ReadSTL` and `ReadOBJ`: the mesh readers, the mesh report and its exact volume. |
| `agent-docs/path-pattern-design.md` | `PathPatternCopies`: copies spaced at equal arc length along a `Path`, and the path sampler. |
| `agent-docs/fillet-chord-design.md` | `FilletChord`: chord-length fillet as a radius, sharing the right-angle gate of `ChamferAngle`. |
| `agent-docs/arrange-design.md` | `Arrange`: shelf packing of body footprints onto a flat bed by translation. |
| `agent-docs/chamfer-angle-design.md` | `ChamferAngle`: distance-and-angle bevel as a second setback, and its right-angle gate. |
| `agent-docs/section-outline-design.md` | `Body.SectionOutline`: the mesh cut by a plane as closed outlines, its on-plane rule and its bound. |
| `agent-docs/boss-design.md` | `Boss`: a round boss and its blind pilot hole in one atomic union-and-cut commit. |
| `agent-docs/mesh-wall-survey-design.md` | The wall and undercut surveys of a faceted body by rays through its held mesh: what they prove and what they cannot. |
| `agent-docs/gcode-design.md` | `export.GCodePerimeters`: the layer-by-layer perimeter toolpath of a body as G-code. |
| `agent-docs/dxf-export-design.md` | `export.DXF`: feature-edge projection, hidden-line runs and the release-12 file. |
| `agent-docs/png-export-design.md` | `export.PNG` and `export.PNGBodies`: the orthographic flat-shaded renderer, its camera and shading rules. |
| `agent-docs/boolean-chain-regroup-design.md` | Rebuilding a refused two-body Boolean from the source bodies its history names: admitted set identities, plan, fallback and fixtures. |
