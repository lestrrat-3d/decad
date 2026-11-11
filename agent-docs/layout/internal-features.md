# decad file layout: internal packages for Loft, prism, placement and blends

Internal packages of Loft, prism and Shell, body placement, the trim graph and the edge blends that rewrite a mesh
Boolean result. The rules for rows live in `agent-docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/pointcone/` | Point-loft cone crossing samples about an exact axis, far-cap subdivision and contact split, cap and triangle crossing bounds, sector cap paths, subdivision, merged angles and root-edge order, and the strict-side meridian offset. See loft-point, loft-face-sections and revolve-axis designs. |
| `internal/pointloft/` | Builds the point-section Loft apex fan and proves its source stations, boundary, area and volume. See loft-point design. |
| `internal/loftrecord/` | The loft and chain loft payloads and builders: `FromProfiles`/`FromChains`, `FromProfilesReusing` (`PairingReuse` carries one build's pairing to a later build), evaluation, placement, topology, bore proofs, corner Fillet, mass readings. See loft §5, §7, §16, §18, evaluator §8. |
| `internal/loftsplit/` | Cuts recorded profile segments at stated parameter fractions into pieces of the same curves, so loft sections of different segment counts pair. See loft §3.1. |
| `internal/loftmesh/` | Loft and chain mesh assembly, point-plane and LoftFaces cap layouts and proofs, stations, bounds, split and collar grids, triangle clearance, and contact scans. See loft-point and tessellation designs. |
| `internal/smoothloft/` | Three-section homothety, constant curved-section admission and independent rectangles with exact height curves for Section; varying curves, four-section rounded rectangles, and twisted sections of one fitted edge with whole lines. See loft-sections design. |
| `internal/loftfillet/` | Loft fit/circle root-fillet candidate, the fillet that rests on a consumed line's fit-spline vertex, tangent-root certificate, connector and carrier departures, section contact audits and circle-bore containment. See loft §17–§18. |
| `internal/loftcylinder/` | Cap and source-circle audits, cap tags, mirrored blank and pairs, Join seams, the Union and bore Cut dispatch with its placed and mirrored forms, cap and mirrored Chamfer, proof teeth, local rebuilds, shell and bounds. See loft-cylinder-boolean and loft-mirror-boolean designs. |
| `internal/loftcontact/` | Varying Loft/prism carrier, hull, exact arithmetic, source partition and paired cap certificates, the output shell with its caps, walls, audit, embedding and volume, quarter-Loft held seams and pair separation, held cap outers, and cap, interval and wall records. See loft-mirror-boolean design. |
| `internal/loftdirect/` | Curved-contact source spans, quarter-arc crossing and wall-order certificates, the Loft/prism output proof, and the bounded direct-shell Union for two live Loft/prism bodies, working on topology nodes. |
| `internal/loftmirror/` | Joins one authenticated axial Loft to its end-cap reflection into one faceted body that records the join. See loft-mirror-boolean design. |
| `internal/axialsection/` | Reads prisms and matching axial lofts as axial extrusions and joins their sections through sketch loop containment into a matching axial loft, placing placed operands first. See axial-section-boolean and axial-pattern-join. |
| `internal/prismboolean/` | Analytic Boolean admission, group-source replay, bounded cap selection and record authentication. See prism-boolean §3 and general-boolean §3 A1, A5, A7, A8. |
| `internal/stackedunion/` | Builds A1 stacked Union slabs, private scenes, and BRep faces after `internal/prismboolean` admits them, and states A7 and A8 blind cuts as BReps. See general-boolean §3 A1, A7, A8. |
| `internal/stackedshell/` | Admits and records the exact stacked shells (nested rectangular boss, circular boss, rectangular blind pocket, coaxial counterbore) from a two-slab record and its face view, and `RefuseClosed`; `breprecord` builds the result. See modify-general §3.1a-§3.1c, §3.1e. |
| `internal/prismshell/` | Prism cap-shell gates with tube, band and closed-shell records; side sections, the side-opening prism, wall bands, nested boss authentication, mesh and proof, closed stack reversal, denoted slab levels and stack faces. See modify §8, shell-opening §3–§5, shell-faceted-boss design. |
| `internal/brepmodify/` | Fillet, Chamfer, Shell engines: brep and stacked routes P (and its displaced reading), E, L, V and S; revolve, loft, Union and cutter routes; asymmetric chamfer, side openings, twisted cups, tapered-circle rim and cup, free-form rim chamfer, faceted views. See brep-modify, modify-reach §8.5. |
| `internal/rimoffset/` | Offsets a smooth closed loop of fit splines and lines for the free-form rim chamfer: sampled offset fit points, joint bisectors, curvature fold and deviation checks. See modify-reach §8.5. |
| `internal/prismrecord/` | The prism record and its node builder (`Build`, `Placed`, extents, map charge); plane-local lifts, point bounds, lift factor, centroid envelope, placement operand, cap frames, mass record, exact box, section-plane test, exact-section gates. Root `prismPayload` aliases it. See evaluator §5, §8. |
| `internal/prismtess/` | Builds a prism payload's chorded mesh node and its face, area and occupied-volume bounds. See tessellation §5. |
| `internal/tessdispatch/` | Routes a body to the tessellator of its payload, and builds the composite Sweep, mitred Sweep, PatchEdges and cap-blend meshes around them. See tessellation design, sweep Table D. |
| `internal/stackedrecord/` | Stacked record and node builder (`Build`, `Placed`, `Plan`, extents, runs, bounds), aliased by root `stackedPrismPayload`; blind-cut slabs via `prismcells`, the opposed cut in `opposed_cut.go` (`ApplyOpposedBlindCut`); interfaces, columns, patches, mesh. See stacked-prism §2.2–§7, evaluator §8. |
| `internal/prismextent/` | Prism and profile boundary extremes, bounds and extent readings. See evaluator §5. |
| `internal/prismplacement/` | Exact prism axis shifts, sweep spans, relative placement, exact rectangular boxes in a reference frame, denoted sweep levels and their exact ranges. See prism-boolean §3, stacked-prism §2.1. |
| `internal/prismcells/` | Analytic prism entry and pair admission, scene construction, §4.2 union, cut, intersect and crossing resolution, merges and audits, cells, charges, trim walks, the placed-axis rectangle gate, overlap volume and arrangement caps. See prism-boolean §4, §6, G3, section design. |
| `internal/prismpattern/` | Prism-pattern Union blank and copy authentication, radial source gaps, wall certificates, fillet contacts, ring cuts, polar sectors, clearance, rounded shell and caps, wall mesh, root chords and section separation. See prism-pattern-union design. |
| `internal/cupwall/` | The cup record and view with denoted levels, its node builder (`Build`, `Placed`, cup roles and face order), the `PlanCapShell` offset stage and the cup wall theorem and morphology recheck. Root `cupPayload` and `cupView` alias it. See modify-reach §9.1, payload verification §4, evaluator §8. |
| `internal/shellsurvey/` | Shell inradius, section cavity limit, work budget and contained-disk witness. See modify §8. |
| `internal/shellseparation/` | Bounded source-shell and held-copy separation, exact slab projections, and placement error. See faceted-vertex-bounds §3.7. |
| `internal/arrangeplan/` | Shelf packing of rectangular footprints onto a bed with a gap; plain millimetre data. See arrange design. |
| `internal/jointinfer/` | Matches cylinder and plane face records of two parts into pin-in-bore, pin-in-slot and face-contact joints; plain millimetre data. See parts design §7.1. |
| `internal/jointcheck/` | Box gaps, interval verdicts, line offsets and the overlap witness of two planar faces for the structural joint check; plain millimetre data. See parts design §7.2. |
| `internal/explodeplan/` | Breadth-first walk of a joint graph from a ground part, with box travels and offsets for an exploded view; plain millimetre boxes. See parts design §5. |
| `internal/pathsample/` | Equal-arc-length points along a path of lines and circular arcs; plain millimetre vectors. See path-pattern design. |
| `internal/glyphs/` | TrueType layout of a line of text and flattening of its glyph outlines to closed polygons. See text design. |
| `internal/igesread/` | IGES 5.3 fixed-format records into entities, delimiters and the unit. See iges-import design. |
| `internal/igesbrep/` | Planar-faced manifold solid B-rep entities into a welded, consistently wound triangle mesh. See iges-import design. |
| `internal/stepread/` | ISO 10303-21 file syntax into entity numbers, names and parameter values. See step-import design. |
| `internal/stepbrep/` | Planar-faced MANIFOLD_SOLID_BREP and BREP_WITH_VOIDS into a welded triangle mesh. See step-import design. |
| `internal/polytri/` | Ear-clipping triangulation of a planar polygon with holes. See step-import design. |
| `internal/anasurf/` | Exact plane tags with interval offsets, three-plane meets as enclosures and strict half-space tests. See trim-graph Component 2. |
| `internal/trimgraph/` | Lifts a retained faceted body into faces, edges and vertices on exact plane tags, and decides edge convexity from the tags. See trim-graph Component 1, Table L. |
| `internal/trimtest/` | Hand-made retained prisms and a pyramid with exact plane frames, for the trim-graph packages' tests. |
| `internal/facemesh/` | Meshes one planar face of a trim graph over `internal/triangulation`, refusing a dropped point or a triangle that fails the exact orientation margin. See trim-graph Component 4. |
| `internal/trimrebuild/` | Splices new faces into a receiver's held mesh, composes the volume, area and diameter bounds, runs the held-embedding gate and builds the body. See trim-graph Component 5. |
| `internal/trimshell/` | Shell over a trim graph: the closed inward shell of a convex planar receiver, the offset-plane vertex meets and gate C1. See trim-graph Component 6. |
| `internal/obliqueboss/` | Recognizes a complete oblique boss junction, admits its junction fillet and builds the mesh with its boundary, volume and area proofs. See oblique-boss-fillet design. |
| `internal/retainedblend/` | Admits Table RR edges and tangent chains between retained faces, rewrites the held mesh into the blend, cuts or fillets an R7 saddle, refuses a strip that meets another facet, and bounds the area and volume change. See retained-edge-blend and retained-modify-reach designs. |
| `internal/saddlecanal/` | The rolling-ball fillet of a saddle in the cross-bore pose, the ball inside or outside the larger cylinder: existence, rows, points and the interval cell proofs of regularity, embedding, chord bound and areas. See retained-edge-blend §4.10. |
