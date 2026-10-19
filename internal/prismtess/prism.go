// Package prismtess assembles a prism payload's chorded mesh over topology nodes.
package prismtess

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Build chords one prism payload into a transient mesh node. ctx and b must
// be non-nil; b must carry the live faces built from pp. The caller births
// the returned node in one root Mesh.
func Build(ctx context.Context, b *topo.Body, pp prismrecord.Payload,
	wallRole func(loop, seg int) string, chord float64, verifyAll bool,
	countFloor func(loop, seg int) int, freeformTarget float64) (*topo.Mesh, error) {
	// sheet is docs/surface-design.md §4.1's own flag, read once: a surface
	// result omits both caps from its wall build (internal/prismrecord/build.go), and every
	// arm below that would otherwise touch a cap face, a cap triangulation or
	// a cap's own area/volume term reads it to skip that half of the work
	// rather than fault a role the build never minted.
	sheet := pp.SurfaceResult

	// Every mesh vertex lands on the RECORDED section, which a payload carrying a
	// section displacement holds only within that displacement of the section its
	// construction DENOTES (docs/prism-boolean-design.md §7). The displacement is
	// therefore part of the mesh's deviation before a single chord is chosen, so
	// docs/tessellation-design.md §5 RESERVES it from the chord budget here, and a
	// tolerance that cannot pay for it refuses, the same shape as
	// facetedrecord.Restate's refusal of a tolerance below a held mesh bound. The
	// reservation keeps chording plus this displacement within tol; it covers no
	// other term, so the per-end axial displacement added at the end of the build
	// can still lift the complete Bound above tol, which §1's Tolerance row
	// allows. Both downward nudges are proven margin:
	// the subtraction's own rounding is at most half an ulp, which the first
	// covers, and the second pays for the upward-rounded sum this bound is
	// published through at the end of the build. Every payload a caller draws
	// carries a zero displacement and chords against the requested tolerance
	// unchanged.
	budget := chord
	if pp.SectionDelta > 0 {
		budget = freeform.DownRound(freeform.DownRound(chord - pp.SectionDelta))
		if budget <= 0 {
			requested := units.Millimeters(chord)
			displacement := units.Millimeters(pp.SectionDelta)
			return nil, fmt.Errorf(`%w: requested tolerance %s leaves no chord budget above the body's own section displacement %s; retry with a tolerance greater than %s`, decaderr.ErrUnsupported, requested, displacement, displacement)
		}
	}

	// Facets remember their source face (docs/evaluator-design.md §9); the
	// provenance roles are how the payload's walks name the faces evalPrism
	// built from them.
	byRole := map[string]*topo.Face{}
	for _, f := range topobuild.Faces(b) {
		for _, o := range f.Origins {
			byRole[o.Role] = f
		}
	}
	faceOfRole := func(role string) (*topo.Face, error) {
		f, ok := byRole[role]
		if !ok {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, decaderr.ErrDegenerate, role)
		}
		return f, nil
	}
	// A sheet's build never attaches capStart/capEnd to the shell
	// (internal/prismrecord/build.go), so byRole carries no such role for one: leave both
	// nil and skip every cap-only step below rather than fault a role this
	// evaluator's own reach never minted.
	var capStart, capEnd *topo.Face
	if !sheet {
		var err error
		capStart, err = faceOfRole(provenance.RoleCapStart)
		if err != nil {
			return nil, err
		}
		capEnd, err = faceOfRole(provenance.RoleCapEnd)
		if err != nil {
			return nil, err
		}
	}

	// One boundary polyline per loop: sample j is walk j's own start (the
	// junction shared with the previous walk) plus, for a circular walk, its
	// interior chord samples. Each 2D sample owns one bottom and one top mesh
	// vertex, so every face that meets a boundary curve reuses the SAME
	// chording — watertightness by construction.
	mesh := new(topo.Mesh)
	var chorded []tessellation.PrismLoop[*topo.Face]
	// One free-form counter for the whole chorded record (see tessellation.ChordLoop).
	work := freeform.NewFreeformWork()
	if freeformTarget > 0 {
		work.RaiseLimit(sweeprecord.CompositeFitWorkLimit)
	}
	// The build that produced this body already resolved every boundary
	// segment's walk and published the set onto the payload (internal/prismrecord/build.go,
	// docs/evaluator-design.md §8). A tessellation is one of the passes
	// docs/spline-design.md §5.2 names as free to REPLAY that recorded charge
	// instead of doing the work again: chording reads the walks back and
	// charges this fresh counter what resolving them cost, so the ceiling binds
	// the chording exactly as resolving would have. A payload carrying no
	// resolution of THIS record — a boolean result, a modify op's rewritten
	// section, a body an older evaluator built — leaves pw nil and every loop
	// resolves through walkOf as before.
	pw := pp.Walks
	if pw.Reusable(pp.Profile) {
		if err := pw.Charge(work); err != nil {
			return nil, err
		}
	} else {
		pw = nil
	}
	loops := append([]sectionrecord.LoopRecord{pp.Profile.Outer}, pp.Profile.Holes...)
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		var floor func(survey2d.SideWalk) int
		if countFloor != nil {
			floor = func(w survey2d.SideWalk) int { return countFloor(li, w.Segs[0]) }
		}
		cl, err := tessellation.ChordLoopWithCountFloor(ctx, loop, budget, pp.Z1-pp.Z0, work, pw, li, func(w survey2d.SideWalk) (*topo.Face, error) {
			return faceOfRole(wallRole(li, w.Segs[0]))
		}, stationbound.ChordStationBound, floor, freeformTarget)
		if err != nil {
			return nil, err
		}
		chorded = append(chorded, tessellation.PrismLoop[*topo.Face]{Samples: cl.Samples, FaceOf: cl.FaceOf,
			SagOf: cl.SagOf, BoundOf: cl.BoundOf, MaxSag: cl.MaxSag,
			WallSlack: cl.WallSlack, CapSlack: cl.CapSlack, SegmentArea: cl.SegmentArea,
			Walks: cl.Walks, PerimeterUpper: cl.PerimeterUpper})
	}
	topology := tessellation.PrismWalls(chorded, sheet, pp.AxialDelta())
	mesh.AreaSlack = topology.AreaSlack
	pts2, sampleBound := topology.Points, topology.Bounds
	faceTrim, faceAxial := topology.FaceTrim, topology.FaceAxial
	walks, perimeterUpper, segmentArea := topology.Walks, topology.PerimeterUpper, topology.SegmentArea

	// The mesh vertices: bottom and top of every boundary sample, placed
	// through the payload — exactly on the analytic boundary.
	mesh.Vertices = make([]r3.Vec, 0, 2*len(pts2))
	// vertexStore is docs/tessellation-reach-design.md §3's deltaStore, one
	// entry per mesh vertex: the sample's own plane-local enclosure gap carried
	// through the frame (proofbound.WalkEndBoundAllow) plus the rounding the frame and
	// placement write commits (prismrecord.Payload.PointRound). Both are zero for a
	// recorded line vertex under an axis-aligned identity payload; neither is
	// ever assumed zero for anything else.
	vertexStore := make([]float64, 0, 2*len(pts2))
	vertexBudget := proofbound.NewWorkBudget(ctx)
	for j, p := range pts2 {
		if err := vertexBudget.Step(); err != nil {
			return nil, err
		}
		lo := pp.Point(p.U, p.V, pp.Z0)
		hi := pp.Point(p.U, p.V, pp.Z1)
		mesh.Vertices = append(mesh.Vertices, lo, hi)
		plane := proofbound.WalkEndBoundAllow(sampleBound[j])
		vertexStore = append(vertexStore,
			proofbound.AbsSumUpper(plane, pp.PointRound(p.U, p.V, pp.Z0, lo)),
			proofbound.AbsSumUpper(plane, pp.PointRound(p.U, p.V, pp.Z1, hi)),
		)
	}
	storeMax, err := tessellation.StoreMax(vertexStore)
	if err != nil {
		return nil, err
	}
	mesh.CoordBound = storeMax

	// Prove loop clearance before both caps reuse the wall rings' vertices.
	if err := tessellation.PrismCaps(ctx, &topology, sheet, capStart, capEnd,
		pp.Z0Delta, pp.Z1Delta, tessellation.RequireLoopClearance, triangulation.Triangulate); err != nil {
		return nil, err
	}
	mesh.Triangles, mesh.Source = topology.Triangles, topology.Sources

	// A reflected placement flips handedness, turning every counter-clockwise
	// winding clockwise; reversing the windings restores outward orientation.
	if pp.Reflected() {
		for i := range mesh.Triangles {
			mesh.Triangles[i][1], mesh.Triangles[i][2] = mesh.Triangles[i][2], mesh.Triangles[i][1]
		}
	}
	// Every vertex sits on the RECORDED boundary at one of the recorded sweep
	// levels — within what its own construction rounded (vertexStore) — and a
	// payload holds each level and each section only within its own displacement
	// of what it denotes: the section's in the plane
	// (docs/prism-boolean-design.md §7), each end's along the normal. So a face
	// deviates by its own trim chording plus its own store, section and axial
	// terms, which is exactly what §3's faceBound composes. The chording above
	// already paid for the section displacement out of the reserved budget, so
	// chording plus that term stays within the requested tolerance; the axial
	// displacement carries no reservation of its own and can make the complete
	// bound larger. The section displacement moves AREA too: the section can
	// differ from the one it denotes by a tube about its own boundary, charged
	// once per cap, and the boundary's own length can differ by that tube's
	// length reading, charged over the sweep height — evalPrism's own composition
	// (2·regionArea + perimeter·height), one dimension at a time. Every such term
	// is zero for a payload a caller draws.
	// The walls and, on a solid, its caps close by construction; this proves
	// the assembled mesh is watertight (or, on a sheet, manifold-with-
	// boundary) and refuses a cracked one rather than return it — the same
	// safety net the cup path already carries (core §11, never a wrong mesh).
	if sheet {
		if err := topobuild.RequireSheetMesh(ctx, b, mesh); err != nil {
			return nil, err
		}
	} else if err := topobuild.RequireClosedMesh(mesh); err != nil {
		return nil, err
	}
	bounds, err := tessellation.FaceBounds(mesh.Triangles, mesh.Source,
		faceTrim, faceAxial, vertexStore, pp.SectionDelta)
	if err != nil {
		return nil, err
	}
	for face, bound := range bounds {
		topobuild.SetFaceBound(mesh, face, bound)
	}
	if !verifyAll {
		// The area slack accumulated above is only part of its composition —
		// the section-displacement and per-facet terms below are the rest —
		// and the occupied-volume proof has not started. The caller asked for
		// neither, so neither is finished and neither is published
		// (tessellateContext's own withholdProofs is what drops them). Every
		// face bound this mesh states is complete and stands.
		return mesh, nil
	}
	if pp.SectionDelta > 0 {
		wallMove := proofbound.ProductUpper(proofbound.SectionDisplacementLength(pp.SectionDelta, walks), math.Abs(pp.Z1-pp.Z0))
		if sheet {
			// A sheet carries no cap, so its section-displacement area charge
			// drops both cap terms and keeps the wall's own alone.
			mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack, wallMove)
		} else {
			capMove := proofbound.SectionDisplacementArea(pp.SectionDelta, walks, perimeterUpper)
			mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack, capMove, capMove, wallMove)
		}
	}
	// Every coordinate the build itself computed can move each facet's own area
	// (docs/tessellation-design.md §5's per-triangle allowance), so the slack
	// carries one such term per facet beside the analytic ones above.
	mesh.AreaSlack = proofbound.AbsSumUpper(mesh.AreaSlack,
		tessellation.StoreAreaAllow(mesh.Vertices, mesh.Triangles, vertexStore))

	if sheet {
		// A sheet encloses no region, so there is no occupied volume to
		// prove: the mesh publishes NO volSymDiff and leaves symDiffOK
		// false (docs/surface-design.md §10). That omission IS the proof's
		// absence, not a placeholder for one still to come — meshbool.OperandSymDiff
		// (boolean.go) is the consumer that reads symDiffOK and refuses a
		// boolean that reaches this operand.
		return mesh, nil
	}

	// Occupied volume (docs/tessellation-reach-design.md §3). The chorded section
	// differs from the section it denotes by the circular segments it omits or
	// adds, over the sweep height; the recorded section differs from the denoted
	// one by its own displacement tube, again over that height; each end level
	// differs by its own axial displacement over the cap it caps; and every
	// computed coordinate sweeps volume at the rate of the surface it moved.
	// Absolute sums throughout — an occupied-volume bound admits no cancellation.
	height := math.Abs(pp.Z1 - pp.Z0)
	areaUpper := tessellation.FaceAreaUpper(mesh.Vertices, mesh.Triangles, mesh.Source, vertexStore)
	terms := []float64{
		proofbound.ProductUpper(height, segmentArea),
		proofbound.ProductUpper(proofbound.SectionDisplacementArea(pp.SectionDelta, walks, perimeterUpper), height),
		proofbound.ProductUpper(pp.Z0Delta, areaUpper[capStart]),
		proofbound.ProductUpper(pp.Z1Delta, areaUpper[capEnd]),
		proofbound.SweptVolumeAllow(storeMax, proofbound.PerturbedAreaUpper(mesh.Vertices, mesh.Triangles, storeMax)),
	}
	total, err := tessellation.SymDiff(terms)
	if err != nil {
		return nil, err
	}
	mesh.VolSymDiff = total
	mesh.SymDiffOK = true
	return mesh, nil
}
