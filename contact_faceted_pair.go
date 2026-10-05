package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/pair"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file admits exact planar solids to the general pair relation of
// docs/multibody-dynamics-design.md §9.1 and caches each body's §9.2
// convexity certificate. internal/pair decides the relation over the exact
// snapshots built here; this file owns admission and the public report.

// planarConvexityEntry is a body's cached convexity certificate. It is read
// at the identity pose: every admitted pose is an affine map with a positive
// determinant, which preserves every orientation sign the certificate reads.
type planarConvexityEntry struct {
	convex bool
}

// classifyExactPlanarPair decides the pair through the exact planar kernel
// when both bodies are admitted. It reports false, leaving report untouched,
// when either body is not.
func classifyExactPlanarPair(ctx context.Context, report *ContactReport) (bool, error) {
	budget := newWorkBudget(ctx)
	if err := budget.err(); err != nil {
		return false, err
	}
	a, okA, err := planarSolidAtPose(ctx, budget, report.A, report.PoseA)
	if err != nil || !okA {
		return false, err
	}
	b, okB, err := planarSolidAtPose(ctx, budget, report.B, report.PoseB)
	if err != nil || !okB {
		return false, err
	}
	result, err := pair.ClassifyPlanar(&a, &b, budget.step)
	if err != nil {
		return false, err
	}
	report.Relation, report.Gap, report.Overlap, report.Manifold = ContactUndecided, nil, nil, nil
	report.Reason = sourceBoxReason(result.Reason)
	switch result.Relation {
	case pair.Separated:
		report.Relation = ContactSeparated
		gap := sourceBoxScalar(*result.Gap)
		report.Gap = &gap
	case pair.Touching, pair.Overlapping:
		report.Relation = ContactOverlapping
		if result.Relation == pair.Touching {
			report.Relation = ContactTouching
			gap := Measurement{Value: units.Millimeters(0), Exactness: Exact, Bound: units.Millimeters(0)}
			report.Gap = &gap
		}
		// §9.3 needs one convex side for a manifold; this stage publishes
		// none, so the reason names which certificate is missing.
		report.Reason = ContactNoNormalProof
		convexA, err := planarConvexity(ctx, budget, report.A)
		if err != nil {
			return false, err
		}
		if !convexA {
			convexB, err := planarConvexity(ctx, budget, report.B)
			if err != nil {
				return false, err
			}
			if !convexB {
				report.Reason = ContactNonConvex
			}
		}
	default:
		if report.Reason == ContactNoReason {
			report.Reason = ContactAmbiguousFeature
		}
	}
	return true, budget.err()
}

// planarConvexity returns the body's cached §9.2 certificate, computing it at
// the identity pose on first use. A canceled computation caches nothing.
func planarConvexity(ctx context.Context, budget *workBudget, b *Body) (bool, error) {
	if entry := b.planarConvexity.Load(); entry != nil {
		return entry.convex, nil
	}
	solid, ok, err := planarSolidAtPose(ctx, budget, b, r3.Identity())
	if err != nil || !ok {
		return false, err
	}
	convex, err := pair.PlanarConvex(&solid, budget.step)
	if err != nil {
		return false, err
	}
	b.planarConvexity.Store(&planarConvexityEntry{convex: convex})
	return convex, nil
}

// planarSolidAtPose builds the exact boundary of an admitted planar solid
// under a query pose (§9): a prism whose section is all whole LineSeg edges
// with zero deltas, or a zero-bound faceted Boolean, either directly or
// through its saved exact mesh and a translation-only placement. Every vertex
// is the exact dyadic image of recorded coordinates under the recorded
// placement and the pose; nothing is read from a rounded transient body.
func planarSolidAtPose(ctx context.Context, budget *workBudget, b *Body,
	pose r3.Transform) (pair.PlanarSolid, bool, error) {
	if b == nil || !b.solid || b.kind != BodySolid || !positiveAffine(pose) {
		return pair.PlanarSolid{}, false, nil
	}
	var solid pair.PlanarSolid
	var ok bool
	var err error
	switch payload := b.payload.(type) {
	case prismPayload:
		solid, ok, err = planarPrismSolid(ctx, budget, payload)
	case facetedPayload:
		solid, ok, err = planarFacetedSolid(budget, payload)
	default:
		return pair.PlanarSolid{}, false, nil
	}
	if err != nil || !ok {
		return pair.PlanarSolid{}, false, err
	}
	for i, v := range solid.Verts {
		if err := budget.step(); err != nil {
			return pair.PlanarSolid{}, false, err
		}
		solid.Verts[i] = exactContactTransform(pose, v)
	}
	audited, err := pair.CheckPlanarSolid(&solid, budget.step)
	if err != nil || !audited {
		return pair.PlanarSolid{}, false, err
	}
	return solid, true, nil
}

// positiveAffine admits a finite transform whose exact basis determinant is
// positive. Such a map preserves every orientation sign, so a snapshot's
// outward winding and convexity survive it.
func positiveAffine(t r3.Transform) bool {
	if !t.IsValid() || !finiteVec(t.Translation()) {
		return false
	}
	basis := t.Basis()
	if !finiteVec(basis.EX) || !finiteVec(basis.EY) || !finiteVec(basis.EZ) {
		return false
	}
	ex, ey, ez := proofarith.DyVec(basis.EX), proofarith.DyVec(basis.EY), proofarith.DyVec(basis.EZ)
	return proofarith.DvDot(ex, proofarith.DvCross(ey, ez)).Sign() > 0
}

// planarPrismSolid lifts an all-LineSeg section through its frame and sweep
// levels. The caps reuse the tessellator's triangulation, which this function
// then proves: every cap triangle is exactly counterclockwise and, through the
// closed-mesh audit, the caps' boundary chain is the section loops, so the
// triangles tile the section exactly.
func planarPrismSolid(ctx context.Context, budget *workBudget,
	pp prismPayload) (pair.PlanarSolid, bool, error) {
	if pp.surfaceResult || pp.sectionDelta != 0 || pp.z0Delta != 0 || pp.z1Delta != 0 ||
		!finiteMeasurementValues(pp.z0, pp.z1) || pp.z0 >= pp.z1 || !positiveAffine(pp.xform) ||
		!finiteVec(pp.frame.Origin()) || !finiteVec(pp.frame.U()) ||
		!finiteVec(pp.frame.V()) || !finiteVec(pp.frame.N()) {
		return pair.PlanarSolid{}, false, nil
	}
	fu, fv, fn := proofarith.DyVec(pp.frame.U()), proofarith.DyVec(pp.frame.V()), proofarith.DyVec(pp.frame.N())
	if proofarith.DvDot(fu, proofarith.DvCross(fv, fn)).Sign() <= 0 {
		return pair.PlanarSolid{}, false, nil
	}
	var pts []Point2
	loops := make([][]int, 0, 1+len(pp.profile.Holes))
	for role, loop := range append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...) {
		indices, ok := planarLoopPoints(loop, &pts)
		if !ok {
			return pair.PlanarSolid{}, false, nil
		}
		area := planarLoopArea(pts, indices)
		if (role == 0 && area.Sign() <= 0) || (role > 0 && area.Sign() >= 0) {
			return pair.PlanarSolid{}, false, nil
		}
		loops = append(loops, indices)
	}
	caps, err := triangulate2DContext(ctx, pts, loops)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return pair.PlanarSolid{}, false, ctxErr
		}
		return pair.PlanarSolid{}, false, nil
	}
	n := len(pts)
	solid := pair.PlanarSolid{Verts: make([]proofarith.DyV3, 2*n)}
	origin := proofarith.DyVec(pp.frame.Origin())
	z := [2]proofarith.Dyadic{proofarith.MustDyOf(pp.z0), proofarith.MustDyOf(pp.z1)}
	for i, p := range pts {
		if err := budget.step(); err != nil {
			return pair.PlanarSolid{}, false, err
		}
		local := proofarith.DvAdd(origin, proofarith.DvAdd(dyScaleVec(fu, proofarith.MustDyOf(p.U)),
			dyScaleVec(fv, proofarith.MustDyOf(p.V))))
		for level := range z {
			solid.Verts[level*n+i] = exactContactTransform(pp.xform,
				proofarith.DvAdd(local, dyScaleVec(fn, z[level])))
		}
	}
	for _, tri := range caps {
		if err := budget.step(); err != nil {
			return pair.PlanarSolid{}, false, err
		}
		if planarCross2(pts[tri[0]], pts[tri[1]], pts[tri[2]]).Sign() <= 0 {
			return pair.PlanarSolid{}, false, nil
		}
		solid.Tris = append(solid.Tris, [3]int{tri[0], tri[2], tri[1]},
			[3]int{n + tri[0], n + tri[1], n + tri[2]})
	}
	for _, loop := range loops {
		for i, from := range loop {
			to := loop[(i+1)%len(loop)]
			solid.Tris = append(solid.Tris, [3]int{from, to, n + to}, [3]int{from, n + to, n + from})
		}
	}
	return solid, true, nil
}

// planarLoopPoints appends one loop's walk-start points to pts and returns
// their indices. It admits only whole LineSeg edges that chain exactly.
func planarLoopPoints(loop LoopRecord, pts *[]Point2) ([]int, bool) {
	if len(loop.Segments) < 3 {
		return nil, false
	}
	indices := make([]int, 0, len(loop.Segments))
	var first, last Point2
	for i, segment := range loop.Segments {
		line, ok := segment.(LineSeg)
		if !ok {
			return nil, false
		}
		start, end := line.Start, line.End
		switch {
		case line.TStart == 0 && line.TEnd == 1:
		case line.TStart == 1 && line.TEnd == 0:
			start, end = end, start
		default:
			return nil, false
		}
		if !finiteMeasurementValues(start.U, start.V, end.U, end.V) || start == end ||
			(i > 0 && start != last) {
			return nil, false
		}
		if i == 0 {
			first = start
		}
		last = end
		indices = append(indices, len(*pts))
		*pts = append(*pts, start)
	}
	if last != first {
		return nil, false
	}
	return indices, true
}

// planarLoopArea is twice the exact signed area of a loop.
func planarLoopArea(pts []Point2, loop []int) proofarith.Dyadic {
	area := proofarith.DyZero()
	for i, from := range loop {
		a, b := pts[from], pts[loop[(i+1)%len(loop)]]
		area = proofarith.DyAdd(area, proofarith.DySubScalar(
			proofarith.DyMul(proofarith.MustDyOf(a.U), proofarith.MustDyOf(b.V)),
			proofarith.DyMul(proofarith.MustDyOf(b.U), proofarith.MustDyOf(a.V))))
	}
	return area
}

// planarCross2 is the exact (b−a)×(c−a) of three section points.
func planarCross2(a, b, c Point2) proofarith.Dyadic {
	au, av := proofarith.MustDyOf(a.U), proofarith.MustDyOf(a.V)
	bu := proofarith.DySubScalar(proofarith.MustDyOf(b.U), au)
	bv := proofarith.DySubScalar(proofarith.MustDyOf(b.V), av)
	cu := proofarith.DySubScalar(proofarith.MustDyOf(c.U), au)
	cv := proofarith.DySubScalar(proofarith.MustDyOf(c.V), av)
	return proofarith.DySubScalar(proofarith.DyMul(bu, cv), proofarith.DyMul(bv, cu))
}

// planarFacetedSolid reads a zero-bound Boolean's held mesh, which is its
// exact boundary. A translation-only placement reads the saved exact source
// mesh moved by the exact placement translation, after a reject-only check
// that every rebuilt vertex lies within the published bound of it.
func planarFacetedSolid(budget *workBudget, pp facetedPayload) (pair.PlanarSolid, bool, error) {
	if len(pp.verts) == 0 || len(pp.tris) == 0 {
		return pair.PlanarSolid{}, false, nil
	}
	source := pp.verts
	placed := false
	if pp.meshBound != 0 || pp.volSymDiff != 0 {
		if !finiteMeasurementValues(pp.meshBound) || pp.meshBound < 0 ||
			!facetedTranslationOnly(pp.xform) || len(pp.exactSourceVerts) != len(pp.verts) ||
			len(pp.exactSourceTris) != len(pp.tris) {
			return pair.PlanarSolid{}, false, nil
		}
		for i, tri := range pp.tris {
			if tri != pp.exactSourceTris[i] {
				return pair.PlanarSolid{}, false, nil
			}
		}
		source, placed = pp.exactSourceVerts, true
	}
	solid := pair.PlanarSolid{Verts: make([]proofarith.DyV3, len(source)),
		Tris: append([][3]int(nil), pp.tris...)}
	bound := proofarith.MustDyOf(pp.meshBound)
	for i, v := range source {
		if err := budget.step(); err != nil {
			return pair.PlanarSolid{}, false, err
		}
		if !finiteVec(v) {
			return pair.PlanarSolid{}, false, nil
		}
		solid.Verts[i] = proofarith.DyVec(v)
		if !placed {
			continue
		}
		if !finiteVec(pp.verts[i]) {
			return pair.PlanarSolid{}, false, nil
		}
		solid.Verts[i] = exactContactTransform(pp.xform, solid.Verts[i])
		difference := proofarith.DvSub(solid.Verts[i], proofarith.DyVec(pp.verts[i]))
		if proofarith.DyCmp(proofarith.DvDot(difference, difference), proofarith.DyMul(bound, bound)) > 0 {
			return pair.PlanarSolid{}, false, nil
		}
	}
	return solid, true, nil
}
