package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/clearance/curvepair"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// This file adapts the curve tiers in internal/clearance/curvepair and
// implements the vertex tiers and ruling certificates of clearance §3–§6.

// feCell dispatches one face × edge pair through §4's curve-tier table.
func (k *pairKernel) feCell(f *clearance.CFace, e *clearance.CEdge, sink *cellSink) {
	cells := curvepair.New(k.ctx, k.tol, k.slack)
	cells.FaceEdge(f, e, sink)
	k.captureCurveCells(cells)
}

// eeCell dispatches one edge pair through §4's curve tiers.
func (k *pairKernel) eeCell(ea, eb *clearance.CEdge, sink *cellSink) {
	cells := curvepair.New(k.ctx, k.tol, k.slack)
	cells.EdgeEdge(ea, eb, sink)
	k.captureCurveCells(cells)
}

func (k *pairKernel) principalCircleEdgeGap(ea, eb *clearance.CEdge, sink *cellSink) bool {
	cells := curvepair.New(k.ctx, k.tol, k.slack)
	ok := cells.PrincipalCircleEdgeGap(ea, eb, sink)
	k.captureCurveCells(cells)
	return ok
}

func (k *pairKernel) captureCurveCells(cells *curvepair.Kernel) {
	if err := cells.Err(); err != nil {
		k.err = err
	}
	k.clearanceRefused = k.clearanceRefused || cells.Refused()
}

// vertexTier runs one vertex (a topological vertex, a synthesized cone apex
// or a spindle axis-collapse point) against the other body's faces and
// edges — every cell closed form (§3's vertex tiers). It continues the
// enumerator's budget through every face, edge, and planar trim scan. A
// face or edge whose box lies beyond the sink's best upper bound is pruned
// (cellSink.Pruned).
func (k *pairKernel) vertexTier(budget *proofbound.WorkBudget, v r3.Vec, other *bodyGeom, sink *cellSink) error {
	at := [2]r3.Vec{v, v}
	for _, f := range other.faces {
		if err := budget.Step(); err != nil {
			return err
		}
		if sink.Pruned(clearance.ClrBoxDist(at, f.Box)) {
			continue
		}
		if err := k.vertexFace(budget, v, f, sink); err != nil {
			return err
		}
	}
	for _, e := range other.edges {
		if err := budget.Step(); err != nil {
			return err
		}
		if sink.Pruned(clearance.ClrBoxDist(at, e.Box)) {
			continue
		}
		k.vertexEdge(v, e, sink)
	}
	return nil
}

func (k *pairKernel) vertexFace(budget *proofbound.WorkBudget, v r3.Vec, f *clearance.CFace, sink *cellSink) error {
	switch f.Kind {
	case clearance.CkPlane:
		h := v.Sub(f.O).Dot(f.N)
		foot := v.Sub(f.N.Scale(h))
		x, y := f.PlaneCoords(foot)
		admit, err := clearance.RegionClassifyBudget(budget, f.Region, x, y, k.tol)
		if err != nil {
			return err
		}
		sink.Candidate(k.tol, admit, math.Abs(h), math.Abs(h), true, foot, v)
	case clearance.CkCone:
		rel := v.Sub(f.Anchor)
		z := rel.Dot(f.Axis)
		perp := rel.Sub(f.Axis.Scale(z))
		rho := perp.Len()
		sinA, cosA := math.Sincos(f.Half)
		var radial r3.Vec
		switch k.oracle().OnAxis(v, f.Anchor, f.Axis) {
		case clearance.DegYes:
			// Provenly on the axis: every azimuth carries the same distance, so
			// the sweep window's midpoint represents the family.
			mid := 0.0
			if !f.Sweep.Full {
				mid = (f.Sweep.Lo + f.Sweep.Hi) / 2
			}
			radial = f.RefU.Scale(math.Cos(mid)).Add(f.RefV.Scale(math.Sin(mid)))
		case clearance.DegNo:
			radial, _ = perp.Normalize()
		default:
			// The distance is azimuth-free, the admission foot is not. The
			// carrier distance is a proven lower bound; it stands as one.
			sink.LoOnly(math.Abs(rho*cosA - z*sinA))
			return nil
		}
		t := z*cosA + rho*sinA
		if t <= k.tol {
			return nil // the apex holds the nearest point; the vertex tiers pair with it
		}
		pf := f.Anchor.Add(f.Axis.Scale(t * cosA)).Add(radial.Scale(t * sinA))
		d := math.Abs(rho*cosA - z*sinA)
		sink.Candidate(k.tol, f.AdmitPoint(pf, k.tol), d, d, true, pf, v)
	default:
		d, foot := clearance.SpineDistOf(f, v)
		if d <= k.tol {
			sink.Unsure = true
			return nil
		}
		dir := v.Sub(foot).Scale(1 / d)
		for _, sf := range []float64{1, -1} {
			pf := foot.Add(dir.Scale(sf * f.Radius))
			raw := d - sf*f.Radius
			sink.Candidate(k.tol, f.AdmitPoint(pf, k.tol), math.Abs(raw), math.Abs(raw), true, pf, v)
		}
	}
	return nil
}

func (k *pairKernel) vertexEdge(v r3.Vec, e *clearance.CEdge, sink *cellSink) {
	if e.Line {
		dir := e.B.Sub(e.A)
		u, ok := dir.Normalize()
		if !ok {
			return
		}
		foot := clearance.LinePoint(e.A, u, v)
		d := v.Sub(foot).Len()
		sink.Candidate(k.tol, clearance.LineParamAdmit(e, foot, k.tol), d, d, true, foot, v)
		return
	}
	crits, ok := k.pointCircleCrits(v, e.Center, e.Axis, e.RefU, e.RefV, e.Radius, e.Ang)
	if !ok {
		// The radial direction is not resolvable, so the arc's own admission
		// cannot be decided — but the distance to the WHOLE circle bounds the
		// distance to any arc of it from below, and that is a proof.
		rel := v.Sub(e.Center)
		z := rel.Dot(e.Axis)
		rho := rel.Sub(e.Axis.Scale(z)).Len()
		sink.LoOnly(math.Hypot(z, math.Abs(rho-e.Radius)))
		return
	}
	for _, c := range crits {
		th := clearance.AngleOf(e, c.Fb.Sub(e.Center))
		d := c.Fa.Sub(c.Fb).Len()
		sink.Candidate(k.tol, clearance.CircleAngleAdmit(e, th, k.tol), d, d, true, c.Fb, v)
	}
}

// rulingContactCertified scans the plane-cylinder and cylinder-cylinder
// face pairs for a §6 ruling certificate. The caller runs it only when both
// bodies' bodyGeom.delta are exactly zero, so every carrier value is the
// boundary it names and every comparison below is exact.
func (k *pairKernel) rulingContactCertified(ctx context.Context) (*clearance.RulingContact, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	for _, fa := range k.a.faces {
		for _, fb := range k.b.faces {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			var ruling *clearance.RulingContact
			switch {
			case fa.Kind == clearance.CkPlane && fb.Kind == clearance.CkCylinder:
				ruling = k.planeCylinderRuling(ctx, fa, fb, k.a.body, k.b.body, false)
			case fa.Kind == clearance.CkCylinder && fb.Kind == clearance.CkPlane:
				ruling = k.planeCylinderRuling(ctx, fb, fa, k.b.body, k.a.body, true)
			case fa.Kind == clearance.CkCylinder && fb.Kind == clearance.CkCylinder:
				ruling = k.cylinderPairRuling(ctx, fa, fb)
			}
			if ruling != nil {
				return ruling, nil
			}
		}
	}
	return nil, ctx.Err()
}

// planeCylinderRuling certifies the cylinder's complete tangent ruling on the
// plane face. The cylinder axis must parallel the plane at exactly its radius
// on the outward side, the tangent azimuth and the whole axial window must be
// on the cylinder face, and the whole ruling must lie inside the plane trim.
// The plane must separate the bodies' complete extents. cylinderFirst names
// the cylinder's body as A.
func (k *pairKernel) planeCylinderRuling(ctx context.Context, plane, cyl *clearance.CFace,
	planeBody, cylBody *Body, cylinderFirst bool) *clearance.RulingContact {
	n, okN := clearance.ExactSignedAxis(plane.N)
	axis, okAxis := clearance.ExactSignedAxis(cyl.Axis)
	o, okO := clearance.DyVecOf(plane.O)
	anchor, okAnchor := clearance.DyVecOf(cyl.Anchor)
	radius, okR := proofarith.DyOf(cyl.Radius)
	lo, okLo := proofarith.DyOf(cyl.ZWin.Lo)
	hi, okHi := proofarith.DyOf(cyl.ZWin.Hi)
	if !okN || !okAxis || !okO || !okAnchor || !okR || !okLo || !okHi ||
		radius.Sign() <= 0 || proofarith.DyCmp(lo, hi) >= 0 ||
		!proofarith.DvDot(n, axis).IsZero() {
		return nil
	}
	offset := proofarith.DvDot(n, o)
	if proofarith.DyCmp(proofarith.DySubScalar(proofarith.DvDot(n, anchor), offset), radius) != 0 {
		return nil
	}
	if !k.tangentAzimuthAdmitted(cyl, plane.N.Scale(-1)) {
		return nil
	}
	base := proofarith.DvSub(anchor, dyScaleVec(n, radius))
	ends := [2]proofarith.DyV3{
		proofarith.DvAdd(base, dyScaleVec(axis, lo)),
		proofarith.DvAdd(base, dyScaleVec(axis, hi)),
	}
	if !k.rulingInsidePlaneTrim(plane, ends) {
		return nil
	}
	_, planeHi, okPlane := payloadExtent(ctx, planeBody, plane.N)
	cylLo, _, okCyl := payloadExtent(ctx, cylBody, plane.N)
	if !okPlane || !okCyl {
		return nil
	}
	planeHiDy, okPlaneHi := proofarith.DyOf(planeHi)
	cylLoDy, okCylLo := proofarith.DyOf(cylLo)
	if !okPlaneHi || !okCylLo || proofarith.DyCmp(planeHiDy, offset) > 0 ||
		proofarith.DyCmp(cylLoDy, offset) < 0 {
		return nil
	}
	ruling := &clearance.RulingContact{FaceA: plane, FaceB: cyl, Normal: n, Offset: offset, Ends: clearance.OrderedRulingEnds(ends)}
	if cylinderFirst {
		ruling.FaceA, ruling.FaceB = cyl, plane
		ruling.Normal = proofarith.DvSub(proofarith.DyV3{}, n)
		ruling.Offset = proofarith.DyNeg(offset)
	}
	return ruling
}

// cylinderPairRuling certifies the common ruling of two parallel external
// cylinders. Their axes must be exactly parallel, their center offset must be
// the radius sum along one signed coordinate axis, both tangent azimuths must
// be on their faces, and the axial windows must overlap with positive
// length. The tangent plane between them must separate the complete extents.
func (k *pairKernel) cylinderPairRuling(ctx context.Context, ca, cb *clearance.CFace) *clearance.RulingContact {
	axisA, okAxisA := clearance.ExactSignedAxis(ca.Axis)
	axisB, okAxisB := clearance.ExactSignedAxis(cb.Axis)
	anchorA, okAnchorA := clearance.DyVecOf(ca.Anchor)
	anchorB, okAnchorB := clearance.DyVecOf(cb.Anchor)
	rA, okRA := proofarith.DyOf(ca.Radius)
	rB, okRB := proofarith.DyOf(cb.Radius)
	loA, okLoA := proofarith.DyOf(ca.ZWin.Lo)
	hiA, okHiA := proofarith.DyOf(ca.ZWin.Hi)
	loB, okLoB := proofarith.DyOf(cb.ZWin.Lo)
	hiB, okHiB := proofarith.DyOf(cb.ZWin.Hi)
	if !okAxisA || !okAxisB || !okAnchorA || !okAnchorB || !okRA || !okRB ||
		!okLoA || !okHiA || !okLoB || !okHiB || rA.Sign() <= 0 || rB.Sign() <= 0 ||
		!proofarith.DvIsZero(proofarith.DvCross(axisA, axisB)) {
		return nil
	}
	delta := proofarith.DvSub(anchorB, anchorA)
	along := proofarith.DvDot(delta, axisA)
	perp := proofarith.DvSub(delta, dyScaleVec(axisA, along))
	normal, ok := clearance.AxisOfLength(perp, proofarith.DyAdd(rA, rB))
	if !ok {
		return nil
	}
	normalVec := clearance.DyAxisVec(normal)
	if !k.tangentAzimuthAdmitted(ca, normalVec) || !k.tangentAzimuthAdmitted(cb, normalVec.Scale(-1)) {
		return nil
	}
	// Map B's axial window into A's axis parameter.
	bLo, bHi := proofarith.DyAdd(along, loB), proofarith.DyAdd(along, hiB)
	if proofarith.DvDot(axisA, axisB).Sign() < 0 {
		bLo, bHi = proofarith.DySubScalar(along, hiB), proofarith.DySubScalar(along, loB)
	}
	lo, hi := dyMax(loA, bLo), dyMin(hiA, bHi)
	if proofarith.DyCmp(lo, hi) >= 0 {
		return nil
	}
	offset := proofarith.DyAdd(proofarith.DvDot(normal, anchorA), rA)
	_, aHi, okA := payloadExtent(ctx, k.a.body, normalVec)
	bLoExtent, _, okB := payloadExtent(ctx, k.b.body, normalVec)
	if !okA || !okB {
		return nil
	}
	aHiDy, okAHi := proofarith.DyOf(aHi)
	bLoDy, okBLo := proofarith.DyOf(bLoExtent)
	if !okAHi || !okBLo || proofarith.DyCmp(aHiDy, offset) > 0 || proofarith.DyCmp(bLoDy, offset) < 0 {
		return nil
	}
	base := proofarith.DvAdd(anchorA, dyScaleVec(normal, rA))
	ends := [2]proofarith.DyV3{
		proofarith.DvAdd(base, dyScaleVec(axisA, lo)),
		proofarith.DvAdd(base, dyScaleVec(axisA, hi)),
	}
	return &clearance.RulingContact{FaceA: ca, FaceB: cb, Normal: normal, Offset: offset, Ends: clearance.OrderedRulingEnds(ends)}
}

// tangentAzimuthAdmitted reports whether the cylinder face's angular trim
// holds the ruling whose outward radial direction is dir, with margin.
func (k *pairKernel) tangentAzimuthAdmitted(f *clearance.CFace, dir r3.Vec) bool {
	phi := math.Atan2(dir.Dot(f.RefV), dir.Dot(f.RefU))
	return f.Sweep.Classify(phi, k.tol/math.Max(f.Radius, 1e-30)) == 1
}

// rulingInsidePlaneTrim admits the whole segment into the plane face's trim:
// one end strictly inside with margin and the segment clear of every trim
// boundary element by more than the margin, so it never leaves the region.
func (k *pairKernel) rulingInsidePlaneTrim(plane *clearance.CFace, ends [2]proofarith.DyV3) bool {
	var coords [2][2]float64
	for i, end := range ends {
		// The nearest float of each end is within an ulp; the trim margin
		// below is many orders wider.
		p := clearance.DyAxisVec(end)
		if !proofbound.FiniteVec(p) {
			return false
		}
		coords[i][0], coords[i][1] = plane.PlaneCoords(p)
	}
	if plane.Region.Classify(coords[0][0], coords[0][1], k.tol) != 1 {
		return false
	}
	for _, e := range plane.Region.Elems {
		if clearance.SegElemDistLB(e, coords[0][0], coords[0][1], coords[1][0], coords[1][1]) <= k.tol {
			return false
		}
	}
	return true
}
