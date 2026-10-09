// Package tier evaluates vertex cells and exact ruling certificates.
package tier

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/clearance/spine"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Kernel holds one tier calculation's tolerance and any spine error.
type Kernel struct {
	tol, slack float64
	ctx        context.Context //nolint:containedctx // The kernel is per-call state.
	err        error
	refused    bool
}

// New creates one clearance tier kernel for the caller's pair.
func New(ctx context.Context, tol, slack float64) *Kernel {
	return &Kernel{ctx: ctx, tol: tol, slack: slack}
}
func (k *Kernel) Err() error               { return k.err }
func (k *Kernel) Refused() bool            { return k.refused }
func (k *Kernel) oracle() clearance.Oracle { return clearance.Oracle{Tol: k.tol} }

func (k *Kernel) pointCircleCrits(p, c, axis, refU, refV r3.Vec, rad float64, win clearance.AngWindow) ([]clearance.SpineCrit, bool) {
	e := &spine.Engine{Context: k.ctx, Tolerance: k.tol, Slack: k.slack}
	out, ok := e.PointCircleCrits(p, c, axis, refU, refV, rad, win)
	if e.Err != nil {
		k.err = e.Err
	}
	k.refused = k.refused || e.Refused
	return out, ok
}

// VertexFace contributes the point-to-face candidates to sink.
func (k *Kernel) VertexFace(budget *proofbound.WorkBudget, v r3.Vec, f *clearance.CFace, sink *clearance.CellSink) error {
	switch f.Kind {
	case clearance.CkPlane:
		h := v.Sub(f.O).Dot(f.N)
		foot := v.Sub(f.N.Scale(h))
		x, y := f.PlaneCoords(foot)
		admit, err := clearance.RegionClassifyBudget(budget, f.Region, x, y, k.tol)
		if err != nil {
			return err
		}
		d := clearance.Height(v, f.O, f.N).Abs().Widen(clearance.DirCharge([]r3.Vec{f.N}, []r3.Vec{v, f.O}))
		sink.CandidateDist(k.tol, admit, d, foot, v)
	case clearance.CkCone:
		rel := v.Sub(f.Anchor)
		z := rel.Dot(f.Axis)
		perp := rel.Sub(f.Axis.Scale(z))
		rho := perp.Len()
		// The meridian reading comes off the cone's slope; its float form only
		// places the foot, and the published distance is ConeDist's enclosure.
		_, t := f.ConeMeridian(z, rho)
		d := f.ConeDist(v).Widen(clearance.DirCharge([]r3.Vec{f.Axis}, []r3.Vec{v, f.Anchor}))
		sinA, cosA := f.ConeSinCos()
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
			sink.LoOnly(d.Lo)
			return nil
		}
		if t <= k.tol {
			return nil // the apex holds the nearest point; the vertex tiers pair with it
		}
		pf := f.Anchor.Add(f.Axis.Scale(t * cosA)).Add(radial.Scale(t * sinA))
		sink.CandidateDist(k.tol, f.AdmitPoint(pf, k.tol), d, pf, v)
	default:
		d, foot := clearance.SpineDistOf(f, v)
		if d <= k.tol {
			sink.Unsure = true
			return nil
		}
		spineDist := clearance.SpineDist(f, v)
		dir := v.Sub(foot).Scale(1 / d)
		for _, sf := range []float64{1, -1} {
			pf := foot.Add(dir.Scale(sf * f.Radius))
			sink.CandidateDist(k.tol, f.AdmitPoint(pf, k.tol), spineDist.Sub(sf*f.Radius).Abs(), pf, v)
		}
	}
	return nil
}

// VertexEdge contributes the point-to-edge candidates to sink.
func (k *Kernel) VertexEdge(v r3.Vec, e *clearance.CEdge, sink *clearance.CellSink) {
	if e.Line {
		dir := e.B.Sub(e.A)
		u, ok := dir.Normalize()
		if !ok {
			return
		}
		foot := clearance.LinePoint(e.A, u, v)
		seg, okSeg := clearance.SegDir(e.A, e.B)
		if !okSeg {
			return
		}
		sink.CandidateDist(k.tol, clearance.LineParamAdmit(e, foot, k.tol), clearance.PointLineDist(v, e.A, seg), foot, v)
		return
	}
	crits, ok := k.pointCircleCrits(v, e.Center, e.Axis, e.RefU, e.RefV, e.Radius, e.Ang)
	if !ok {
		// The radial direction is not resolvable, so the arc's own admission
		// cannot be decided — but the distance to the WHOLE circle bounds the
		// distance to any arc of it from below, and that is a proof.
		d := clearance.PointCircleDist(v, e.Center, e.Axis, e.Radius, 1).
			Widen(clearance.DirCharge([]r3.Vec{e.Axis}, []r3.Vec{v, e.Center}, e.Radius))
		sink.LoOnly(d.Lo)
		return
	}
	for _, c := range crits {
		th := clearance.AngleOf(e, c.Fb.Sub(e.Center))
		sink.Candidate(k.tol, clearance.CircleAngleAdmit(e, th, k.tol), c.Lo, c.Hi, c.Exact, c.Fb, v)
	}
}

// Extent reads a body's directional extent after the carrier gates pass.
type Extent func(r3.Vec) (float64, float64, bool)

// PlaneCylinderRuling certifies the cylinder's complete tangent ruling on the
// plane face. The cylinder axis must parallel the plane at exactly its radius
// on the outward side, the tangent azimuth and the whole axial window must be
// on the cylinder face, and the whole ruling must lie inside the plane trim.
// The plane must separate the bodies' complete extents. cylinderFirst names
// the cylinder's body as A.
func (k *Kernel) PlaneCylinderRuling(plane, cyl *clearance.CFace,
	planeExtent, cylExtent Extent, cylinderFirst bool) *clearance.RulingContact {
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
	base := proofarith.DvSub(anchor, proofarith.DvScale(n, radius))
	ends := [2]proofarith.DyV3{
		proofarith.DvAdd(base, proofarith.DvScale(axis, lo)),
		proofarith.DvAdd(base, proofarith.DvScale(axis, hi)),
	}
	if !k.rulingInsidePlaneTrim(plane, ends) {
		return nil
	}
	_, planeHi, okPlane := planeExtent(plane.N)
	cylLo, _, okCyl := cylExtent(plane.N)
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

// CylinderPairRuling certifies the common ruling of two parallel external
// cylinders. Their axes must be exactly parallel, their center offset must be
// the radius sum along one signed coordinate axis, both tangent azimuths must
// be on their faces, and the axial windows must overlap with positive
// length. The tangent plane between them must separate the complete extents.
func (k *Kernel) CylinderPairRuling(ca, cb *clearance.CFace, aExtent, bExtent Extent) *clearance.RulingContact {
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
	perp := proofarith.DvSub(delta, proofarith.DvScale(axisA, along))
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
	_, aHi, okA := aExtent(normalVec)
	bLoExtent, _, okB := bExtent(normalVec)
	if !okA || !okB {
		return nil
	}
	aHiDy, okAHi := proofarith.DyOf(aHi)
	bLoDy, okBLo := proofarith.DyOf(bLoExtent)
	if !okAHi || !okBLo || proofarith.DyCmp(aHiDy, offset) > 0 || proofarith.DyCmp(bLoDy, offset) < 0 {
		return nil
	}
	base := proofarith.DvAdd(anchorA, proofarith.DvScale(normal, rA))
	ends := [2]proofarith.DyV3{
		proofarith.DvAdd(base, proofarith.DvScale(axisA, lo)),
		proofarith.DvAdd(base, proofarith.DvScale(axisA, hi)),
	}
	return &clearance.RulingContact{FaceA: ca, FaceB: cb, Normal: normal, Offset: offset, Ends: clearance.OrderedRulingEnds(ends)}
}

// tangentAzimuthAdmitted reports whether the cylinder face's angular trim
// holds the ruling whose outward radial direction is dir, with margin.
func (k *Kernel) tangentAzimuthAdmitted(f *clearance.CFace, dir r3.Vec) bool {
	phi := math.Atan2(dir.Dot(f.RefV), dir.Dot(f.RefU))
	return f.Sweep.Classify(phi, k.tol/math.Max(f.Radius, 1e-30)) == 1
}

// rulingInsidePlaneTrim admits the whole segment into the plane face's trim:
// one end strictly inside with margin and the segment clear of every trim
// boundary element by more than the margin, so it never leaves the region.
func (k *Kernel) rulingInsidePlaneTrim(plane *clearance.CFace, ends [2]proofarith.DyV3) bool {
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

func dyMax(a, b proofarith.Dyadic) proofarith.Dyadic {
	if proofarith.DyCmp(a, b) >= 0 {
		return a
	}
	return b
}

func dyMin(a, b proofarith.Dyadic) proofarith.Dyadic {
	if proofarith.DyCmp(a, b) <= 0 {
		return a
	}
	return b
}
