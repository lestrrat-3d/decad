package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type loftCircleWallKind uint8

const (
	loftCircleWallNone loftCircleWallKind = iota
	loftCircleWallCylinder
	loftCircleWallCone
	loftCircleWallRuled
	loftCircleWallQuarterTurn
)

// directLoftCircularHole reads only authenticated section records. Matching
// frame axes and zero alignment make a circle pair interpolate with center
// C(z) and radius r(z). A fixed center gives a right cylinder or cone; a
// moving center gives a rational ruled surface. Exact cardinal quarter-turn
// frames also give a rational ruled surface under the common angle parameter.
func directLoftCircularHole(pl loftPayload) loftCircleWallKind {
	if !pl.authenticatedReconstruction || pl.surfaceResult ||
		len(pl.profile0.Holes) != 1 || len(pl.profile1.Holes) != 1 ||
		len(pl.profile0.Holes[0].Segments) != 1 || len(pl.profile1.Holes[0].Segments) != 1 ||
		!sectionrecord.IdenticalRecord(pl.profile0.Outer, pl.profile1.Outer) {
		return loftCircleWallNone
	}
	for _, offset := range pl.alignment {
		if offset != 0 {
			return loftCircleWallNone
		}
	}
	bottom, ok0 := pl.profile0.Holes[0].Segments[0].(circleSeg)
	top, ok1 := pl.profile1.Holes[0].Segments[0].(circleSeg)
	if !ok0 || !ok1 || bottom.CCW || top.CCW ||
		bottom.TStart != 1 || bottom.TEnd != 0 || top.TStart != 1 || top.TEnd != 0 {
		return loftCircleWallNone
	}
	level := pl.frame0.ToLocal(pl.frame1.Origin())
	if level.X != 0 || level.Y != 0 ||
		!loftBoreHasExactAxis(pl.frame0, pl.frame1, 0, 0, level.Z) {
		return loftCircleWallNone
	}
	sameAxes := pl.frame0.U() == pl.frame1.U() && pl.frame0.V() == pl.frame1.V()
	if !sameAxes {
		// A cardinal quarter-turn gives an exact ruled circle pair and an
		// angularly constant area integrand. Both local centers must be on
		// the common axis; other frame rotations need their own area bound.
		quarterTurn := pl.frame1.U() == (r3.Vec{Y: 1}) && pl.frame1.V() == (r3.Vec{X: -1})
		quarterTurn = quarterTurn ||
			pl.frame1.U() == (r3.Vec{Y: -1}) && pl.frame1.V() == (r3.Vec{X: 1})
		if !quarterTurn || bottom.Center.U != 0 || bottom.Center.V != 0 ||
			top.Center.U != 0 || top.Center.V != 0 {
			return loftCircleWallNone
		}
	}
	// The cylindrical builder uses the first section's entire circle record
	// for both rims, so equal world radii alone do not authorize that reuse.
	if sameAxes && sectionrecord.IdenticalRecord(bottom, top) {
		if loftBoreHasExactAxis(pl.frame0, pl.frame1,
			bottom.Center.U, bottom.Center.V, level.Z) {
			return loftCircleWallCylinder
		}
		return loftCircleWallNone
	}
	if sameAxes && bottom.Center == top.Center && bottom.Radius.Base() == top.Radius.Base() {
		return loftCircleWallNone
	}
	// The varying case uses exact world-XY coordinates. This makes its
	// section-to-world control net exact and lets Cone's half-angle bound
	// read the exact rational height between the two recorded planes.
	if pl.xform != r3.Identity() || pl.frame0.U() != (r3.Vec{X: 1}) ||
		pl.frame0.V() != (r3.Vec{Y: 1}) ||
		pl.frame0.Origin().X != 0 || pl.frame0.Origin().Y != 0 ||
		pl.frame1.Origin().X != 0 || pl.frame1.Origin().Y != 0 {
		return loftCircleWallNone
	}
	height := new(big.Rat).Sub(proofarith.FloatRat(pl.plane1.Origin.Z),
		proofarith.FloatRat(pl.plane0.Origin.Z))
	if height.Cmp(proofarith.FloatRat(level.Z)) != 0 {
		return loftCircleWallNone
	}
	if !sameAxes {
		return loftCircleWallQuarterTurn
	}
	if bottom.Center == top.Center {
		return loftCircleWallCone
	}
	return loftCircleWallRuled
}

// loftConeWallArea bounds the lateral area of the exact circular frustum
// denoted by two coaxial circle records. The surface is independent of the
// outer Loft wall, so a curved outer loop does not enter this proof.
func loftConeWallArea(pl loftPayload, bottom, top circleSeg) (proofbound.BoundedScalar, bool) {
	r0, r1 := bottom.Radius.Base(), top.Radius.Base()
	h := new(big.Rat).Sub(proofarith.FloatRat(pl.plane1.Origin.Z),
		proofarith.FloatRat(pl.plane0.Origin.Z))
	dr := new(big.Rat).Sub(proofarith.FloatRat(r1), proofarith.FloatRat(r0))
	slantSquared := new(big.Rat).Add(new(big.Rat).Mul(h, h), new(big.Rat).Mul(dr, dr))
	slant, ok := proofbound.IntervalSqrt(proofbound.PointInterval(slantSquared))
	if !ok {
		return proofbound.BoundedScalar{}, false
	}
	radiusSum := new(big.Rat).Add(proofarith.FloatRat(r0), proofarith.FloatRat(r1))
	areaInterval := proofbound.IntervalMul(
		proofbound.IntervalScale(proofbound.TwoPiInterval(),
			new(big.Rat).Quo(radiusSum, big.NewRat(2, 1))), slant)
	held := math.Pi * (r0 + r1) * math.Hypot(pl.frame0.ToLocal(pl.frame1.Origin()).Z, r1-r0)
	bound := proofbound.IntervalFloatError(areaInterval, held)
	if proofbound.IsNonFinite(held) || proofbound.IsNonFinite(bound) {
		return proofbound.BoundedScalar{}, false
	}
	return proofbound.MeasuredScalar(held, bound), true
}

// loftMovingCircleWallArea bounds the wall ruled between corresponding circle
// points. Its speed is r(t)*sqrt(h²+(dr+dx*cosθ+dy*sinθ)²); integrating the
// linear radius over t leaves the mean radius times one angular integral.
// The angular speed is |(dx,dy)|-Lipschitz, so midpoint quadrature has an
// absolute error at most |(dx,dy)|*π²/cells.
func loftMovingCircleWallArea(pl loftPayload, bottom, top circleSeg) (proofbound.BoundedScalar, bool) {
	r0, r1 := proofarith.FloatRat(bottom.Radius.Base()), proofarith.FloatRat(top.Radius.Base())
	dr := new(big.Rat).Sub(r1, r0)
	dx := new(big.Rat).Sub(proofarith.FloatRat(top.Center.U), proofarith.FloatRat(bottom.Center.U))
	dy := new(big.Rat).Sub(proofarith.FloatRat(top.Center.V), proofarith.FloatRat(bottom.Center.V))
	h := new(big.Rat).Sub(proofarith.FloatRat(pl.plane1.Origin.Z), proofarith.FloatRat(pl.plane0.Origin.Z))
	h2 := new(big.Rat).Mul(h, h)
	const cells int64 = 512
	memo := proofbound.NewTurnTrigMemo()
	sum := proofbound.PointInterval(new(big.Rat))
	for i := range cells {
		turn := big.NewRat(2*i+1, 2*cells)
		sin, cos := memo.SinCos(turn)
		slope := proofbound.IntervalAdd(proofbound.PointInterval(dr),
			proofbound.IntervalAdd(proofbound.IntervalScale(cos, dx), proofbound.IntervalScale(sin, dy)))
		speed, ok := proofbound.IntervalSqrt(proofbound.IntervalAdd(
			proofbound.PointInterval(h2), proofbound.IntervalSquare(slope)))
		if !ok {
			return proofbound.BoundedScalar{}, false
		}
		sum = proofbound.IntervalAdd(sum, speed)
	}
	d2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
	lipschitz := proofarith.FloatRat(proofbound.RatSqrtUp(d2))
	if lipschitz == nil {
		return proofbound.BoundedScalar{}, false
	}
	pi2 := new(big.Rat).Mul(proofbound.PiUpper, proofbound.PiUpper)
	quadratureError := new(big.Rat).Quo(new(big.Rat).Mul(lipschitz, pi2), big.NewRat(cells, 1))
	integral := proofbound.IntervalWiden(proofbound.IntervalMul(
		proofbound.IntervalScale(proofbound.TwoPiInterval(), big.NewRat(1, cells)), sum), quadratureError)
	meanRadius := new(big.Rat).Quo(new(big.Rat).Add(r0, r1), big.NewRat(2, 1))
	areaInterval := proofbound.IntervalScale(integral, meanRadius)
	midpoint := new(big.Rat).Quo(new(big.Rat).Add(areaInterval.Lo, areaInterval.Hi), big.NewRat(2, 1))
	held, _ := midpoint.Float64()
	bound := proofbound.IntervalFloatError(areaInterval, held)
	if proofbound.IsNonFinite(held) || proofbound.IsNonFinite(bound) {
		return proofbound.BoundedScalar{}, false
	}
	return proofbound.MeasuredScalar(held, bound), true
}

// loftQuarterTurnWallArea encloses the lateral area of a pair of concentric
// circles whose section frames differ by exactly a quarter-turn. Writing the
// planar ruled point as q(t) exp(iθ), q(t)=(1-t)r0+i t r1, removes θ from the
// area integrand. Its square is a positive quadratic in t. Each interval's
// endpoint maximum and endpoint-or-vertex minimum bound the integrand there;
// outward square roots and π then bound the full integral.
func loftQuarterTurnWallArea(pl loftPayload, bottom, top circleSeg) (proofbound.BoundedScalar, bool) {
	r0, r1 := proofarith.FloatRat(bottom.Radius.Base()), proofarith.FloatRat(top.Radius.Base())
	h := new(big.Rat).Sub(proofarith.FloatRat(pl.plane1.Origin.Z),
		proofarith.FloatRat(pl.plane0.Origin.Z))
	r0sq, r1sq := new(big.Rat).Mul(r0, r0), new(big.Rat).Mul(r1, r1)
	sum := new(big.Rat).Add(r0sq, r1sq)
	minimumAt := new(big.Rat).Quo(r0sq, sum)
	squaredSpeed := func(t *big.Rat) *big.Rat {
		oneMinus := new(big.Rat).Sub(big.NewRat(1, 1), t)
		q0 := new(big.Rat).Mul(oneMinus, r0)
		q1 := new(big.Rat).Mul(t, r1)
		planar := new(big.Rat).Add(new(big.Rat).Mul(q0, q0), new(big.Rat).Mul(q1, q1))
		radial := new(big.Rat).Sub(new(big.Rat).Mul(oneMinus, r0sq), new(big.Rat).Mul(t, r1sq))
		return new(big.Rat).Add(new(big.Rat).Mul(new(big.Rat).Mul(h, h), planar),
			new(big.Rat).Mul(radial, radial))
	}
	const cells int64 = 512
	loSum, hiSum := new(big.Rat), new(big.Rat)
	for i := range cells {
		a, b := big.NewRat(i, cells), big.NewRat(i+1, cells)
		qa, qb := squaredSpeed(a), squaredSpeed(b)
		lo, hi := qa, qb
		if qa.Cmp(qb) > 0 {
			lo, hi = qb, qa
		}
		if a.Cmp(minimumAt) <= 0 && minimumAt.Cmp(b) <= 0 {
			lo = squaredSpeed(minimumAt)
		}
		rootLo, rootHi := proofbound.RatSqrtDown(lo), proofbound.RatSqrtUp(hi)
		if proofbound.IsNonFinite(rootLo) || proofbound.IsNonFinite(rootHi) {
			return proofbound.BoundedScalar{}, false
		}
		loSum.Add(loSum, proofarith.FloatRat(rootLo))
		hiSum.Add(hiSum, proofarith.FloatRat(rootHi))
	}
	areaInterval := proofbound.IntervalMul(proofbound.TwoPiInterval(),
		proofbound.IntervalOwned(new(big.Rat).Quo(loSum, big.NewRat(cells, 1)),
			new(big.Rat).Quo(hiSum, big.NewRat(cells, 1))))
	midpoint := new(big.Rat).Quo(new(big.Rat).Add(areaInterval.Lo, areaInterval.Hi), big.NewRat(2, 1))
	held, _ := midpoint.Float64()
	bound := proofbound.IntervalFloatError(areaInterval, held)
	if proofbound.IsNonFinite(held) || proofbound.IsNonFinite(bound) {
		return proofbound.BoundedScalar{}, false
	}
	return proofbound.MeasuredScalar(held, bound), true
}

// loftCircularRim uses a section's own whole-circle record and the Loft
// assembly's first circle station. The station carries a.Delta to the exact
// seam; CircleCurveBound separately reaches the entire recorded circle from
// the published Circle3 under the frame and placement.
func loftCircularRim(pl loftPayload, a loftmesh.Assembly, side int) (*Edge, bool, error) {
	profile, frame, indices := pl.profile0, pl.frame0, a.VIdx[1]
	if side == 1 {
		profile, frame, indices = pl.profile1, pl.frame1, a.WIdx[1]
	}
	if len(indices) == 0 {
		return nil, false, nil
	}
	circle, ok := profile.Holes[0].Segments[0].(circleSeg)
	if !ok {
		return nil, false, nil
	}
	walk, err := boundarywalk.WalkOf(circle, nil)
	if err != nil {
		return nil, false, err
	}
	pp := prismPayload{Payload: prismrecord.Payload{
		Frame: frame, Z0: 0, Z1: 1, Xform: pl.xform,
		Z0Delta: a.Delta, Z1Delta: a.Delta,
	}}
	center := pp.Point(circle.Center.U, circle.Center.V, 0)
	axis := pp.Dir(0, 0, 1)
	curveBound, bounded := pp.CircleCurveBound(circle.Center.U, circle.Center.V, 0,
		a.Delta, 0, circle.Radius.Base(), 0, center, axis)
	if !bounded {
		return nil, false, nil
	}
	// A clockwise hole walks opposite the circle's geometric axis; a
	// reflection reverses that walk again.
	edgeAxis := axis.Scale(-1)
	if pp.Reflected() {
		edgeAxis = axis
	}
	vertex := loftVertex(a.Verts[indices[0]], a.Delta)
	return bornEdge(&Edge{n: topo.Edge{
		Convex: false, Length: walk.Length, LengthBound: walk.LengthBound,
		Curve:      surfacegeom.Circle3{Center: center, Axis: edgeAxis, Radius: circle.Radius},
		CurveBound: curveBound, CurveBounded: true,
		Start: vertexNode(vertex), End: vertexNode(vertex),
	}}), true, nil
}

// A whole circle has the rational quadratic quarter-arc parametrization
// ((1-t²)/(1+t²), 2t/(1+t²)). In Bernstein form its homogeneous weights
// are [1,1,2], with controls [(1,0),(1,1),(0,1)]; rotations give the other
// three quarters. Both authenticated circles use this same rational basis.
// Linear interpolation of their world-space weighted controls gives the
// exact ruled circle-to-circle surface, including a moving center or an
// exact cardinal quarter-turn of the second section. The source records
// therefore determine an exact private degree-(2,1) control net; its tag is
// NURBSSurface when that wall is neither a right cylinder nor a right cone.
func loftCircularWallSurface(pl loftPayload, bottom, top circleSeg,
	kind loftCircleWallKind) (surfacegeom.Surface, float64) {
	if kind == loftCircleWallRuled || kind == loftCircleWallQuarterTurn {
		return surfacegeom.NURBSSurface{}, 0
	}
	r0, r1 := bottom.Radius.Base(), top.Radius.Base()
	pp0 := prismPayload{Payload: prismrecord.Payload{Frame: pl.frame0, Xform: pl.xform}}
	pp1 := prismPayload{Payload: prismrecord.Payload{Frame: pl.frame1, Xform: pl.xform}}
	if kind == loftCircleWallCylinder {
		return surfacegeom.Cylinder{Origin: pp0.Point(bottom.Center.U, bottom.Center.V, 0),
			Axis: pp0.Dir(0, 0, 1), Radius: bottom.Radius}, 0
	}
	level := pl.frame0.ToLocal(pl.frame1.Origin()).Z
	angle := units.Radians(math.Atan2(math.Abs(r1-r0), level))
	// The radius difference and height are exact rationals of the recorded
	// floats. Atan2Interval encloses their true slope angle; this term
	// charges the Cone tag's rounded HalfAngle to every NormalAt reading.
	r0Rat, r1Rat := proofarith.FloatRat(r0), proofarith.FloatRat(r1)
	dr := new(big.Rat).Abs(new(big.Rat).Sub(r1Rat, r0Rat))
	h := new(big.Rat).Sub(proofarith.FloatRat(pl.plane1.Origin.Z),
		proofarith.FloatRat(pl.plane0.Origin.Z))
	angleIv := proofbound.Atan2Interval(dr, h, false)
	heldAngle := proofarith.FloatRat(angle.Base())
	angleBound := math.Max(proofbound.RatDistanceUp(heldAngle, angleIv.Lo),
		proofbound.RatDistanceUp(heldAngle, angleIv.Hi))
	if r1 > r0 {
		return surfacegeom.Cone{Origin: pp0.Point(bottom.Center.U, bottom.Center.V, 0),
			Axis: pp0.Dir(0, 0, 1), Radius: bottom.Radius, HalfAngle: angle}, angleBound
	}
	return surfacegeom.Cone{Origin: pp1.Point(top.Center.U, top.Center.V, 0),
		Axis: pp1.Dir(0, 0, -1), Radius: top.Radius, HalfAngle: angle}, angleBound
}

// buildLoftVaryingCircularHoleTopology uses the source circle pair to replace
// only the hole wall and its two rims. A circle whose placement cannot bound
// its analytic curve keeps the original faceted topology instead.
func buildLoftVaryingCircularHoleTopology(ctx context.Context, body *Body, pl loftPayload,
	kind loftCircleWallKind, a loftmesh.Assembly, start, end *Face,
	walls []*Face) ([]*Face, *Face, error) {
	if len(a.VIdx) != 2 || len(start.loops) != 2 || len(end.loops) != 2 || len(walls) != a.Walls {
		return walls, nil, nil
	}
	bottomSeg, ok0 := pl.profile0.Holes[0].Segments[0].(circleSeg)
	topSeg, ok1 := pl.profile1.Holes[0].Segments[0].(circleSeg)
	if !ok0 || !ok1 {
		return walls, nil, nil
	}
	bottom, ok, err := loftCircularRim(pl, a, 0)
	if err != nil || !ok {
		return walls, nil, err
	}
	top, ok, err := loftCircularRim(pl, a, 1)
	if err != nil || !ok {
		return walls, nil, err
	}
	kept := make([]*Face, 0, len(walls))
	var origins []provenance.Ref
	for k, face := range walls {
		if err := ctx.Err(); err != nil {
			return nil, nil, err
		}
		if a.Cell[k][0] == 1 {
			origins = append(origins, face.n.Origins...)
			continue
		}
		kept = append(kept, face)
	}
	if len(origins) < 2 {
		return walls, nil, nil
	}
	surface, normalBound := loftCircularWallSurface(pl, bottomSeg, topSeg, kind)
	wall := bornFace(&Face{n: topo.Face{
		Surface: surface, Reversed: kind != loftCircleWallRuled && kind != loftCircleWallQuarterTurn,
		NormalBound: normalBound, Origins: origins, Body: bodyNode(body),
	}})
	wall.loops = []*Loop{
		{n: topo.Loop{Outer: true, CoEdges: loftLoopCoedges([]topo.CoEdge{{Edge: edgeNode(bottom), Forward: true}}, a.Reversed)}},
		{n: topo.Loop{Outer: true, CoEdges: loftLoopCoedges([]topo.CoEdge{{Edge: edgeNode(top), Forward: false}}, a.Reversed)}},
	}
	start.loops[1] = &Loop{n: topo.Loop{Outer: start.loops[1].n.Outer, CoEdges: loftLoopCoedges([]topo.CoEdge{{Edge: edgeNode(bottom), Forward: false}}, a.Reversed)}}
	end.loops[1] = &Loop{n: topo.Loop{Outer: end.loops[1].n.Outer, CoEdges: loftLoopCoedges([]topo.CoEdge{{Edge: edgeNode(top), Forward: true}}, a.Reversed)}}
	if err := loftCircleCapAreas(ctx, pl, a, start, end); err != nil {
		return nil, nil, err
	}
	return append(kept, wall), wall, nil
}

// loftBoreHasExactAxis compares the recorded frames' denoted lifts over
// rationals. The proposed axial sweep uses frame0's axis and the rounded
// level, so both section origins and the selected centers must agree exactly.
func loftBoreHasExactAxis(frame0, frame1 r3.Frame, u, v, level float64) bool {
	z := proofarith.FloatRat(level)
	if z == nil || z.Sign() <= 0 {
		return false
	}
	axis0, err := massmoment.PlaneMap(frame0, r3.Identity())
	if err != nil {
		return false
	}
	axis1, err := massmoment.PlaneMap(frame1, r3.Identity())
	if err != nil {
		return false
	}
	for i := range axis0 {
		if axis0[i][2].Cmp(axis1[i][2]) != 0 {
			return false
		}
	}
	for _, center := range [][2]float64{{0, 0}, {u, v}} {
		bottom, err := massmoment.RevolveAnchor(frame0, center[0], center[1])
		if err != nil {
			return false
		}
		top, err := massmoment.RevolveAnchor(frame1, center[0], center[1])
		if err != nil {
			return false
		}
		for i := range bottom {
			lift := new(big.Rat).Add(bottom[i], new(big.Rat).Mul(z, axis0[i][2]))
			if lift.Cmp(top[i]) != 0 {
				return false
			}
		}
	}
	return true
}

func loftBoreTopologyCircle(pl loftPayload, a loftmesh.Assembly, start, end *Face, walls []*Face) (circleSeg, bool) {
	if len(pl.profile0.Holes) != 1 || len(pl.profile0.Holes[0].Segments) != 1 ||
		len(a.VIdx) != 2 || len(start.loops) != 2 || len(end.loops) != 2 || len(walls) != a.Walls {
		return circleSeg{}, false
	}
	circle, ok := pl.profile0.Holes[0].Segments[0].(circleSeg)
	if !ok || circle.CCW || circle.TStart != 1 || circle.TEnd != 0 {
		return circleSeg{}, false
	}
	return circle, true
}

// buildLoftAnalyticBoreTopology replaces only the certified hole's triangle
// faces. The loft assembly and its mass and tessellation proofs stay intact:
// each former hole triangle role maps to the one cylindrical source face.
func buildLoftAnalyticBoreTopology(ctx context.Context, body *Body, ref producerID, pl loftPayload,
	a loftmesh.Assembly, start, end *Face, walls []*Face) ([]*Face, error) {
	circle, ok := loftBoreTopologyCircle(pl, a, start, end, walls)
	if !ok {
		return nil, fmt.Errorf("%w: a through-bore loft lost its certified hole", ErrUnsupported)
	}
	level := pl.frame0.ToLocal(pl.frame1.Origin())
	if level.X != 0 || level.Y != 0 ||
		!loftBoreHasExactAxis(pl.frame0, pl.frame1, circle.Center.U, circle.Center.V, level.Z) {
		return nil, fmt.Errorf("%w: a through-bore loft has no common circle axis", ErrUnsupported)
	}
	pp := prismPayload{Payload: prismrecord.Payload{
		Frame: pl.frame0, Z0: 0, Z1: level.Z, Xform: pl.xform,
		Z0Delta: a.Delta, Z1Delta: a.Delta,
	}}
	faces, bottom, top, _, err := buildLoopSidesAs(ctx, body, ref, pp, 1, true,
		pl.profile0.Holes[0], freeform.NewFreeformWork(), nil, levelToken{}, levelToken{}, false)
	if err != nil {
		return nil, err
	}
	if len(faces) != 1 || len(bottom) != 1 || len(top) != 1 {
		return nil, fmt.Errorf("%w: a through-bore circle did not build one wall", ErrUnsupported)
	}
	wall := faces[0]
	wall.n.Origins = nil
	// The prism builder states these circle loops in its local sweep order.
	// Carry the Loft assembly's whole-shell reversal onto the cylinder too;
	// the cap loops below already receive the same reversal.
	for _, loop := range wall.loops {
		loop.n.CoEdges = loftLoopCoedges(loop.n.CoEdges, a.Reversed)
	}
	// buildLoopSidesAs links its side face immediately, while evalLoft links
	// every live face after this rewrite. Clear those two private links once.
	bottom[0].Edge.Faces = nil
	top[0].Edge.Faces = nil
	filtered := make([]*Face, 0, len(walls))
	for k, face := range walls {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if a.Cell[k][0] == 1 {
			wall.n.Origins = append(wall.n.Origins, face.n.Origins...)
			continue
		}
		filtered = append(filtered, face)
	}
	if len(wall.n.Origins) < 2 {
		return nil, fmt.Errorf("%w: a through-bore has no source wall cells", ErrUnsupported)
	}
	filtered = append(filtered, wall)
	start.loops[1] = &Loop{n: topo.Loop{Outer: start.loops[1].n.Outer, CoEdges: loftLoopCoedges([]topo.CoEdge{{Edge: bottom[0].Edge, Forward: false}}, a.Reversed)}}
	end.loops[1] = &Loop{n: topo.Loop{Outer: end.loops[1].n.Outer, CoEdges: loftLoopCoedges([]topo.CoEdge{{Edge: top[0].Edge, Forward: true}}, a.Reversed)}}
	if err := loftCircleCapAreas(ctx, pl, a, start, end); err != nil {
		return nil, err
	}
	return filtered, nil
}

// loftCircleCapAreas replaces each held hole polygon's area with its own
// recorded whole-circle integral. Each cap keeps its existing triangle and
// placement bounds; the subtraction composes the circle integral's bound.
func loftCircleCapAreas(ctx context.Context, pl loftPayload, a loftmesh.Assembly, start, end *Face) error {
	for cap, face := range []*Face{start, end} {
		hole := pl.profile0.Holes[0]
		pts, loops := a.Pts0, a.LoopIdx0
		if cap == 1 {
			hole = pl.profile1.Holes[0]
			pts, loops = a.Pts1, a.LoopIdx1
		}
		holeArea, err := momentinput.LoopEnclosedArea(ctx, hole)
		if err != nil {
			return err
		}
		outer := loftmesh.CapPolygonAreaRat(pts, [][]int{loops[0]})
		value, _ := outer.Float64()
		if proofbound.IsNonFinite(value) {
			return fmt.Errorf("%w: a circular-hole cap area is not finite", ErrUnsupported)
		}
		area := proofbound.BoundedSub(
			proofbound.MeasuredScalar(value, proofarith.RationalFloatError(outer, value)), holeArea)
		face.n.Area = area.Value
		face.n.AreaBound = proofbound.AbsSumUpper(area.Bound, face.n.AreaBound)
	}
	return nil
}
