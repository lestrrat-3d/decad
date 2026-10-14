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
)

// directLoftCircularHole reads only authenticated section records. Matching
// frame axes and zero alignment make the two circle walks use the same angle
// parameter, so their linear interpolation has center C(z) and radius r(z).
// A fixed center gives a right cylinder or cone; a moving center gives a
// rational degree-(2,1) ruled surface, never a right cylinder or cone.
func directLoftCircularHole(pl loftPayload) loftCircleWallKind {
	if !pl.authenticatedReconstruction || pl.surfaceResult ||
		len(pl.profile0.Holes) != 1 || len(pl.profile1.Holes) != 1 ||
		len(pl.profile0.Holes[0].Segments) != 1 || len(pl.profile1.Holes[0].Segments) != 1 ||
		!sectionrecord.IdenticalRecord(pl.profile0.Outer, pl.profile1.Outer) ||
		pl.frame0.U() != pl.frame1.U() || pl.frame0.V() != pl.frame1.V() {
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
	if level.X != 0 || level.Y != 0 || level.Z <= 0 ||
		pl.frame0.ToWorld(r3.NewVec(0, 0, level.Z)) != pl.frame1.Origin() {
		return loftCircleWallNone
	}
	// The cylindrical builder uses the first section's entire circle record
	// for both rims, so equal world radii alone do not authorize that reuse.
	if sectionrecord.IdenticalRecord(bottom, top) {
		if pl.frame0.ToWorld(r3.NewVec(bottom.Center.U, bottom.Center.V, level.Z)) ==
			pl.frame1.ToWorldUV(top.Center.U, top.Center.V) {
			return loftCircleWallCylinder
		}
		return loftCircleWallNone
	}
	if bottom.Center == top.Center && bottom.Radius.Base() == top.Radius.Base() {
		return loftCircleWallNone
	}
	// The retained outer wall faces must each bound their true area before
	// the complete Loft area can be used to isolate this one curved wall.
	// Equal straight segments give planar outer wall cells.
	for _, segment := range pl.profile0.Outer.Segments {
		if _, ok := segment.(lineSeg); !ok {
			return loftCircleWallNone
		}
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
	if bottom.Center != top.Center {
		return loftCircleWallRuled
	}
	return loftCircleWallCone
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
	return &Edge{n: topo.Edge{
		Convex: false, Length: walk.Length, LengthBound: walk.LengthBound,
		Curve:      surfacegeom.Circle3{Center: center, Axis: edgeAxis, Radius: circle.Radius},
		CurveBound: curveBound, CurveBounded: true,
		Start: vertexNode(vertex), End: vertexNode(vertex),
	}}, true, nil
}

// A whole circle has the rational quadratic quarter-arc parametrization
// ((1-t²)/(1+t²), 2t/(1+t²)). In Bernstein form its homogeneous weights
// are [1,1,2], with controls [(1,0),(1,1),(0,1)]; rotations give the other
// three quarters. Both authenticated circles use this same rational basis.
// Linear interpolation of their weighted controls gives the exact ruled
// circle-to-circle surface, including a moving center. The source records
// therefore determine an exact private degree-(2,1) control net; its tag is
// NURBSSurface when that wall is neither a right cylinder nor a right cone.
func loftCircularWallSurface(pl loftPayload, bottom, top circleSeg,
	kind loftCircleWallKind) (surfacegeom.Surface, float64) {
	if kind == loftCircleWallRuled {
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
func buildLoftVaryingCircularHoleTopology(ctx context.Context, pl loftPayload,
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
	wall := &Face{n: topo.Face{
		Surface: surface, Reversed: kind != loftCircleWallRuled,
		NormalBound: normalBound, Origins: origins,
	}}
	wall.loops = []*Loop{
		{n: topo.Loop{Outer: true}, coedges: loftLoopCoedges([]coedge{{edge: bottom, forward: true}}, a.Reversed)},
		{n: topo.Loop{Outer: true}, coedges: loftLoopCoedges([]coedge{{edge: top, forward: false}}, a.Reversed)},
	}
	start.loops[1] = &Loop{n: start.loops[1].n,
		coedges: loftLoopCoedges([]coedge{{edge: bottom, forward: false}}, a.Reversed)}
	end.loops[1] = &Loop{n: end.loops[1].n,
		coedges: loftLoopCoedges([]coedge{{edge: top, forward: true}}, a.Reversed)}
	if err := loftCircleCapAreas(ctx, pl, a, start, end); err != nil {
		return nil, nil, err
	}
	return append(kept, wall), wall, nil
}

// loftBoreHasExactAxis compares the recorded frames' denoted lifts over
// rationals. The proposed cylinder uses frame0's axis and the rounded level,
// so both section origins and circle centers must lie exactly on that axis.
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
		loop.coedges = loftLoopCoedges(loop.coedges, a.Reversed)
	}
	// buildLoopSidesAs links its side face immediately, while evalLoft links
	// every live face after this rewrite. Clear those two private links once.
	bottom[0].edge.faces = nil
	top[0].edge.faces = nil
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
	start.loops[1] = &Loop{n: start.loops[1].n,
		coedges: loftLoopCoedges([]coedge{{edge: bottom[0].edge, forward: false}}, a.Reversed)}
	end.loops[1] = &Loop{n: end.loops[1].n,
		coedges: loftLoopCoedges([]coedge{{edge: top[0].edge, forward: true}}, a.Reversed)}
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
