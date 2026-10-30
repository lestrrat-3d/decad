package sectionclip

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// LoftCircleSectionRecord reads a parallel section directly from an
// authenticated circle-pair Loft. Its identical outer records denote the
// same region boundary at every level. Equal-angle correspondence makes the
// moving hole a circle with linearly interpolated center and radius.
func LoftCircleSectionRecord(pl loftPayload, plane r3.Frame) (profileRecord, bool) {
	// An exact-axis Cut is refused by analyticBore. The direct-circle gate
	// refuses non-analytic Cut and Fillet rewrites; their markers remain
	// reject-only backstops if those gates change.
	if pl.ConstructionDelta != 0 || len(pl.BlendSegs) != 0 ||
		pl.ThroughBoreCut || pl.AnalyticBore || pl.Xform != r3.Identity() ||
		loftrecord.DirectCircularHole(pl) == loftrecord.WallNone {
		return profileRecord{}, false
	}
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	if pl.Frame0.U() != x || pl.Frame0.V() != y || pl.Frame0.N() != z ||
		pl.Frame1.U() != x || pl.Frame1.V() != y || pl.Frame1.N() != z ||
		plane.U() != x || plane.V() != y || plane.N() != z {
		return profileRecord{}, false
	}
	if pl.Frame0.Origin().X != pl.Frame1.Origin().X ||
		pl.Frame0.Origin().Y != pl.Frame1.Origin().Y ||
		pl.Frame0.Origin().X != plane.Origin().X ||
		pl.Frame0.Origin().Y != plane.Origin().Y ||
		pl.Plane0.Origin.Z != pl.Frame0.Origin().Z ||
		pl.Plane1.Origin.Z != pl.Frame1.Origin().Z {
		return profileRecord{}, false
	}
	z0, z1 := proofarith.FloatRat(pl.Frame0.Origin().Z), proofarith.FloatRat(pl.Frame1.Origin().Z)
	cut := proofarith.FloatRat(plane.Origin().Z)
	if z0 == nil || z1 == nil || cut == nil || cut.Cmp(z0) <= 0 || cut.Cmp(z1) >= 0 {
		return profileRecord{}, false
	}
	fraction := new(big.Rat).Quo(new(big.Rat).Sub(cut, z0), new(big.Rat).Sub(z1, z0))
	bottom, okBottom := pl.Profile0.Holes[0].Segments[0].(circleSeg)
	top, okTop := pl.Profile1.Holes[0].Segments[0].(circleSeg)
	if !okBottom || !okTop {
		return profileRecord{}, false
	}
	u, okU := loftSectionLerpExact(bottom.Center.U, top.Center.U, fraction)
	v, okV := loftSectionLerpExact(bottom.Center.V, top.Center.V, fraction)
	radius, okRadius := loftSectionLerpExact(bottom.Radius.Base(), top.Radius.Base(), fraction)
	if !okU || !okV || !okRadius || radius <= 0 {
		return profileRecord{}, false
	}
	hole := bottom
	hole.Center.U, hole.Center.V = u, v
	hole.Radius = units.Millimeters(radius)
	return profileRecord{
		Outer: pl.Profile0.Outer,
		Holes: []loopRecord{{Segments: []curveSegment{hole}}},
	}, true
}

// LoftVaryingCircularOuterSectionRecord intersects equal-angle circle pairs
// from two authenticated source profiles. At a fixed height their ruled
// boundary is a circle with linearly interpolated center and radius.
func LoftVaryingCircularOuterSectionRecord(pl loftPayload, plane r3.Frame) (profileRecord, bool) {
	// Cut/Fillet markers, hole-count disagreement, invalid alignment shapes
	// and plane/frame disagreement are reject-only for current direct Lofts.
	if !pl.AuthenticatedReconstruction || pl.SurfaceResult ||
		pl.ConstructionDelta != 0 || len(pl.BlendSegs) != 0 ||
		pl.ThroughBoreCut || pl.AnalyticBore || pl.Xform != r3.Identity() ||
		len(pl.Profile0.Holes) != len(pl.Profile1.Holes) ||
		(len(pl.Alignment) != 0 && len(pl.Alignment) != 1+len(pl.Profile0.Holes)) {
		return profileRecord{}, false
	}
	for _, offset := range pl.Alignment {
		if offset != 0 {
			return profileRecord{}, false
		}
	}
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	if pl.Frame0.U() != x || pl.Frame0.V() != y || pl.Frame0.N() != z ||
		pl.Frame1.U() != x || pl.Frame1.V() != y || pl.Frame1.N() != z ||
		plane.U() != x || plane.V() != y || plane.N() != z ||
		pl.Plane0.U != x || pl.Plane0.V != y ||
		pl.Plane1.U != x || pl.Plane1.V != y ||
		pl.Plane0.Origin != pl.Frame0.Origin() || pl.Plane1.Origin != pl.Frame1.Origin() ||
		pl.Frame0.Origin().X != pl.Frame1.Origin().X ||
		pl.Frame0.Origin().Y != pl.Frame1.Origin().Y ||
		pl.Frame0.Origin().X != plane.Origin().X ||
		pl.Frame0.Origin().Y != plane.Origin().Y {
		return profileRecord{}, false
	}
	z0, z1 := proofarith.FloatRat(pl.Frame0.Origin().Z), proofarith.FloatRat(pl.Frame1.Origin().Z)
	cut := proofarith.FloatRat(plane.Origin().Z)
	if z0 == nil || z1 == nil || cut == nil || cut.Cmp(z0) <= 0 || cut.Cmp(z1) >= 0 {
		return profileRecord{}, false
	}
	fraction := new(big.Rat).Quo(new(big.Rat).Sub(cut, z0), new(big.Rat).Sub(z1, z0))
	outer, outerPair, ok := loftSectionCirclePair(pl.Profile0.Outer, pl.Profile1.Outer, true, fraction)
	if !ok {
		return profileRecord{}, false
	}
	profile := profileRecord{Outer: outer, Holes: make([]loopRecord, len(pl.Profile0.Holes))}
	holePairs := make([][2]circleSeg, len(profile.Holes))
	for i := range profile.Holes {
		hole, pair, ok := loftSectionCirclePair(pl.Profile0.Holes[i], pl.Profile1.Holes[i], false, fraction)
		if !ok || !LoftSectionCircleGapAllLevels(outerPair, pair, true) {
			return profileRecord{}, false
		}
		for j := range i {
			if !LoftSectionCircleGapAllLevels(pair, holePairs[j], false) {
				return profileRecord{}, false
			}
		}
		profile.Holes[i] = hole
		holePairs[i] = pair
	}
	return profile, true
}

func loftSectionCirclePair(lower, upper loopRecord, ccw bool, fraction *big.Rat) (loopRecord, [2]circleSeg, bool) {
	if len(lower.Segments) != 1 || len(upper.Segments) != 1 {
		return loopRecord{}, [2]circleSeg{}, false
	}
	bottom, okBottom := lower.Segments[0].(circleSeg)
	top, okTop := upper.Segments[0].(circleSeg)
	start, end := 0.0, 1.0
	if !ccw {
		start, end = 1, 0
	}
	if !okBottom || !okTop || bottom.CCW != ccw || top.CCW != ccw ||
		bottom.TStart != start || bottom.TEnd != end ||
		top.TStart != start || top.TEnd != end ||
		!loftSectionFiniteCircle(bottom) || !loftSectionFiniteCircle(top) {
		return loopRecord{}, [2]circleSeg{}, false
	}
	u, okU := loftSectionLerpExact(bottom.Center.U, top.Center.U, fraction)
	v, okV := loftSectionLerpExact(bottom.Center.V, top.Center.V, fraction)
	radius, okRadius := loftSectionLerpExact(bottom.Radius.Base(), top.Radius.Base(), fraction)
	if !okU || !okV || !okRadius || radius <= 0 {
		return loopRecord{}, [2]circleSeg{}, false
	}
	circle := bottom
	circle.Center.U, circle.Center.V = u, v
	circle.Radius = units.Millimeters(radius)
	return loopRecord{Segments: []curveSegment{circle}}, [2]circleSeg{bottom, top}, true
}

// LoftSectionCircleGapAllLevels proves two paired circles remain nested or
// separated for the whole Loft. Both center differences and the radius gap
// or sum are linear in the sweep fraction, so their squared gap is quadratic.
func LoftSectionCircleGapAllLevels(a, b [2]circleSeg, contained bool) bool {
	x := [2]*big.Rat{}
	y := [2]*big.Rat{}
	r := [2]*big.Rat{}
	for i := range a {
		x[i] = new(big.Rat).Sub(proofarith.FloatRat(a[i].Center.U), proofarith.FloatRat(b[i].Center.U))
		y[i] = new(big.Rat).Sub(proofarith.FloatRat(a[i].Center.V), proofarith.FloatRat(b[i].Center.V))
		r[i] = proofarith.FloatRat(a[i].Radius.Base())
		if contained {
			r[i].Sub(r[i], proofarith.FloatRat(b[i].Radius.Base()))
		} else {
			r[i].Add(r[i], proofarith.FloatRat(b[i].Radius.Base()))
		}
		if r[i].Sign() <= 0 {
			return false
		}
	}
	dx := new(big.Rat).Sub(x[1], x[0])
	dy := new(big.Rat).Sub(y[1], y[0])
	dr := new(big.Rat).Sub(r[1], r[0])
	sq := func(v *big.Rat) *big.Rat { return new(big.Rat).Mul(v, v) }
	quadratic := new(big.Rat).Sub(new(big.Rat).Add(sq(dx), sq(dy)), sq(dr))
	linear := new(big.Rat).Sub(new(big.Rat).Add(
		new(big.Rat).Mul(x[0], dx), new(big.Rat).Mul(y[0], dy)), new(big.Rat).Mul(r[0], dr))
	linear.Mul(linear, big.NewRat(2, 1))
	constant := new(big.Rat).Sub(new(big.Rat).Add(sq(x[0]), sq(y[0])), sq(r[0]))
	if contained {
		quadratic.Neg(quadratic)
		linear.Neg(linear)
		constant.Neg(constant)
	}
	return loftSectionQuadraticPositive(quadratic, linear, constant)
}

// loftSectionQuadraticPositive requires a·t²+b·t+c > 0 on closed [0,1].
func loftSectionQuadraticPositive(a, b, c *big.Rat) bool {
	atOne := new(big.Rat).Add(new(big.Rat).Add(a, b), c)
	if c.Sign() <= 0 || atOne.Sign() <= 0 {
		return false
	}
	if a.Sign() <= 0 || b.Sign() >= 0 || b.Cmp(new(big.Rat).Neg(new(big.Rat).Mul(a, big.NewRat(2, 1)))) <= 0 {
		return true
	}
	// The vertex lies inside (0,1), and 4a times its value is 4ac-b².
	minimum := new(big.Rat).Sub(new(big.Rat).Mul(big.NewRat(4, 1), new(big.Rat).Mul(a, c)),
		new(big.Rat).Mul(b, b))
	return minimum.Sign() > 0
}

func loftSectionFiniteCircle(circle circleSeg) bool {
	return !math.IsNaN(circle.Center.U) && !math.IsInf(circle.Center.U, 0) &&
		!math.IsNaN(circle.Center.V) && !math.IsInf(circle.Center.V, 0) &&
		!math.IsNaN(circle.Radius.Base()) && !math.IsInf(circle.Radius.Base(), 0) &&
		circle.Radius.Base() > 0
}

// loftSectionLerpExact returns an interpolation only when binary64 holds its
// exact rational value.
func loftSectionLerpExact(a, b float64, fraction *big.Rat) (float64, bool) {
	value := new(big.Rat).Add(proofarith.FloatRat(a),
		new(big.Rat).Mul(fraction, new(big.Rat).Sub(proofarith.FloatRat(b), proofarith.FloatRat(a))))
	held, exact := value.Float64()
	return held, exact && !math.IsInf(held, 0)
}

// LoftMatchingSectionRecord reads an interior parallel section from a direct
// solid Loft whose two authenticated profile records are identical. Zero
// alignment pairs every outer and hole boundary point with itself, so every
// ruled wall is vertical and every intermediate material region matches.
func LoftMatchingSectionRecord(pl loftPayload, plane r3.Frame) (profileRecord, bool) {
	if !pl.AuthenticatedReconstruction || pl.SurfaceResult ||
		pl.ConstructionDelta != 0 || len(pl.BlendSegs) != 0 ||
		pl.ThroughBoreCut || pl.AnalyticBore || pl.Xform != r3.Identity() ||
		!sectionrecord.IdenticalRecord(pl.Profile0, pl.Profile1) {
		return profileRecord{}, false
	}
	for _, offset := range pl.Alignment {
		if offset != 0 {
			return profileRecord{}, false
		}
	}
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	if pl.Frame0.U() != x || pl.Frame0.V() != y || pl.Frame0.N() != z ||
		pl.Frame1.U() != x || pl.Frame1.V() != y || pl.Frame1.N() != z ||
		plane.U() != x || plane.V() != y || plane.N() != z {
		return profileRecord{}, false
	}
	if pl.Frame0.Origin().X != pl.Frame1.Origin().X ||
		pl.Frame0.Origin().Y != pl.Frame1.Origin().Y ||
		pl.Frame0.Origin().X != plane.Origin().X ||
		pl.Frame0.Origin().Y != plane.Origin().Y ||
		pl.Plane0.Origin.Z != pl.Frame0.Origin().Z ||
		pl.Plane1.Origin.Z != pl.Frame1.Origin().Z {
		return profileRecord{}, false
	}
	z0, z1 := proofarith.FloatRat(pl.Frame0.Origin().Z), proofarith.FloatRat(pl.Frame1.Origin().Z)
	cut := proofarith.FloatRat(plane.Origin().Z)
	if z0 == nil || z1 == nil || cut == nil || cut.Cmp(z0) <= 0 || cut.Cmp(z1) >= 0 {
		return profileRecord{}, false
	}
	return pl.Profile0, true
}

// LoftThroughBoreSectionRecord reads a D-shaped through-bore loft from its
// authenticated, matching section records. The supporting wall is vertical, so
// every interior world-XY section has the recorded profile.
func LoftThroughBoreSectionRecord(pl loftPayload, plane r3.Frame) (profileRecord, bool) {
	if !pl.AuthenticatedReconstruction || pl.SurfaceResult || !pl.ThroughBoreCut || pl.AnalyticBore ||
		pl.ConstructionDelta != 0 || len(pl.BlendSegs) != 0 || pl.Xform != r3.Identity() ||
		!sectionrecord.IdenticalRecord(pl.Profile0, pl.Profile1) || len(pl.Profile0.Holes) != 1 ||
		!isDLoop(pl.Profile0.Holes[0]) {
		return profileRecord{}, false
	}
	for _, offset := range pl.Alignment {
		if offset != 0 {
			return profileRecord{}, false
		}
	}
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	if pl.Frame0.U() != x || pl.Frame0.V() != y || pl.Frame0.N() != z ||
		pl.Frame1.U() != x || pl.Frame1.V() != y || pl.Frame1.N() != z ||
		plane.U() != x || plane.V() != y || plane.N() != z {
		return profileRecord{}, false
	}
	if pl.Frame0.Origin().X != pl.Frame1.Origin().X ||
		pl.Frame0.Origin().Y != pl.Frame1.Origin().Y ||
		pl.Frame0.Origin().X != plane.Origin().X ||
		pl.Frame0.Origin().Y != plane.Origin().Y ||
		pl.Plane0.Origin.Z != pl.Frame0.Origin().Z ||
		pl.Plane1.Origin.Z != pl.Frame1.Origin().Z {
		return profileRecord{}, false
	}
	z0, z1 := proofarith.FloatRat(pl.Frame0.Origin().Z), proofarith.FloatRat(pl.Frame1.Origin().Z)
	cut := proofarith.FloatRat(plane.Origin().Z)
	if z0 == nil || z1 == nil || cut == nil || cut.Cmp(z0) <= 0 || cut.Cmp(z1) >= 0 {
		return profileRecord{}, false
	}
	return pl.Profile0, true
}

func isDLoop(loop sectionrecord.LoopRecord) bool {
	if len(loop.Segments) != 2 {
		return false
	}
	line, circle := 0, 0
	for _, segment := range loop.Segments {
		switch segment.(type) {
		case sectionrecord.LineSeg:
			line++
		case sectionrecord.CircleSeg:
			circle++
		case sectionrecord.ArcSeg:
			circle++
		default:
			return false
		}
	}
	return line == 1 && circle == 1
}

// LoftRuledLineSectionRecord intersects the exact triangular walls of a
// direct line-only Loft. Each line pair contributes two triangles with one
// diagonal, so its interior section can contain a kink at that diagonal.
// Zero vertex displacement ties the held triangle corners to the records.
// Placement, source-origin, cap-level, and rewrite-marker guards are reject-only
// for current Loft producers; keep them if those producers gain new routes.
func LoftRuledLineSectionRecord(pl loftPayload, plane r3.Frame) (profileRecord, bool) {
	if !pl.AuthenticatedReconstruction || pl.SurfaceResult || pl.Xform != r3.Identity() ||
		pl.Delta != 0 || pl.SectionDelta != 0 || pl.Chorded ||
		pl.ConstructionDelta != 0 || len(pl.BlendSegs) != 0 ||
		pl.ThroughBoreCut || pl.AnalyticBore ||
		len(pl.Profile0.Holes) != len(pl.Profile1.Holes) ||
		(len(pl.Alignment) != 0 && len(pl.Alignment) != 1+len(pl.Profile0.Holes)) {
		return profileRecord{}, false
	}
	for _, offset := range pl.Alignment {
		if offset != 0 {
			return profileRecord{}, false
		}
	}
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	if pl.Frame0.U() != x || pl.Frame0.V() != y || pl.Frame0.N() != z ||
		pl.Frame1.U() != x || pl.Frame1.V() != y || pl.Frame1.N() != z ||
		plane.U() != x || plane.V() != y || plane.N() != z ||
		pl.Plane0.U != x || pl.Plane0.V != y ||
		pl.Plane1.U != x || pl.Plane1.V != y ||
		pl.Plane0.Origin != pl.Frame0.Origin() || pl.Plane1.Origin != pl.Frame1.Origin() ||
		pl.Frame0.Origin().X != pl.Frame1.Origin().X ||
		pl.Frame0.Origin().Y != pl.Frame1.Origin().Y ||
		pl.Frame0.Origin().X != plane.Origin().X ||
		pl.Frame0.Origin().Y != plane.Origin().Y {
		return profileRecord{}, false
	}
	z0, z1 := proofarith.FloatRat(pl.Frame0.Origin().Z), proofarith.FloatRat(pl.Frame1.Origin().Z)
	cut := proofarith.FloatRat(plane.Origin().Z)
	if z0 == nil || z1 == nil || cut == nil || cut.Cmp(z0) <= 0 || cut.Cmp(z1) >= 0 {
		return profileRecord{}, false
	}
	fraction := new(big.Rat).Quo(new(big.Rat).Sub(cut, z0), new(big.Rat).Sub(z1, z0))
	outer, ok := loftRuledLineSectionLoop(pl.Profile0.Outer, pl.Profile1.Outer, fraction)
	if !ok {
		return profileRecord{}, false
	}
	holes := make([]loopRecord, len(pl.Profile0.Holes))
	for i := range holes {
		holes[i], ok = loftRuledLineSectionLoop(pl.Profile0.Holes[i], pl.Profile1.Holes[i], fraction)
		if !ok {
			return profileRecord{}, false
		}
	}
	return profileRecord{Outer: outer, Holes: holes}, true
}

func loftRuledLineSectionLoop(lower, upper loopRecord, fraction *big.Rat) (loopRecord, bool) {
	if len(lower.Segments) == 0 || len(lower.Segments) != len(upper.Segments) {
		return loopRecord{}, false
	}
	section := loopRecord{Segments: make([]curveSegment, 0, 2*len(lower.Segments))}
	for i, lowerSegment := range lower.Segments {
		v0, v1, ok := loftSectionFullLineEnds(lowerSegment)
		if !ok {
			return loopRecord{}, false
		}
		w0, w1, ok := loftSectionFullLineEnds(upper.Segments[i])
		if !ok {
			return loopRecord{}, false
		}
		start, ok := loftRuledLineSectionPoint(v0, w0, fraction)
		if !ok {
			return loopRecord{}, false
		}
		diagonal, ok := loftRuledLineSectionPoint(v0, w1, fraction)
		if !ok {
			return loopRecord{}, false
		}
		end, ok := loftRuledLineSectionPoint(v1, w1, fraction)
		if !ok || start == diagonal || diagonal == end {
			return loopRecord{}, false
		}
		section.Segments = append(section.Segments,
			lineSeg{Start: start, End: diagonal, TStart: 0, TEnd: 1},
			lineSeg{Start: diagonal, End: end, TStart: 0, TEnd: 1})
	}
	return section, true
}

func loftSectionFullLineEnds(segment curveSegment) (sectionrecord.Point2, sectionrecord.Point2, bool) {
	line, ok := segment.(lineSeg)
	if !ok {
		return sectionrecord.Point2{}, sectionrecord.Point2{}, false
	}
	switch {
	case line.TStart == 0 && line.TEnd == 1:
		return line.Start, line.End, true
	case line.TStart == 1 && line.TEnd == 0:
		return line.End, line.Start, true
	default:
		return sectionrecord.Point2{}, sectionrecord.Point2{}, false
	}
}

func loftRuledLineSectionPoint(a, b sectionrecord.Point2, fraction *big.Rat) (sectionrecord.Point2, bool) {
	u, okU := loftSectionLerpExact(a.U, b.U, fraction)
	v, okV := loftSectionLerpExact(a.V, b.V, fraction)
	return sectionrecord.Point2{U: u, V: v}, okU && okV
}
