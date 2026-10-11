package decad

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// bevelBlankSource is the six-line, unplaced axial section whose middle
// generator is the tooth's seating face. Its points are the recorded Sketch
// values, not values reconstructed from the rounded public Cone tag.
type bevelBlankSource struct {
	toeAxis, heelAxis Point2
	toeOuter, toeRoot Point2
	ded, heelOuter    Point2
	edgeIndex         [6]int
	rootFace          *Face
	rootCone          Cone
	// coneGap bounds the radial difference between the recorded straight
	// meridian and the public cone across the entire finite root segment.
	coneGap float64
}

type bevelToothBoundary struct {
	insideArc int
	lineIndex [2]int
	crossing  [2]sketch.ConeLineCrossing
	sides     [6]sketch.ConeCurveSide
}

// certifyBevelToothBoundary authenticates the two complete connector lines
// and four whole source curves. The Sketch certificates describe the reported
// cone; the caller must also account for blank.coneGap before accepting their
// signs as statements about the recorded finite meridian.
func certifyBevelToothBoundary(record *pointSectionRecord,
	blank *bevelBlankSource, base facetedPayload) (*bevelToothBoundary, bool) {
	if !pointSourceCurrent(record) || blank == nil || len(record.profile.Outer.Segments) != 6 {
		return nil, false
	}
	implicitCost, ok := bevelImplicitDisplacement(blank, base)
	if !ok {
		return nil, false
	}
	cone := blank.coneSection()
	src := record.source
	result := &bevelToothBoundary{insideArc: -1}
	lineCount, fitCount, arcCount := 0, 0, 0
	for i, entity := range src.entities {
		if src.profile.Outer[i].Partial {
			return nil, false
		}
		switch typed := entity.(type) {
		case *sketch.Line:
			if _, ok := record.profile.Outer.Segments[i].(lineSeg); !ok || lineCount >= 2 {
				return nil, false
			}
			cert, err := src.sketch.CertifyConeLineCrossing(typed, cone)
			if err != nil || cert.Line() != typed || cert.Cone() != cone ||
				cert.SketchRevision() != src.revision || cert.IsStale() || !cert.TExact() {
				return nil, false
			}
			lo, hi := cert.TBounds()
			point := cert.Point()
			if !(0 < lo && lo < hi && hi < 1) ||
				!proofbound.FiniteVec(r3.Vec{X: cert.T(), Y: point[0], Z: point[1]}) {
				return nil, false
			}
			result.lineIndex[lineCount], result.crossing[lineCount] = i, cert
			lineCount++
		case *sketch.FitSpline, *sketch.Arc:
			if _, isFit := entity.(*sketch.FitSpline); isFit {
				if _, ok := record.profile.Outer.Segments[i].(fitSplineSeg); !ok {
					return nil, false
				}
				fitCount++
			} else {
				if _, ok := record.profile.Outer.Segments[i].(arcSeg); !ok {
					return nil, false
				}
				arcCount++
			}
			cert, err := src.sketch.CertifyConeCurveSide(entity, cone)
			if err != nil || cert.Entity() != entity || cert.Cone() != cone || cert.IsStale() {
				return nil, false
			}
			margin := cert.ImplicitMargin()
			if margin == nil || margin.Cmp(proofarith.FloatRat(implicitCost)) <= 0 {
				return nil, false
			}
			if cert.Side() == sketch.ConeInside {
				if _, isArc := entity.(*sketch.Arc); !isArc || result.insideArc >= 0 {
					return nil, false
				}
				result.insideArc = i
			} else if cert.Side() != sketch.ConeOutside {
				return nil, false
			}
			result.sides[i] = cert
		default:
			return nil, false
		}
	}
	if lineCount != 2 || fitCount != 2 || arcCount != 2 || result.insideArc < 0 {
		return nil, false
	}
	left := (result.insideArc + 5) % 6
	right := (result.insideArc + 1) % 6
	if (result.lineIndex[0] != left || result.lineIndex[1] != right) &&
		(result.lineIndex[0] != right || result.lineIndex[1] != left) {
		return nil, false
	}
	return result, true
}

// bevelImplicitDisplacement pays for the original point-loft evaluator and
// chording as well as the discrepancy between the recorded root meridian and
// the public cone. The latter is checked over the far profile's axial range.
// With the admitted unit +X cone, |dF| <= (1+s²)(2R*d+d²).
func bevelImplicitDisplacement(blank *bevelBlankSource, base facetedPayload) (float64, bool) {
	if blank == nil || !proofbound.FiniteVec(r3.Vec{X: base.meshBound}) ||
		base.meshBound < 0 || blank.toeRoot.U <= 0 || len(base.verts) < 4 {
		return 0, false
	}
	minX, maxX, reach := math.Inf(1), math.Inf(-1), 0.0
	for _, world := range base.verts[1:] {
		if !proofbound.FiniteVec(world) || world.X <= 0 {
			return 0, false
		}
		minX, maxX = min(minX, world.X), max(maxX, world.X)
		reach = max(reach, proofbound.DvLenUpper(proofbound.HeldDelta(world, r3.Vec{})))
	}
	minX = math.Nextafter(minX-base.meshBound, math.Inf(-1))
	maxX = proofbound.AbsSumUpper(maxX, base.meshBound)
	slope := blank.coneSection().Slope
	meridianAtFar := bevelMeridianExtensionGap(blank, minX, maxX)
	d := proofbound.AbsSumUpper(base.meshBound, meridianAtFar)
	r := proofbound.AbsSumUpper(reach, d)
	cost := proofbound.ProductUpper(proofbound.AbsSumUpper(1,
		proofbound.ProductUpper(slope, slope)),
		proofbound.ProductUpper(d, proofbound.AbsSumUpper(proofbound.ProductUpper(2, r), d)))
	return cost, !proofbound.IsNonFinite(cost)
}

// pointSourceCurrent checks the source identity held when LoftFromPoint
// recorded the profile. Certificate getters are checked separately at use.
func pointSourceCurrent(record *pointSectionRecord) bool {
	if record == nil || record.source == nil {
		return false
	}
	src := record.source
	if src.sketch == nil || src.profile == nil || src.profile.Sketch() != src.sketch ||
		src.profile.IsStale() || src.sketch.Revision() != src.revision ||
		src.profile.Revision() != src.revision ||
		len(src.entities) != len(record.profile.Outer.Segments) ||
		len(src.profile.Outer) != len(src.entities) {
		return false
	}
	if _, _, _, err := recordProfile(src.sketch, src.profile); err != nil {
		return false
	}
	frame, err := src.sketch.Plane().Frame()
	if err != nil || frame.Origin() != src.frame.Origin() ||
		frame.U() != src.frame.U() || frame.V() != src.frame.V() {
		return false
	}
	for i, entity := range src.entities {
		if entity == nil || src.profile.Outer[i].Entity != entity {
			return false
		}
	}
	return true
}

func exactBevelBlank(b *Body) (*bevelBlankSource, bool) {
	rp, ok := b.payload.(revolvePayload)
	if !ok || !rp.full || rp.surfaceResult || rp.sectionDelta != 0 ||
		rp.xform != r3.Identity() || len(rp.profile.Holes) != 0 ||
		len(rp.profile.Outer.Segments) != 6 ||
		rp.frame.Origin() != (r3.Vec{}) || rp.frame.U() != (r3.Vec{X: 1}) ||
		rp.frame.V() != (r3.Vec{Y: 1}) ||
		rp.ax.AU != 0 || rp.ax.AV != 0 || rp.ax.DU != 1 || rp.ax.DV != 0 ||
		rp.ax.AUBound != 0 || rp.ax.AVBound != 0 ||
		rp.ax.DUBound != 0 || rp.ax.DVBound != 0 {
		return nil, false
	}
	segments := make([]lineSeg, 6)
	points := map[Point2]struct{}{}
	for i, segment := range rp.profile.Outer.Segments {
		line, lineOK := segment.(lineSeg)
		if !lineOK || !((line.TStart == 0 && line.TEnd == 1) ||
			(line.TStart == 1 && line.TEnd == 0)) ||
			!proofbound.FiniteVec(r3.Vec{X: line.Start.U, Y: line.Start.V}) ||
			!proofbound.FiniteVec(r3.Vec{X: line.End.U, Y: line.End.V}) ||
			line.Start == line.End {
			return nil, false
		}
		segments[i] = line
		points[line.Start], points[line.End] = struct{}{}, struct{}{}
	}
	if len(points) != 6 {
		return nil, false
	}
	var axis, outer []Point2
	for point := range points {
		if point.U <= 0 || point.V < 0 {
			return nil, false
		}
		if point.V == 0 {
			axis = append(axis, point)
		} else {
			outer = append(outer, point)
		}
	}
	if len(axis) != 2 || len(outer) != 4 {
		return nil, false
	}
	if axis[0].U > axis[1].U {
		axis[0], axis[1] = axis[1], axis[0]
	}
	result := &bevelBlankSource{toeAxis: axis[0], heelAxis: axis[1]}
	interior := make([]Point2, 0, 2)
	for _, point := range outer {
		switch point.U {
		case result.toeAxis.U:
			if result.toeOuter != (Point2{}) {
				return nil, false
			}
			result.toeOuter = point
		case result.heelAxis.U:
			if result.heelOuter != (Point2{}) {
				return nil, false
			}
			result.heelOuter = point
		default:
			interior = append(interior, point)
		}
	}
	if len(interior) != 2 || result.toeOuter == (Point2{}) ||
		result.heelOuter == (Point2{}) {
		return nil, false
	}
	if interior[0].U > interior[1].U {
		interior[0], interior[1] = interior[1], interior[0]
	}
	result.toeRoot, result.ded = interior[0], interior[1]
	if !(result.toeRoot.U < result.toeAxis.U &&
		result.toeAxis.U < result.ded.U && result.ded.U < result.heelAxis.U) {
		return nil, false
	}
	cycle := [6]Point2{result.toeAxis, result.heelAxis, result.heelOuter,
		result.ded, result.toeRoot, result.toeOuter}
	rootIndex := -1
	var seen [6]bool
	for i, line := range segments {
		matched := false
		for j := range cycle {
			a, z := cycle[j], cycle[(j+1)%len(cycle)]
			if line.Start == a && line.End == z || line.Start == z && line.End == a {
				if seen[j] {
					return nil, false
				}
				seen[j] = true
				matched = true
				result.edgeIndex[j] = i
				if j == 3 {
					rootIndex = i
				}
				break
			}
		}
		if !matched {
			return nil, false
		}
	}
	if rootIndex < 0 {
		return nil, false
	}
	role := fmt.Sprintf("side(0,%d)", rootIndex)
	for _, face := range b.Faces() {
		found := false
		for _, origin := range face.Origins() {
			found = found || origin.Role == role
		}
		if !found {
			continue
		}
		cone, coneOK := face.Surface().(Cone)
		if !coneOK || cone.Origin != (r3.Vec{}) || cone.Axis != (r3.Vec{X: 1}) {
			return nil, false
		}
		radius, radiusErr := cone.Radius.In(units.Millimeter)
		angle, angleErr := cone.HalfAngle.In(units.Radian)
		if radiusErr != nil || angleErr != nil || radius != 0 ||
			!proofbound.FiniteVec(r3.Vec{X: angle}) || angle <= 0 || angle >= math.Pi/2 {
			return nil, false
		}
		slope := math.Tan(angle)
		if !proofbound.FiniteVec(r3.Vec{X: slope}) || slope <= 0 {
			return nil, false
		}
		result.rootFace, result.rootCone = face, cone
		result.coneGap = max(bevelMeridianResidual(result.toeRoot, slope),
			bevelMeridianResidual(result.ded, slope))
		return result, !proofbound.IsNonFinite(result.coneGap)
	}
	return nil, false
}

// A straight meridian and a cone generator differ linearly in station, so
// the endpoint maximum bounds every point between them without sampling.
func bevelMeridianResidual(point Point2, slope float64) float64 {
	r := new(big.Rat).Sub(proofarith.FloatRat(point.V),
		new(big.Rat).Mul(proofarith.FloatRat(slope), proofarith.FloatRat(point.U)))
	if r.Sign() < 0 {
		r.Neg(r)
	}
	return proofbound.RatFloatUp(r)
}

func (source *bevelBlankSource) meridianEquation() (*big.Rat, *big.Rat) {
	x0, x1 := proofarith.FloatRat(source.toeRoot.U), proofarith.FloatRat(source.ded.U)
	r0, r1 := proofarith.FloatRat(source.toeRoot.V), proofarith.FloatRat(source.ded.V)
	m := new(big.Rat).Quo(new(big.Rat).Sub(r1, r0), new(big.Rat).Sub(x1, x0))
	b := new(big.Rat).Sub(r0, new(big.Rat).Mul(m, x0))
	return m, b
}

// Absolute residual is convex for this affine meridian extension, so its
// endpoint maximum bounds every axial station in the interval.
func bevelMeridianExtensionGap(source *bevelBlankSource, xLo, xHi float64) float64 {
	m, b := source.meridianEquation()
	slope := proofarith.FloatRat(source.coneSection().Slope)
	gap := func(x float64) *big.Rat {
		value := new(big.Rat).Add(new(big.Rat).Mul(
			new(big.Rat).Sub(m, slope), proofarith.FloatRat(x)), b)
		return value.Abs(value)
	}
	a, z := gap(xLo), gap(xHi)
	if a.Cmp(z) < 0 {
		a = z
	}
	return proofbound.RatFloatUp(a)
}

// bevelToolFaceGap compares each exact tool cone to the blank's recorded
// straight toe or heel meridian. The endpoint maximum covers the whole
// finite face. The root intersection station is independent of azimuth;
// solving its two line equations bounds its departure from the shared ring.
func bevelToolFaceGap(blank *bevelBlankSource, record *pointConeTrimRecord) (float64, bool) {
	if blank == nil || record == nil || record.lower == nil || record.upper == nil {
		return 0, false
	}
	m, b := blank.meridianEquation()
	maxGap := 0.0
	for _, item := range []struct {
		limit       *pointConeLimit
		root, outer Point2
	}{{record.lower, blank.toeRoot, blank.toeOuter},
		{record.upper, blank.ded, blank.heelOuter}} {
		residual := func(p Point2) *big.Rat {
			value := new(big.Rat).Sub(new(big.Rat).Add(
				proofarith.FloatRat(p.U),
				new(big.Rat).Mul(item.limit.slope, proofarith.FloatRat(p.V))),
				item.limit.apex)
			return value.Abs(value)
		}
		maxGap = max(maxGap, proofbound.RatFloatUp(residual(item.root)),
			proofbound.RatFloatUp(residual(item.outer)))
		den := new(big.Rat).Add(big.NewRat(1, 1), new(big.Rat).Mul(item.limit.slope, m))
		if den.Sign() <= 0 {
			return 0, false
		}
		crossX := new(big.Rat).Quo(new(big.Rat).Sub(item.limit.apex,
			new(big.Rat).Mul(item.limit.slope, b)), den)
		dx := new(big.Rat).Sub(crossX, proofarith.FloatRat(item.root.U))
		dr := new(big.Rat).Mul(m, dx)
		distance2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dr, dr))
		maxGap = max(maxGap, proofbound.RatSqrtUp(distance2))
	}
	return maxGap, !proofbound.IsNonFinite(maxGap)
}

func (source *bevelBlankSource) coneSection() sketch.ConeSection {
	angle, _ := source.rootCone.HalfAngle.In(units.Radian)
	return sketch.ConeSection{Apex: source.rootCone.Origin,
		Axis: source.rootCone.Axis, Slope: math.Tan(angle)}
}

// bevelCrossingPoint lifts Sketch's rounded line evaluation and bounds the
// true cone crossing enclosed by its parameter interval. The returned point
// is only a mesh station; pointBound must enter every seam displacement proof.
func bevelCrossingPoint(src *pointSectionSource, cert sketch.ConeLineCrossing) (
	r3.Vec, float64, bool) {
	if src == nil || cert.IsStale() || !cert.TExact() ||
		cert.SketchRevision() != src.revision {
		return r3.Vec{}, 0, false
	}
	line := cert.Line()
	if line == nil {
		return r3.Vec{}, 0, false
	}
	start, end := line.Start.Geometry(), line.End.Geometry()
	point := cert.Point()
	t, lo, hi := cert.T(), 0.0, 0.0
	lo, hi = cert.TBounds()
	if !(0 < lo && lo <= t && t <= hi && hi < 1) ||
		!proofbound.FiniteVec(r3.Vec{X: point[0], Y: point[1], Z: t}) {
		return r3.Vec{}, 0, false
	}
	q := src.frame.ToWorldUV(point[0], point[1])
	if !proofbound.FiniteVec(q) {
		return r3.Vec{}, 0, false
	}
	delta := proofbound.HeldDelta(r3.Vec{X: end.X, Y: end.Y},
		r3.Vec{X: start.X, Y: start.Y})
	lineLength := proofbound.DvLenUpper(delta)
	loGap := new(big.Rat).Sub(proofarith.FloatRat(t), proofarith.FloatRat(lo))
	hiGap := new(big.Rat).Sub(proofarith.FloatRat(hi), proofarith.FloatRat(t))
	if loGap.Sign() < 0 || hiGap.Sign() < 0 {
		return r3.Vec{}, 0, false
	}
	if hiGap.Cmp(loGap) > 0 {
		loGap = hiGap
	}
	paramGap := proofbound.RatFloatUp(loGap)
	coordError := func(a, b, rounded float64) float64 {
		exact := new(big.Rat).Add(proofarith.FloatRat(a),
			new(big.Rat).Mul(new(big.Rat).Sub(proofarith.FloatRat(b),
				proofarith.FloatRat(a)), proofarith.FloatRat(t)))
		return proofbound.RatAbsDiff(exact, rounded)
	}
	uError := coordError(start.X, end.X, point[0])
	vError := coordError(start.Y, end.Y, point[1])
	uLength := proofbound.DvLenUpper(proofbound.HeldDelta(src.frame.U(), r3.Vec{}))
	vLength := proofbound.DvLenUpper(proofbound.HeldDelta(src.frame.V(), r3.Vec{}))
	bound := proofbound.AbsSumUpper(
		proofbound.ProductUpper(paramGap, proofbound.ProductUpper(lineLength,
			proofbound.AbsSumUpper(uLength, vLength))),
		proofbound.ProductUpper(uError, uLength),
		proofbound.ProductUpper(vError, vLength),
		loftmesh.FrameLiftRoundAllow(src.frame, max(math.Abs(point[0]), math.Abs(point[1]))))
	return q, bound, !proofbound.IsNonFinite(bound)
}

// bevelSeatPoint extends the reported-cone crossing to the exact straight
// meridian recorded by Revolve. F is strictly monotone along the whole line;
// its endpoint margins dominate the meridian perturbation. The inverse slope
// then encloses how far the true source contact can move along the connector.
func bevelSeatPoint(src *pointSectionSource, cert sketch.ConeLineCrossing,
	blank *bevelBlankSource, base facetedPayload) (r3.Vec, float64, bool) {
	point, pointBound, ok := bevelCrossingPoint(src, cert)
	if !ok || blank == nil || len(base.verts) < 4 || base.meshBound < 0 {
		return r3.Vec{}, 0, false
	}
	line := cert.Line()
	start, end := line.Start.Geometry(), line.End.Geometry()
	frame := src.frame
	fr := [3][3]*big.Rat{}
	for axis, values := range [3][3]float64{
		{frame.Origin().X, frame.U().X, frame.V().X},
		{frame.Origin().Y, frame.U().Y, frame.V().Y},
		{frame.Origin().Z, frame.U().Z, frame.V().Z},
	} {
		for j, v := range values {
			fr[axis][j] = proofarith.FloatRat(v)
		}
	}
	var q, d [3]*big.Rat
	u0, v0 := proofarith.FloatRat(start.X), proofarith.FloatRat(start.Y)
	du := new(big.Rat).Sub(proofarith.FloatRat(end.X), u0)
	dv := new(big.Rat).Sub(proofarith.FloatRat(end.Y), v0)
	for i := range q {
		q[i] = new(big.Rat).Add(fr[i][0], new(big.Rat).Add(
			new(big.Rat).Mul(fr[i][1], u0), new(big.Rat).Mul(fr[i][2], v0)))
		d[i] = new(big.Rat).Add(new(big.Rat).Mul(fr[i][1], du),
			new(big.Rat).Mul(fr[i][2], dv))
	}
	slope := proofarith.FloatRat(blank.coneSection().Slope)
	slope2 := new(big.Rat).Mul(slope, slope)
	form := func(p [3]*big.Rat) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Add(
			new(big.Rat).Mul(p[1], p[1]), new(big.Rat).Mul(p[2], p[2])),
			new(big.Rat).Mul(slope2, new(big.Rat).Mul(p[0], p[0])))
	}
	endQ := [3]*big.Rat{}
	for i := range q {
		endQ[i] = new(big.Rat).Add(q[i], d[i])
	}
	f0, f1 := form(q), form(endQ)
	derivative := func(p [3]*big.Rat) *big.Rat {
		return new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Sub(
			new(big.Rat).Add(new(big.Rat).Mul(p[1], d[1]),
				new(big.Rat).Mul(p[2], d[2])),
			new(big.Rat).Mul(slope2, new(big.Rat).Mul(p[0], d[0]))))
	}
	d0, d1 := derivative(q), derivative(endQ)
	if d0.Sign() == 0 || d0.Sign() != d1.Sign() || f0.Sign() == f1.Sign() {
		return r3.Vec{}, 0, false
	}
	m, intercept := blank.meridianEquation()
	sourceRadius := func(p [3]*big.Rat) *big.Rat {
		return new(big.Rat).Add(new(big.Rat).Mul(m, p[0]), intercept)
	}
	sourceForm := func(p [3]*big.Rat) *big.Rat {
		radius := sourceRadius(p)
		return new(big.Rat).Sub(new(big.Rat).Add(
			new(big.Rat).Mul(p[1], p[1]), new(big.Rat).Mul(p[2], p[2])),
			new(big.Rat).Mul(radius, radius))
	}
	if sourceRadius(q).Sign() <= 0 || sourceRadius(endQ).Sign() <= 0 ||
		sourceForm(q).Sign() == sourceForm(endQ).Sign() {
		return r3.Vec{}, 0, false
	}
	minD := new(big.Rat).Abs(d0)
	if minD.Cmp(new(big.Rat).Abs(d1)) > 0 {
		minD.Abs(d1)
	}
	reach := 0.0
	for _, p := range base.verts[1:] {
		reach = max(reach, proofbound.DvLenUpper(proofbound.HeldDelta(p, r3.Vec{})))
	}
	reach = proofbound.AbsSumUpper(reach, base.meshBound)
	xLo := proofbound.RatFloatDown(proofbound.RatMin(q[0], endQ[0]))
	xHi := proofbound.RatFloatUp(proofbound.RatMax(q[0], endQ[0]))
	meridianGap := bevelMeridianExtensionGap(blank, xLo, xHi)
	fCost := proofbound.ProductUpper(meridianGap,
		proofbound.AbsSumUpper(proofbound.ProductUpper(2, reach), meridianGap))
	if proofbound.IsNonFinite(fCost) ||
		new(big.Rat).Abs(f0).Cmp(proofarith.FloatRat(fCost)) <= 0 ||
		new(big.Rat).Abs(f1).Cmp(proofarith.FloatRat(fCost)) <= 0 {
		return r3.Vec{}, 0, false
	}
	shift := proofbound.RatFloatUp(new(big.Rat).Quo(proofarith.FloatRat(fCost), minD))
	lo, hi := cert.TBounds()
	if proofbound.IsNonFinite(shift) || !(lo > shift && hi+shift < 1) {
		return r3.Vec{}, 0, false
	}
	localLength := proofbound.DvLenUpper(proofbound.HeldDelta(
		r3.Vec{X: end.X, Y: end.Y}, r3.Vec{X: start.X, Y: start.Y}))
	basis := proofbound.AbsSumUpper(
		proofbound.DvLenUpper(proofbound.HeldDelta(frame.U(), r3.Vec{})),
		proofbound.DvLenUpper(proofbound.HeldDelta(frame.V(), r3.Vec{})))
	worldShift := proofbound.ProductUpper(shift,
		proofbound.ProductUpper(localLength, basis))
	bound := proofbound.AbsSumUpper(pointBound, worldShift)
	return point, bound, !proofbound.IsNonFinite(bound)
}
