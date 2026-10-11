package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// twistedSweepPayload keeps the true rigidly rotating section separate from
// the Loft polyhedron used as its held boundary. In particular, consumers
// must not mistake the held facets for the exact shape of the sweep.
type twistedSweepPayload struct {
	profile    profileRecord
	plane      planeRecord
	path       *Path
	angle      units.Value
	recordArea float64
	xform      r3.Transform
	held       loftPayload
	// sourceRoles maps Loft's private triangle roles to the merged live
	// Sweep faces. Both wall triangles of one profile edge share one face.
	sourceRoles map[string]*Face
	delta       float64
	areaSlack   float64
	volSymDiff  float64
}

func (p twistedSweepPayload) transform() r3.Transform { return p.xform }

func (p twistedSweepPayload) placed(ctx context.Context, d *Document, ref producerID, xform r3.Transform) (*Body, error) {
	return evalTwistedSweep(ctx, d, ref, p.profile, p.plane, p.path, p.angle, p.recordArea, xform)
}

type twistPolygon struct {
	points         []Point2
	area           *big.Rat
	perimeterUpper float64
	maxRadiusL1    float64
	maxEdgeL1      float64
}

func twistRat(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }

// twistPolygonOf admits only a strictly convex polygon whose exact dyadic
// centroid is on the path axis. The z coordinate is monotone, so every true
// section is a simple rotated copy of this polygon and sections cannot meet.
func twistPolygonOf(profile profileRecord) (twistPolygon, error) {
	if len(profile.Holes) != 0 {
		return twistPolygon{}, fmt.Errorf(`%w: a twisted sweep with holes is not implemented`, ErrUnsupported)
	}
	segments := profile.Outer.Segments
	if len(segments) < 3 || len(segments) > 256 {
		return twistPolygon{}, fmt.Errorf(`%w: a twisted sweep needs 3 to 256 straight profile edges`, ErrUnsupported)
	}
	points := make([]Point2, len(segments))
	for i, segment := range segments {
		line, ok := segment.(lineSeg)
		if !ok || line.TStart != 0 || line.TEnd != 1 {
			return twistPolygon{}, fmt.Errorf(`%w: a twisted sweep needs whole straight profile edges`, ErrUnsupported)
		}
		points[i] = line.Start
	}
	area2, momentX6, momentY6 := new(big.Rat), new(big.Rat), new(big.Rat)
	perimeter := new(big.Rat)
	maxRadius, maxEdge := new(big.Rat), new(big.Rat)
	sign := 0
	for i, p := range points {
		q, r := points[(i+1)%len(points)], points[(i+2)%len(points)]
		px, py, qx, qy := twistRat(p.U), twistRat(p.V), twistRat(q.U), twistRat(q.V)
		dx, dy := new(big.Rat).Sub(qx, px), new(big.Rat).Sub(qy, py)
		cross := new(big.Rat).Sub(new(big.Rat).Mul(px, qy), new(big.Rat).Mul(qx, py))
		area2.Add(area2, cross)
		momentX6.Add(momentX6, new(big.Rat).Mul(new(big.Rat).Add(px, qx), cross))
		momentY6.Add(momentY6, new(big.Rat).Mul(new(big.Rat).Add(py, qy), cross))
		rx, ry := new(big.Rat).Sub(twistRat(r.U), qx), new(big.Rat).Sub(twistRat(r.V), qy)
		turn := new(big.Rat).Sub(new(big.Rat).Mul(dx, ry), new(big.Rat).Mul(dy, rx))
		if turn.Sign() == 0 || (sign != 0 && turn.Sign() != sign) {
			return twistPolygon{}, fmt.Errorf(`%w: a twisted sweep needs a strictly convex profile`, ErrUnsupported)
		}
		sign = turn.Sign()
		length2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
		length, ok := proofbound.SqrtFixed(length2)
		if !ok {
			return twistPolygon{}, fmt.Errorf(`%w: the twisted profile perimeter has no bound`, ErrUnsupported)
		}
		perimeter.Add(perimeter, length.Hi)
		radiusL1 := new(big.Rat).Add(new(big.Rat).Abs(px), new(big.Rat).Abs(py))
		if radiusL1.Cmp(maxRadius) > 0 {
			maxRadius = radiusL1
		}
		edgeL1 := new(big.Rat).Add(new(big.Rat).Abs(dx), new(big.Rat).Abs(dy))
		if edgeL1.Cmp(maxEdge) > 0 {
			maxEdge = edgeL1
		}
	}
	if area2.Sign() == 0 || momentX6.Sign() != 0 || momentY6.Sign() != 0 {
		return twistPolygon{}, fmt.Errorf(`%w: the twisted profile must have its centroid on the path axis`, ErrUnsupported)
	}
	return twistPolygon{
		points: points, area: new(big.Rat).Quo(new(big.Rat).Abs(area2), big.NewRat(2, 1)),
		perimeterUpper: proofbound.RatFloatUp(perimeter),
		maxRadiusL1:    proofbound.RatFloatUp(maxRadius),
		maxEdgeL1:      proofbound.RatFloatUp(maxEdge),
	}, nil
}

// twistAreaInterval integrates |dS/ds x dS/dt|. On an edge p(s)=p0+s*v,
// that norm is sqrt(h²|v|² + angle²(p(s)·v)²), independent of path station t.
// The primitive of sqrt(a+u²) is
// (u*sqrt(a+u²)+a*asinh(u/sqrt(a)))/2. All inputs and operations below are
// exact rational intervals, including the stated angle and asinh enclosure.
func twistAreaInterval(poly twistPolygon, height *big.Rat, angle proofbound.RatInterval) (proofbound.RatInterval, error) {
	result := proofbound.PointInterval(new(big.Rat).Mul(poly.area, big.NewRat(2, 1)))
	absAngle := angle
	if angle.Hi.Sign() < 0 {
		absAngle = proofbound.IntervalNeg(angle)
	}
	if absAngle.Lo.Sign() <= 0 {
		return proofbound.RatInterval{}, fmt.Errorf(`%w: the twisted sweep's angle interval includes zero`, ErrUnsupported)
	}
	h2 := new(big.Rat).Mul(height, height)
	for i, p := range poly.points {
		q := poly.points[(i+1)%len(poly.points)]
		x, y := twistRat(p.U), twistRat(p.V)
		dx, dy := new(big.Rat).Sub(twistRat(q.U), x), new(big.Rat).Sub(twistRat(q.V), y)
		len2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
		base := new(big.Rat).Mul(h2, len2)
		rootA, ok := proofbound.SqrtInterval(proofbound.PointInterval(base))
		if !ok || rootA.Lo.Sign() <= 0 {
			return proofbound.RatInterval{}, fmt.Errorf(`%w: a twisted wall has no area bound`, ErrUnsupported)
		}
		primitive := func(dot *big.Rat) (proofbound.RatInterval, bool) {
			u := proofbound.IntervalScale(absAngle, dot)
			root, ok := proofbound.SqrtInterval(proofbound.IntervalAdd(
				proofbound.PointInterval(base), proofbound.IntervalSquare(u)))
			if !ok {
				return proofbound.RatInterval{}, false
			}
			ratio, ok := proofbound.IntervalQuo(u, rootA)
			if !ok {
				return proofbound.RatInterval{}, false
			}
			return proofbound.IntervalScale(proofbound.IntervalAdd(
				proofbound.IntervalMul(u, root),
				proofbound.IntervalScale(proofbound.AsinhInterval(ratio), base),
			), big.NewRat(1, 2)), true
		}
		initial := new(big.Rat).Add(new(big.Rat).Mul(x, dx), new(big.Rat).Mul(y, dy))
		finish := new(big.Rat).Add(initial, len2)
		f0, ok0 := primitive(initial)
		f1, ok1 := primitive(finish)
		if !ok0 || !ok1 {
			return proofbound.RatInterval{}, fmt.Errorf(`%w: a twisted wall has no area integral`, ErrUnsupported)
		}
		term, ok := proofbound.IntervalQuo(proofbound.IntervalSub(f1, f0),
			proofbound.IntervalScale(absAngle, len2))
		if !ok {
			return proofbound.RatInterval{}, fmt.Errorf(`%w: a twisted wall has no positive angle divisor`, ErrUnsupported)
		}
		result = proofbound.IntervalAdd(result, term)
	}
	return result, nil
}

// twistDeparture encloses the true wall against Loft's flat two-triangle
// wall. Linear interpolation of the rotation has error ≤ R|angle|²/8; the
// bilinear ruled quad against its diagonal has error ≤ |edge|(|angle|+e)/4.
// e covers the endpoint frame's enclosed sin/cos discrepancy.
func twistDeparture(poly twistPolygon, angleUpper, endpointCoordError, loftDelta float64) float64 {
	interp := proofbound.UpRound(poly.maxRadiusL1 * angleUpper * angleUpper / 8)
	warp := proofbound.UpRound(poly.maxEdgeL1 * (angleUpper + 2*endpointCoordError) / 4)
	end := proofbound.UpRound(2 * poly.maxRadiusL1 * endpointCoordError)
	return proofbound.AbsSumUpper(interp, warp, end, loftDelta)
}

// twistSymDiff bounds each horizontal symmetric difference by the two-sided
// delta tube of the convex true polygon. The section has perimeter P, so the
// tube area is at most 2Pδ+2πδ²; integrate over the exact path height.
func twistSymDiff(height float64, perimeter, delta float64) float64 {
	h, p, d := twistRat(height), twistRat(perimeter), twistRat(delta)
	linear := new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Mul(p, d))
	quadratic := new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Mul(proofbound.PiUpper, new(big.Rat).Mul(d, d)))
	return proofbound.RatFloatUp(new(big.Rat).Mul(h, new(big.Rat).Add(linear, quadratic)))
}

// mergeTwistWalls removes the diagonal Loft exposes between each pair of
// wall triangles. The two triangles remain in the held mesh, but their one
// true twisted patch is one live Faceted face with role side(0,0,j).
func mergeTwistWalls(ctx context.Context, body *Body, ref producerID, count int) (map[string]*Face, error) {
	oldFaces := body.Faces()
	if len(oldFaces) != 2+2*count {
		return nil, fmt.Errorf(`%w: the twist's held wall count changed`, ErrUnsupported)
	}
	sources := map[string]*Face{
		roleCapStart: oldFaces[0], roleCapEnd: oldFaces[1],
	}
	merged := make([]*Face, 0, 2+count)
	merged = append(merged, oldFaces[:2]...)
	for j := range count {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		lower, upper := oldFaces[2+2*j], oldFaces[3+2*j]
		if len(lower.loops) != 1 || len(upper.loops) != 1 ||
			len(lower.loops[0].coedges) != 3 || len(upper.loops[0].coedges) != 3 {
			return nil, fmt.Errorf(`%w: a twisted wall has no paired Loft triangles`, ErrUnsupported)
		}
		lo, up := lower.loops[0].coedges, upper.loops[0].coedges
		li, ui := -1, -1
		for a, x := range lo {
			for b, y := range up {
				if x.edge == y.edge {
					li, ui = a, b
				}
			}
		}
		if li < 0 {
			return nil, fmt.Errorf(`%w: paired twisted facets have no shared diagonal`, ErrUnsupported)
		}
		outline := []coedge{
			lo[(li+1)%3], lo[(li+2)%3], up[(ui+1)%3], up[(ui+2)%3],
		}
		face := &Face{
			surface: Faceted{}, loops: []*Loop{{outer: true, coedges: outline}},
			origins:   []FeatureRef{{producer: ref, Role: fmt.Sprintf("side(0,0,%d)", j)}},
			body:      body,
			area:      proofbound.AbsSumUpper(lower.area, upper.area),
			areaBound: proofbound.AbsSumUpper(lower.areaBound, upper.areaBound),
		}
		sources[lower.origins[0].Role] = face
		sources[upper.origins[0].Role] = face
		merged = append(merged, face)
	}
	// Detach the old triangle uses before attaching the merged faces. The
	// unused diagonal edges then disappear from Body.Edges traversal.
	for _, edge := range body.Edges() {
		edge.faces = nil
	}
	body.lumps[0].shells[0].faces = merged
	if err := attachFaceLoopsContext(ctx, merged); err != nil {
		return nil, err
	}
	return sources, nil
}

func evalTwistedSweep(ctx context.Context, d *Document, ref producerID, profile profileRecord,
	plane planeRecord, path *Path, angle units.Value, recordArea float64, xform r3.Transform) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if len(path.records) != 1 || len(path.segments) != 1 ||
		plane.Origin != (r3.Vec{}) || plane.U != r3.NewVec(1, 0, 0) || plane.V != r3.NewVec(0, 1, 0) ||
		path.Start() != (r3.Vec{}) || path.End().X != 0 || path.End().Y != 0 || path.End().Z <= 0 {
		return nil, fmt.Errorf(`%w: nonzero twist currently needs one positive Z line from an origin XY sketch`, ErrUnsupported)
	}
	if _, ok := path.segments[0].(LineTo); !ok {
		return nil, fmt.Errorf(`%w: nonzero twist currently needs one straight path span`, ErrUnsupported)
	}
	poly, err := twistPolygonOf(profile)
	if err != nil {
		return nil, err
	}
	theta, err := angle.In(units.Radian)
	if err != nil {
		return nil, err
	}
	denoted := revolveangle.FromValue(angle)
	angleIv, ok := denoted.Enclosure()
	if !ok {
		return nil, fmt.Errorf(`%w: this twist angle has no certified denotation`, ErrUnsupported)
	}
	upper := proofbound.RatFloatUp(proofbound.IntervalAbsUpper(angleIv))
	if upper > 1 || upper == 0 {
		return nil, fmt.Errorf(`%w: the twisted sweep admits an angle up to one radian`, ErrUnsupported)
	}
	sinIv, cosIv, ok := denoted.SinCosFor(theta)
	if !ok {
		return nil, fmt.Errorf(`%w: the twisted sweep cannot enclose the endpoint rotation`, ErrUnsupported)
	}
	sinHeld, cosHeld := math.Sincos(theta)
	end := planeRecord{Origin: path.End(), U: r3.NewVec(cosHeld, sinHeld, 0), V: r3.NewVec(-sinHeld, cosHeld, 0)}
	f0, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	if err != nil {
		return nil, err
	}
	f1, err := r3.NewFrame(end.Origin, end.U, end.V)
	if err != nil {
		return nil, err
	}
	// Compare the frame Loft actually lifts against the ideal rotation. These
	// exact interval-to-float distances include SinCos and NewFrame rounding.
	coordError := max(
		proofbound.IntervalFloatError(cosIv, f1.U().X),
		proofbound.IntervalFloatError(sinIv, f1.U().Y),
		proofbound.IntervalFloatError(proofbound.IntervalNeg(sinIv), f1.V().X),
		proofbound.IntervalFloatError(cosIv, f1.V().Y),
	)
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	heldBody, err := evalLoft(ctx, d, ref, loftPayload{
		profile0: profile, profile1: profile, plane0: plane, plane1: end,
		frame0: f0, frame1: f1, xform: xform,
		recordArea: [2]float64{recordArea, recordArea},
	}, proofbound.NewWorkBudget(ctx), work0, work1)
	if err != nil {
		return nil, err
	}
	held := heldBody.payload.(loftPayload)
	sourceRoles, err := mergeTwistWalls(ctx, heldBody, ref, len(poly.points))
	if err != nil {
		return nil, err
	}
	idealDelta := twistDeparture(poly, upper, coordError, 0)
	delta := proofbound.AbsSumUpper(idealDelta, held.delta)
	if proofbound.IsNonFinite(delta) || delta <= 0 {
		return nil, fmt.Errorf(`%w: the twisted boundary has no finite displacement proof`, ErrUnsupported)
	}
	areaIv, err := twistAreaInterval(poly, twistRat(path.End().Z), angleIv)
	if err != nil {
		return nil, err
	}
	trueArea, trueAreaBound := mitredEnclosure(areaIv.Lo, areaIv.Hi)
	heldArea := heldBody.area
	heldAreaIv := proofbound.Interval(
		new(big.Rat).Sub(twistRat(heldArea.Value.Base()), twistRat(heldArea.Bound.Base())),
		new(big.Rat).Add(twistRat(heldArea.Value.Base()), twistRat(heldArea.Bound.Base())),
	)
	areaGap := proofbound.RatFloatUp(proofbound.IntervalAbsUpper(proofbound.IntervalSub(areaIv, heldAreaIv)))
	areaSlack := proofbound.AbsSumUpper(areaGap, held.proof.AreaSlack)
	// The horizontal-section tube compares the true twist with the exact
	// unrounded Loft facets. Loft's own occupied-volume proof then covers
	// every held-vertex displacement, including axial placement rounding.
	volSymDiff := proofbound.AbsSumUpper(
		twistSymDiff(path.End().Z, poly.perimeterUpper, idealDelta),
		held.proof.VolSymDiff,
	)
	if proofbound.IsNonFinite(trueArea) || proofbound.IsNonFinite(trueAreaBound) ||
		proofbound.IsNonFinite(areaSlack) || proofbound.IsNonFinite(volSymDiff) {
		return nil, fmt.Errorf(`%w: the twisted sweep's proof is not finite`, ErrUnsupported)
	}
	volume := new(big.Rat).Mul(poly.area, twistRat(path.End().Z))
	vol, volBound := mitredEnclosure(volume, volume)
	heldBody.volume = Measurement{Value: units.CubicMillimeters(vol), Exactness: exactnessOf(volBound), Bound: units.CubicMillimeters(volBound)}
	heldBody.area = Measurement{Value: units.SquareMillimeters(trueArea), Exactness: Approximate, Bound: units.SquareMillimeters(trueAreaBound)}
	center := r3.NewVec(0, 0, path.End().Z/2)
	heldCenter := xform.Apply(center)
	centerBound := 0.0
	if xform != r3.Identity() {
		tr := xform.Translation()
		centerBound = proofbound.RigidRoundAllow(path.End().Z/2, max(math.Abs(tr.X), math.Abs(tr.Y), math.Abs(tr.Z)))
	}
	heldBody.centroid = VecMeasurement{Value: heldCenter, Exactness: exactnessOf(centerBound), Bound: units.Millimeters(centerBound)}
	heldBody.bounds.Bound = units.Millimeters(proofbound.AbsSumUpper(heldBody.bounds.Bound.Base(), delta))
	heldBody.bounds.Exactness = Approximate
	for _, vertex := range heldBody.Vertices() {
		vertex.bound = units.Millimeters(proofbound.AbsSumUpper(vertex.bound.Base(), delta))
	}
	for _, edge := range heldBody.Edges() {
		// A helix edge is not the chord the held Loft exposes. Its true length
		// is at most h+R|angle|; a cap edge is shorter than the perimeter.
		edge.curve = FacetedCurve{Bound: units.Millimeters(delta)}
		edge.lengthBound = proofbound.AbsSumUpper(edge.lengthBound,
			edge.length, path.End().Z, proofbound.UpRound(poly.maxRadiusL1*upper), poly.perimeterUpper)
	}
	for _, face := range heldBody.Faces() {
		if face.origins[0].Role != roleCapStart && face.origins[0].Role != roleCapEnd {
			face.surface = Faceted{Bound: units.Millimeters(delta)}
		}
		// Each triangular parameter patch is a subset of a wall whose area
		// is at most the body's total area upper bound. The wide face bound
		// is intentional until a triangular-domain quadrature is added.
		face.areaBound = proofbound.AbsSumUpper(face.areaBound, face.area, trueArea, trueAreaBound)
		face.axialDelta, face.hasAxialDelta = delta, true
	}
	if err := validateLoftBodyMeasurements(heldBody); err != nil {
		return nil, err
	}
	heldBody.payload = twistedSweepPayload{
		profile: profile, plane: plane, path: path, angle: angle,
		recordArea: recordArea, xform: xform, held: held, sourceRoles: sourceRoles,
		delta: delta, areaSlack: areaSlack, volSymDiff: volSymDiff,
	}
	return heldBody, nil
}
