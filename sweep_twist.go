package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/twistsweep"
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
	mesh       *Mesh
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

// twistAreaInterval integrates |dS/ds x dS/dt|. On an edge p(s)=p0+s*v,
// that norm is sqrt(h²|v|² + angle²(p(s)·v)²), independent of path station t.
// The primitive of sqrt(a+u²) is
// (u*sqrt(a+u²)+a*asinh(u/sqrt(a)))/2. All inputs and operations below are
// exact rational intervals, including the stated angle and asinh enclosure.
func twistAreaInterval(poly twistsweep.TwistPolygon, height *big.Rat, angle proofbound.RatInterval) (proofbound.RatInterval, error) {
	result := proofbound.PointInterval(new(big.Rat).Mul(poly.Area, big.NewRat(2, 1)))
	absAngle := angle
	if angle.Hi.Sign() < 0 {
		absAngle = proofbound.IntervalNeg(angle)
	}
	if absAngle.Lo.Sign() <= 0 {
		return proofbound.RatInterval{}, fmt.Errorf(`%w: the twisted sweep's angle interval includes zero`, ErrUnsupported)
	}
	h2 := new(big.Rat).Mul(height, height)
	for i, p := range poly.Points {
		q := poly.Points[(i+1)%len(poly.Points)]
		x, y := twistsweep.TwistRat(p.U), twistsweep.TwistRat(p.V)
		dx, dy := new(big.Rat).Sub(twistsweep.TwistRat(q.U), x), new(big.Rat).Sub(twistsweep.TwistRat(q.V), y)
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

// For an arc-length parameter s, a rotating vertical wall has Jacobian
// sqrt(h² + theta²*(p dot dp/ds)²). The two-sided perimeter and source-radius
// enclosures bound this non-cancelling integral for both lines and arcs.
func twistMixedAreaInterval(poly twistsweep.TwistPolygon, height *big.Rat,
	angle proofbound.RatInterval) (proofbound.RatInterval, error) {
	if height.Sign() <= 0 || poly.PerimeterLower <= 0 || poly.PerimeterUpper <= 0 ||
		poly.MaxRadiusL1 <= 0 || proofbound.IsNonFinite(poly.PerimeterUpper) ||
		proofbound.IsNonFinite(poly.MaxRadiusL1) {
		return proofbound.RatInterval{}, fmt.Errorf(`%w: a mixed twisted section has no wall-area bound`, ErrUnsupported)
	}
	thetaR := new(big.Rat).Mul(proofbound.IntervalAbsUpper(angle), twistsweep.TwistRat(poly.MaxRadiusL1))
	ratio := new(big.Rat).Quo(thetaR, height)
	factor, ok := proofbound.SqrtInterval(proofbound.PointInterval(
		new(big.Rat).Add(big.NewRat(1, 1), new(big.Rat).Mul(ratio, ratio))))
	if !ok {
		return proofbound.RatInterval{}, fmt.Errorf(`%w: a mixed twisted section has no wall-area root`, ErrUnsupported)
	}
	wallLo := new(big.Rat).Mul(height, twistsweep.TwistRat(poly.PerimeterLower))
	wallHi := new(big.Rat).Mul(new(big.Rat).Mul(height, twistsweep.TwistRat(poly.PerimeterUpper)), factor.Hi)
	return proofbound.IntervalAdd(proofbound.IntervalScale(poly.AreaInterval, big.NewRat(2, 1)),
		proofbound.Interval(wallLo, wallHi)), nil
}

// twistDeparture encloses the true wall against Loft's flat two-triangle
// wall. Linear interpolation of the rotation has error ≤ R|angle|²/8; the
// bilinear ruled quad against its diagonal has error ≤ |edge|(|angle|+e)/4.
// e covers the endpoint frame's enclosed sin/cos discrepancy.
func twistDeparture(poly twistsweep.TwistPolygon, angleUpper, endpointCoordError, loftDelta float64) float64 {
	interp := proofbound.UpRound(poly.MaxRadiusL1 * angleUpper * angleUpper / 8)
	warp := proofbound.UpRound(poly.MaxEdgeL1 * (angleUpper + 2*endpointCoordError) / 4)
	end := proofbound.UpRound(2 * poly.MaxRadiusL1 * endpointCoordError)
	return proofbound.AbsSumUpper(interp, warp, end, loftDelta)
}

// twistSymDiff bounds each horizontal symmetric difference by the two-sided
// delta tube of the true closed source walk. A rectifiable walk of perimeter
// P has a delta-neighbourhood of area at most 2Pδ+2πδ². Integrate over height.
func twistSymDiff(height float64, perimeter, delta float64) float64 {
	h, p, d := twistsweep.TwistRat(height), twistsweep.TwistRat(perimeter), twistsweep.TwistRat(delta)
	linear := new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Mul(p, d))
	quadratic := new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Mul(proofbound.PiUpper, new(big.Rat).Mul(d, d)))
	return proofbound.RatFloatUp(new(big.Rat).Mul(h, new(big.Rat).Add(linear, quadratic)))
}

// mergeTwistWalls removes internal Loft edges from each source wall's live
// face. Its triangles remain in the held mesh; its outline becomes one live
// Faceted face with role side(0,0,j).
func mergeTwistWalls(ctx context.Context, body *Body, ref producerID, held loftPayload, count int) (map[string]*Face, error) {
	oldFaces := body.Faces()
	if len(oldFaces) != 2+held.walls || len(held.cell) != held.walls ||
		len(held.segment) != 1 {
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
		uses := map[*Edge][]coedge{}
		area, areaBound, firstFacet := 0.0, 0.0, -1
		for k := range held.walls {
			cell := held.cell[k]
			if cell[0] != 0 || cell[1] < 0 || cell[1] >= len(held.segment[0]) {
				return nil, fmt.Errorf(`%w: a twisted wall lost its source segment`, ErrUnsupported)
			}
			if held.segment[0][cell[1]] != j {
				continue
			}
			face := oldFaces[2+k]
			if len(face.loops) != 1 || len(face.loops[0].coedges) != 3 ||
				len(face.origins) == 0 {
				return nil, fmt.Errorf(`%w: a twisted wall has no Loft triangle`, ErrUnsupported)
			}
			if firstFacet < 0 {
				firstFacet = k
			}
			for _, use := range face.loops[0].coedges {
				uses[use.edge] = append(uses[use.edge], use)
			}
			area = proofbound.AbsSumUpper(area, face.area)
			areaBound = proofbound.AbsSumUpper(areaBound, face.areaBound)
		}
		if firstFacet < 0 {
			return nil, fmt.Errorf(`%w: a twisted source wall has no facets`, ErrUnsupported)
		}
		boundary := map[*Vertex]coedge{}
		for _, edgeUses := range uses {
			switch len(edgeUses) {
			case 1:
				use := edgeUses[0]
				if _, exists := boundary[use.Start()]; exists {
					return nil, fmt.Errorf(`%w: a twisted wall boundary branches`, ErrUnsupported)
				}
				boundary[use.Start()] = use
			case 2:
				if edgeUses[0].forward == edgeUses[1].forward {
					return nil, fmt.Errorf(`%w: twisted wall facets use a seam in the same direction`, ErrUnsupported)
				}
			default:
				return nil, fmt.Errorf(`%w: a twisted wall edge has an unsupported use count`, ErrUnsupported)
			}
		}
		if len(boundary) < 4 {
			return nil, fmt.Errorf(`%w: a twisted wall has no closed boundary`, ErrUnsupported)
		}
		var first *Vertex
		for _, use := range oldFaces[2+firstFacet].loops[0].coedges {
			if kept, ok := boundary[use.Start()]; ok && kept.edge == use.edge {
				first = use.Start()
				break
			}
		}
		if first == nil {
			return nil, fmt.Errorf(`%w: a twisted wall has no deterministic outline start`, ErrUnsupported)
		}
		outline := make([]coedge, 0, len(boundary))
		current := first
		for range len(boundary) {
			use, ok := boundary[current]
			if !ok {
				return nil, fmt.Errorf(`%w: a twisted wall outline is disconnected`, ErrUnsupported)
			}
			outline = append(outline, use)
			current = use.End()
			delete(boundary, use.Start())
		}
		if current != first || len(boundary) != 0 {
			return nil, fmt.Errorf(`%w: a twisted wall outline is not one cycle`, ErrUnsupported)
		}
		face := &Face{
			surface: Faceted{}, loops: []*Loop{{outer: true, coedges: outline}},
			origins:   []FeatureRef{{producer: ref, Role: fmt.Sprintf("side(0,0,%d)", j)}},
			body:      body,
			area:      area,
			areaBound: areaBound,
		}
		for k := range held.walls {
			cell := held.cell[k]
			if held.segment[0][cell[1]] == j {
				sources[oldFaces[2+k].origins[0].Role] = face
			}
		}
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
	if len(path.records) == 2 {
		return evalCompositeTwistedSweep(ctx, d, ref, profile, plane, path, angle, recordArea, xform)
	}
	if len(path.records) != 1 || len(path.segments) != 1 ||
		plane.Origin != (r3.Vec{}) || plane.U != r3.NewVec(1, 0, 0) || plane.V != r3.NewVec(0, 1, 0) ||
		path.Start() != (r3.Vec{}) || path.End().X != 0 || path.End().Y != 0 || path.End().Z <= 0 {
		return nil, fmt.Errorf(`%w: nonzero twist currently needs one positive Z line from an origin XY sketch`, ErrUnsupported)
	}
	if _, ok := path.segments[0].(LineTo); !ok {
		return nil, fmt.Errorf(`%w: nonzero twist currently needs one straight path span`, ErrUnsupported)
	}
	poly, err := twistsweep.TwistPolygonOf(profile)
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
	if upper == 0 {
		return nil, fmt.Errorf(`%w: the twisted sweep admits an angle up to one radian`, ErrUnsupported)
	}
	if upper > 1 || poly.HasArc {
		return evalStationedTwistedSweep(ctx, d, ref, profile, plane, path, angle, recordArea, xform)
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
	held, ok := heldBody.payload.(loftPayload)
	if !ok {
		return nil, fmt.Errorf(`%w: the twisted sweep lost its held Loft proof`, ErrUnsupported)
	}
	sourceRoles, err := mergeTwistWalls(ctx, heldBody, ref, held, len(poly.Points))
	if err != nil {
		return nil, err
	}
	idealDelta := twistDeparture(poly, upper, coordError, 0)
	delta := proofbound.AbsSumUpper(idealDelta, held.delta)
	if proofbound.IsNonFinite(delta) || delta <= 0 {
		return nil, fmt.Errorf(`%w: the twisted boundary has no finite displacement proof`, ErrUnsupported)
	}
	areaIv, err := twistAreaInterval(poly, twistsweep.TwistRat(path.End().Z), angleIv)
	if err != nil {
		return nil, err
	}
	trueArea, trueAreaBound := mitredEnclosure(areaIv.Lo, areaIv.Hi)
	heldArea := heldBody.area
	heldAreaIv := proofbound.Interval(
		new(big.Rat).Sub(twistsweep.TwistRat(heldArea.Value.Base()), twistsweep.TwistRat(heldArea.Bound.Base())),
		new(big.Rat).Add(twistsweep.TwistRat(heldArea.Value.Base()), twistsweep.TwistRat(heldArea.Bound.Base())),
	)
	areaGap := proofbound.RatFloatUp(proofbound.IntervalAbsUpper(proofbound.IntervalSub(areaIv, heldAreaIv)))
	areaSlack := proofbound.AbsSumUpper(areaGap, held.proof.AreaSlack)
	// The horizontal-section tube compares the true twist with the exact
	// unrounded Loft facets. Loft's own occupied-volume proof then covers
	// every held-vertex displacement, including axial placement rounding.
	volSymDiff := proofbound.AbsSumUpper(
		twistSymDiff(path.End().Z, poly.PerimeterUpper, idealDelta),
		held.proof.VolSymDiff,
	)
	if proofbound.IsNonFinite(trueArea) || proofbound.IsNonFinite(trueAreaBound) ||
		proofbound.IsNonFinite(areaSlack) || proofbound.IsNonFinite(volSymDiff) {
		return nil, fmt.Errorf(`%w: the twisted sweep's proof is not finite`, ErrUnsupported)
	}
	volume := new(big.Rat).Mul(poly.Area, twistsweep.TwistRat(path.End().Z))
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
			edge.length, path.End().Z, proofbound.UpRound(poly.MaxRadiusL1*upper), poly.PerimeterUpper)
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
