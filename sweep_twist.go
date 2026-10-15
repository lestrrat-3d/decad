package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
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
	path       *sweepinput.Path
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

func (p twistedSweepPayload) Transform() r3.Transform { return p.xform }

func (p twistedSweepPayload) Placed(ctx context.Context, env topo.Env, xform r3.Transform) (*topo.Body, error) {
	d, ref := documentHandle(env.Doc), env.Ref
	return placedNode(evalTwistedSweep(ctx, d, ref, p.profile, p.plane, p.path, p.angle, p.recordArea, xform))
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
		provenance.RoleCapStart: oldFaces[0], provenance.RoleCapEnd: oldFaces[1],
	}
	merged := make([]*Face, 0, 2+count)
	merged = append(merged, oldFaces[:2]...)
	for j := range count {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		uses := map[*Edge][]topo.CoEdge{}
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
			if len(face.n.Loops) != 1 || len(face.n.Loops[0].CoEdges) != 3 ||
				len(face.n.Origins) == 0 {
				return nil, fmt.Errorf(`%w: a twisted wall has no Loft triangle`, ErrUnsupported)
			}
			if firstFacet < 0 {
				firstFacet = k
			}
			for _, use := range face.n.Loops[0].CoEdges {
				uses[edgeHandle(use.Edge)] = append(uses[edgeHandle(use.Edge)], use)
			}
			area = proofbound.AbsSumUpper(area, face.n.Area)
			areaBound = proofbound.AbsSumUpper(areaBound, face.n.AreaBound)
		}
		if firstFacet < 0 {
			return nil, fmt.Errorf(`%w: a twisted source wall has no facets`, ErrUnsupported)
		}
		boundary := map[*Vertex]topo.CoEdge{}
		for _, edgeUses := range uses {
			switch len(edgeUses) {
			case 1:
				use := edgeUses[0]
				if _, exists := boundary[vertexHandle(coedgeStart(use))]; exists {
					return nil, fmt.Errorf(`%w: a twisted wall boundary branches`, ErrUnsupported)
				}
				boundary[vertexHandle(coedgeStart(use))] = use
			case 2:
				if edgeUses[0].Forward == edgeUses[1].Forward {
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
		for _, use := range oldFaces[2+firstFacet].n.Loops[0].CoEdges {
			if kept, ok := boundary[vertexHandle(coedgeStart(use))]; ok && kept.Edge == use.Edge {
				first = vertexHandle(coedgeStart(use))
				break
			}
		}
		if first == nil {
			return nil, fmt.Errorf(`%w: a twisted wall has no deterministic outline start`, ErrUnsupported)
		}
		outline := make([]topo.CoEdge, 0, len(boundary))
		current := first
		for range len(boundary) {
			use, ok := boundary[current]
			if !ok {
				return nil, fmt.Errorf(`%w: a twisted wall outline is disconnected`, ErrUnsupported)
			}
			outline = append(outline, use)
			current = vertexHandle(coedgeEnd(use))
			delete(boundary, vertexHandle(coedgeStart(use)))
		}
		if current != first || len(boundary) != 0 {
			return nil, fmt.Errorf(`%w: a twisted wall outline is not one cycle`, ErrUnsupported)
		}
		face := bornFace(&Face{
			n: topo.Face{
				Area:      area,
				AreaBound: areaBound,
				Surface:   surfacegeom.Faceted{},
				Origins:   []provenance.Ref{{Producer: ref, Role: fmt.Sprintf("side(0,0,%d)", j)}},
				Body:      bodyNode(body),
				Loops:     []*topo.Loop{loopNode(bornLoop(&Loop{n: topo.Loop{Outer: true, CoEdges: outline}}))},
			},
		})
		for k := range held.walls {
			cell := held.cell[k]
			if held.segment[0][cell[1]] == j {
				sources[oldFaces[2+k].n.Origins[0].Role] = face
			}
		}
		merged = append(merged, face)
	}
	// Detach the old triangle uses before attaching the merged faces. The
	// unused diagonal edges then disappear from Body.Edges traversal.
	for _, edge := range body.Edges() {
		edge.n.Faces = nil
	}
	body.n.Lumps[0].Shells[0].Faces = faceNodes(merged)
	if err := topobuild.AttachFaceLoops(ctx, faceNodes(merged)); err != nil {
		return nil, err
	}
	return sources, nil
}

func evalTwistedSweep(ctx context.Context, d *Document, ref producerID, profile profileRecord,
	plane planeRecord, path *sweepinput.Path, angle units.Value, recordArea float64, xform r3.Transform) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if len(path.Records) == 2 {
		return evalCompositeTwistedSweep(ctx, d, ref, profile, plane, path, angle, recordArea, xform)
	}
	if len(path.Records) != 1 || len(path.Segments) != 1 ||
		plane.Origin != (r3.Vec{}) || plane.U != r3.NewVec(1, 0, 0) || plane.V != r3.NewVec(0, 1, 0) ||
		path.Start != (r3.Vec{}) || path.End.X != 0 || path.End.Y != 0 || path.End.Z <= 0 {
		return nil, fmt.Errorf(`%w: nonzero twist currently needs one positive Z line from an origin XY sketch`, ErrUnsupported)
	}
	if path.Segments[0].Arc {
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
	end := planeRecord{Origin: path.End, U: r3.NewVec(cosHeld, sinHeld, 0), V: r3.NewVec(-sinHeld, cosHeld, 0)}
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
	held, ok := heldBody.n.Payload.(loftPayload)
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
	areaIv, err := twistsweep.TwistAreaInterval(poly, twistsweep.TwistRat(path.End.Z), angleIv)
	if err != nil {
		return nil, err
	}
	trueArea, trueAreaBound := sweepmitre.Enclosure(areaIv.Lo, areaIv.Hi)
	heldArea := heldBody.n.Area
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
		twistSymDiff(path.End.Z, poly.PerimeterUpper, idealDelta),
		held.proof.VolSymDiff,
	)
	if proofbound.IsNonFinite(trueArea) || proofbound.IsNonFinite(trueAreaBound) ||
		proofbound.IsNonFinite(areaSlack) || proofbound.IsNonFinite(volSymDiff) {
		return nil, fmt.Errorf(`%w: the twisted sweep's proof is not finite`, ErrUnsupported)
	}
	volume := new(big.Rat).Mul(poly.Area, twistsweep.TwistRat(path.End.Z))
	vol, volBound := sweepmitre.Enclosure(volume, volume)
	heldBody.n.Volume = measurement.Measurement{Value: units.CubicMillimeters(vol), Exactness: measurement.ExactnessOf(volBound), Bound: units.CubicMillimeters(volBound)}
	heldBody.n.Area = measurement.Measurement{Value: units.SquareMillimeters(trueArea), Exactness: measurement.Approximate, Bound: units.SquareMillimeters(trueAreaBound)}
	center := r3.NewVec(0, 0, path.End.Z/2)
	heldCenter := xform.Apply(center)
	centerBound := 0.0
	if xform != r3.Identity() {
		tr := xform.Translation()
		centerBound = proofbound.RigidRoundAllow(path.End.Z/2, max(math.Abs(tr.X), math.Abs(tr.Y), math.Abs(tr.Z)))
	}
	heldBody.n.Centroid = measurement.VecMeasurement{Value: heldCenter, Exactness: measurement.ExactnessOf(centerBound), Bound: units.Millimeters(centerBound)}
	heldBody.n.Bounds.Bound = units.Millimeters(proofbound.AbsSumUpper(heldBody.n.Bounds.Bound.Base(), delta))
	heldBody.n.Bounds.Exactness = measurement.Approximate
	for _, vertex := range heldBody.Vertices() {
		vertex.n.Bound = units.Millimeters(proofbound.AbsSumUpper(vertex.n.Bound.Base(), delta))
	}
	for _, edge := range heldBody.Edges() {
		// A helix edge is not the chord the held Loft exposes. Its true length
		// is at most h+R|angle|; a cap edge is shorter than the perimeter.
		edge.n.Curve = surfacegeom.FacetedCurve{Bound: units.Millimeters(delta)}
		edge.n.LengthBound = proofbound.AbsSumUpper(edge.n.LengthBound,
			edge.n.Length, path.End.Z, proofbound.UpRound(poly.MaxRadiusL1*upper), poly.PerimeterUpper)
	}
	for _, face := range heldBody.Faces() {
		if face.n.Origins[0].Role != provenance.RoleCapStart && face.n.Origins[0].Role != provenance.RoleCapEnd {
			face.n.Surface = surfacegeom.Faceted{Bound: units.Millimeters(delta)}
		}
		// Each triangular parameter patch is a subset of a wall whose area
		// is at most the body's total area upper bound. The wide face bound
		// is intentional until a triangular-domain quadrature is added.
		face.n.AreaBound = proofbound.AbsSumUpper(face.n.AreaBound, face.n.Area, trueArea, trueAreaBound)
		face.n.AxialDelta, face.n.HasAxialDelta = delta, true
	}
	if err := topobuild.ValidateLoftBodyMeasurements(&heldBody.n); err != nil {
		return nil, err
	}
	heldBody.n.Payload = twistedSweepPayload{
		profile: profile, plane: plane, path: path, angle: angle,
		recordArea: recordArea, xform: xform, held: held, sourceRoles: sourceRoles,
		delta: delta, areaSlack: areaSlack, volSymDiff: volSymDiff,
	}
	return heldBody, nil
}
