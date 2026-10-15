package twistsweep

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// For an arc-length parameter s, a rotating vertical wall has Jacobian
// sqrt(h² + theta²*(p dot dp/ds)²). The two-sided perimeter and source-radius
// enclosures bound this non-cancelling integral for both lines and arcs.
func twistMixedAreaInterval(poly TwistPolygon, height *big.Rat,
	angle proofbound.RatInterval) (proofbound.RatInterval, error) {
	if height.Sign() <= 0 || poly.PerimeterLower <= 0 || poly.PerimeterUpper <= 0 ||
		poly.MaxRadiusL1 <= 0 || proofbound.IsNonFinite(poly.PerimeterUpper) ||
		proofbound.IsNonFinite(poly.MaxRadiusL1) {
		return proofbound.RatInterval{}, fmt.Errorf(`%w: a mixed twisted section has no wall-area bound`, decaderr.ErrUnsupported)
	}
	thetaR := new(big.Rat).Mul(proofbound.IntervalAbsUpper(angle), TwistRat(poly.MaxRadiusL1))
	ratio := new(big.Rat).Quo(thetaR, height)
	factor, ok := proofbound.SqrtInterval(proofbound.PointInterval(
		new(big.Rat).Add(big.NewRat(1, 1), new(big.Rat).Mul(ratio, ratio))))
	if !ok {
		return proofbound.RatInterval{}, fmt.Errorf(`%w: a mixed twisted section has no wall-area root`, decaderr.ErrUnsupported)
	}
	wallLo := new(big.Rat).Mul(height, TwistRat(poly.PerimeterLower))
	wallHi := new(big.Rat).Mul(new(big.Rat).Mul(height, TwistRat(poly.PerimeterUpper)), factor.Hi)
	return proofbound.IntervalAdd(proofbound.IntervalScale(poly.AreaInterval, big.NewRat(2, 1)),
		proofbound.Interval(wallLo, wallHi)), nil
}

// twistDeparture encloses the true wall against Loft's flat two-triangle
// wall. Linear interpolation of the rotation has error ≤ R|angle|²/8; the
// bilinear ruled quad against its diagonal has error ≤ |edge|(|angle|+e)/4.
// e covers the endpoint frame's enclosed sin/cos discrepancy.
func twistDeparture(poly TwistPolygon, angleUpper, endpointCoordError, loftDelta float64) float64 {
	interp := proofbound.UpRound(poly.MaxRadiusL1 * angleUpper * angleUpper / 8)
	warp := proofbound.UpRound(poly.MaxEdgeL1 * (angleUpper + 2*endpointCoordError) / 4)
	end := proofbound.UpRound(2 * poly.MaxRadiusL1 * endpointCoordError)
	return proofbound.AbsSumUpper(interp, warp, end, loftDelta)
}

// twistSymDiff bounds each horizontal symmetric difference by the two-sided
// delta tube of the true closed source walk. A rectifiable walk of perimeter
// P has a delta-neighbourhood of area at most 2Pδ+2πδ². Integrate over height.
func twistSymDiff(height float64, perimeter, delta float64) float64 {
	h, p, d := TwistRat(height), TwistRat(perimeter), TwistRat(delta)
	linear := new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Mul(p, d))
	quadratic := new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Mul(proofbound.PiUpper, new(big.Rat).Mul(d, d)))
	return proofbound.RatFloatUp(new(big.Rat).Mul(h, new(big.Rat).Add(linear, quadratic)))
}

// mergeTwistWalls removes internal Loft edges from each source wall's live
// face. Its triangles remain in the held mesh; its outline becomes one live
// Faceted face with role side(0,0,j).
func mergeTwistWalls(ctx context.Context, env topo.Env, body *topo.Body, held loftrecord.Payload, count int) (map[string]*topo.Face, error) {
	oldFaces := topobuild.Faces(body)
	if len(oldFaces) != 2+held.Walls || len(held.Cell) != held.Walls ||
		len(held.Segment) != 1 {
		return nil, fmt.Errorf(`%w: the twist's held wall count changed`, decaderr.ErrUnsupported)
	}
	sources := map[string]*topo.Face{
		provenance.RoleCapStart: oldFaces[0], provenance.RoleCapEnd: oldFaces[1],
	}
	merged := make([]*topo.Face, 0, 2+count)
	merged = append(merged, oldFaces[:2]...)
	for j := range count {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		uses := map[*topo.Edge][]topo.CoEdge{}
		area, areaBound, firstFacet := 0.0, 0.0, -1
		for k := range held.Walls {
			cell := held.Cell[k]
			if cell[0] != 0 || cell[1] < 0 || cell[1] >= len(held.Segment[0]) {
				return nil, fmt.Errorf(`%w: a twisted wall lost its source segment`, decaderr.ErrUnsupported)
			}
			if held.Segment[0][cell[1]] != j {
				continue
			}
			face := oldFaces[2+k]
			if len(face.Loops) != 1 || len(face.Loops[0].CoEdges) != 3 ||
				len(face.Origins) == 0 {
				return nil, fmt.Errorf(`%w: a twisted wall has no Loft triangle`, decaderr.ErrUnsupported)
			}
			if firstFacet < 0 {
				firstFacet = k
			}
			for _, use := range face.Loops[0].CoEdges {
				uses[use.Edge] = append(uses[use.Edge], use)
			}
			area = proofbound.AbsSumUpper(area, face.Area)
			areaBound = proofbound.AbsSumUpper(areaBound, face.AreaBound)
		}
		if firstFacet < 0 {
			return nil, fmt.Errorf(`%w: a twisted source wall has no facets`, decaderr.ErrUnsupported)
		}
		boundary := map[*topo.Vertex]topo.CoEdge{}
		for _, edgeUses := range uses {
			switch len(edgeUses) {
			case 1:
				use := edgeUses[0]
				if _, exists := boundary[topobuild.CoEdgeStart(use)]; exists {
					return nil, fmt.Errorf(`%w: a twisted wall boundary branches`, decaderr.ErrUnsupported)
				}
				boundary[topobuild.CoEdgeStart(use)] = use
			case 2:
				if edgeUses[0].Forward == edgeUses[1].Forward {
					return nil, fmt.Errorf(`%w: twisted wall facets use a seam in the same direction`, decaderr.ErrUnsupported)
				}
			default:
				return nil, fmt.Errorf(`%w: a twisted wall edge has an unsupported use count`, decaderr.ErrUnsupported)
			}
		}
		if len(boundary) < 4 {
			return nil, fmt.Errorf(`%w: a twisted wall has no closed boundary`, decaderr.ErrUnsupported)
		}
		var first *topo.Vertex
		for _, use := range oldFaces[2+firstFacet].Loops[0].CoEdges {
			if kept, ok := boundary[topobuild.CoEdgeStart(use)]; ok && kept.Edge == use.Edge {
				first = topobuild.CoEdgeStart(use)
				break
			}
		}
		if first == nil {
			return nil, fmt.Errorf(`%w: a twisted wall has no deterministic outline start`, decaderr.ErrUnsupported)
		}
		outline := make([]topo.CoEdge, 0, len(boundary))
		current := first
		for range len(boundary) {
			use, ok := boundary[current]
			if !ok {
				return nil, fmt.Errorf(`%w: a twisted wall outline is disconnected`, decaderr.ErrUnsupported)
			}
			outline = append(outline, use)
			current = topobuild.CoEdgeEnd(use)
			delete(boundary, topobuild.CoEdgeStart(use))
		}
		if current != first || len(boundary) != 0 {
			return nil, fmt.Errorf(`%w: a twisted wall outline is not one cycle`, decaderr.ErrUnsupported)
		}
		face := topobuild.NewFace(env.Alloc, topo.Face{
			Area:      area,
			AreaBound: areaBound,
			Surface:   surfacegeom.Faceted{},
			Origins:   []provenance.Ref{{Producer: env.Ref, Role: fmt.Sprintf("side(0,0,%d)", j)}},
			Body:      body,
			Loops:     []*topo.Loop{topobuild.NewLoop(env.Alloc, topo.Loop{Outer: true, CoEdges: outline})},
		})
		for k := range held.Walls {
			cell := held.Cell[k]
			if held.Segment[0][cell[1]] == j {
				sources[oldFaces[2+k].Origins[0].Role] = face
			}
		}
		merged = append(merged, face)
	}
	// Detach the old triangle uses before attaching the merged faces. The
	// unused diagonal edges then disappear from Body.Edges traversal.
	for _, edge := range topobuild.Edges(body) {
		edge.Faces = nil
	}
	body.Lumps[0].Shells[0].Faces = merged
	if err := topobuild.AttachFaceLoops(ctx, merged); err != nil {
		return nil, err
	}
	return sources, nil
}

// Eval evaluates a twisted Sweep of profile along path and publishes its body
// under env, never committing it. The body holds a Payload, or a
// CompositePayload for a line-and-arc path.
func Eval(ctx context.Context, env topo.Env, profile profileRecord,
	plane planeRecord, path *sweepinput.Path, angle units.Value, recordArea float64, xform r3.Transform) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if len(path.Records) == 2 {
		return evalComposite(ctx, env, profile, plane, path, angle, recordArea, xform)
	}
	if len(path.Records) != 1 || len(path.Segments) != 1 ||
		plane.Origin != (r3.Vec{}) || plane.U != r3.NewVec(1, 0, 0) || plane.V != r3.NewVec(0, 1, 0) ||
		path.Start != (r3.Vec{}) || path.End.X != 0 || path.End.Y != 0 || path.End.Z <= 0 {
		return nil, fmt.Errorf(`%w: nonzero twist currently needs one positive Z line from an origin XY sketch`, decaderr.ErrUnsupported)
	}
	if path.Segments[0].Arc {
		return nil, fmt.Errorf(`%w: nonzero twist currently needs one straight path span`, decaderr.ErrUnsupported)
	}
	poly, err := TwistPolygonOf(profile)
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
		return nil, fmt.Errorf(`%w: this twist angle has no certified denotation`, decaderr.ErrUnsupported)
	}
	upper := proofbound.RatFloatUp(proofbound.IntervalAbsUpper(angleIv))
	if upper == 0 {
		return nil, fmt.Errorf(`%w: the twisted sweep admits an angle up to one radian`, decaderr.ErrUnsupported)
	}
	if upper > 1 || poly.HasArc {
		return evalStationed(ctx, env, profile, plane, path, angle, recordArea, xform)
	}
	sinIv, cosIv, ok := denoted.SinCosFor(theta)
	if !ok {
		return nil, fmt.Errorf(`%w: the twisted sweep cannot enclose the endpoint rotation`, decaderr.ErrUnsupported)
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
	heldBody, err := loftrecord.Build(ctx, env, loftrecord.Payload{
		Profile0: profile, Profile1: profile, Plane0: plane, Plane1: end,
		Frame0: f0, Frame1: f1, Xform: xform,
		RecordArea: [2]float64{recordArea, recordArea},
	}, proofbound.NewWorkBudget(ctx), work0, work1)
	if err != nil {
		return nil, err
	}
	held, ok := heldBody.Payload.(loftrecord.Payload)
	if !ok {
		return nil, fmt.Errorf(`%w: the twisted sweep lost its held Loft proof`, decaderr.ErrUnsupported)
	}
	sourceRoles, err := mergeTwistWalls(ctx, env, heldBody, held, len(poly.Points))
	if err != nil {
		return nil, err
	}
	idealDelta := twistDeparture(poly, upper, coordError, 0)
	delta := proofbound.AbsSumUpper(idealDelta, held.Delta)
	if proofbound.IsNonFinite(delta) || delta <= 0 {
		return nil, fmt.Errorf(`%w: the twisted boundary has no finite displacement proof`, decaderr.ErrUnsupported)
	}
	areaIv, err := TwistAreaInterval(poly, TwistRat(path.End.Z), angleIv)
	if err != nil {
		return nil, err
	}
	trueArea, trueAreaBound := sweepmitre.Enclosure(areaIv.Lo, areaIv.Hi)
	heldArea := heldBody.Area
	heldAreaIv := proofbound.Interval(
		new(big.Rat).Sub(TwistRat(heldArea.Value.Base()), TwistRat(heldArea.Bound.Base())),
		new(big.Rat).Add(TwistRat(heldArea.Value.Base()), TwistRat(heldArea.Bound.Base())),
	)
	areaGap := proofbound.RatFloatUp(proofbound.IntervalAbsUpper(proofbound.IntervalSub(areaIv, heldAreaIv)))
	areaSlack := proofbound.AbsSumUpper(areaGap, held.Proof.AreaSlack)
	// The horizontal-section tube compares the true twist with the exact
	// unrounded Loft facets. Loft's own occupied-volume proof then covers
	// every held-vertex displacement, including axial placement rounding.
	volSymDiff := proofbound.AbsSumUpper(
		twistSymDiff(path.End.Z, poly.PerimeterUpper, idealDelta),
		held.Proof.VolSymDiff,
	)
	if proofbound.IsNonFinite(trueArea) || proofbound.IsNonFinite(trueAreaBound) ||
		proofbound.IsNonFinite(areaSlack) || proofbound.IsNonFinite(volSymDiff) {
		return nil, fmt.Errorf(`%w: the twisted sweep's proof is not finite`, decaderr.ErrUnsupported)
	}
	volume := new(big.Rat).Mul(poly.Area, TwistRat(path.End.Z))
	vol, volBound := sweepmitre.Enclosure(volume, volume)
	heldBody.Volume = measurement.Measurement{Value: units.CubicMillimeters(vol), Exactness: measurement.ExactnessOf(volBound), Bound: units.CubicMillimeters(volBound)}
	heldBody.Area = measurement.Measurement{Value: units.SquareMillimeters(trueArea), Exactness: measurement.Approximate, Bound: units.SquareMillimeters(trueAreaBound)}
	center := r3.NewVec(0, 0, path.End.Z/2)
	heldCenter := xform.Apply(center)
	centerBound := 0.0
	if xform != r3.Identity() {
		tr := xform.Translation()
		centerBound = proofbound.RigidRoundAllow(path.End.Z/2, max(math.Abs(tr.X), math.Abs(tr.Y), math.Abs(tr.Z)))
	}
	heldBody.Centroid = measurement.VecMeasurement{Value: heldCenter, Exactness: measurement.ExactnessOf(centerBound), Bound: units.Millimeters(centerBound)}
	heldBody.Bounds.Bound = units.Millimeters(proofbound.AbsSumUpper(heldBody.Bounds.Bound.Base(), delta))
	heldBody.Bounds.Exactness = measurement.Approximate
	for _, vertex := range topobuild.Vertices(heldBody) {
		vertex.Bound = units.Millimeters(proofbound.AbsSumUpper(vertex.Bound.Base(), delta))
	}
	for _, edge := range topobuild.Edges(heldBody) {
		// A helix edge is not the chord the held Loft exposes. Its true length
		// is at most h+R|angle|; a cap edge is shorter than the perimeter.
		edge.Curve = surfacegeom.FacetedCurve{Bound: units.Millimeters(delta)}
		edge.LengthBound = proofbound.AbsSumUpper(edge.LengthBound,
			edge.Length, path.End.Z, proofbound.UpRound(poly.MaxRadiusL1*upper), poly.PerimeterUpper)
	}
	for _, face := range topobuild.Faces(heldBody) {
		if face.Origins[0].Role != provenance.RoleCapStart && face.Origins[0].Role != provenance.RoleCapEnd {
			face.Surface = surfacegeom.Faceted{Bound: units.Millimeters(delta)}
		}
		// Each triangular parameter patch is a subset of a wall whose area
		// is at most the body's total area upper bound. The wide face bound
		// is intentional until a triangular-domain quadrature is added.
		face.AreaBound = proofbound.AbsSumUpper(face.AreaBound, face.Area, trueArea, trueAreaBound)
		face.AxialDelta, face.HasAxialDelta = delta, true
	}
	if err := topobuild.ValidateLoftBodyMeasurements(heldBody); err != nil {
		return nil, err
	}
	heldBody.Payload = Payload{
		Profile: profile, Plane: plane, Path: path, Angle: angle,
		RecordArea: recordArea, Xform: xform, Held: held, SourceRoles: sourceRoles,
		Delta: delta, AreaSlack: areaSlack, VolSymDiff: volSymDiff,
	}
	return heldBody, nil
}
