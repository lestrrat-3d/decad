package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

const twistArcCells = 8

type compositeTwistedSweepPayload struct {
	profile    profileRecord
	plane      planeRecord
	path       *Path
	angle      units.Value
	recordArea float64
	xform      r3.Transform
	mesh       *Mesh
	delta      float64
}

func (p compositeTwistedSweepPayload) transform() r3.Transform { return p.xform }

func (p compositeTwistedSweepPayload) placed(ctx context.Context, d *Document,
	ref producerID, xform r3.Transform) (*Body, error) {
	return evalCompositeTwistedSweep(ctx, d, ref, p.profile, p.plane, p.path, p.angle, p.recordArea, xform)
}

type twistCardinalPath struct{ height, radius float64 }

func twistCardinalPathOf(path *Path, plane planeRecord, poly twistPolygon) (twistCardinalPath, error) {
	if path == nil || len(path.records) != 2 ||
		plane.Origin != (r3.Vec{}) || plane.U != r3.NewVec(1, 0, 0) ||
		plane.V != r3.NewVec(0, 1, 0) || path.Start() != (r3.Vec{}) {
		return twistCardinalPath{}, fmt.Errorf(`%w: composite twist needs an origin XY sketch and two path spans`, ErrUnsupported)
	}
	line, arc := path.records[0], path.records[1]
	if line.Arc != nil || arc.Arc == nil || line.End.X != 0 || line.End.Y != 0 || line.End.Z <= 0 ||
		arc.Start != line.End {
		return twistCardinalPath{}, fmt.Errorf(`%w: composite twist needs a positive Z line followed by an arc`, ErrUnsupported)
	}
	r := arc.End.X
	h := line.End.Z
	carrier := arc.Arc
	if r <= 0 || arc.End.Y != 0 || arc.End.Z != h+r ||
		carrier.Center[0].Cmp(twistRat(r)) != 0 || carrier.Center[1].Sign() != 0 ||
		carrier.Center[2].Cmp(twistRat(h)) != 0 ||
		carrier.RadiusStart[0].Cmp(new(big.Rat).Neg(twistRat(r))) != 0 ||
		carrier.RadiusStart[1].Sign() != 0 || carrier.RadiusStart[2].Sign() != 0 ||
		carrier.RadiusEnd[0].Sign() != 0 || carrier.RadiusEnd[1].Sign() != 0 ||
		carrier.RadiusEnd[2].Cmp(twistRat(r)) != 0 ||
		carrier.Axis[0].Sign() != 0 || carrier.Axis[1].Sign() <= 0 || carrier.Axis[2].Sign() != 0 ||
		arc.ArcAngle.Turn == nil || arc.ArcAngle.Turn.Cmp(big.NewRat(1, 4)) != 0 {
		return twistCardinalPath{}, fmt.Errorf(`%w: composite twist needs a positive cardinal quarter arc in XZ`, ErrUnsupported)
	}
	// The profile stays on the inside of the arc's polar radius. This also
	// separates the true arc from the preceding line except at their join.
	if !proofbound.FiniteVec(arc.End) ||
		twistRat(r).Cmp(new(big.Rat).Mul(big.NewRat(8, 1), twistRat(poly.maxRadiusL1))) <= 0 {
		return twistCardinalPath{}, fmt.Errorf(`%w: the twisted profile approaches the arc axis`, ErrUnsupported)
	}
	return twistCardinalPath{height: h, radius: r}, nil
}

type twistStation struct {
	plane      planeRecord
	frame      r3.Frame
	pointError float64
}

func compositeTwistArea(poly twistPolygon, geometry twistCardinalPath,
	joinAngle, fullAngle proofbound.RatInterval) (proofbound.RatInterval, error) {
	straight, err := twistAreaInterval(poly, twistRat(geometry.height), joinAngle)
	if err != nil {
		return proofbound.RatInterval{}, err
	}
	h, radius, r := twistRat(geometry.height), twistRat(geometry.radius), twistRat(poly.maxRadiusL1)
	pathLength := proofbound.IntervalAdd(proofbound.PointInterval(h), proofbound.IntervalScale(proofbound.HalfPiIv, radius))
	arcRate, ok := proofbound.IntervalQuo(proofbound.IntervalScale(fullAngle, radius), pathLength)
	if !ok {
		return proofbound.RatInterval{}, fmt.Errorf(`%w: arc twist rate is unbounded`, ErrUnsupported)
	}
	rate := proofbound.IntervalAbsUpper(arcRate)
	lower := new(big.Rat).Mul(twistRat(poly.perimeterLower),
		new(big.Rat).Mul(new(big.Rat).Sub(radius, r), proofbound.HalfPiIv.Lo))
	upperSpeed := new(big.Rat).Add(new(big.Rat).Add(radius, r), new(big.Rat).Mul(r, rate))
	upper := new(big.Rat).Mul(twistRat(poly.perimeterUpper),
		new(big.Rat).Mul(upperSpeed, proofbound.HalfPiIv.Hi))
	return proofbound.IntervalAdd(straight, proofbound.Interval(lower, upper)), nil
}

// The true boundary and the ideal Loft boundary share a triangle-domain
// parameterization on each profile edge. Linear interpolation between them
// sweeps at most displacement times the product of upper bounds on its two
// surface derivatives. Both end-cap homotopies are charged as well. Loft's
// own proof later covers the ideal-to-stored vertex displacement separately.
func compositeTwistCellProof(poly twistPolygon, geometry twistCardinalPath,
	fullAngle proofbound.RatInterval, start, end twistStation, held loftPayload) (float64, float64, error) {
	dPhi := proofbound.RatFloatUp(new(big.Rat).Quo(proofbound.HalfPiIv.Hi, big.NewRat(twistArcCells, 1)))
	pathLengthLo := new(big.Rat).Add(twistRat(geometry.height),
		new(big.Rat).Mul(twistRat(geometry.radius), proofbound.HalfPiIv.Lo))
	dBetaRat := new(big.Rat).Quo(new(big.Rat).Mul(
		proofbound.IntervalAbsUpper(fullAngle),
		new(big.Rat).Mul(twistRat(geometry.radius),
			new(big.Rat).Quo(proofbound.HalfPiIv.Hi, big.NewRat(twistArcCells, 1)))), pathLengthLo)
	dBeta := proofbound.RatFloatUp(dBetaRat)
	eps := max(start.pointError, end.pointError)
	turn := proofbound.AbsSumUpper(dPhi, dBeta)
	interp := proofbound.AbsSumUpper(
		proofbound.ProductUpper(proofbound.ProductUpper(geometry.radius, proofbound.ProductUpper(dPhi, dPhi)), 0.125),
		proofbound.ProductUpper(proofbound.ProductUpper(poly.maxRadiusL1, proofbound.ProductUpper(turn, turn)), 0.125))
	warp := proofbound.ProductUpper(proofbound.ProductUpper(poly.maxEdgeL1, turn), 0.25)
	idealDelta := proofbound.AbsSumUpper(interp, warp, eps)
	wallEdgeUpper := proofbound.AbsSumUpper(poly.perimeterUpper,
		proofbound.ProductUpper(float64(2*len(poly.points)), eps))
	speedUpper := proofbound.AbsSumUpper(
		proofbound.ProductUpper(proofbound.AbsSumUpper(geometry.radius, poly.maxRadiusL1), dPhi),
		proofbound.ProductUpper(poly.maxRadiusL1, dBeta), proofbound.ProductUpper(2, eps))
	wallAreaUpper := proofbound.ProductUpper(wallEdgeUpper, speedUpper)
	if held.walls != 2*len(poly.points) || len(held.tris) < held.walls+held.capStartCount {
		return 0, 0, fmt.Errorf(`%w: twisted Loft cap cell counts changed`, ErrUnsupported)
	}
	capStart := proofbound.PerturbedAreaUpper(held.verts,
		held.tris[held.walls:held.walls+held.capStartCount],
		proofbound.AbsSumUpper(start.pointError, held.delta))
	capEnd := proofbound.PerturbedAreaUpper(held.verts,
		held.tris[held.walls+held.capStartCount:],
		proofbound.AbsSumUpper(end.pointError, held.delta))
	// The cap homotopy moves only by its station error; the larger wall
	// departure is used for one simple upper bound on the whole cell.
	swept := proofbound.ProductUpper(idealDelta,
		proofbound.AbsSumUpper(wallAreaUpper, capStart, capEnd))
	if proofbound.IsNonFinite(swept) || proofbound.IsNonFinite(idealDelta) {
		return 0, 0, fmt.Errorf(`%w: twisted arc cell has no finite boundary-tube proof`, ErrUnsupported)
	}
	return idealDelta, swept, nil
}

func compositeTwistMesh(ctx context.Context, body *Body, cells []loftPayload,
	profileEdges int, delta, volSymDiff float64) (*Mesh, proofbound.RatInterval, error) {
	if len(cells) < 2 {
		return nil, proofbound.RatInterval{}, fmt.Errorf(`%w: composite twist has too few held cells`, ErrDegenerate)
	}
	byRole := map[string]*Face{}
	for _, face := range body.Faces() {
		for _, origin := range face.origins {
			byRole[origin.Role] = face
		}
	}
	mesh := &Mesh{bound: delta, volSymDiff: volSymDiff, symDiffOK: true}
	spanOf := make([]int, 0, len(cells)*(2*profileEdges+4))
	for cell, held := range cells {
		if err := ctx.Err(); err != nil {
			return nil, proofbound.RatInterval{}, err
		}
		if len(held.verts) != 2*profileEdges || held.walls != 2*profileEdges ||
			len(held.tris) <= held.walls+held.capStartCount {
			return nil, proofbound.RatInterval{}, fmt.Errorf(`%w: twisted Loft has an unexpected station table`, ErrUnsupported)
		}
		if cell == 0 {
			mesh.vertices = append(mesh.vertices, held.verts[:profileEdges]...)
		} else {
			for j := range profileEdges {
				if mesh.vertices[cell*profileEdges+j] != held.verts[j] {
					return nil, proofbound.RatInterval{}, fmt.Errorf(`%w: twisted Loft cells disagree at a shared station`, ErrUnsupported)
				}
			}
		}
		mesh.vertices = append(mesh.vertices, held.verts[profileEdges:]...)
		mesh.coordBound = max(mesh.coordBound, held.delta)
		for t, triangle := range held.tris {
			if t < held.walls && (t >= len(held.cell) ||
				held.cell[t][0] != 0 || held.cell[t][1] != t/2) {
				return nil, proofbound.RatInterval{}, fmt.Errorf(`%w: twisted Loft wall order lost its profile provenance`, ErrUnsupported)
			}
			if t >= held.walls && t < held.walls+held.capStartCount && cell != 0 ||
				t >= held.walls+held.capStartCount && cell != len(cells)-1 {
				continue
			}
			role := roleCapStart
			if t < held.walls {
				pathSpan := 1
				if cell == 0 {
					pathSpan = 0
				}
				role = fmt.Sprintf("side(%d,0,%d)", pathSpan, t/2)
			} else if t >= held.walls+held.capStartCount {
				role = roleCapEnd
			}
			face := byRole[role]
			if face == nil {
				return nil, proofbound.RatInterval{}, fmt.Errorf(`%w: twisted Loft mesh has no live face for %s`, ErrDegenerate, role)
			}
			mapped := [3]int{}
			for i, local := range triangle {
				if local < 0 || local >= 2*profileEdges {
					return nil, proofbound.RatInterval{}, fmt.Errorf(`%w: twisted Loft triangle has an invalid station`, ErrDegenerate)
				}
				mapped[i] = cell*profileEdges + local
			}
			mesh.addTriangle(mapped, face)
			mesh.setFaceBound(face, delta)
			spanOf = append(spanOf, cell)
		}
	}
	if err := requireMeshAudit(ctx, false, body, mesh); err != nil {
		return nil, proofbound.RatInterval{}, err
	}
	if err := liftTessellationError(tessellation.RequireVertexLinks(ctx, len(mesh.vertices), mesh.triangles)); err != nil {
		return nil, proofbound.RatInterval{}, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	auditTris, err := revolvemesh.RequireRevolveFacetAreas(budget, mesh.vertices, mesh.triangles, mesh.coordBound)
	if err != nil {
		return nil, proofbound.RatInterval{}, err
	}
	if err := compositeCrossSpanContactAudit(budget, auditTris, mesh.triangles, spanOf, mesh.coordBound); err != nil {
		return nil, proofbound.RatInterval{}, err
	}
	if tessellation.OrientationSign(mesh.vertices, mesh.triangles, r3.Vec{}) <= 0 {
		return nil, proofbound.RatInterval{}, fmt.Errorf(`%w: twisted Loft mesh has nonpositive orientation`, ErrUnsupported)
	}
	area := proofbound.PointInterval(new(big.Rat))
	for _, triangle := range mesh.triangles {
		var points [3]proofbound.IvVec3
		for i, index := range triangle {
			v := mesh.vertices[index]
			points[i] = proofbound.IvVec3{
				proofbound.PointInterval(twistRat(v.X)),
				proofbound.PointInterval(twistRat(v.Y)),
				proofbound.PointInterval(twistRat(v.Z)),
			}
		}
		twice, ok := revolvemesh.IvTwoTriangleArea(points[0], points[1], points[2])
		if !ok {
			return nil, proofbound.RatInterval{}, fmt.Errorf(`%w: twisted Loft facet has no area enclosure`, ErrUnsupported)
		}
		area = proofbound.IntervalAdd(area, proofbound.IntervalScale(twice, big.NewRat(1, 2)))
	}
	return mesh, area, nil
}

func evalCompositeTwistedSweep(ctx context.Context, d *Document, ref producerID,
	profile profileRecord, plane planeRecord, path *Path, angle units.Value,
	recordArea float64, xform r3.Transform) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	poly, err := twistPolygonOf(profile)
	if err != nil {
		return nil, err
	}
	geometry, err := twistCardinalPathOf(path, plane, poly)
	if err != nil {
		return nil, err
	}
	theta, err := angle.In(units.Radian)
	if err != nil {
		return nil, err
	}
	thetaIv, ok := revolveangle.FromValue(angle).Enclosure()
	if !ok || thetaIv.Lo.Sign() <= 0 && thetaIv.Hi.Sign() >= 0 ||
		proofbound.RatFloatUp(proofbound.IntervalAbsUpper(thetaIv)) > 1 {
		return nil, fmt.Errorf(`%w: composite twist needs a nonzero certified angle up to one radian`, ErrUnsupported)
	}
	stations, joinAngle, err := compositeTwistStations(thetaIv, theta, geometry, poly)
	if err != nil {
		return nil, err
	}
	parts := make([]compositeSpanPart, len(stations)-1)
	held := make([]loftPayload, len(parts))
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	budget := proofbound.NewWorkBudget(ctx)
	for i := range parts {
		part, buildErr := evalLoft(ctx, d, ref, loftPayload{
			profile0: profile, profile1: profile,
			plane0: stations[i].plane, plane1: stations[i+1].plane,
			frame0: stations[i].frame, frame1: stations[i+1].frame,
			xform: xform, recordArea: [2]float64{recordArea, recordArea},
		}, budget, work0, work1)
		if buildErr != nil {
			return nil, fmt.Errorf("twisted sweep cell %d: %w", i, buildErr)
		}
		var heldOK bool
		held[i], heldOK = part.payload.(loftPayload)
		if !heldOK {
			return nil, fmt.Errorf(`%w: a twisted sweep cell lost its Loft payload`, ErrUnsupported)
		}
		if _, err := mergeTwistWalls(ctx, part, ref, len(poly.points)); err != nil {
			return nil, fmt.Errorf("twisted sweep cell %d: %w", i, err)
		}
		parts[i], err = compositeSpanPartOf(part)
		if err != nil {
			return nil, fmt.Errorf("twisted sweep cell %d: %w", i, err)
		}
	}
	body, err := assembleCompositeSweepBodyWithJoin(ctx, d, ref, parts, false,
		func(ctx context.Context, spans []sweepAuditSpan) error {
			if err := ctx.Err(); err != nil {
				return err
			}
			if len(spans) != len(held) {
				return fmt.Errorf(`%w: twisted sweep span audit count changed`, ErrDegenerate)
			}
			for i, span := range spans {
				if span.body == nil || span.body.payload == nil ||
					span.startCap == nil || span.endCap == nil || len(held[i].verts) != 2*len(poly.points) {
					return fmt.Errorf(`%w: twisted sweep cell %d has incomplete Loft geometry`, ErrUnsupported, i)
				}
			}
			return nil
		}, sewTwistLoftJoin)
	if err != nil {
		return nil, err
	}
	if _, err := mergeTwistCellFaces(ctx, body, ref, len(poly.points),
		[]twistCellGroup{{from: 0, through: 1, pathSpan: 0},
			{from: 1, through: len(held), pathSpan: 1}}); err != nil {
		return nil, err
	}
	upperTheta := proofbound.RatFloatUp(proofbound.IntervalAbsUpper(thetaIv))
	upperJoin := proofbound.RatFloatUp(proofbound.IntervalAbsUpper(joinAngle))
	lineEps := max(stations[0].pointError, stations[1].pointError)
	lineIdeal := proofbound.AbsSumUpper(
		proofbound.ProductUpper(proofbound.ProductUpper(poly.maxRadiusL1,
			proofbound.ProductUpper(upperJoin, upperJoin)), 0.125),
		proofbound.ProductUpper(proofbound.ProductUpper(poly.maxEdgeL1, upperJoin), 0.25),
		lineEps)
	delta := proofbound.AbsSumUpper(lineIdeal, held[0].delta)
	volSymDiff := proofbound.AbsSumUpper(
		twistSymDiff(geometry.height, poly.perimeterUpper, lineIdeal), held[0].proof.VolSymDiff)
	for i := 1; i < len(held); i++ {
		ideal, swept, proofErr := compositeTwistCellProof(poly, geometry, thetaIv,
			stations[i], stations[i+1], held[i])
		if proofErr != nil {
			return nil, fmt.Errorf("twisted sweep cell %d: %w", i, proofErr)
		}
		delta = max(delta, proofbound.AbsSumUpper(ideal, held[i].delta))
		volSymDiff = proofbound.AbsSumUpper(volSymDiff, swept, held[i].proof.VolSymDiff)
	}
	if proofbound.IsNonFinite(delta) || proofbound.IsNonFinite(volSymDiff) {
		return nil, fmt.Errorf(`%w: twisted sweep has no finite boundary proof`, ErrUnsupported)
	}
	length := proofbound.IntervalAdd(proofbound.PointInterval(twistRat(geometry.height)),
		proofbound.IntervalScale(proofbound.HalfPiIv, twistRat(geometry.radius)))
	volumeIv := proofbound.IntervalScale(length, poly.area)
	// This gate rejects a certificate that merely permits the whole true
	// body to be missing. The admitted mesh must have a local, useful bound.
	if twistRat(volSymDiff).Cmp(volumeIv.Lo) >= 0 {
		return nil, fmt.Errorf(`%w: twisted sweep boundary tube exceeds the true occupied volume`, ErrUnsupported)
	}
	mesh, heldAreaIv, err := compositeTwistMesh(ctx, body, held, len(poly.points), delta, volSymDiff)
	if err != nil {
		return nil, err
	}
	trueAreaIv, err := compositeTwistArea(poly, geometry, joinAngle, thetaIv)
	if err != nil {
		return nil, err
	}
	mesh.areaSlack = proofbound.RatFloatUp(proofbound.IntervalAbsUpper(
		proofbound.IntervalSub(trueAreaIv, heldAreaIv)))
	if proofbound.IsNonFinite(mesh.areaSlack) {
		return nil, fmt.Errorf(`%w: twisted sweep area proof is not finite`, ErrUnsupported)
	}
	volume, volumeBound := mitredEnclosure(volumeIv.Lo, volumeIv.Hi)
	area, areaBound := mitredEnclosure(trueAreaIv.Lo, trueAreaIv.Hi)
	body.volume = Measurement{Value: units.CubicMillimeters(volume), Bound: units.CubicMillimeters(volumeBound),
		Exactness: exactnessOf(volumeBound)}
	body.area = Measurement{Value: units.SquareMillimeters(area), Bound: units.SquareMillimeters(areaBound),
		Exactness: exactnessOf(areaBound)}
	if err := publishCompositeTwistCentroid(body, xform, geometry, poly, length); err != nil {
		return nil, err
	}
	minV, maxV := mesh.vertices[0], mesh.vertices[0]
	for _, vertex := range mesh.vertices[1:] {
		minV.X, minV.Y, minV.Z = min(minV.X, vertex.X), min(minV.Y, vertex.Y), min(minV.Z, vertex.Z)
		maxV.X, maxV.Y, maxV.Z = max(maxV.X, vertex.X), max(maxV.Y, vertex.Y), max(maxV.Z, vertex.Z)
	}
	body.bounds = Box{Min: minV, Max: maxV, Bound: units.Millimeters(delta), Exactness: Approximate}
	for _, vertex := range body.Vertices() {
		vertex.bound = units.Millimeters(proofbound.AbsSumUpper(vertex.bound.Base(), delta))
	}
	longitudinalExtra := proofbound.ProductUpper(poly.maxRadiusL1,
		proofbound.AbsSumUpper(proofbound.RatFloatUp(proofbound.HalfPiIv.Hi), upperTheta))
	for _, edge := range body.Edges() {
		edge.curve = FacetedCurve{Bound: units.Millimeters(delta)}
		edge.lengthBound = proofbound.AbsSumUpper(edge.lengthBound, edge.length,
			proofbound.RatFloatUp(length.Hi), longitudinalExtra, poly.perimeterUpper)
	}
	for _, face := range body.Faces() {
		if face.origins[0].Role != roleCapStart && face.origins[0].Role != roleCapEnd {
			face.surface = Faceted{Bound: units.Millimeters(delta)}
		}
		face.areaBound = proofbound.AbsSumUpper(face.areaBound, face.area, area, areaBound)
		face.axialDelta, face.hasAxialDelta = delta, true
	}
	if err := validateAnalyticBodyMeasurements(body); err != nil {
		return nil, err
	}
	body.payload = compositeTwistedSweepPayload{
		profile: profile, plane: plane, path: path, angle: angle,
		recordArea: recordArea, xform: xform, mesh: mesh, delta: delta,
	}
	return body, nil
}

func publishCompositeTwistCentroid(body *Body, xform r3.Transform,
	geometry twistCardinalPath, poly twistPolygon, length proofbound.RatInterval) error {
	h, radius := twistRat(geometry.height), twistRat(geometry.radius)
	xNum := proofbound.IntervalScale(proofbound.IntervalSub(proofbound.HalfPiIv,
		proofbound.PointInterval(big.NewRat(1, 1))), new(big.Rat).Mul(radius, radius))
	zNum := proofbound.IntervalAdd(proofbound.PointInterval(new(big.Rat).Add(
		new(big.Rat).Quo(new(big.Rat).Mul(h, h), big.NewRat(2, 1)),
		new(big.Rat).Mul(radius, radius))),
		proofbound.IntervalScale(proofbound.HalfPiIv, new(big.Rat).Mul(h, radius)))
	xIv, xOK := proofbound.IntervalQuo(xNum, length)
	zIv, zOK := proofbound.IntervalQuo(zNum, length)
	if !xOK || !zOK {
		return fmt.Errorf(`%w: twisted sweep centroid has no path moment`, ErrUnsupported)
	}
	x, xb := mitredEnclosure(xIv.Lo, xIv.Hi)
	z, zb := mitredEnclosure(zIv.Lo, zIv.Hi)
	// Curvature weights each section by 1-a/R. The exact profile centroid
	// is at the path axis, so only its bounded second moment can shift this.
	correction := proofbound.RatFloatUp(new(big.Rat).Quo(
		new(big.Rat).Mul(twistRat(poly.maxRadiusL1), twistRat(poly.maxRadiusL1)),
		twistRat(geometry.radius)))
	local := r3.NewVec(x, 0, z)
	world := xform.Apply(local)
	translation := xform.Translation()
	round := proofbound.RigidRoundAllow(max(math.Abs(x), math.Abs(z)),
		max(math.Abs(translation.X), math.Abs(translation.Y), math.Abs(translation.Z)))
	bound := proofbound.AbsSumUpper(xb, zb, correction, round)
	if proofbound.IsNonFinite(bound) || !proofbound.FiniteVec(world) {
		return fmt.Errorf(`%w: twisted sweep centroid has no finite bound`, ErrUnsupported)
	}
	body.centroid = VecMeasurement{Value: world, Bound: units.Millimeters(bound), Exactness: exactnessOf(bound)}
	return nil
}

func compositeTwistStations(theta proofbound.RatInterval, heldTheta float64,
	geometry twistCardinalPath, poly twistPolygon) ([]twistStation, proofbound.RatInterval, error) {
	h, radius := twistRat(geometry.height), twistRat(geometry.radius)
	length := proofbound.IntervalAdd(proofbound.PointInterval(h), proofbound.IntervalScale(proofbound.HalfPiIv, radius))
	if length.Lo.Sign() <= 0 {
		return nil, proofbound.RatInterval{}, fmt.Errorf(`%w: twisted path length has no positive lower bound`, ErrUnsupported)
	}
	joinFraction, ok := proofbound.IntervalQuo(proofbound.PointInterval(h), length)
	if !ok {
		return nil, proofbound.RatInterval{}, fmt.Errorf(`%w: twisted path join has no parameter bound`, ErrUnsupported)
	}
	joinAngle := proofbound.IntervalMul(theta, joinFraction)
	stations := make([]twistStation, twistArcCells+2)
	arcLength := geometry.radius * math.Pi / 2
	for index := range stations {
		var phiIv proofbound.RatInterval
		var phiHeld float64
		if index <= 1 {
			phiIv = proofbound.PointInterval(new(big.Rat))
		} else {
			fraction := big.NewRat(int64(index-1), twistArcCells)
			phiIv = proofbound.IntervalScale(proofbound.HalfPiIv, fraction)
			phiHeld = math.Pi / 2 * float64(index-1) / twistArcCells
		}
		pathFraction, ok := proofbound.IntervalQuo(proofbound.IntervalAdd(
			proofbound.PointInterval(h), proofbound.IntervalScale(phiIv, radius)), length)
		if !ok {
			return nil, proofbound.RatInterval{}, fmt.Errorf(`%w: twist station has no path fraction`, ErrUnsupported)
		}
		betaIv := proofbound.IntervalMul(theta, pathFraction)
		betaHeld := heldTheta * (geometry.height + geometry.radius*phiHeld) / (geometry.height + arcLength)
		if index == 0 {
			betaIv = proofbound.PointInterval(new(big.Rat))
			betaHeld = 0
		}
		sPhi, cPhi, okPhi := proofbound.RadSinCosSpan(phiIv)
		sBeta, cBeta, okBeta := proofbound.RadSinCosSpan(betaIv)
		if !okPhi || !okBeta {
			return nil, proofbound.RatInterval{}, fmt.Errorf(`%w: twist station has no trigonometric enclosure`, ErrUnsupported)
		}
		sph, cph := math.Sincos(phiHeld)
		sbe, cbe := math.Sincos(betaHeld)
		origin := r3.NewVec(geometry.radius*(1-cph), 0, geometry.height+geometry.radius*sph)
		if index == 0 {
			origin = r3.Vec{}
		}
		u := r3.NewVec(cph*cbe, sbe, -sph*cbe)
		v := r3.NewVec(-cph*sbe, cbe, sph*sbe)
		frame, err := r3.NewFrame(origin, u, v)
		if err != nil {
			return nil, proofbound.RatInterval{}, fmt.Errorf(`%w: twist station has no held frame: %s`, ErrUnsupported, err)
		}
		if index == 0 {
			frame, err = r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
			if err != nil {
				return nil, proofbound.RatInterval{}, err
			}
		}
		trueOriginX := proofbound.IntervalScale(proofbound.IntervalSub(
			proofbound.PointInterval(big.NewRat(1, 1)), cPhi), radius)
		trueOriginZ := proofbound.IntervalAdd(proofbound.PointInterval(h), proofbound.IntervalScale(sPhi, radius))
		if index == 0 {
			trueOriginZ = proofbound.PointInterval(new(big.Rat))
		}
		originError := proofbound.AbsSumUpper(
			proofbound.IntervalFloatError(trueOriginX, frame.Origin().X),
			proofbound.IntervalFloatError(trueOriginZ, frame.Origin().Z))
		idealU := [3]proofbound.RatInterval{
			proofbound.IntervalMul(cPhi, cBeta), sBeta,
			proofbound.IntervalNeg(proofbound.IntervalMul(sPhi, cBeta)),
		}
		idealV := [3]proofbound.RatInterval{
			proofbound.IntervalNeg(proofbound.IntervalMul(cPhi, sBeta)), cBeta,
			proofbound.IntervalMul(sPhi, sBeta),
		}
		actualU, actualV := frame.U(), frame.V()
		frameError := 0.0
		for coordinate, actual := range [2][3]float64{{actualU.X, actualU.Y, actualU.Z},
			{actualV.X, actualV.Y, actualV.Z}} {
			ideal := idealU
			if coordinate == 1 {
				ideal = idealV
			}
			for axis, value := range actual {
				frameError = max(frameError, proofbound.IntervalFloatError(ideal[axis], value))
			}
		}
		pointError := proofbound.AbsSumUpper(originError,
			proofbound.ProductUpper(3*poly.maxRadiusL1, frameError))
		if !proofbound.FiniteVec(frame.Origin()) || proofbound.IsNonFinite(pointError) {
			return nil, proofbound.RatInterval{}, fmt.Errorf(`%w: twist station has no finite displacement`, ErrUnsupported)
		}
		stations[index] = twistStation{
			plane: planeRecord{Origin: frame.Origin(), U: frame.U(), V: frame.V()},
			frame: frame, pointError: pointError,
		}
	}
	return stations, joinAngle, nil
}

// A Loft's start and end cap loops walk in opposite directions. Match the
// recorded station coordinates at a join instead of pairing coedge positions.
// Both adjoining Lofts use the same held frame, so equality is exact here.
func sewTwistLoftJoin(ctx context.Context, from, to *Face, previous *Body, sewn map[*Edge]struct{}) error {
	if len(from.loops) != 1 || len(to.loops) != 1 {
		return fmt.Errorf(`%w: a twisted Loft join needs one section loop`, ErrUnsupported)
	}
	for _, old := range from.loops[0].coedges {
		if err := ctx.Err(); err != nil {
			return err
		}
		var shared coedge
		matches := 0
		for _, candidate := range to.loops[0].coedges {
			if old.Start().position == candidate.Start().position &&
				old.End().position == candidate.End().position ||
				old.Start().position == candidate.End().position &&
					old.End().position == candidate.Start().position {
				shared = candidate
				matches++
			}
		}
		if matches != 1 {
			return fmt.Errorf(`%w: a twisted Loft join has %d matching rim edges`, ErrUnsupported, matches)
		}
		var start, end *Vertex
		if old.Start().position == shared.Start().position {
			start, end = shared.Start(), shared.End()
		} else {
			start, end = shared.End(), shared.Start()
		}
		rewriteCompositeVertex(previous, old.Start(), start)
		rewriteCompositeVertex(previous, old.End(), end)
		discarded := old.edge
		rewriteCompositeEdge(previous, discarded, shared.edge)
		shared.edge.faces = append(shared.edge.faces, discarded.faces...)
		sewn[shared.edge] = struct{}{}
	}
	return nil
}

type twistCellGroup struct{ from, through, pathSpan int }

// Each Loft cell contributes one quad for each profile edge. Internal section
// edges cancel in pairs, leaving one public wall patch per input path span
// and profile edge, as Sweep's source-role contract requires.
func mergeTwistCellFaces(ctx context.Context, body *Body, ref producerID,
	profileEdges int, groups []twistCellGroup) (map[*Face]*Face, error) {
	old := body.Faces()
	cells := 0
	for _, group := range groups {
		if group.from != cells || group.through <= group.from {
			return nil, fmt.Errorf(`%w: twisted Loft cell groups are not consecutive`, ErrDegenerate)
		}
		cells = group.through
	}
	if len(old) != 2+cells*profileEdges {
		return nil, fmt.Errorf(`%w: twisted Loft cells have unexpected wall topology`, ErrUnsupported)
	}
	merged := []*Face{old[0]}
	redirect := make(map[*Face]*Face, cells*profileEdges)
	for _, group := range groups {
		for j := range profileEdges {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			uses := map[*Edge][]coedge{}
			area, areaBound := 0.0, 0.0
			for cell := group.from; cell < group.through; cell++ {
				face := old[1+cell*profileEdges+j]
				if len(face.loops) != 1 {
					return nil, fmt.Errorf(`%w: a twisted Loft wall has multiple loops`, ErrUnsupported)
				}
				for _, use := range face.loops[0].coedges {
					uses[use.edge] = append(uses[use.edge], use)
				}
				area = proofbound.AbsSumUpper(area, face.area)
				areaBound = proofbound.AbsSumUpper(areaBound, face.areaBound)
			}
			boundary := make(map[*Vertex]coedge)
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
						return nil, fmt.Errorf(`%w: twisted wall cells use a seam in the same direction`, ErrUnsupported)
					}
				default:
					return nil, fmt.Errorf(`%w: a twisted wall edge has an unsupported use count`, ErrUnsupported)
				}
			}
			if len(boundary) < 4 {
				return nil, fmt.Errorf(`%w: a twisted wall has no closed boundary`, ErrUnsupported)
			}
			var first *Vertex
			firstCell := old[1+group.from*profileEdges+j]
			for _, use := range firstCell.loops[0].coedges {
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
				origins: []FeatureRef{{producer: ref, Role: fmt.Sprintf("side(%d,0,%d)", group.pathSpan, j)}},
				body:    body, area: area, areaBound: areaBound,
			}
			for cell := group.from; cell < group.through; cell++ {
				redirect[old[1+cell*profileEdges+j]] = face
			}
			merged = append(merged, face)
		}
	}
	merged = append(merged, old[len(old)-1])
	for _, edge := range body.Edges() {
		edge.faces = nil
	}
	body.lumps[0].shells[0].faces = merged
	if err := attachFaceLoopsContext(ctx, merged); err != nil {
		return nil, err
	}
	if err := auditCompositeBoundary(ctx, body, nil); err != nil {
		return nil, err
	}
	return redirect, nil
}
