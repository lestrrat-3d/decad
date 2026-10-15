package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/twistsweep"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type compositeTwistedSweepPayload struct {
	profile    profileRecord
	plane      planeRecord
	path       *sweepinput.Path
	angle      units.Value
	recordArea float64
	xform      r3.Transform
	mesh       *Mesh
	delta      float64
}

func (p compositeTwistedSweepPayload) Transform() r3.Transform { return p.xform }

func (p compositeTwistedSweepPayload) Placed(ctx context.Context, env topo.Env, xform r3.Transform) (*topo.Body, error) {
	d, ref := documentHandle(env.Doc), env.Ref
	return placedNode(evalCompositeTwistedSweep(ctx, d, ref, p.profile, p.plane, p.path, p.angle, p.recordArea, xform))
}

// The true boundary and the ideal Loft boundary share a triangle-domain
// parameterization on each profile edge. Linear interpolation between them
// sweeps at most displacement times the product of upper bounds on its two
// surface derivatives. Both end-cap homotopies are charged as well. Loft's
// own proof later covers the ideal-to-stored vertex displacement separately.
func compositeTwistCellProof(poly twistsweep.TwistPolygon, geometry twistsweep.TwistCardinalPath,
	fullAngle proofbound.RatInterval, start, end twistsweep.TwistStation, held loftPayload) (float64, float64, error) {
	dPhi := proofbound.RatFloatUp(new(big.Rat).Quo(proofbound.HalfPiIv.Hi, big.NewRat(twistsweep.TwistArcCells, 1)))
	pathLengthLo := new(big.Rat).Add(twistsweep.TwistRat(geometry.Height),
		new(big.Rat).Mul(twistsweep.TwistRat(geometry.Radius), proofbound.HalfPiIv.Lo))
	dBetaRat := new(big.Rat).Quo(new(big.Rat).Mul(
		proofbound.IntervalAbsUpper(fullAngle),
		new(big.Rat).Mul(twistsweep.TwistRat(geometry.Radius),
			new(big.Rat).Quo(proofbound.HalfPiIv.Hi, big.NewRat(twistsweep.TwistArcCells, 1)))), pathLengthLo)
	dBeta := proofbound.RatFloatUp(dBetaRat)
	eps := max(start.PointError, end.PointError)
	turn := proofbound.AbsSumUpper(dPhi, dBeta)
	interp := proofbound.AbsSumUpper(
		proofbound.ProductUpper(proofbound.ProductUpper(geometry.Radius, proofbound.ProductUpper(dPhi, dPhi)), 0.125),
		proofbound.ProductUpper(proofbound.ProductUpper(poly.MaxRadiusL1, proofbound.ProductUpper(turn, turn)), 0.125))
	warp := proofbound.ProductUpper(proofbound.ProductUpper(poly.MaxEdgeL1, turn), 0.25)
	idealDelta := proofbound.AbsSumUpper(interp, warp, eps)
	wallEdgeUpper := proofbound.AbsSumUpper(poly.PerimeterUpper,
		proofbound.ProductUpper(float64(2*len(poly.Points)), eps))
	speedUpper := proofbound.AbsSumUpper(
		proofbound.ProductUpper(proofbound.AbsSumUpper(geometry.Radius, poly.MaxRadiusL1), dPhi),
		proofbound.ProductUpper(poly.MaxRadiusL1, dBeta), proofbound.ProductUpper(2, eps))
	wallAreaUpper := proofbound.ProductUpper(wallEdgeUpper, speedUpper)
	if held.walls != 2*len(poly.Points) || len(held.tris) < held.walls+held.capStartCount {
		return 0, 0, fmt.Errorf(`%w: twisted Loft cap cell counts changed`, ErrUnsupported)
	}
	capStart := proofbound.PerturbedAreaUpper(held.verts,
		held.tris[held.walls:held.walls+held.capStartCount],
		proofbound.AbsSumUpper(start.PointError, held.delta))
	capEnd := proofbound.PerturbedAreaUpper(held.verts,
		held.tris[held.walls+held.capStartCount:],
		proofbound.AbsSumUpper(end.PointError, held.delta))
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
	profileEdges int, pathSpanOfCell []int, delta, volSymDiff float64) (*Mesh, proofbound.RatInterval,
	map[*topo.Face]proofbound.RatInterval, error) {
	if len(cells) < 2 {
		return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: composite twist has too few held cells`, ErrDegenerate)
	}
	if len(pathSpanOfCell) != len(cells) {
		return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft cells have no path-span map`, ErrDegenerate)
	}
	byRole := map[string]*Face{}
	for _, face := range body.Faces() {
		for _, origin := range face.n.Origins {
			byRole[origin.Role] = face
		}
	}
	mesh := bornMesh(&Mesh{n: topo.Mesh{Bound: delta, VolSymDiff: volSymDiff, SymDiffOK: true}})
	stationPoints := len(cells[0].verts) / 2
	if stationPoints < profileEdges || len(cells[0].verts) != 2*stationPoints {
		return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft has no complete station table`, ErrUnsupported)
	}
	spanOf := make([]int, 0, len(cells)*(2*stationPoints+4))
	for cell, held := range cells {
		if err := ctx.Err(); err != nil {
			return nil, proofbound.RatInterval{}, nil, err
		}
		if len(held.verts) != 2*stationPoints || held.walls != 2*stationPoints ||
			len(held.segment) != 1 || len(held.segment[0]) != stationPoints ||
			len(held.tris) <= held.walls+held.capStartCount {
			return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft has an unexpected station table`, ErrUnsupported)
		}
		if cell == 0 {
			mesh.n.Vertices = append(mesh.n.Vertices, held.verts[:stationPoints]...)
		} else {
			for j := range stationPoints {
				if mesh.n.Vertices[cell*stationPoints+j] != held.verts[j] {
					return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft cells disagree at a shared station`, ErrUnsupported)
				}
			}
		}
		mesh.n.Vertices = append(mesh.n.Vertices, held.verts[stationPoints:]...)
		mesh.n.CoordBound = max(mesh.n.CoordBound, held.delta)
		for t, triangle := range held.tris {
			if t < held.walls && (t >= len(held.cell) ||
				held.cell[t][0] != 0 || held.cell[t][1] != t/2) {
				return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft wall order lost its profile provenance`, ErrUnsupported)
			}
			if t >= held.walls && t < held.walls+held.capStartCount && cell != 0 ||
				t >= held.walls+held.capStartCount && cell != len(cells)-1 {
				continue
			}
			role := provenance.RoleCapStart
			if t < held.walls {
				segment := held.segment[0][t/2]
				if segment < 0 || segment >= profileEdges {
					return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft lost its source segment`, ErrUnsupported)
				}
				role = fmt.Sprintf("side(%d,0,%d)", pathSpanOfCell[cell], segment)
			} else if t >= held.walls+held.capStartCount {
				role = provenance.RoleCapEnd
			}
			face := byRole[role]
			if face == nil {
				return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft mesh has no live face for %s`, ErrDegenerate, role)
			}
			mapped := [3]int{}
			for i, local := range triangle {
				if local < 0 || local >= 2*stationPoints {
					return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft triangle has an invalid station`, ErrDegenerate)
				}
				mapped[i] = cell*stationPoints + local
			}
			mesh.addTriangle(mapped, faceNode(face))
			mesh.setFaceBound(faceNode(face), delta)
			spanOf = append(spanOf, cell)
		}
	}
	if err := requireMeshAudit(ctx, false, body, mesh); err != nil {
		return nil, proofbound.RatInterval{}, nil, err
	}
	if err := liftTessellationError(tessellation.RequireVertexLinks(ctx, len(mesh.n.Vertices), mesh.n.Triangles)); err != nil {
		return nil, proofbound.RatInterval{}, nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	auditTris, err := revolvemesh.RequireRevolveFacetAreas(budget, mesh.n.Vertices, mesh.n.Triangles, mesh.n.CoordBound)
	if err != nil {
		return nil, proofbound.RatInterval{}, nil, err
	}
	if err := compositesweep.CrossSpanContactAudit(budget, auditTris, mesh.n.Triangles, spanOf, mesh.n.CoordBound); err != nil {
		return nil, proofbound.RatInterval{}, nil, err
	}
	if tessellation.OrientationSign(mesh.n.Vertices, mesh.n.Triangles, r3.Vec{}) <= 0 {
		return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft mesh has nonpositive orientation`, ErrUnsupported)
	}
	area := proofbound.PointInterval(new(big.Rat))
	byFace := make(map[*topo.Face]proofbound.RatInterval, len(body.Faces()))
	for index, triangle := range mesh.n.Triangles {
		var points [3]proofbound.IvVec3
		for i, index := range triangle {
			v := mesh.n.Vertices[index]
			points[i] = proofbound.IvVec3{
				proofbound.PointInterval(twistsweep.TwistRat(v.X)),
				proofbound.PointInterval(twistsweep.TwistRat(v.Y)),
				proofbound.PointInterval(twistsweep.TwistRat(v.Z)),
			}
		}
		twice, ok := revolvemesh.IvTwoTriangleArea(points[0], points[1], points[2])
		if !ok {
			return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft facet has no area enclosure`, ErrUnsupported)
		}
		facet := proofbound.IntervalScale(twice, big.NewRat(1, 2))
		area = proofbound.IntervalAdd(area, facet)
		face := mesh.n.Source[index]
		if previous, exists := byFace[face]; exists {
			byFace[face] = proofbound.IntervalAdd(previous, facet)
		} else {
			byFace[face] = facet
		}
	}
	return mesh, area, byFace, nil
}

func evalCompositeTwistedSweep(ctx context.Context, d *Document, ref producerID,
	profile profileRecord, plane planeRecord, path *sweepinput.Path, angle units.Value,
	recordArea float64, xform r3.Transform) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	poly, err := twistsweep.TwistPolygonOf(profile)
	if err != nil {
		return nil, err
	}
	if poly.HasArc {
		return nil, fmt.Errorf(`%w: a composite twisted sweep needs straight profile edges`, ErrUnsupported)
	}
	geometry, err := twistsweep.TwistCardinalPathOf(path, plane, poly)
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
	stations, joinAngle, err := twistsweep.CompositeTwistStations(thetaIv, theta, geometry, poly)
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
			plane0: stations[i].Plane, plane1: stations[i+1].Plane,
			frame0: stations[i].Frame, frame1: stations[i+1].Frame,
			xform: xform, recordArea: [2]float64{recordArea, recordArea},
		}, budget, work0, work1)
		if buildErr != nil {
			return nil, fmt.Errorf("twisted sweep cell %d: %w", i, buildErr)
		}
		var heldOK bool
		held[i], heldOK = part.n.Payload.(loftPayload)
		if !heldOK {
			return nil, fmt.Errorf(`%w: a twisted sweep cell lost its Loft payload`, ErrUnsupported)
		}
		if _, err := mergeTwistWalls(ctx, part, ref, held[i], len(poly.Points)); err != nil {
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
				if span.body == nil || span.body.n.Payload == nil ||
					span.startCap == nil || span.endCap == nil || len(held[i].verts) != 2*len(poly.Points) {
					return fmt.Errorf(`%w: twisted sweep cell %d has incomplete Loft geometry`, ErrUnsupported, i)
				}
			}
			return nil
		}, sewTwistLoftJoin)
	if err != nil {
		return nil, err
	}
	if err := mergeTwistCellFaces(ctx, body, ref, len(poly.Points),
		[]twistCellGroup{{from: 0, through: 1, pathSpan: 0},
			{from: 1, through: len(held), pathSpan: 1}}); err != nil {
		return nil, err
	}
	upperTheta := proofbound.RatFloatUp(proofbound.IntervalAbsUpper(thetaIv))
	upperJoin := proofbound.RatFloatUp(proofbound.IntervalAbsUpper(joinAngle))
	lineEps := max(stations[0].PointError, stations[1].PointError)
	lineIdeal := proofbound.AbsSumUpper(
		proofbound.ProductUpper(proofbound.ProductUpper(poly.MaxRadiusL1,
			proofbound.ProductUpper(upperJoin, upperJoin)), 0.125),
		proofbound.ProductUpper(proofbound.ProductUpper(poly.MaxEdgeL1, upperJoin), 0.25),
		lineEps)
	delta := proofbound.AbsSumUpper(lineIdeal, held[0].delta)
	volSymDiff := proofbound.AbsSumUpper(
		twistSymDiff(geometry.Height, poly.PerimeterUpper, lineIdeal), held[0].proof.VolSymDiff)
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
	length := proofbound.IntervalAdd(proofbound.PointInterval(twistsweep.TwistRat(geometry.Height)),
		proofbound.IntervalScale(proofbound.HalfPiIv, twistsweep.TwistRat(geometry.Radius)))
	volumeIv := proofbound.IntervalScale(length, poly.Area)
	// This gate rejects a certificate that merely permits the whole true
	// body to be missing. The admitted mesh must have a local, useful bound.
	if twistsweep.TwistRat(volSymDiff).Cmp(volumeIv.Lo) >= 0 {
		return nil, fmt.Errorf(`%w: twisted sweep boundary tube exceeds the true occupied volume`, ErrUnsupported)
	}
	pathSpanOfCell := make([]int, len(held))
	for i := 1; i < len(pathSpanOfCell); i++ {
		pathSpanOfCell[i] = 1
	}
	mesh, heldAreaIv, _, err := compositeTwistMesh(ctx, body, held, len(poly.Points),
		pathSpanOfCell, delta, volSymDiff)
	if err != nil {
		return nil, err
	}
	trueAreaIv, err := twistsweep.CompositeTwistArea(poly, geometry, joinAngle, thetaIv)
	if err != nil {
		return nil, err
	}
	mesh.n.AreaSlack = proofbound.RatFloatUp(proofbound.IntervalAbsUpper(
		proofbound.IntervalSub(trueAreaIv, heldAreaIv)))
	if proofbound.IsNonFinite(mesh.n.AreaSlack) {
		return nil, fmt.Errorf(`%w: twisted sweep area proof is not finite`, ErrUnsupported)
	}
	volume, volumeBound := sweepmitre.Enclosure(volumeIv.Lo, volumeIv.Hi)
	area, areaBound := sweepmitre.Enclosure(trueAreaIv.Lo, trueAreaIv.Hi)
	body.n.Volume = measurement.Measurement{Value: units.CubicMillimeters(volume), Bound: units.CubicMillimeters(volumeBound),
		Exactness: measurement.ExactnessOf(volumeBound)}
	body.n.Area = measurement.Measurement{Value: units.SquareMillimeters(area), Bound: units.SquareMillimeters(areaBound),
		Exactness: measurement.ExactnessOf(areaBound)}
	if err := publishCompositeTwistCentroid(body, xform, geometry, poly, length); err != nil {
		return nil, err
	}
	minV, maxV := mesh.n.Vertices[0], mesh.n.Vertices[0]
	for _, vertex := range mesh.n.Vertices[1:] {
		minV.X, minV.Y, minV.Z = min(minV.X, vertex.X), min(minV.Y, vertex.Y), min(minV.Z, vertex.Z)
		maxV.X, maxV.Y, maxV.Z = max(maxV.X, vertex.X), max(maxV.Y, vertex.Y), max(maxV.Z, vertex.Z)
	}
	body.n.Bounds = measurement.Box{Min: minV, Max: maxV, Bound: units.Millimeters(delta), Exactness: measurement.Approximate}
	for _, vertex := range body.Vertices() {
		vertex.n.Bound = units.Millimeters(proofbound.AbsSumUpper(vertex.n.Bound.Base(), delta))
	}
	longitudinalExtra := proofbound.ProductUpper(poly.MaxRadiusL1,
		proofbound.AbsSumUpper(proofbound.RatFloatUp(proofbound.HalfPiIv.Hi), upperTheta))
	for _, edge := range body.Edges() {
		edge.n.Curve = surfacegeom.FacetedCurve{Bound: units.Millimeters(delta)}
		edge.n.LengthBound = proofbound.AbsSumUpper(edge.n.LengthBound, edge.n.Length,
			proofbound.RatFloatUp(length.Hi), longitudinalExtra, poly.PerimeterUpper)
	}
	for _, face := range body.Faces() {
		if face.n.Origins[0].Role != provenance.RoleCapStart && face.n.Origins[0].Role != provenance.RoleCapEnd {
			face.n.Surface = surfacegeom.Faceted{Bound: units.Millimeters(delta)}
		}
		face.n.AreaBound = proofbound.AbsSumUpper(face.n.AreaBound, face.n.Area, area, areaBound)
		face.n.AxialDelta, face.n.HasAxialDelta = delta, true
	}
	if err := topobuild.ValidateAnalyticMeasurements(bodyNode(body)); err != nil {
		return nil, err
	}
	body.n.Payload = compositeTwistedSweepPayload{
		profile: profile, plane: plane, path: path, angle: angle,
		recordArea: recordArea, xform: xform, mesh: mesh, delta: delta,
	}
	return body, nil
}

func publishCompositeTwistCentroid(body *Body, xform r3.Transform,
	geometry twistsweep.TwistCardinalPath, poly twistsweep.TwistPolygon, length proofbound.RatInterval) error {
	h, radius := twistsweep.TwistRat(geometry.Height), twistsweep.TwistRat(geometry.Radius)
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
	x, xb := sweepmitre.Enclosure(xIv.Lo, xIv.Hi)
	z, zb := sweepmitre.Enclosure(zIv.Lo, zIv.Hi)
	// Curvature weights each section by 1-a/R. The exact profile centroid
	// is at the path axis, so only its bounded second moment can shift this.
	correction := proofbound.RatFloatUp(new(big.Rat).Quo(
		new(big.Rat).Mul(twistsweep.TwistRat(poly.MaxRadiusL1), twistsweep.TwistRat(poly.MaxRadiusL1)),
		twistsweep.TwistRat(geometry.Radius)))
	local := r3.NewVec(x, 0, z)
	world := xform.Apply(local)
	translation := xform.Translation()
	round := proofbound.RigidRoundAllow(max(math.Abs(x), math.Abs(z)),
		max(math.Abs(translation.X), math.Abs(translation.Y), math.Abs(translation.Z)))
	bound := proofbound.AbsSumUpper(xb, zb, correction, round)
	if proofbound.IsNonFinite(bound) || !proofbound.FiniteVec(world) {
		return fmt.Errorf(`%w: twisted sweep centroid has no finite bound`, ErrUnsupported)
	}
	body.n.Centroid = measurement.VecMeasurement{Value: world, Bound: units.Millimeters(bound), Exactness: measurement.ExactnessOf(bound)}
	return nil
}

// A Loft's start and end cap loops walk in opposite directions. Match the
// recorded station coordinates at a join instead of pairing coedge positions.
// Both adjoining Lofts use the same held frame, so equality is exact here.
func sewTwistLoftJoin(ctx context.Context, from, to *Face, previous *Body, sewn map[*Edge]struct{}) error {
	if len(from.n.Loops) != 1 || len(to.n.Loops) != 1 {
		return fmt.Errorf(`%w: a twisted Loft join needs one section loop`, ErrUnsupported)
	}
	for _, old := range from.n.Loops[0].CoEdges {
		if err := ctx.Err(); err != nil {
			return err
		}
		var shared topo.CoEdge
		matches := 0
		for _, candidate := range to.n.Loops[0].CoEdges {
			if coedgeStart(old).Position == coedgeStart(candidate).Position &&
				coedgeEnd(old).Position == coedgeEnd(candidate).Position ||
				coedgeStart(old).Position == coedgeEnd(candidate).Position &&
					coedgeEnd(old).Position == coedgeStart(candidate).Position {
				shared = candidate
				matches++
			}
		}
		if matches != 1 {
			return fmt.Errorf(`%w: a twisted Loft join has %d matching rim edges`, ErrUnsupported, matches)
		}
		var start, end *Vertex
		if coedgeStart(old).Position == coedgeStart(shared).Position {
			start, end = vertexHandle(coedgeStart(shared)), vertexHandle(coedgeEnd(shared))
		} else {
			start, end = vertexHandle(coedgeEnd(shared)), vertexHandle(coedgeStart(shared))
		}
		rewriteCompositeVertex(previous, vertexHandle(coedgeStart(old)), start)
		rewriteCompositeVertex(previous, vertexHandle(coedgeEnd(old)), end)
		discarded := old.Edge
		rewriteCompositeEdge(previous, edgeHandle(discarded), edgeHandle(shared.Edge))
		shared.Edge.Faces = append(shared.Edge.Faces, discarded.Faces...)
		sewn[edgeHandle(shared.Edge)] = struct{}{}
	}
	return nil
}

type twistCellGroup struct{ from, through, pathSpan int }

// Each Loft cell contributes one quad for each profile edge. Internal section
// edges cancel in pairs, leaving one public wall patch per input path span
// and profile edge, as Sweep's source-role contract requires.
func mergeTwistCellFaces(ctx context.Context, body *Body, ref producerID,
	profileEdges int, groups []twistCellGroup) error {
	old := body.Faces()
	cells := 0
	for _, group := range groups {
		if group.from != cells || group.through <= group.from {
			return fmt.Errorf(`%w: twisted Loft cell groups are not consecutive`, ErrDegenerate)
		}
		cells = group.through
	}
	if len(old) != 2+cells*profileEdges {
		return fmt.Errorf(`%w: twisted Loft cells have unexpected wall topology`, ErrUnsupported)
	}
	merged := []*Face{old[0]}
	for _, group := range groups {
		for j := range profileEdges {
			if err := ctx.Err(); err != nil {
				return err
			}
			uses := map[*Edge][]topo.CoEdge{}
			area, areaBound := 0.0, 0.0
			for cell := group.from; cell < group.through; cell++ {
				face := old[1+cell*profileEdges+j]
				if len(face.n.Loops) != 1 {
					return fmt.Errorf(`%w: a twisted Loft wall has multiple loops`, ErrUnsupported)
				}
				for _, use := range face.n.Loops[0].CoEdges {
					uses[edgeHandle(use.Edge)] = append(uses[edgeHandle(use.Edge)], use)
				}
				area = proofbound.AbsSumUpper(area, face.n.Area)
				areaBound = proofbound.AbsSumUpper(areaBound, face.n.AreaBound)
			}
			boundary := make(map[*Vertex]topo.CoEdge)
			for _, edgeUses := range uses {
				switch len(edgeUses) {
				case 1:
					use := edgeUses[0]
					if _, exists := boundary[vertexHandle(coedgeStart(use))]; exists {
						return fmt.Errorf(`%w: a twisted wall boundary branches`, ErrUnsupported)
					}
					boundary[vertexHandle(coedgeStart(use))] = use
				case 2:
					if edgeUses[0].Forward == edgeUses[1].Forward {
						return fmt.Errorf(`%w: twisted wall cells use a seam in the same direction`, ErrUnsupported)
					}
				default:
					return fmt.Errorf(`%w: a twisted wall edge has an unsupported use count`, ErrUnsupported)
				}
			}
			if len(boundary) < 4 {
				return fmt.Errorf(`%w: a twisted wall has no closed boundary`, ErrUnsupported)
			}
			var first *Vertex
			firstCell := old[1+group.from*profileEdges+j]
			for _, use := range firstCell.n.Loops[0].CoEdges {
				if kept, ok := boundary[vertexHandle(coedgeStart(use))]; ok && kept.Edge == use.Edge {
					first = vertexHandle(coedgeStart(use))
					break
				}
			}
			if first == nil {
				return fmt.Errorf(`%w: a twisted wall has no deterministic outline start`, ErrUnsupported)
			}
			outline := make([]topo.CoEdge, 0, len(boundary))
			current := first
			for range len(boundary) {
				use, ok := boundary[current]
				if !ok {
					return fmt.Errorf(`%w: a twisted wall outline is disconnected`, ErrUnsupported)
				}
				outline = append(outline, use)
				current = vertexHandle(coedgeEnd(use))
				delete(boundary, vertexHandle(coedgeStart(use)))
			}
			if current != first || len(boundary) != 0 {
				return fmt.Errorf(`%w: a twisted wall outline is not one cycle`, ErrUnsupported)
			}
			face := bornFace(&Face{n: topo.Face{Area: area, AreaBound: areaBound, Surface: surfacegeom.Faceted{}, Origins: []provenance.Ref{{Producer: ref, Role: fmt.Sprintf("side(%d,0,%d)", group.pathSpan, j)}}, Body: bodyNode(body), Loops: []*topo.Loop{loopNode(bornLoop(&Loop{n: topo.Loop{Outer: true, CoEdges: outline}}))}}})
			merged = append(merged, face)
		}
	}
	merged = append(merged, old[len(old)-1])
	for _, edge := range body.Edges() {
		edge.n.Faces = nil
	}
	body.n.Lumps[0].Shells[0].Faces = faceNodes(merged)
	if err := topobuild.AttachFaceLoops(ctx, faceNodes(merged)); err != nil {
		return err
	}
	if err := auditCompositeBoundary(ctx, body, nil); err != nil {
		return err
	}
	return nil
}
