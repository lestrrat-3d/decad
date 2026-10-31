package twistsweep

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/massmoment"
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
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// The true boundary and the ideal Loft boundary share a triangle-domain
// parameterization on each profile edge. Linear interpolation between them
// sweeps at most displacement times the product of upper bounds on its two
// surface derivatives. Both end-cap homotopies are charged as well. Loft's
// own proof later covers the ideal-to-stored vertex displacement separately.
func compositeTwistCellProof(poly TwistPolygon, geometry TwistCardinalPath,
	fullAngle proofbound.RatInterval, start, end TwistStation, held loftrecord.Payload) (float64, float64, error) {
	dPhi := proofbound.RatFloatUp(new(big.Rat).Quo(proofbound.HalfPiIv.Hi, big.NewRat(TwistArcCells, 1)))
	pathLengthLo := new(big.Rat).Add(TwistRat(geometry.Height),
		new(big.Rat).Mul(TwistRat(geometry.Radius), proofbound.HalfPiIv.Lo))
	dBetaRat := new(big.Rat).Quo(new(big.Rat).Mul(
		proofbound.IntervalAbsUpper(fullAngle),
		new(big.Rat).Mul(TwistRat(geometry.Radius),
			new(big.Rat).Quo(proofbound.HalfPiIv.Hi, big.NewRat(TwistArcCells, 1)))), pathLengthLo)
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
	if held.Walls != 2*len(poly.Points) || len(held.Tris) < held.Walls+held.CapStartCount {
		return 0, 0, fmt.Errorf(`%w: twisted Loft cap cell counts changed`, decaderr.ErrUnsupported)
	}
	capStart := proofbound.PerturbedAreaUpper(held.Verts,
		held.Tris[held.Walls:held.Walls+held.CapStartCount],
		proofbound.AbsSumUpper(start.PointError, held.Delta))
	capEnd := proofbound.PerturbedAreaUpper(held.Verts,
		held.Tris[held.Walls+held.CapStartCount:],
		proofbound.AbsSumUpper(end.PointError, held.Delta))
	// The cap homotopy moves only by its station error; the larger wall
	// departure is used for one simple upper bound on the whole cell.
	swept := proofbound.ProductUpper(idealDelta,
		proofbound.AbsSumUpper(wallAreaUpper, capStart, capEnd))
	if proofbound.IsNonFinite(swept) || proofbound.IsNonFinite(idealDelta) {
		return 0, 0, fmt.Errorf(`%w: twisted arc cell has no finite boundary-tube proof`, decaderr.ErrUnsupported)
	}
	return idealDelta, swept, nil
}

// CompositeTwistMesh sews the held cells of body into one authenticated mesh,
// audits it, and returns it with its facet area interval and the area
// interval of each live face. A later Boolean reads those per-face intervals.
func CompositeTwistMesh(ctx context.Context, body *topo.Body, cells []loftrecord.Payload,
	profileEdges int, pathSpanOfCell []int, delta, volSymDiff float64) (*topo.Mesh, proofbound.RatInterval,
	map[*topo.Face]proofbound.RatInterval, error) {
	if len(cells) < 2 {
		return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: composite twist has too few held cells`, decaderr.ErrDegenerate)
	}
	if len(pathSpanOfCell) != len(cells) {
		return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft cells have no path-span map`, decaderr.ErrDegenerate)
	}
	byRole := map[string]*topo.Face{}
	for _, face := range topobuild.Faces(body) {
		for _, origin := range face.Origins {
			byRole[origin.Role] = face
		}
	}
	mesh := &topo.Mesh{Bound: delta, VolSymDiff: volSymDiff, SymDiffOK: true}
	stationPoints := len(cells[0].Verts) / 2
	if stationPoints < profileEdges || len(cells[0].Verts) != 2*stationPoints {
		return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft has no complete station table`, decaderr.ErrUnsupported)
	}
	spanOf := make([]int, 0, len(cells)*(2*stationPoints+4))
	for cell, held := range cells {
		if err := ctx.Err(); err != nil {
			return nil, proofbound.RatInterval{}, nil, err
		}
		if len(held.Verts) != 2*stationPoints || held.Walls != 2*stationPoints ||
			len(held.Segment) != 1 || len(held.Segment[0]) != stationPoints ||
			len(held.Tris) <= held.Walls+held.CapStartCount {
			return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft has an unexpected station table`, decaderr.ErrUnsupported)
		}
		if cell == 0 {
			mesh.Vertices = append(mesh.Vertices, held.Verts[:stationPoints]...)
		} else {
			for j := range stationPoints {
				if mesh.Vertices[cell*stationPoints+j] != held.Verts[j] {
					return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft cells disagree at a shared station`, decaderr.ErrUnsupported)
				}
			}
		}
		mesh.Vertices = append(mesh.Vertices, held.Verts[stationPoints:]...)
		mesh.CoordBound = max(mesh.CoordBound, held.Delta)
		for t, triangle := range held.Tris {
			if t < held.Walls && (t >= len(held.Cell) ||
				held.Cell[t][0] != 0 || held.Cell[t][1] != t/2) {
				return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft wall order lost its profile provenance`, decaderr.ErrUnsupported)
			}
			if t >= held.Walls && t < held.Walls+held.CapStartCount && cell != 0 ||
				t >= held.Walls+held.CapStartCount && cell != len(cells)-1 {
				continue
			}
			role := provenance.RoleCapStart
			if t < held.Walls {
				segment := held.Segment[0][t/2]
				if segment < 0 || segment >= profileEdges {
					return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft lost its source segment`, decaderr.ErrUnsupported)
				}
				role = fmt.Sprintf("side(%d,0,%d)", pathSpanOfCell[cell], segment)
			} else if t >= held.Walls+held.CapStartCount {
				role = provenance.RoleCapEnd
			}
			face := byRole[role]
			if face == nil {
				return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft mesh has no live face for %s`, decaderr.ErrDegenerate, role)
			}
			mapped := [3]int{}
			for i, local := range triangle {
				if local < 0 || local >= 2*stationPoints {
					return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft triangle has an invalid station`, decaderr.ErrDegenerate)
				}
				mapped[i] = cell*stationPoints + local
			}
			topobuild.AddTriangle(mesh, mapped, face)
			topobuild.SetFaceBound(mesh, face, delta)
			spanOf = append(spanOf, cell)
		}
	}
	if err := topobuild.RequireClosedMesh(mesh); err != nil {
		return nil, proofbound.RatInterval{}, nil, err
	}
	if err := topobuild.LiftAuditError(tessellation.RequireVertexLinks(ctx, len(mesh.Vertices), mesh.Triangles)); err != nil {
		return nil, proofbound.RatInterval{}, nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	auditTris, err := revolvemesh.RequireRevolveFacetAreas(budget, mesh.Vertices, mesh.Triangles, mesh.CoordBound)
	if err != nil {
		return nil, proofbound.RatInterval{}, nil, err
	}
	if err := compositesweep.CrossSpanContactAudit(budget, auditTris, mesh.Triangles, spanOf, mesh.CoordBound); err != nil {
		return nil, proofbound.RatInterval{}, nil, err
	}
	if tessellation.OrientationSign(mesh.Vertices, mesh.Triangles, r3.Vec{}) <= 0 {
		return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft mesh has nonpositive orientation`, decaderr.ErrUnsupported)
	}
	area := proofbound.PointInterval(new(big.Rat))
	byFace := make(map[*topo.Face]proofbound.RatInterval, len(topobuild.Faces(body)))
	vertexIntervals := make([]proofbound.IvVec3, len(mesh.Vertices))
	vertexIntervalsInitialized := make([]bool, len(mesh.Vertices))
	for index, triangle := range mesh.Triangles {
		var points [3]proofbound.IvVec3
		for i, vertexIndex := range triangle {
			if !vertexIntervalsInitialized[vertexIndex] {
				iv, ok := proofbound.IvVec3Of(mesh.Vertices[vertexIndex])
				if !ok {
					return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft facet has no area enclosure`, decaderr.ErrUnsupported)
				}
				vertexIntervals[vertexIndex] = iv
				vertexIntervalsInitialized[vertexIndex] = true
			}
			points[i] = vertexIntervals[vertexIndex]
		}
		twice, ok := revolvemesh.IvTwoTriangleArea(points[0], points[1], points[2])
		if !ok {
			return nil, proofbound.RatInterval{}, nil, fmt.Errorf(`%w: twisted Loft facet has no area enclosure`, decaderr.ErrUnsupported)
		}
		facet := proofbound.IntervalScale(twice, big.NewRat(1, 2))
		area = proofbound.IntervalAdd(area, facet)
		face := mesh.Source[index]
		if previous, exists := byFace[face]; exists {
			byFace[face] = proofbound.IntervalAdd(previous, facet)
		} else {
			byFace[face] = facet
		}
	}
	return mesh, area, byFace, nil
}

func evalComposite(ctx context.Context, env topo.Env,
	profile profileRecord, plane planeRecord, path *sweepinput.Path, angle units.Value,
	recordArea float64, xform r3.Transform) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	poly, err := TwistPolygonOf(profile)
	if err != nil {
		return nil, err
	}
	if poly.HasArc {
		return nil, fmt.Errorf(`%w: a composite twisted sweep needs straight profile edges`, decaderr.ErrUnsupported)
	}
	if poly.OffAxis() {
		return nil, fmt.Errorf(`%w: a composite twisted sweep needs its centroid on the path axis`, decaderr.ErrUnsupported)
	}
	geometry, err := TwistCardinalPathOf(path, plane, poly)
	if err != nil {
		return nil, err
	}
	charge, err := placementCharge(xform)
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
		return nil, fmt.Errorf(`%w: composite twist needs a nonzero certified angle up to one radian`, decaderr.ErrUnsupported)
	}
	stations, joinAngle, err := CompositeTwistStations(thetaIv, theta, geometry, poly)
	if err != nil {
		return nil, err
	}
	parts := make([]compositesweep.SpanPart, len(stations)-1)
	held := make([]loftrecord.Payload, len(parts))
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	budget := proofbound.NewWorkBudget(ctx)
	for i := range parts {
		part, buildErr := loftrecord.Build(ctx, env, loftrecord.Payload{
			Profile0: profile, Profile1: profile,
			Plane0: stations[i].Plane, Plane1: stations[i+1].Plane,
			Frame0: stations[i].Frame, Frame1: stations[i+1].Frame,
			Xform: xform, RecordArea: [2]float64{recordArea, recordArea},
		}, budget, work0, work1)
		if buildErr != nil {
			return nil, fmt.Errorf("twisted sweep cell %d: %w", i, buildErr)
		}
		var heldOK bool
		held[i], heldOK = part.Payload.(loftrecord.Payload)
		if !heldOK {
			return nil, fmt.Errorf(`%w: a twisted sweep cell lost its Loft payload`, decaderr.ErrUnsupported)
		}
		if _, err := mergeTwistWalls(ctx, env, part, held[i], len(poly.Points)); err != nil {
			return nil, fmt.Errorf("twisted sweep cell %d: %w", i, err)
		}
		parts[i], err = compositesweep.SpanPartOf(part)
		if err != nil {
			return nil, fmt.Errorf("twisted sweep cell %d: %w", i, err)
		}
	}
	body, err := compositesweep.Assemble(ctx, env, parts, false,
		func(ctx context.Context, spans []compositesweep.SpanPart) error {
			if err := ctx.Err(); err != nil {
				return err
			}
			if len(spans) != len(held) {
				return fmt.Errorf(`%w: twisted sweep span audit count changed`, decaderr.ErrDegenerate)
			}
			for i, span := range spans {
				if span.Body == nil || span.Body.Payload == nil ||
					span.StartCap == nil || span.EndCap == nil || len(held[i].Verts) != 2*len(poly.Points) {
					return fmt.Errorf(`%w: twisted sweep cell %d has incomplete Loft geometry`, decaderr.ErrUnsupported, i)
				}
			}
			return nil
		}, compositesweep.SewTwistLoftJoin, false)
	if err != nil {
		return nil, err
	}
	if err := mergeTwistCellFaces(ctx, env, body, len(poly.Points),
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
	delta := proofbound.AbsSumUpper(lineIdeal, held[0].Delta)
	volSymDiff := proofbound.AbsSumUpper(
		twistSymDiff(geometry.Height, poly.PerimeterUpper, lineIdeal), held[0].Proof.VolSymDiff)
	for i := 1; i < len(held); i++ {
		ideal, swept, proofErr := compositeTwistCellProof(poly, geometry, thetaIv,
			stations[i], stations[i+1], held[i])
		if proofErr != nil {
			return nil, fmt.Errorf("twisted sweep cell %d: %w", i, proofErr)
		}
		delta = max(delta, proofbound.AbsSumUpper(ideal, held[i].Delta))
		volSymDiff = proofbound.AbsSumUpper(volSymDiff, swept, held[i].Proof.VolSymDiff)
	}
	// The cell departures and section tubes are plane-local; the mesh and
	// every reading below are of their images under the placement.
	delta, volSymDiff = chargeBoundary(charge, delta, volSymDiff)
	if proofbound.IsNonFinite(delta) || proofbound.IsNonFinite(volSymDiff) {
		return nil, fmt.Errorf(`%w: twisted sweep has no finite boundary proof`, decaderr.ErrUnsupported)
	}
	length := proofbound.IntervalAdd(proofbound.PointInterval(TwistRat(geometry.Height)),
		proofbound.IntervalScale(proofbound.HalfPiIv, TwistRat(geometry.Radius)))
	volumeIv := charge.VolumeInterval(proofbound.IntervalScale(length, poly.Area))
	// This gate rejects a certificate that merely permits the whole true
	// body to be missing. The admitted mesh must have a local, useful bound.
	if TwistRat(volSymDiff).Cmp(volumeIv.Lo) >= 0 {
		return nil, fmt.Errorf(`%w: twisted sweep boundary tube exceeds the true occupied volume`, decaderr.ErrUnsupported)
	}
	pathSpanOfCell := make([]int, len(held))
	for i := 1; i < len(pathSpanOfCell); i++ {
		pathSpanOfCell[i] = 1
	}
	mesh, heldAreaIv, _, err := CompositeTwistMesh(ctx, body, held, len(poly.Points),
		pathSpanOfCell, delta, volSymDiff)
	if err != nil {
		return nil, err
	}
	trueAreaIv, err := CompositeTwistArea(poly, geometry, joinAngle, thetaIv)
	if err != nil {
		return nil, err
	}
	trueAreaIv = charge.AreaInterval(trueAreaIv)
	mesh.AreaSlack = proofbound.RatFloatUp(proofbound.IntervalAbsUpper(
		proofbound.IntervalSub(trueAreaIv, heldAreaIv)))
	if proofbound.IsNonFinite(mesh.AreaSlack) {
		return nil, fmt.Errorf(`%w: twisted sweep area proof is not finite`, decaderr.ErrUnsupported)
	}
	volume, volumeBound := sweepmitre.Enclosure(volumeIv.Lo, volumeIv.Hi)
	area, areaBound := sweepmitre.Enclosure(trueAreaIv.Lo, trueAreaIv.Hi)
	body.Volume = measurement.Measurement{Value: units.CubicMillimeters(volume), Bound: units.CubicMillimeters(volumeBound),
		Exactness: measurement.ExactnessOf(volumeBound)}
	body.Area = measurement.Measurement{Value: units.SquareMillimeters(area), Bound: units.SquareMillimeters(areaBound),
		Exactness: measurement.ExactnessOf(areaBound)}
	if err := publishCompositeTwistCentroid(body, xform, charge, geometry, poly, length); err != nil {
		return nil, err
	}
	minV, maxV := mesh.Vertices[0], mesh.Vertices[0]
	for _, vertex := range mesh.Vertices[1:] {
		minV.X, minV.Y, minV.Z = min(minV.X, vertex.X), min(minV.Y, vertex.Y), min(minV.Z, vertex.Z)
		maxV.X, maxV.Y, maxV.Z = max(maxV.X, vertex.X), max(maxV.Y, vertex.Y), max(maxV.Z, vertex.Z)
	}
	body.Bounds = measurement.Box{Min: minV, Max: maxV, Bound: units.Millimeters(delta), Exactness: measurement.Approximate}
	for _, vertex := range topobuild.Vertices(body) {
		vertex.Bound = units.Millimeters(proofbound.AbsSumUpper(vertex.Bound.Base(), delta))
	}
	longitudinalExtra := proofbound.ProductUpper(poly.MaxRadiusL1,
		proofbound.AbsSumUpper(proofbound.RatFloatUp(proofbound.HalfPiIv.Hi), upperTheta))
	for _, edge := range topobuild.Edges(body) {
		edge.Curve = surfacegeom.FacetedCurve{Bound: units.Millimeters(delta)}
		edge.LengthBound = proofbound.AbsSumUpper(edge.LengthBound, edge.Length,
			proofbound.RatFloatUp(length.Hi), longitudinalExtra, poly.PerimeterUpper)
	}
	for _, face := range topobuild.Faces(body) {
		if face.Origins[0].Role != provenance.RoleCapStart && face.Origins[0].Role != provenance.RoleCapEnd {
			face.Surface = surfacegeom.Faceted{Bound: units.Millimeters(delta)}
		}
		face.AreaBound = proofbound.AbsSumUpper(face.AreaBound, face.Area, area, areaBound)
		face.AxialDelta, face.HasAxialDelta = delta, true
	}
	if err := topobuild.ValidateAnalyticMeasurements(body); err != nil {
		return nil, err
	}
	body.Payload = CompositePayload{
		Profile: profile, Plane: plane, Path: path, Angle: angle,
		RecordArea: recordArea, Xform: xform, Mesh: mesh, Delta: delta,
	}
	return body, nil
}

func publishCompositeTwistCentroid(body *topo.Body, xform r3.Transform, charge massmoment.MapCharge,
	geometry TwistCardinalPath, poly TwistPolygon, length proofbound.RatInterval) error {
	h, radius := TwistRat(geometry.Height), TwistRat(geometry.Radius)
	xNum := proofbound.IntervalScale(proofbound.IntervalSub(proofbound.HalfPiIv,
		proofbound.PointInterval(big.NewRat(1, 1))), new(big.Rat).Mul(radius, radius))
	zNum := proofbound.IntervalAdd(proofbound.PointInterval(new(big.Rat).Add(
		new(big.Rat).Quo(new(big.Rat).Mul(h, h), big.NewRat(2, 1)),
		new(big.Rat).Mul(radius, radius))),
		proofbound.IntervalScale(proofbound.HalfPiIv, new(big.Rat).Mul(h, radius)))
	xIv, xOK := proofbound.IntervalQuo(xNum, length)
	zIv, zOK := proofbound.IntervalQuo(zNum, length)
	if !xOK || !zOK {
		return fmt.Errorf(`%w: twisted sweep centroid has no path moment`, decaderr.ErrUnsupported)
	}
	x, xb := sweepmitre.Enclosure(xIv.Lo, xIv.Hi)
	z, zb := sweepmitre.Enclosure(zIv.Lo, zIv.Hi)
	// Curvature weights each section by 1-a/R. The exact profile centroid
	// is at the path axis, so only its bounded second moment can shift this.
	correction := proofbound.RatFloatUp(new(big.Rat).Quo(
		new(big.Rat).Mul(TwistRat(poly.MaxRadiusL1), TwistRat(poly.MaxRadiusL1)),
		TwistRat(geometry.Radius)))
	local := r3.NewVec(x, 0, z)
	world := xform.Apply(local)
	translation := xform.Translation()
	round := proofbound.RigidRoundAllow(max(math.Abs(x), math.Abs(z)),
		max(math.Abs(translation.X), math.Abs(translation.Y), math.Abs(translation.Z)))
	bound := proofbound.AbsSumUpper(xb, zb, correction, round)
	if charge.Stretch > 0 {
		// The plane-local centroid's error moves through B, which stretches
		// a length by at most its defect; Apply's rounding is already placed.
		bound = proofbound.AbsSumUpper(bound,
			proofbound.ProductUpper(charge.Stretch, proofbound.AbsSumUpper(xb, zb, correction)))
	}
	if proofbound.IsNonFinite(bound) || !proofbound.FiniteVec(world) {
		return fmt.Errorf(`%w: twisted sweep centroid has no finite bound`, decaderr.ErrUnsupported)
	}
	body.Centroid = measurement.VecMeasurement{Value: world, Bound: units.Millimeters(bound), Exactness: measurement.ExactnessOf(bound)}
	return nil
}

type twistCellGroup struct{ from, through, pathSpan int }

// Each Loft cell contributes one quad for each profile edge. Internal section
// edges cancel in pairs, leaving one public wall patch per input path span
// and profile edge, as Sweep's source-role contract requires.
func mergeTwistCellFaces(ctx context.Context, env topo.Env, body *topo.Body,
	profileEdges int, groups []twistCellGroup) error {
	old := topobuild.Faces(body)
	cells := 0
	for _, group := range groups {
		if group.from != cells || group.through <= group.from {
			return fmt.Errorf(`%w: twisted Loft cell groups are not consecutive`, decaderr.ErrDegenerate)
		}
		cells = group.through
	}
	if len(old) != 2+cells*profileEdges {
		return fmt.Errorf(`%w: twisted Loft cells have unexpected wall topology`, decaderr.ErrUnsupported)
	}
	merged := []*topo.Face{old[0]}
	for _, group := range groups {
		for j := range profileEdges {
			if err := ctx.Err(); err != nil {
				return err
			}
			uses := map[*topo.Edge][]topo.CoEdge{}
			area, areaBound := 0.0, 0.0
			for cell := group.from; cell < group.through; cell++ {
				face := old[1+cell*profileEdges+j]
				if len(face.Loops) != 1 {
					return fmt.Errorf(`%w: a twisted Loft wall has multiple loops`, decaderr.ErrUnsupported)
				}
				for _, use := range face.Loops[0].CoEdges {
					uses[use.Edge] = append(uses[use.Edge], use)
				}
				area = proofbound.AbsSumUpper(area, face.Area)
				areaBound = proofbound.AbsSumUpper(areaBound, face.AreaBound)
			}
			boundary := make(map[*topo.Vertex]topo.CoEdge)
			for _, edgeUses := range uses {
				switch len(edgeUses) {
				case 1:
					use := edgeUses[0]
					if _, exists := boundary[topobuild.CoEdgeStart(use)]; exists {
						return fmt.Errorf(`%w: a twisted wall boundary branches`, decaderr.ErrUnsupported)
					}
					boundary[topobuild.CoEdgeStart(use)] = use
				case 2:
					if edgeUses[0].Forward == edgeUses[1].Forward {
						return fmt.Errorf(`%w: twisted wall cells use a seam in the same direction`, decaderr.ErrUnsupported)
					}
				default:
					return fmt.Errorf(`%w: a twisted wall edge has an unsupported use count`, decaderr.ErrUnsupported)
				}
			}
			if len(boundary) < 4 {
				return fmt.Errorf(`%w: a twisted wall has no closed boundary`, decaderr.ErrUnsupported)
			}
			var first *topo.Vertex
			firstCell := old[1+group.from*profileEdges+j]
			for _, use := range firstCell.Loops[0].CoEdges {
				if kept, ok := boundary[topobuild.CoEdgeStart(use)]; ok && kept.Edge == use.Edge {
					first = topobuild.CoEdgeStart(use)
					break
				}
			}
			if first == nil {
				return fmt.Errorf(`%w: a twisted wall has no deterministic outline start`, decaderr.ErrUnsupported)
			}
			outline := make([]topo.CoEdge, 0, len(boundary))
			current := first
			for range len(boundary) {
				use, ok := boundary[current]
				if !ok {
					return fmt.Errorf(`%w: a twisted wall outline is disconnected`, decaderr.ErrUnsupported)
				}
				outline = append(outline, use)
				current = topobuild.CoEdgeEnd(use)
				delete(boundary, topobuild.CoEdgeStart(use))
			}
			if current != first || len(boundary) != 0 {
				return fmt.Errorf(`%w: a twisted wall outline is not one cycle`, decaderr.ErrUnsupported)
			}
			face := topobuild.NewFace(env.Alloc, topo.Face{
				Area:      area,
				AreaBound: areaBound,
				Surface:   surfacegeom.Faceted{},
				Origins: []provenance.Ref{{Producer: env.Ref,
					Role: fmt.Sprintf("side(%d,0,%d)", group.pathSpan, j)}},
				Body: body,
				Loops: []*topo.Loop{topobuild.NewLoop(env.Alloc,
					topo.Loop{Outer: true, CoEdges: outline})},
			})
			merged = append(merged, face)
		}
	}
	merged = append(merged, old[len(old)-1])
	for _, edge := range topobuild.Edges(body) {
		edge.Faces = nil
	}
	body.Lumps[0].Shells[0].Faces = merged
	if err := topobuild.AttachFaceLoops(ctx, merged); err != nil {
		return err
	}
	if err := compositesweep.AuditBoundary(ctx, body, nil); err != nil {
		return err
	}
	return nil
}
