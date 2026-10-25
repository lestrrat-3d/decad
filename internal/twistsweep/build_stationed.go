package twistsweep

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
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

// Each station keeps one Loft's turn small enough for its ruled facets to
// remain simple. The fixed ceiling also bounds the number of exact Loft and
// cross-cell facet checks before any body is built.
const maxStationedTwistCells = 64

// StraightTwistStation is one station of a straight-path twist, with the
// axial rounding of its held height.
type StraightTwistStation struct {
	TwistStation
	axialError float64
}

// StationedTwistCellCounts returns the base and the optional finer Loft cell
// counts of a stationed twist through angleUpper radians.
func StationedTwistCellCounts(poly TwistPolygon, angleUpper float64) (int, int, error) {
	if angleUpper <= 0 || proofbound.IsNonFinite(angleUpper) ||
		poly.MaxRadiusL1 <= 0 || proofbound.IsNonFinite(poly.MaxRadiusL1) {
		return 0, 0, fmt.Errorf(`%w: stationed twist has no finite angle or profile radius`, decaderr.ErrUnsupported)
	}
	// The curved source needs a finer station than the straight polygon.
	// The final proof still decides the caller's requested mesh tolerance.
	targetScale := 0.04
	if poly.HasArc {
		targetScale = 0.03
	}
	baseTarget := proofbound.ProductUpper(poly.MaxRadiusL1, targetScale)
	base, err := stationedTwistCountAtTarget(poly, angleUpper, baseTarget)
	if err != nil {
		return 0, 0, err
	}
	if !poly.HasArc || baseTarget <= 0.08 {
		return base, base, nil
	}
	// The finer target is optional. When it exceeds the fixed station or
	// facet-pair ceiling, the original certified build stays available.
	refined, err := stationedTwistCountAtTarget(poly, angleUpper, 0.08)
	if err == nil {
		return base, refined, nil
	}
	return base, base, nil
}

func stationedTwistCountAtTarget(poly TwistPolygon, angleUpper, target float64) (int, error) {
	for cells := 2; cells <= maxStationedTwistCells; cells++ {
		perCell := proofbound.DivUpper(angleUpper, float64(cells))
		if perCell > 1 || twistDeparture(poly, perCell, 0, 0) > target {
			continue
		}
		facets := 2*len(poly.Points)*cells + 2*(len(poly.Points)-2)
		pairs, ok := proofbound.WallChoose2(uint64(facets))
		if !ok || pairs > proofbound.MaxFacetPairTestsPerCall {
			err := fmt.Errorf(`%w: stationed twist exceeds the fixed facet-pair work ceiling`, decaderr.ErrUnsupported)
			if ok {
				return 0, proofbound.WithWorkLimit(err, "facet_pairs", pairs, proofbound.MaxFacetPairTestsPerCall)
			}
			return 0, err
		}
		return cells, nil
	}
	return 0, fmt.Errorf(`%w: stationed twist needs more than %d cells`, decaderr.ErrUnsupported, maxStationedTwistCells)
}

// StraightTwistStations places the cells+1 stations of a straight-path twist
// of heldAngle radians over height.
func StraightTwistStations(height, heldAngle float64, angle proofbound.RatInterval,
	poly TwistPolygon, cells int) ([]StraightTwistStation, error) {
	stations := make([]StraightTwistStation, cells+1)
	for index := range stations {
		fraction := big.NewRat(int64(index), int64(cells))
		betaIv := proofbound.IntervalScale(angle, fraction)
		betaHeld := heldAngle * float64(index) / float64(cells)
		z := height * float64(index) / float64(cells)
		switch index {
		case 0:
			betaHeld, z = 0, 0
		case cells:
			betaHeld, z = heldAngle, height
		}
		sinIv, cosIv, ok := proofbound.RadSinCosSpan(betaIv)
		if !ok {
			return nil, fmt.Errorf(`%w: twist station has no trigonometric enclosure`, decaderr.ErrUnsupported)
		}
		sinHeld, cosHeld := math.Sincos(betaHeld)
		origin := r3.NewVec(0, 0, z)
		frame, err := r3.NewFrame(origin, r3.NewVec(cosHeld, sinHeld, 0),
			r3.NewVec(-sinHeld, cosHeld, 0))
		if err != nil {
			return nil, fmt.Errorf(`%w: twist station has no held frame: %s`, decaderr.ErrUnsupported, err)
		}
		frameError := max(
			proofbound.IntervalFloatError(cosIv, frame.U().X),
			proofbound.IntervalFloatError(sinIv, frame.U().Y),
			proofbound.IntervalFloatError(proofbound.IntervalNeg(sinIv), frame.V().X),
			proofbound.IntervalFloatError(cosIv, frame.V().Y),
		)
		trueZ := new(big.Rat).Mul(TwistRat(height), fraction)
		axialError := proofbound.IntervalFloatError(proofbound.PointInterval(trueZ), frame.Origin().Z)
		pointError := proofbound.AbsSumUpper(axialError,
			proofbound.ProductUpper(3*poly.MaxRadiusL1, frameError))
		if proofbound.IsNonFinite(pointError) || !proofbound.FiniteVec(frame.Origin()) {
			return nil, fmt.Errorf(`%w: twist station has no finite coordinate bound`, decaderr.ErrUnsupported)
		}
		stations[index] = StraightTwistStation{
			Plane: planeRecord{Origin: frame.Origin(), U: frame.U(), V: frame.V()},
			Frame: frame, PointError: pointError,
			axialError: axialError,
		}
	}
	return stations, nil
}

// The true rotating section and the comparison Loft share exact source
// heights. A wall point differs by the rotation's interpolation error, the
// quad diagonal's warp, and the station point enclosures. The later axial
// sweep proof moves this comparison Loft to the rounded held heights.
func straightTwistCellDeparture(poly TwistPolygon, angleUpper, stationError float64) float64 {
	interp := proofbound.ProductUpper(proofbound.ProductUpper(poly.MaxRadiusL1,
		proofbound.ProductUpper(angleUpper, angleUpper)), 0.125)
	warp := proofbound.ProductUpper(proofbound.ProductUpper(poly.MaxEdgeL1, angleUpper), 0.25)
	return proofbound.AbsSumUpper(interp, warp, stationError)
}

func evalStationed(ctx context.Context, env topo.Env,
	profile profileRecord, plane planeRecord, path *sweepinput.Path, angle units.Value,
	recordArea float64, xform r3.Transform) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	poly, err := TwistPolygonOf(profile)
	if err != nil {
		return nil, err
	}
	theta, err := angle.In(units.Radian)
	if err != nil {
		return nil, err
	}
	thetaIv, ok := revolveangle.FromValue(angle).Enclosure()
	if !ok {
		return nil, fmt.Errorf(`%w: stationed twist has no certified angle`, decaderr.ErrUnsupported)
	}
	absTheta := proofbound.IntervalAbsUpper(thetaIv)
	if absTheta.Cmp(proofbound.TwoPiIv.Hi) > 0 {
		return nil, fmt.Errorf(`%w: stationed twist admits at most one full turn`, decaderr.ErrUnsupported)
	}
	baseCells, cells, err := StationedTwistCellCounts(poly, proofbound.RatFloatUp(absTheta))
	if err != nil {
		return nil, err
	}
	if cells > baseCells {
		body, refinedErr := evalStationedCells(ctx, env, profile, plane, path,
			angle, recordArea, xform, poly, theta, thetaIv, absTheta, cells, true)
		if refinedErr == nil {
			return body, nil
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if !errors.Is(refinedErr, decaderr.ErrUnsupported) && !errors.Is(refinedErr, decaderr.ErrDegenerate) {
			return nil, refinedErr
		}
	}
	return evalStationedCells(ctx, env, profile, plane, path,
		angle, recordArea, xform, poly, theta, thetaIv, absTheta, baseCells, false)
}

// An optional finer build can refuse without changing the original
// independently certified station count or committing a partial body. There
// are at most two attempts, each with its own fixed cell and audit ceiling.
func evalStationedCells(ctx context.Context, env topo.Env,
	profile profileRecord, plane planeRecord, path *sweepinput.Path, angle units.Value,
	recordArea float64, xform r3.Transform, poly TwistPolygon, theta float64,
	thetaIv proofbound.RatInterval, absTheta *big.Rat, cells int, optional bool) (*topo.Body, error) {
	stations, err := StraightTwistStations(path.End.Z, theta, thetaIv, poly, cells)
	if err != nil {
		return nil, err
	}
	parts := make([]compositesweep.SpanPart, cells)
	held := make([]loftrecord.Payload, cells)
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	budget := proofbound.NewWorkBudget(ctx)
	for i := range parts {
		part, buildErr := loftrecord.Build(ctx, env, loftrecord.Payload{
			Profile0: profile, Profile1: profile,
			Plane0: stations[i].Plane, Plane1: stations[i+1].Plane,
			Frame0: stations[i].Frame, Frame1: stations[i+1].Frame,
			Xform: xform, RecordArea: [2]float64{recordArea, recordArea},
			RetainTwistAreaCells: poly.HasArc,
		}, budget, work0, work1)
		if buildErr != nil {
			return nil, fmt.Errorf("twisted sweep cell %d: %w", i, buildErr)
		}
		var heldOK bool
		held[i], heldOK = part.Payload.(loftrecord.Payload)
		if !heldOK {
			return nil, fmt.Errorf(`%w: twisted sweep cell lost its Loft payload`, decaderr.ErrUnsupported)
		}
		// Arc subdivision can make the actual Loft much larger than the
		// source-edge estimate. A first-cell projection only skips an optional
		// attempt; the complete mesh still receives its own exact work audit.
		if optional && i == 0 && !stationedTwistFirstCellFits(held[i], cells) {
			return nil, fmt.Errorf(`%w: refined twist exceeds the fixed facet-pair work ceiling`, decaderr.ErrUnsupported)
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
			if len(spans) != cells {
				return fmt.Errorf(`%w: twisted sweep cell count changed during sewing`, decaderr.ErrDegenerate)
			}
			return nil
		}, compositesweep.SewTwistLoftJoin, false)
	if err != nil {
		return nil, err
	}
	if err := mergeTwistCellFaces(ctx, env, body, len(poly.Points),
		[]twistCellGroup{{from: 0, through: cells, pathSpan: 0}}); err != nil {
		return nil, err
	}
	cellAngle := proofbound.RatFloatUp(new(big.Rat).Quo(absTheta, big.NewRat(int64(cells), 1)))
	cellHeight := proofbound.RatFloatUp(new(big.Rat).Quo(TwistRat(path.End.Z), big.NewRat(int64(cells), 1)))
	delta, volSymDiff := 0.0, 0.0
	for i, loft := range held {
		stationError := max(stations[i].PointError, stations[i+1].PointError)
		axialError := max(stations[i].axialError, stations[i+1].axialError)
		ideal := straightTwistCellDeparture(poly, cellAngle, stationError)
		// Loft's source-to-facet proof includes the arc chord and ruled-wall
		// departure; its vertex-only delta does not.
		delta = max(delta, proofbound.AbsSumUpper(ideal, loft.Proof.FacetDeparture))
		// The horizontal tube compares cells at their exact source heights.
		// Move that comparison Loft to the rounded station heights separately:
		// each triangle vertex moves axially by at most axialError, and the
		// area bound covers every intermediate triangle after placement.
		axialArea := proofbound.PerturbedAreaUpper(loft.Verts, loft.Tris,
			proofbound.AbsSumUpper(axialError, loft.Delta))
		axialMove := proofbound.SweptVolumeAllow(axialError, axialArea)
		volSymDiff = proofbound.AbsSumUpper(volSymDiff,
			twistSymDiff(cellHeight, poly.PerimeterUpper, ideal),
			axialMove, loft.Proof.VolSymDiff)
	}
	if proofbound.IsNonFinite(delta) || proofbound.IsNonFinite(volSymDiff) {
		return nil, fmt.Errorf(`%w: stationed twist has no finite boundary proof`, decaderr.ErrUnsupported)
	}
	volumeIv := proofbound.IntervalScale(poly.AreaInterval, TwistRat(path.End.Z))
	if TwistRat(volSymDiff).Cmp(volumeIv.Lo) >= 0 {
		return nil, fmt.Errorf(`%w: stationed twist boundary tube exceeds the true occupied volume`, decaderr.ErrUnsupported)
	}
	pathSpanOfCell := make([]int, cells)
	mesh, heldAreaIv, heldByFace, err := CompositeTwistMesh(ctx, body, held, len(poly.Points),
		pathSpanOfCell, delta, volSymDiff)
	if err != nil {
		return nil, err
	}
	var trueAreaIv proofbound.RatInterval
	if poly.HasArc {
		trueAreaIv, err = twistMixedAreaInterval(poly, TwistRat(path.End.Z), thetaIv)
	} else {
		trueAreaIv, err = TwistAreaInterval(poly, TwistRat(path.End.Z), thetaIv)
	}
	if err != nil {
		return nil, err
	}
	if poly.HasArc {
		wallBound, wallErr := TwistMixedWallAreaL1(held, stations, poly,
			proofbound.RatFloatUp(absTheta), path.End.Z)
		if wallErr != nil {
			return nil, wallErr
		}
		mesh.AreaSlack, err = TwistMixedAreaSlackLocal(profile, poly, body, heldByFace, wallBound)
		if err != nil {
			return nil, err
		}
	} else {
		mesh.AreaSlack = proofbound.RatFloatUp(proofbound.IntervalAbsUpper(
			proofbound.IntervalSub(trueAreaIv, heldAreaIv)))
	}
	if proofbound.IsNonFinite(mesh.AreaSlack) {
		return nil, fmt.Errorf(`%w: stationed twist has no finite area proof`, decaderr.ErrUnsupported)
	}
	volume, volumeBound := sweepmitre.Enclosure(volumeIv.Lo, volumeIv.Hi)
	area, areaBound := sweepmitre.Enclosure(trueAreaIv.Lo, trueAreaIv.Hi)
	body.Volume = measurement.Measurement{Value: units.CubicMillimeters(volume), Bound: units.CubicMillimeters(volumeBound),
		Exactness: measurement.ExactnessOf(volumeBound)}
	body.Area = measurement.Measurement{Value: units.SquareMillimeters(area), Bound: units.SquareMillimeters(areaBound),
		Exactness: measurement.ExactnessOf(areaBound)}
	center := r3.NewVec(0, 0, path.End.Z/2)
	worldCenter := xform.Apply(center)
	centerBound := 0.0
	if xform != r3.Identity() {
		tr := xform.Translation()
		centerBound = proofbound.RigidRoundAllow(path.End.Z/2,
			max(math.Abs(tr.X), math.Abs(tr.Y), math.Abs(tr.Z)))
	}
	body.Centroid = measurement.VecMeasurement{Value: worldCenter, Bound: units.Millimeters(centerBound),
		Exactness: measurement.ExactnessOf(centerBound)}
	minV, maxV := mesh.Vertices[0], mesh.Vertices[0]
	for _, vertex := range mesh.Vertices[1:] {
		minV.X, minV.Y, minV.Z = min(minV.X, vertex.X), min(minV.Y, vertex.Y), min(minV.Z, vertex.Z)
		maxV.X, maxV.Y, maxV.Z = max(maxV.X, vertex.X), max(maxV.Y, vertex.Y), max(maxV.Z, vertex.Z)
	}
	body.Bounds = measurement.Box{Min: minV, Max: maxV, Bound: units.Millimeters(delta), Exactness: measurement.Approximate}
	for _, vertex := range topobuild.Vertices(body) {
		vertex.Bound = units.Millimeters(proofbound.AbsSumUpper(vertex.Bound.Base(), delta))
	}
	upperTheta := proofbound.RatFloatUp(absTheta)
	for _, edge := range topobuild.Edges(body) {
		edge.Curve = surfacegeom.FacetedCurve{Bound: units.Millimeters(delta)}
		edge.LengthBound = proofbound.AbsSumUpper(edge.LengthBound, edge.Length,
			path.End.Z, proofbound.ProductUpper(poly.MaxRadiusL1, upperTheta), poly.PerimeterUpper)
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
	body.Payload = Payload{
		Profile: profile, Plane: plane, Path: path, Angle: angle,
		RecordArea: recordArea, Xform: xform, Mesh: mesh,
		Delta: delta, AreaSlack: mesh.AreaSlack, VolSymDiff: volSymDiff,
	}
	return body, nil
}

func stationedTwistFirstCellFits(first loftrecord.Payload, cells int) bool {
	if cells <= 0 || first.Walls <= 0 || first.Walls >= len(first.Tris) {
		return false
	}
	walls := uint64(first.Walls)
	if walls > ^uint64(0)/uint64(cells) {
		return false
	}
	facets := walls * uint64(cells)
	caps := uint64(len(first.Tris) - first.Walls)
	if caps > ^uint64(0)-facets {
		return false
	}
	pairs, ok := proofbound.WallChoose2(facets + caps)
	return ok && pairs <= proofbound.MaxFacetPairTestsPerCall
}
