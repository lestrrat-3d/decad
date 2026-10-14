package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/r3"
)

const (
	crossBoreHalfStations = 2048
	crossBoreRestStations = 512
	crossBoreAuditScans   = 1_200_000_000
)

type crossBoreMesh struct {
	verts []r3.Vec
	tris  [][3]int
	src   []int
	// cellHeld[i] holds the indices of the held vertices at whole-turn cell
	// i's angle 2πi/n: the outer foot, the bore foot and the negative-mouth
	// point. The cell proof compares exactly these vertices.
	cellHeld          [][3]int
	groups            []facetGroup
	outerStationError float64
	outerAngularGap   float64
}

// crossBoreStationError bounds a held outer-ring vertex against the exact
// source circle at this station's recorded angle. It covers both radial
// rounding and the angle conversion used to build a cap or outer-wall ring.
func crossBoreStationError(p r3.Vec, radius *big.Rat, phi float64) (float64, bool) {
	s, c, ok := proofbound.RadSinCosSpan(proofbound.PointInterval(proofarith.FloatRat(phi)))
	if !ok {
		return 0, false
	}
	x := proofbound.IntervalMul(c, proofbound.PointInterval(radius))
	y := proofbound.IntervalMul(s, proofbound.PointInterval(radius))
	err := max(proofbound.IntervalFloatError(x, p.X), proofbound.IntervalFloatError(y, p.Y))
	return proofbound.Radius3D(err), true
}

// crossBoreStationErrors computes crossBoreStationError for the four vertices
// of every station (idx[i] holds their indices in the order bottom, lower,
// upper, top), evaluating each station's sine and cosine enclosure once.
// Stations run on up to workers goroutines, each writing only its own slot.
// The result is errs[i][k], the error of vertex idx[i][k]. The first failing
// station in index order gives the error: a station without a circular bound
// or with a non-finite error.
func crossBoreStationErrors(ctx context.Context, verts []r3.Vec, stations []crossBoreStation,
	idx [][4]int, radius *big.Rat, workers int) ([][4]float64, error) {
	errs := make([][4]float64, len(stations))
	_, err := orderedwork.Each(len(stations), workers, 16, ctx.Err, func(i int) error {
		fail := fmt.Errorf("%w: an outer-ring station has no circular bound", ErrUnsupported)
		s, c, ok := proofbound.RadSinCosSpan(proofbound.PointInterval(proofarith.FloatRat(stations[i].phi)))
		if !ok {
			return fail
		}
		x := proofbound.IntervalMul(c, proofbound.PointInterval(radius))
		y := proofbound.IntervalMul(s, proofbound.PointInterval(radius))
		for k, vertex := range idx[i] {
			p := verts[vertex]
			e := proofbound.Radius3D(max(proofbound.IntervalFloatError(x, p.X), proofbound.IntervalFloatError(y, p.Y)))
			if proofbound.IsNonFinite(e) {
				return fail
			}
			errs[i][k] = e
		}
		return nil
	})
	if err != nil {
		return nil, err
	}
	return errs, nil
}

func (m *crossBoreMesh) point(p r3.Vec) int {
	i := len(m.verts)
	m.verts = append(m.verts, p)
	return i
}

func (m *crossBoreMesh) triangle(a, b, c, source int) {
	m.tris = append(m.tris, [3]int{a, b, c})
	m.src = append(m.src, source)
}

func (m *crossBoreMesh) quad(a, b, c, d, source int) {
	m.triangle(a, b, c, source)
	m.triangle(a, c, d, source)
}

// crossBoreCapPatches returns the bottom cap fan (source 0) and the top
// cap fan (source 1) as projected patches over the XY axes, each in
// triangle order. Each cap is a fan at one exact height; a proof shows it
// is one non-self-touching graph over XY, so its own pairs can meet only
// at indexed edges and vertices.
func crossBoreCapPatches(src []int) []loftmesh.ProjectedPatch {
	patches := []loftmesh.ProjectedPatch{{Axes: [2]int{0, 1}}, {Axes: [2]int{0, 1}}}
	for i, s := range src {
		if s == 0 || s == 1 {
			patches[s].Triangles = append(patches[s].Triangles, i)
		}
	}
	return patches
}

type crossBoreStation struct {
	phi          float64
	lower, upper r3.Vec
	kind, j      int
}

const (
	crossBorePositive = iota
	crossBoreNegative
	crossBoreRest
)

// crossBoreFeet starts from the exact cylinder-cylinder seam. The outer and
// inner points have equal intrinsic setback on their respective unrolled
// cylinders. The source intersection is x=sqrt(R²-r²cos²t), y=r cos t,
// z=c+r sin t; Q=sqrt(R²-r²cos⁴t) makes both normal offsets explicit.
func crossBoreFeet(rec *crossCylinderBoreRecord, setback, theta float64) (r3.Vec, r3.Vec) {
	R, r, c := rec.radius, rec.bore, rec.centerZ
	s, co := math.Sincos(theta)
	x := math.Sqrt(R*R - r*r*co*co)
	Q := math.Sqrt(R*R - r*r*co*co*co*co)
	do := setback * co * x / (R * Q)
	sdo, cdo := math.Sincos(do)
	outer := r3.Vec{
		X: x*cdo - r*co*sdo,
		Y: x*sdo + r*co*cdo,
		Z: c + r*s + setback*R*s/Q,
	}
	db := setback * co * s / Q
	sdb, cdb := math.Sincos(db)
	inner := r3.Vec{
		X: x * (1 - setback/Q),
		Y: r * (co*cdb - s*sdb),
		Z: c + r*(s*cdb+co*sdb),
	}
	return outer, inner
}

func crossBoreFaceGroup(body *Body, role string) (facetGroup, bool) {
	for _, face := range body.Faces() {
		for _, origin := range face.originRefs() {
			if origin.Producer == body.n.Origin.Producer && origin.Role == role {
				return facetGroup{origins: face.originRefs(), planar: face.isPlanar()}, true
			}
		}
	}
	return facetGroup{}, false
}

func crossBoreMeshGroups(rec *crossCylinderBoreRecord) ([]facetGroup, error) {
	groups := make([]facetGroup, 0, 5)
	for _, source := range []struct {
		body *Body
		role string
	}{
		{rec.target, provenance.RoleCapStart},
		{rec.target, provenance.RoleCapEnd},
		{rec.target, prismrecord.OuterFirstSideRole},
		{rec.tool, prismrecord.OuterFirstSideRole},
	} {
		group, ok := crossBoreFaceGroup(source.body, source.role)
		if !ok {
			return nil, fmt.Errorf("%w: a cross-bore source lost face %q", ErrUnsupported, source.role)
		}
		groups = append(groups, group)
	}
	groups = append(groups, facetGroup{origins: []provenance.Ref{{Role: "boreMouthChamfer"}}})
	return groups, nil
}

func crossBoreStationPlan(rec *crossCylinderBoreRecord, setback float64) ([]crossBoreStation, error) {
	const half, rest = crossBoreHalfStations, crossBoreRestStations
	positive := make([]crossBoreStation, half+1)
	negative := make([]crossBoreStation, half+1)
	for j := range positive {
		theta := math.Pi * float64(j) / half
		upper, _ := crossBoreFeet(rec, setback, theta)
		lower, _ := crossBoreFeet(rec, setback, 2*math.Pi-theta)
		positive[half-j] = crossBoreStation{
			phi: math.Atan2(upper.Y, upper.X), lower: lower, upper: upper,
			kind: crossBorePositive, j: j,
		}
		co, s := math.Cos(theta), math.Sin(theta)
		x, y := -math.Sqrt(rec.radius*rec.radius-rec.bore*rec.bore*co*co), rec.bore*co
		negative[j] = crossBoreStation{
			phi:   math.Atan2(y, x),
			lower: r3.Vec{X: x, Y: y, Z: rec.centerZ - rec.bore*s},
			upper: r3.Vec{X: x, Y: y, Z: rec.centerZ + rec.bore*s},
			kind:  crossBoreNegative, j: j,
		}
		if negative[j].phi < 0 {
			negative[j].phi += 2 * math.Pi
		}
	}
	for _, stations := range [][]crossBoreStation{positive, negative} {
		for i := 0; i+1 < len(stations); i++ {
			if !(stations[i].phi < stations[i+1].phi) {
				return nil, fmt.Errorf("%w: a cross-bore mouth turns backward", ErrUnsupported)
			}
		}
	}
	addRest := func(stations []crossBoreStation, first, last float64) []crossBoreStation {
		for k := 1; k < rest; k++ {
			phi := first + (last-first)*float64(k)/rest
			p := r3.Vec{X: rec.radius * math.Cos(phi),
				Y: rec.radius * math.Sin(phi), Z: rec.centerZ}
			stations = append(stations, crossBoreStation{
				phi: phi, lower: p, upper: p, kind: crossBoreRest,
			})
		}
		return stations
	}
	stations := make([]crossBoreStation, 0, 2*(half+rest))
	stations = append(stations, positive...)
	stations = addRest(stations, positive[half].phi, negative[0].phi)
	stations = append(stations, negative...)
	stations = addRest(stations, negative[half].phi, positive[0].phi+2*math.Pi)
	for i, station := range stations {
		if !proofbound.FiniteVec(station.lower) || !proofbound.FiniteVec(station.upper) ||
			(i > 0 && !(stations[i-1].phi < station.phi)) {
			return nil, fmt.Errorf("%w: a cross-bore station has no finite ordered position", ErrUnsupported)
		}
	}
	return stations, nil
}

// crossBoreStationGap certifies the largest source-cylinder angle covered
// by one outer-wall panel, including the wrap across a full turn. It uses
// each held station angle as an exact rational parameter and keeps the
// full-turn endpoint outward because π is irrational.
func crossBoreStationGap(stations []crossBoreStation) (float64, error) {
	maximum := new(big.Rat)
	for i := 0; i+1 < len(stations); i++ {
		gap := new(big.Rat).Sub(proofarith.FloatRat(stations[i+1].phi),
			proofarith.FloatRat(stations[i].phi))
		if gap.Sign() <= 0 {
			return 0, fmt.Errorf("%w: an outer-ring angle is not increasing", ErrUnsupported)
		}
		if gap.Cmp(maximum) > 0 {
			maximum = gap
		}
	}
	start := proofarith.FloatRat(stations[0].phi)
	end := proofarith.FloatRat(stations[len(stations)-1].phi)
	wrapLower := new(big.Rat).Add(new(big.Rat).Mul(proofbound.PiLower, big.NewRat(2, 1)), start)
	wrapLower.Sub(wrapLower, end)
	if wrapLower.Sign() <= 0 {
		return 0, fmt.Errorf("%w: an outer-ring wrap is not increasing", ErrUnsupported)
	}
	wrapUpper := new(big.Rat).Add(new(big.Rat).Mul(proofbound.PiUpper, big.NewRat(2, 1)), start)
	wrapUpper.Sub(wrapUpper, end)
	if wrapUpper.Cmp(maximum) > 0 {
		maximum = wrapUpper
	}
	return proofbound.RatFloatUp(maximum), nil
}

func buildCrossBoreChamferMesh(ctx context.Context,
	rec *crossCylinderBoreRecord, setback float64) (*crossBoreMesh, error) {
	return buildCrossBoreChamferMeshWorkers(ctx, rec, setback, -1)
}

// crossBoreStationParallelMin is the station count from which the station
// errors are computed on several goroutines.
const crossBoreStationParallelMin = 256

// buildCrossBoreChamferMeshWorkers is buildCrossBoreChamferMesh with the
// station-error fan-out width given; a negative workers picks
// orderedwork.EachWorkers for the station count. The mesh does not depend on
// workers.
func buildCrossBoreChamferMeshWorkers(ctx context.Context,
	rec *crossCylinderBoreRecord, setback float64, workers int) (*crossBoreMesh, error) {
	groups, err := crossBoreMeshGroups(rec)
	if err != nil {
		return nil, err
	}
	stations, err := crossBoreStationPlan(rec, setback)
	if err != nil {
		return nil, err
	}
	outerGap, err := crossBoreStationGap(stations)
	if err != nil {
		return nil, err
	}
	m := &crossBoreMesh{groups: groups}
	m.outerAngularGap = outerGap
	radiusRat := proofarith.FloatRat(rec.radius)
	bottom := make([]int, len(stations))
	lower := make([]int, len(stations))
	upper := make([]int, len(stations))
	top := make([]int, len(stations))
	vertexIndex := make([][4]int, len(stations))
	n := 2 * crossBoreHalfStations
	positiveOuter, negativeOuter := make([]int, n), make([]int, n)
	for i, station := range stations {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		x, y := rec.radius*math.Cos(station.phi), rec.radius*math.Sin(station.phi)
		bottom[i] = m.point(r3.Vec{X: x, Y: y})
		lower[i] = m.point(station.lower)
		upper[i] = lower[i]
		if station.lower.Sub(station.upper).Len() >= 1e-13 {
			upper[i] = m.point(station.upper)
		}
		top[i] = m.point(r3.Vec{X: x, Y: y, Z: rec.height})
		vertexIndex[i] = [4]int{bottom[i], lower[i], upper[i], top[i]}
		if station.kind == crossBorePositive {
			positiveOuter[station.j] = upper[i]
			positiveOuter[(n-station.j)%n] = lower[i]
		}
		if station.kind == crossBoreNegative {
			negativeOuter[station.j] = upper[i]
			negativeOuter[(n-station.j)%n] = lower[i]
		}
	}
	if workers < 0 {
		workers = orderedwork.EachWorkers(len(stations), crossBoreStationParallelMin)
	}
	errs, err := crossBoreStationErrors(ctx, m.verts, stations, vertexIndex, radiusRat, workers)
	if err != nil {
		return nil, err
	}
	for _, station := range errs {
		for _, e := range station {
			m.outerStationError = max(m.outerStationError, e)
		}
	}
	bottomCenter, topCenter := m.point(r3.Vec{}), m.point(r3.Vec{Z: rec.height})
	for i := range stations {
		j := (i + 1) % len(stations)
		m.quad(bottom[i], bottom[j], lower[j], lower[i], 2)
		m.quad(upper[i], upper[j], top[j], top[i], 2)
		m.triangle(bottomCenter, bottom[j], bottom[i], 0)
		m.triangle(topCenter, top[i], top[j], 1)
	}
	inner := make([]int, n)
	m.cellHeld = make([][3]int, n)
	for i := range inner {
		theta := 2 * math.Pi * float64(i) / float64(n)
		_, foot := crossBoreFeet(rec, setback, theta)
		inner[i] = m.point(foot)
		m.cellHeld[i] = [3]int{positiveOuter[i], inner[i], negativeOuter[i]}
	}
	for i := range inner {
		j := (i + 1) % n
		m.quad(negativeOuter[i], inner[i], inner[j], negativeOuter[j], 3)
		m.quad(positiveOuter[i], positiveOuter[j], inner[j], inner[i], 4)
	}
	return m, nil
}
