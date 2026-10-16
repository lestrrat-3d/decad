// Package crossbore holds the exact proof arithmetic and the mesh plan of the
// perpendicular cylinder bore-mouth chamfer: the station plan over both
// mouths and the rest of the outer ring, the seam feet of the positive mouth,
// the held outer-ring station errors, the dual-interval proof of the 1,025
// quarter-turn base cells, and the map from the 4,096 whole-turn cells onto
// them. The root
// package reads the source bodies, assembles the mesh and sums the volume and
// area bounds. See docs/cross-cylinder-bore-chamfer-design.md.
package crossbore

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

const (
	// HalfStations is the number of intervals between the stations of a
	// mouth's half turn; StationPlan places HalfStations+1 stations on each
	// mouth. The positive-mouth proof splits the full turn into
	// 2*HalfStations cells.
	HalfStations = 2048
	restStations = 512
	// AuditScans is the scan limit the root passes to the streaming crossing
	// audit of the chamfer mesh.
	AuditScans = 1_200_000_000
)

// Mesh is the held chamfer mesh: vertices, triangles, and for each triangle
// the index in Groups of its source face (0 bottom cap, 1 top cap, 2 outer
// wall, 3 bore wall, 4 chamfer band).
type Mesh struct {
	Verts []r3.Vec
	Tris  [][3]int
	Src   []int
	// CellHeld[i] holds the indices of the held vertices at whole-turn cell
	// i's angle 2πi/n: the outer foot, the bore foot and the negative-mouth
	// point. The cell proof compares exactly these vertices.
	CellHeld [][3]int
	Groups   []facetedrecord.Group
	// OuterStationError is the largest StationErrors reading over the held
	// outer-ring vertices.
	OuterStationError float64
	// OuterAngularGap is the StationGap of the station plan.
	OuterAngularGap float64
}

// StationError bounds a held outer-ring vertex against the exact
// source circle at this station's recorded angle. It covers both radial
// rounding and the angle conversion used to build a cap or outer-wall ring.
func StationError(p r3.Vec, radius *big.Rat, phi float64) (float64, bool) {
	s, c, ok := proofbound.RadSinCosSpan(proofbound.PointInterval(proofarith.FloatRat(phi)))
	if !ok {
		return 0, false
	}
	x := proofbound.IntervalMul(c, proofbound.PointInterval(radius))
	y := proofbound.IntervalMul(s, proofbound.PointInterval(radius))
	err := max(proofbound.IntervalFloatError(x, p.X), proofbound.IntervalFloatError(y, p.Y))
	return proofbound.Radius3D(err), true
}

// StationErrors computes StationError for the four vertices
// of every station (idx[i] holds their indices in the order bottom, lower,
// upper, top), evaluating each station's sine and cosine enclosure once.
// Stations run on up to workers goroutines, each writing only its own slot.
// The result is errs[i][k], the error of vertex idx[i][k]. The first failing
// station in index order gives the error: a station without a circular bound
// or with a non-finite error.
func StationErrors(ctx context.Context, verts []r3.Vec, stations []Station,
	idx [][4]int, radius *big.Rat, workers int) ([][4]float64, error) {
	errs := make([][4]float64, len(stations))
	_, err := orderedwork.Each(len(stations), workers, 16, ctx.Err, func(i int) error {
		fail := fmt.Errorf("%w: an outer-ring station has no circular bound", decaderr.ErrUnsupported)
		s, c, ok := proofbound.RadSinCosSpan(proofbound.PointInterval(proofarith.FloatRat(stations[i].Phi)))
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

func (m *Mesh) Point(p r3.Vec) int {
	i := len(m.Verts)
	m.Verts = append(m.Verts, p)
	return i
}

func (m *Mesh) Triangle(a, b, c, source int) {
	m.Tris = append(m.Tris, [3]int{a, b, c})
	m.Src = append(m.Src, source)
}

func (m *Mesh) Quad(a, b, c, d, source int) {
	m.Triangle(a, b, c, source)
	m.Triangle(a, c, d, source)
}

// CapPatches returns the bottom cap fan (source 0) and the top
// cap fan (source 1) as projected patches over the XY axes, each in
// triangle order. Each cap is a fan at one exact height; a proof shows it
// is one non-self-touching graph over XY, so its own pairs can meet only
// at indexed edges and vertices.
func CapPatches(src []int) []loftmesh.ProjectedPatch {
	patches := []loftmesh.ProjectedPatch{{Axes: [2]int{0, 1}}, {Axes: [2]int{0, 1}}}
	for i, s := range src {
		if s == 0 || s == 1 {
			patches[s].Triangles = append(patches[s].Triangles, i)
		}
	}
	return patches
}

// Station is one angular station of the outer ring. Phi is its angle and Kind
// is Positive, Negative or Rest. For a Positive or Negative station, Lower and
// Upper are the mouth's lower and upper vertices and J is the station's index
// within that mouth's half turn. For a Rest station, Lower and Upper are the
// same outer-ring point and J is zero.
type Station struct {
	Phi          float64
	Lower, Upper r3.Vec
	Kind, J      int
}

// The Kind values of a Station.
const (
	Positive = iota
	Negative
	Rest
)

// Feet starts from the exact cylinder-cylinder seam. The outer and
// inner points have equal intrinsic setback on their respective unrolled
// cylinders. The source intersection is x=sqrt(R²-r²cos²t), y=r cos t,
// z=c+r sin t; Q=sqrt(R²-r²cos⁴t) makes both normal offsets explicit.
func Feet(rec *facetedrecord.CrossCylinderBoreRecord, setback, theta float64) (r3.Vec, r3.Vec) {
	R, r, c := rec.Radius, rec.Bore, rec.CenterZ
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

func StationPlan(rec *facetedrecord.CrossCylinderBoreRecord, setback float64) ([]Station, error) {
	const half, rest = HalfStations, restStations
	positive := make([]Station, half+1)
	negative := make([]Station, half+1)
	for j := range positive {
		theta := math.Pi * float64(j) / half
		upper, _ := Feet(rec, setback, theta)
		lower, _ := Feet(rec, setback, 2*math.Pi-theta)
		positive[half-j] = Station{
			Phi: math.Atan2(upper.Y, upper.X), Lower: lower, Upper: upper,
			Kind: Positive, J: j,
		}
		co, s := math.Cos(theta), math.Sin(theta)
		x, y := -math.Sqrt(rec.Radius*rec.Radius-rec.Bore*rec.Bore*co*co), rec.Bore*co
		negative[j] = Station{
			Phi:   math.Atan2(y, x),
			Lower: r3.Vec{X: x, Y: y, Z: rec.CenterZ - rec.Bore*s},
			Upper: r3.Vec{X: x, Y: y, Z: rec.CenterZ + rec.Bore*s},
			Kind:  Negative, J: j,
		}
		if negative[j].Phi < 0 {
			negative[j].Phi += 2 * math.Pi
		}
	}
	for _, stations := range [][]Station{positive, negative} {
		for i := 0; i+1 < len(stations); i++ {
			if !(stations[i].Phi < stations[i+1].Phi) {
				return nil, fmt.Errorf("%w: a cross-bore mouth turns backward", decaderr.ErrUnsupported)
			}
		}
	}
	addRest := func(stations []Station, first, last float64) []Station {
		for k := 1; k < rest; k++ {
			phi := first + (last-first)*float64(k)/rest
			p := r3.Vec{X: rec.Radius * math.Cos(phi),
				Y: rec.Radius * math.Sin(phi), Z: rec.CenterZ}
			stations = append(stations, Station{
				Phi: phi, Lower: p, Upper: p, Kind: Rest,
			})
		}
		return stations
	}
	stations := make([]Station, 0, 2*(half+rest))
	stations = append(stations, positive...)
	stations = addRest(stations, positive[half].Phi, negative[0].Phi)
	stations = append(stations, negative...)
	stations = addRest(stations, negative[half].Phi, positive[0].Phi+2*math.Pi)
	for i, station := range stations {
		if !proofbound.FiniteVec(station.Lower) || !proofbound.FiniteVec(station.Upper) ||
			(i > 0 && !(stations[i-1].Phi < station.Phi)) {
			return nil, fmt.Errorf("%w: a cross-bore station has no finite ordered position", decaderr.ErrUnsupported)
		}
	}
	return stations, nil
}

// StationGap certifies the largest source-cylinder angle covered
// by one outer-wall panel, including the wrap across a full turn. It uses
// each held station angle as an exact rational parameter and keeps the
// full-turn endpoint outward because π is irrational.
func StationGap(stations []Station) (float64, error) {
	maximum := new(big.Rat)
	for i := 0; i+1 < len(stations); i++ {
		gap := new(big.Rat).Sub(proofarith.FloatRat(stations[i+1].Phi),
			proofarith.FloatRat(stations[i].Phi))
		if gap.Sign() <= 0 {
			return 0, fmt.Errorf("%w: an outer-ring angle is not increasing", decaderr.ErrUnsupported)
		}
		if gap.Cmp(maximum) > 0 {
			maximum = gap
		}
	}
	start := proofarith.FloatRat(stations[0].Phi)
	end := proofarith.FloatRat(stations[len(stations)-1].Phi)
	wrapLower := new(big.Rat).Add(new(big.Rat).Mul(proofbound.PiLower, big.NewRat(2, 1)), start)
	wrapLower.Sub(wrapLower, end)
	if wrapLower.Sign() <= 0 {
		return 0, fmt.Errorf("%w: an outer-ring wrap is not increasing", decaderr.ErrUnsupported)
	}
	wrapUpper := new(big.Rat).Add(new(big.Rat).Mul(proofbound.PiUpper, big.NewRat(2, 1)), start)
	wrapUpper.Sub(wrapUpper, end)
	if wrapUpper.Cmp(maximum) > 0 {
		maximum = wrapUpper
	}
	return proofbound.RatFloatUp(maximum), nil
}

// StationParallelMin is the station count from which the station
// errors are computed on several goroutines.
const StationParallelMin = 256
