package crossbore

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// BuildMesh assembles the held cross-bore chamfer mesh from source face groups.
// A negative workers value selects the usual station-error fan-out width.
func BuildMesh(ctx context.Context, rec *facetedrecord.CrossCylinderBoreRecord,
	setback float64, groups []facetedrecord.Group, workers int) (*Mesh, error) {
	stations, err := StationPlan(rec, setback)
	if err != nil {
		return nil, err
	}
	outerGap, err := StationGap(stations)
	if err != nil {
		return nil, err
	}
	m := &Mesh{Groups: groups}
	m.OuterAngularGap = outerGap
	radiusRat := proofarith.FloatRat(rec.Radius)
	bottom := make([]int, len(stations))
	lower := make([]int, len(stations))
	upper := make([]int, len(stations))
	top := make([]int, len(stations))
	vertexIndex := make([][4]int, len(stations))
	n := 2 * HalfStations
	positiveOuter, negativeOuter := make([]int, n), make([]int, n)
	for i, station := range stations {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		x, y := rec.Radius*math.Cos(station.Phi), rec.Radius*math.Sin(station.Phi)
		bottom[i] = m.Point(r3.Vec{X: x, Y: y})
		lower[i] = m.Point(station.Lower)
		upper[i] = lower[i]
		if station.Lower.Sub(station.Upper).Len() >= 1e-13 {
			upper[i] = m.Point(station.Upper)
		}
		top[i] = m.Point(r3.Vec{X: x, Y: y, Z: rec.Height})
		vertexIndex[i] = [4]int{bottom[i], lower[i], upper[i], top[i]}
		if station.Kind == Positive {
			positiveOuter[station.J] = upper[i]
			positiveOuter[(n-station.J)%n] = lower[i]
		}
		if station.Kind == Negative {
			negativeOuter[station.J] = upper[i]
			negativeOuter[(n-station.J)%n] = lower[i]
		}
	}
	if workers < 0 {
		workers = orderedwork.EachWorkers(len(stations), StationParallelMin)
	}
	errs, err := StationErrors(ctx, m.Verts, stations, vertexIndex, radiusRat, workers)
	if err != nil {
		return nil, err
	}
	for _, station := range errs {
		for _, e := range station {
			m.OuterStationError = max(m.OuterStationError, e)
		}
	}
	bottomCenter, topCenter := m.Point(r3.Vec{}), m.Point(r3.Vec{Z: rec.Height})
	for i := range stations {
		j := (i + 1) % len(stations)
		m.Quad(bottom[i], bottom[j], lower[j], lower[i], 2)
		m.Quad(upper[i], upper[j], top[j], top[i], 2)
		m.Triangle(bottomCenter, bottom[j], bottom[i], 0)
		m.Triangle(topCenter, top[i], top[j], 1)
	}
	inner := make([]int, n)
	m.CellHeld = make([][3]int, n)
	for i := range inner {
		theta := 2 * math.Pi * float64(i) / float64(n)
		_, foot := Feet(rec, setback, theta)
		inner[i] = m.Point(foot)
		m.CellHeld[i] = [3]int{positiveOuter[i], inner[i], negativeOuter[i]}
	}
	for i := range inner {
		j := (i + 1) % n
		m.Quad(negativeOuter[i], inner[i], inner[j], negativeOuter[j], 3)
		m.Quad(positiveOuter[i], positiveOuter[j], inner[j], inner[i], 4)
	}
	return m, nil
}
