package smoothloft

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

type roundedStation struct {
	sign    int64
	xOffset *big.Rat
	y       *big.Rat
	segment int
}

// roundedUnitPolygon has exact cardinal vertices, so straight side stations
// and quarter-arc stations share the same held vertex. Its exact tests retain
// circlePolygon's inscribed outer disk and 1023/1024 inner disk enclosure.
func roundedUnitPolygon(ctx context.Context) ([]sectionrecord.Point2, error) {
	unit, err := circlePolygon(ctx, rint(1))
	if err != nil {
		return nil, err
	}
	unit[0] = sectionrecord.Point2{U: 1}
	unit[24] = sectionrecord.Point2{V: 1}
	unit[48] = sectionrecord.Point2{U: -1}
	unit[72] = sectionrecord.Point2{V: -1}
	inner2 := mul(big.NewRat(1023, 1024), big.NewRat(1023, 1024))
	for i, p := range unit {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		q, next := unit[(i+1)%len(unit)], unit[(i+2)%len(unit)]
		px, py := rf(p.U), rf(p.V)
		qx, qy := rf(q.U), rf(q.V)
		dx, dy := sub(qx, px), sub(qy, py)
		cross := sub(mul(px, qy), mul(py, qx))
		turn := sub(mul(dx, sub(rf(next.V), qy)), mul(dy, sub(rf(next.U), qx)))
		if add(mul(px, px), mul(py, py)).Cmp(rint(1)) > 0 || cross.Sign() <= 0 ||
			turn.Sign() <= 0 ||
			mul(cross, cross).Cmp(mul(inner2, add(mul(dx, dx), mul(dy, dy)))) < 0 {
			return nil, fmt.Errorf("%w: rounded corner polygon has no exact disk enclosure", decaderr.ErrUnsupported)
		}
	}
	return unit, nil
}

func roundedStations(unit []sectionrecord.Point2, h, r *big.Rat, sourceStart int) []roundedStation {
	stations := make([]roundedStation, 0, 100)
	appendStation := func(sign int64, x, y *big.Rat, segment int) {
		stations = append(stations, roundedStation{sign: sign, xOffset: x, y: y,
			segment: (sourceStart + segment) % 8})
	}
	low, high := new(big.Rat).Neg(h), h
	appendStation(-1, r, low, 0)
	for i := 72; i < 96; i++ {
		appendStation(1, add(new(big.Rat).Neg(r), mul(r, rf(unit[i].U))),
			add(add(low, r), mul(r, rf(unit[i].V))), 1)
	}
	appendStation(1, rint(0), add(low, r), 2)
	for i := range 24 {
		appendStation(1, add(new(big.Rat).Neg(r), mul(r, rf(unit[i].U))),
			add(sub(h, r), mul(r, rf(unit[i].V))), 3)
	}
	appendStation(1, new(big.Rat).Neg(r), high, 4)
	for i := 24; i < 48; i++ {
		appendStation(-1, add(r, mul(r, rf(unit[i].U))),
			add(sub(h, r), mul(r, rf(unit[i].V))), 5)
	}
	appendStation(-1, rint(0), sub(h, r), 6)
	for i := 48; i < 72; i++ {
		appendStation(-1, add(r, mul(r, rf(unit[i].U))),
			add(add(low, r), mul(r, rf(unit[i].V))), 7)
	}
	return stations
}

func roundedExactFloat(q *big.Rat) (float64, error) {
	f, _ := q.Float64()
	if math.IsInf(f, 0) || math.IsNaN(f) || rf(f).Cmp(q) != 0 {
		return 0, fmt.Errorf("%w: four rounded section station cannot be held exactly in float64",
			decaderr.ErrUnsupported)
	}
	return f, nil
}

// roundedMesh chords the cubic width in Z while keeping one certified corner
// polygon at every ring. Each side quad is planar: one circular or vertical
// side translates equally at both ends, and a horizontal side has constant Y.
func roundedMesh(ctx context.Context, unit []sectionrecord.Point2, b [4]*big.Rat,
	h, r *big.Rat, origin r3.Vec, height *big.Rat, bands, sourceStart int,
) ([]r3.Vec, [][3]int, []int, error) {
	stations := roundedStations(unit, h, r, sourceStart)
	p := len(stations)
	vertices := make([]r3.Vec, (bands+1)*p)
	for i := 0; i <= bands; i++ {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, err
		}
		t := big.NewRat(int64(i), int64(bands))
		width := roundedCubicAt(b, t)
		z, err := roundedExactFloat(add(rf(origin.Z), mul(height, t)))
		if err != nil {
			return nil, nil, nil, err
		}
		for j, station := range stations {
			x, err := roundedExactFloat(add(rf(origin.X),
				add(mul(rint(station.sign), width), station.xOffset)))
			if err != nil {
				return nil, nil, nil, err
			}
			y, err := roundedExactFloat(add(rf(origin.Y), station.y))
			if err != nil {
				return nil, nil, nil, err
			}
			vertices[i*p+j] = r3.Vec{X: x, Y: y, Z: z}
		}
	}
	triangles := make([][3]int, 0, 2*bands*p+2*(p-2))
	sources := make([]int, 0, cap(triangles))
	for band := range bands {
		for j, station := range stations {
			next := (j + 1) % p
			a0, b0 := band*p+j, band*p+next
			a1, b1 := (band+1)*p+j, (band+1)*p+next
			triangles = append(triangles, [3]int{a0, b0, b1}, [3]int{a0, b1, a1})
			sources = append(sources, station.segment, station.segment)
		}
	}
	capPoints := make([]sectionrecord.Point2, p)
	cycle := make([]int, p)
	for j := range stations {
		capPoints[j] = sectionrecord.Point2{U: vertices[j].X, V: vertices[j].Y}
		cycle[j] = j
	}
	capTriangles, err := triangulation.Triangulate(ctx, capPoints, [][]int{cycle})
	if err != nil {
		return nil, nil, nil, triangulation.WrapLoftError(err)
	}
	for _, tri := range capTriangles {
		triangles = append(triangles, [3]int{tri[0], tri[2], tri[1]},
			[3]int{bands*p + tri[0], bands*p + tri[1], bands*p + tri[2]})
		sources = append(sources, 8, 9)
	}
	if err := loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), vertices, triangles); err != nil {
		return nil, nil, nil, err
	}
	return vertices, triangles, sources, nil
}
