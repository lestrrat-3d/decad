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

const (
	twistMaxU         = 64
	twistMaxT         = 32
	twistMaxTriangles = 60000
	twistMaxScans     = 128_000_000
)

var (
	twistTarget = big.NewRat(1, 50) // mesh distance each patch is refined to, mm
	twistLimit  = big.NewRat(1, 20) // refusal ceiling for the published bound, mm
)

// twistTerms are §13.3's three distance terms of one patch at counts nu, nt.
func twistTerms(b twistBounds, nu, nt int) (uu, tt, ut *big.Rat) {
	u, t := rint(int64(nu)), rint(int64(nt))
	uu = div(b.muu, mul(rint(8), mul(u, u)))
	tt = div(b.mtt, mul(rint(8), mul(t, t)))
	ut = div(b.mut, mul(rint(4), mul(u, t)))
	return uu, tt, ut
}

// ceilSqrt returns the least positive integer n with x/(8n²) <= limit.
func twistCountFor(x, limit *big.Rat) int {
	if x.Sign() == 0 {
		return 1
	}
	n := int(math.Ceil(math.Sqrt(ratFloat(div(x, mul(rint(8), limit))))))
	n = max(n-1, 1)
	for ; div(x, mul(rint(8), mul(rint(int64(n)), rint(int64(n))))).Cmp(limit) > 0; n++ {
	}
	return n
}

func ratFloat(q *big.Rat) float64 {
	f, _ := q.Float64()
	return f
}

// twistCounts picks the station counts. Each of the three distance terms of
// every patch gets a third of the target; the integer counts are the least
// that meet it, so the table is shared and conforming.
func twistCounts(ctx context.Context, bounds [][]twistBounds) ([]int, []int, error) {
	third := div(twistTarget, rint(3))
	nu, nt := make([]int, len(bounds)), make([]int, len(bounds[0]))
	for i := range nt {
		nt[i] = 1
		for k := range bounds {
			nt[i] = max(nt[i], twistCountFor(bounds[k][i].mtt, third))
		}
		if nt[i] > twistMaxT {
			return nil, nil, twistUnsupported("twisted loft exceeds its height-band ceiling for a 0.05 mm mesh bound")
		}
	}
	for k := range bounds {
		if err := ctx.Err(); err != nil {
			return nil, nil, err
		}
		nu[k] = 1
		for i, b := range bounds[k] {
			nu[k] = max(nu[k], twistCountFor(b.muu, third))
			// The twist term mut/(4 nu nt) <= third.
			need := int(math.Ceil(ratFloat(div(b.mut, mul(rint(4), mul(third, rint(int64(nt[i]))))))))
			for need > 1 && div(b.mut, mul(rint(4), mul(rint(int64(need-1)), rint(int64(nt[i]))))).Cmp(third) <= 0 {
				need--
			}
			for div(b.mut, mul(rint(4), mul(rint(int64(need)), rint(int64(nt[i]))))).Cmp(third) > 0 {
				need++
			}
			nu[k] = max(nu[k], need)
		}
		if nu[k] > twistMaxU {
			return nil, nil, twistUnsupported("twisted loft exceeds its station ceiling for a 0.05 mm mesh bound")
		}
	}
	return nu, nt, nil
}

type twistStation struct {
	span int
	u    *big.Rat
}

func buildTwistedMesh(ctx context.Context, patches [][]twistPatch, edgeOf []int, edges int,
	planes []sectionrecord.PlaneRecord, zs []*big.Rat) (Build, error) {
	bounds := make([][]twistBounds, len(patches))
	for k := range patches {
		bounds[k] = make([]twistBounds, len(patches[k]))
		for i := range patches[k] {
			bounds[k][i] = patches[k][i].bounds()
		}
	}
	nu, nt, err := twistCounts(ctx, bounds)
	if err != nil {
		return Build{}, err
	}
	var stations []twistStation
	var stationEdge []int
	for k := range patches {
		for a := range nu[k] {
			stations = append(stations, twistStation{span: k, u: big.NewRat(int64(a), int64(nu[k]))})
			stationEdge = append(stationEdge, edgeOf[k])
		}
	}
	ringSize := len(stations)
	type ring struct{ span, band int }
	var rings []ring
	for i := range nt {
		for b := range nt[i] + 1 {
			if i > 0 && b == 0 {
				continue
			}
			rings = append(rings, ring{span: i, band: b})
		}
	}
	if triangles := 2*ringSize*(len(rings)-1) + 4*(ringSize-2); triangles > twistMaxTriangles {
		return Build{}, twistUnsupported("twisted loft mesh of %d triangles exceeds %d", triangles, twistMaxTriangles)
	}

	origin := planes[0].Origin
	rho := new(big.Rat)
	verts := make([]r3.Vec, len(rings)*ringSize)
	for r, rg := range rings {
		if err := ctx.Err(); err != nil {
			return Build{}, err
		}
		t := big.NewRat(int64(rg.band), int64(nt[rg.span]))
		z := add(zs[rg.span], mul(sub(zs[rg.span+1], zs[rg.span]), t))
		zf, _ := z.Float64()
		for q, st := range stations {
			p := patches[st.span][rg.span].At(st.u, t)
			x, y := add(p.U, rf(origin.X)), add(p.V, rf(origin.Y))
			xf, _ := x.Float64()
			yf, _ := y.Float64()
			v := r3.Vec{X: xf, Y: yf, Z: zf}
			if !proofbound.FiniteVec(v) {
				return Build{}, twistUnsupported("twisted loft station exceeds float64")
			}
			verts[r*ringSize+q] = v
			round := add(add(abs(sub(x, rf(xf))), abs(sub(y, rf(yf)))), abs(sub(z, rf(zf))))
			rho = maxRat(rho, round)
		}
	}
	tris := make([][3]int, 0, 2*ringSize*(len(rings)-1)+4*(ringSize-2))
	sources := make([]int, 0, cap(tris))
	for r := range len(rings) - 1 {
		for q := range ringSize {
			next := (q + 1) % ringSize
			a0, b0 := r*ringSize+q, r*ringSize+next
			a1, b1 := (r+1)*ringSize+q, (r+1)*ringSize+next
			tris = append(tris, [3]int{a0, b0, b1}, [3]int{a0, b1, a1})
			sources = append(sources, stationEdge[q], stationEdge[q])
		}
	}
	capPoints := make([]sectionrecord.Point2, ringSize)
	cycle := make([]int, ringSize)
	for q := range ringSize {
		capPoints[q] = sectionrecord.Point2{U: verts[q].X, V: verts[q].Y}
		cycle[q] = q
	}
	capTris, err := triangulation.Triangulate(ctx, capPoints, [][]int{cycle})
	if err != nil {
		return Build{}, triangulation.WrapLoftError(err)
	}
	last := (len(rings) - 1) * ringSize
	for _, tri := range capTris {
		tris = append(tris, [3]int{tri[0], tri[2], tri[1]}, [3]int{last + tri[0], last + tri[1], last + tri[2]})
		sources = append(sources, edges, edges+1)
	}
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx), verts, tris,
		twistMaxTriangles, twistMaxScans); err != nil {
		return Build{}, err
	}

	// Certificates (§13.3, §13.4). Every term is an exact rational rounded
	// upward once, at the end.
	meshBound, volume, area, radius := new(big.Rat), new(big.Rat), new(big.Rat), new(big.Rat)
	two := mul(rint(2), rho)
	for k := range patches {
		u := rint(int64(nu[k]))
		for i, b := range bounds[k] {
			t := rint(int64(nt[i]))
			uu, tt, ut := twistTerms(b, nu[k], nt[i])
			d := add(add(add(uu, tt), ut), rho)
			meshBound = maxRat(meshBound, d)
			radius = maxRat(radius, b.radius)
			speedU := add(b.bu, mul(two, u))
			step := sub(zs[i+1], zs[i])
			speedT := add(add(b.bt, step), mul(two, t))
			// The sweep integrates the pointwise distance, whose mean over
			// a cell is 2/3 of the sup for a linear-interpolation term and
			// 1/3 of it for the twist term (§13.4).
			mean := add(add(mul(big.NewRat(2, 3), add(uu, tt)), mul(big.NewRat(1, 3), ut)), rho)
			volume = add(volume, mul(mean, mul(speedU, speedT)))
			// Mean derivative deviation over a cell: 1/3 of the sup along
			// the edge being averaged, 1/2 across it (§13.4).
			devU := add(add(div(b.muu, mul(rint(3), u)), div(b.mut, mul(rint(2), t))), mul(two, u))
			devT := add(add(div(b.mut, mul(rint(2), u)), div(b.mtt, mul(rint(3), t))), mul(two, t))
			area = add(area, add(mul(devU, add(b.bt, step)), mul(speedU, devT)))
		}
		// Each cap differs from its ring polygon by the area swept between
		// the true ring and the polygon: the sagitta times the ring speed.
		for _, i := range [...]int{0, len(bounds[k]) - 1} {
			b := bounds[k][i]
			sag := add(div(b.muu, mul(rint(8), mul(u, u))), rho)
			area = add(area, mul(sag, add(b.bu, mul(two, u))))
		}
	}
	if meshBound.Cmp(twistLimit) > 0 {
		return Build{}, twistUnsupported("twisted loft mesh bound exceeds 0.05 mm")
	}
	height := sub(zs[len(zs)-1], zs[0])
	result := Build{Vertices: verts, Triangles: tris, Sources: sources, EdgeCount: edges,
		MeshBound: proofbound.RatFloatUp(meshBound), VolSymDiff: proofbound.RatFloatUp(volume),
		AreaSlack: proofbound.RatFloatUp(area),
		Diameter:  proofbound.RatFloatUp(add(mul(rint(2), radius), height))}
	for _, v := range [...]float64{result.MeshBound, result.VolSymDiff, result.AreaSlack, result.Diameter} {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return Build{}, fmt.Errorf("%w: twisted loft proof exceeds float64", decaderr.ErrUnsupported)
		}
	}
	return result, nil
}
