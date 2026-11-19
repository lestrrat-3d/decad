package smoothloft

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

const (
	twistMaxU         = 64
	twistMaxT         = 32
	twistMaxTriangles = 500_000
	twistMaxScans     = 4096 * twistMaxTriangles
)

var (
	twistLimit = big.NewRat(1, 20) // refusal ceiling for the published bound, mm
	// twistTargets are the mesh distances each patch may be refined to, finest
	// first. The first that fits twistMaxTriangles is used.
	twistTargets = []*big.Rat{big.NewRat(1, 50), big.NewRat(3, 100), big.NewRat(49, 1000)}
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
func twistCounts(ctx context.Context, bounds [][]twistBounds, target *big.Rat) ([]int, []int, error) {
	third := div(target, rint(3))
	nu, nt := make([]int, len(bounds)), make([]int, len(bounds[0]))
	for i := range nt {
		nt[i] = 1
		for k := range bounds {
			maxTerm := mul(mul(rint(8), third), rint(twistMaxT*twistMaxT))
			if bounds[k][i].mtt.Cmp(maxTerm) > 0 {
				return nil, nil, twistUnsupported("twisted loft exceeds its height-band ceiling for a 0.05 mm mesh bound")
			}
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
			maxTerm := mul(mul(rint(8), third), rint(twistMaxU*twistMaxU))
			if b.muu.Cmp(maxTerm) > 0 {
				return nil, nil, twistUnsupported("twisted loft exceeds its station ceiling for a 0.05 mm mesh bound")
			}
			nu[k] = max(nu[k], twistCountFor(b.muu, third))
			// The twist term mut/(4 nu nt) <= third.
			mixedMax := mul(mul(rint(4*twistMaxU), rint(int64(nt[i]))), third)
			if b.mut.Cmp(mixedMax) > 0 {
				return nil, nil, twistUnsupported("twisted loft exceeds its station ceiling for a 0.05 mm mesh bound")
			}
			need := max(1, int(math.Ceil(ratFloat(div(b.mut, mul(rint(4), mul(third, rint(int64(nt[i])))))))))
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

// twistStation is the station u = a/nu[span].
type twistStation struct {
	span int
	a    int64
}

// twistSelectCounts tries twistTargets finest first and returns the station
// counts of the first whose table holds at most maxTriangles triangles. A
// target whose counts exceed a station ceiling is skipped. It reports
// ErrUnsupported when no target fits, and ctx.Err() when ctx is done.
func twistSelectCounts(ctx context.Context, bounds [][]twistBounds, maxTriangles int) ([]int, []int, error) {
	for _, target := range twistTargets {
		nu, nt, err := twistCounts(ctx, bounds, target)
		if err != nil {
			if ctxErr := ctx.Err(); ctxErr != nil {
				return nil, nil, ctxErr
			}
			continue
		}
		if twistTriangleCount(nu, nt) <= maxTriangles {
			return nu, nt, nil
		}
	}
	return nil, nil, twistUnsupported("twisted loft needs more than %d triangles or its station ceilings for a 0.05 mm mesh bound",
		maxTriangles)
}

func buildTwistedMesh(ctx context.Context, patches [][]twistPatch, edgeOf []int, edges int,
	planes []sectionrecord.PlaneRecord, zs []*big.Rat) (Build, error) {
	return buildSectionMesh(ctx, patches, nil, edgeOf, edges, planes, zs)
}

func buildSectionMesh(ctx context.Context, patches [][]twistPatch, rational [][]rationalTransition,
	edgeOf []int, edges int, planes []sectionrecord.PlaneRecord, zs []*big.Rat) (Build, error) {
	return buildSectionMeshMode(ctx, patches, rational, edgeOf, edges, planes, zs, nil, 0)
}

func buildSectionMeshMode(ctx context.Context, patches [][]twistPatch, rational [][]rationalTransition,
	edgeOf []int, edges int, planes []sectionrecord.PlaneRecord, zs []*big.Rat,
	emerging []bool, firstFit int) (Build, error) {
	count := len(patches)
	if rational != nil {
		count = len(rational)
	}
	bounds := make([][]twistBounds, count)
	commons := make([][]*twistCommon, count)
	for k := range bounds {
		if rational != nil {
			bounds[k] = make([]twistBounds, len(rational[k]))
			for i, p := range rational[k] {
				if err := ctx.Err(); err != nil {
					return Build{}, err
				}
				var err error
				bounds[k][i], err = p.bounds()
				if err != nil {
					return Build{}, err
				}
			}
			continue
		}
		bounds[k] = make([]twistBounds, len(patches[k]))
		commons[k] = make([]*twistCommon, len(patches[k]))
		for i := range patches[k] {
			commons[k][i] = patches[k][i].overCommon()
			bounds[k][i] = commons[k][i].bounds()
		}
	}
	nu, nt, err := twistSelectCounts(ctx, bounds, twistMaxTriangles)
	if err != nil {
		return Build{}, err
	}
	var stations []twistStation
	var stationEdge []int
	for k := range bounds {
		for a := range nu[k] {
			stations = append(stations, twistStation{span: k, a: int64(a)})
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
	origin := planes[0].Origin
	o := twistOriginOf(rf(origin.X), rf(origin.Y))
	weights := make([][4]int64, ringSize)
	spanStart := make([]int, len(bounds)+1)
	for q, st := range stations {
		weights[q] = twistBernsteinScaled(st.a, int64(nu[st.span]))
		spanStart[st.span+1] = q + 1
	}
	rhos := make([]*big.Rat, len(rings))
	verts := make([]r3.Vec, len(rings)*ringSize)
	err = twistParallel(ctx, len(rings), func(r int) error {
		rg := rings[r]
		t := big.NewRat(int64(rg.band), int64(nt[rg.span]))
		z := add(zs[rg.span], mul(sub(zs[rg.span+1], zs[rg.span]), t))
		zf, _ := z.Float64()
		wt := twistBernsteinScaled(int64(rg.band), int64(nt[rg.span]))
		var rho twistRoundMax
		exactRho := new(big.Rat)
		var x, y big.Int
		for k := range bounds {
			if rational != nil {
				for q := spanStart[k]; q < spanStart[k+1]; q++ {
					u := big.NewRat(stations[q].a, int64(nu[k]))
					p := rational[k][rg.span].at(u, t)
					x, y := add(p.U, rf(origin.X)), add(p.V, rf(origin.Y))
					xf, yf := ratFloat(x), ratFloat(y)
					v := r3.Vec{X: xf, Y: yf, Z: zf}
					if !proofbound.FiniteVec(v) {
						return twistUnsupported("circular fitted loft station exceeds float64")
					}
					verts[r*ringSize+q] = v
					exactRho = maxRat(exactRho, add(abs(sub(x, rf(xf))), abs(sub(y, rf(yf)))))
				}
				continue
			}
			curve := commons[k][rg.span].curveAt(wt, nu[k], nt[rg.span], o)
			for q := spanStart[k]; q < spanStart[k+1]; q++ {
				curve.point(weights[q], &x, &y)
				xf := proof.FracFloat64(&x, &curve.den)
				yf := proof.FracFloat64(&y, &curve.den)
				v := r3.Vec{X: xf, Y: yf, Z: zf}
				if !proofbound.FiniteVec(v) {
					return twistUnsupported("twisted loft station exceeds float64")
				}
				verts[r*ringSize+q] = v
				rho.keep(&x, &y, &curve.den, xf, yf)
			}
		}
		// Z is the same at every station of the ring, so its rounding adds
		// to the largest planar rounding.
		rhos[r] = add(maxRat(rho.rat(), exactRho), abs(sub(z, rf(zf))))
		return nil
	})
	if err != nil {
		return Build{}, err
	}
	rho := new(big.Rat)
	for _, r := range rhos {
		rho = maxRat(rho, r)
	}
	// Keep the virtual grid for bounds, but alias the inactive flat to its
	// authenticated source vertex in the emitted topology.
	indices := make([]int, len(verts))
	if len(emerging) == 0 || !emerging[0] {
		for i := range indices {
			indices[i] = i
		}
	} else {
		flatSpans := 0
		for flatSpans < len(emerging) && emerging[flatSpans] {
			flatSpans++
		}
		flatEnd := spanStart[flatSpans]
		compact := make([]r3.Vec, 0, len(verts))
		for r, rg := range rings {
			collapsed := rg.span < firstFit-1 || (rg.span == firstFit-1 && rg.band == 0)
			apex := -1
			for q := range ringSize {
				if collapsed && q <= flatEnd {
					if apex < 0 {
						apex = len(compact)
						compact = append(compact, verts[r*ringSize+q])
					}
					indices[r*ringSize+q] = apex
				} else {
					indices[r*ringSize+q] = len(compact)
					compact = append(compact, verts[r*ringSize+q])
				}
			}
		}
		verts = compact
	}
	tris := make([][3]int, 0, 2*ringSize*(len(rings)-1)+4*(ringSize-2))
	sources := make([]int, 0, cap(tris))
	appendTri := func(a, b, c, source int) {
		if a != b && b != c && c != a {
			tris = append(tris, [3]int{a, b, c})
			sources = append(sources, source)
		}
	}
	for r := range len(rings) - 1 {
		for q := range ringSize {
			next := (q + 1) % ringSize
			a0, b0 := indices[r*ringSize+q], indices[r*ringSize+next]
			a1, b1 := indices[(r+1)*ringSize+q], indices[(r+1)*ringSize+next]
			appendTri(a0, b0, b1, stationEdge[q])
			appendTri(a0, b1, a1, stationEdge[q])
		}
	}
	for capIndex, r := range [...]int{0, len(rings) - 1} {
		cycle := make([]int, 0, ringSize)
		points := make([]sectionrecord.Point2, 0, ringSize)
		ids := make([]int, 0, ringSize)
		for q := range ringSize {
			id := indices[r*ringSize+q]
			if len(ids) > 0 && id == ids[len(ids)-1] {
				continue
			}
			ids = append(ids, id)
		}
		if len(ids) > 1 && ids[0] == ids[len(ids)-1] {
			ids = ids[:len(ids)-1]
		}
		for i, id := range ids {
			cycle = append(cycle, i)
			points = append(points, sectionrecord.Point2{U: verts[id].X, V: verts[id].Y})
		}
		capTris, err := triangulation.Triangulate(ctx, points, [][]int{cycle})
		if err != nil {
			return Build{}, triangulation.WrapLoftError(err)
		}
		for _, tri := range capTris {
			if capIndex == 0 {
				tri[1], tri[2] = tri[2], tri[1]
			}
			appendTri(ids[tri[0]], ids[tri[1]], ids[tri[2]], edges+capIndex)
		}
	}
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx), verts, tris,
		twistMaxTriangles, twistMaxScans); err != nil {
		return Build{}, err
	}

	// Certificates (§13.3, §13.4). Every term is an exact rational rounded
	// upward once, at the end.
	meshBound, volume, area, radius := new(big.Rat), new(big.Rat), new(big.Rat), new(big.Rat)
	two := mul(rint(2), rho)
	for k := range bounds {
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

// twistRoundMax keeps the largest exact planar rounding |x/d − xf| +
// |y/d − yf| of a ring's stations as the fraction num/den, comparing
// fractions by cross multiplication so no GCD reduces them. Its zero value
// holds zero.
type twistRoundMax struct {
	num, den    big.Int
	n, dn, t, u big.Int
}

// keep charges one station: x/d and y/d are its exact coordinates, d > 0,
// and xf, yf the finite floats they round to. With f = m·2^e, |x/d − f| is
// |x·2^s − m·d·2^(e+s)| / (d·2^s) for any s >= −e.
func (r *twistRoundMax) keep(x, y, d *big.Int, xf, yf float64) {
	mx, ex := twistMantExp(xf)
	my, ey := twistMantExp(yf)
	s := max(0, -ex, -ey)
	r.n.Lsh(x, uint(s))
	r.t.Lsh(r.t.Mul(d, big.NewInt(mx)), uint(ex+s))
	r.n.Sub(&r.n, &r.t)
	r.n.Abs(&r.n)
	r.u.Lsh(y, uint(s))
	r.t.Lsh(r.t.Mul(d, big.NewInt(my)), uint(ey+s))
	r.u.Sub(&r.u, &r.t)
	r.n.Add(&r.n, r.u.Abs(&r.u))
	r.dn.Lsh(d, uint(s))
	if r.den.Sign() == 0 {
		r.num.Set(&r.n)
		r.den.Set(&r.dn)
		return
	}
	// n/dn > num/den exactly when n·den > num·dn, both denominators positive.
	if r.t.Mul(&r.n, &r.den).Cmp(r.u.Mul(&r.num, &r.dn)) > 0 {
		r.num.Set(&r.n)
		r.den.Set(&r.dn)
	}
}

// rat is the largest charged rounding, zero when nothing was charged.
func (r *twistRoundMax) rat() *big.Rat {
	if r.den.Sign() == 0 {
		return new(big.Rat)
	}
	return new(big.Rat).SetFrac(&r.num, &r.den)
}

// twistMantExp writes a finite f as m·2^e with m an integer of at most 53
// bits.
func twistMantExp(f float64) (int64, int) {
	if f == 0 {
		return 0, 0
	}
	frac, exp := math.Frexp(f)
	return int64(frac * (1 << 53)), exp - 53
}

// twistTriangleCount is the held triangle count of a station table.
func twistTriangleCount(nu, nt []int) int {
	ring, bands := 0, 0
	for _, n := range nu {
		ring += n
	}
	for _, n := range nt {
		bands += n
	}
	return 2*ring*bands + 4*(ring-2)
}
