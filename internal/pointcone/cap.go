package pointcone

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Subdivide splits the point-loft source fan (verts, tris and src, with the
// apex as vertex 0) into its far cap and refines it rounds times by midpoints.
// It returns the far-cap vertices, the cap triangles, the boundary sides, and
// the midpoint rounding bound.
func Subdivide(ctx context.Context, verts []r3.Vec, tris [][3]int, src []int, rounds int) (
	[]r3.Vec, [][3]int, []Side, float64, error) {
	if len(verts) < 4 {
		return nil, nil, nil, 0, fmt.Errorf("%w: the point-cone source has no fan", decaderr.ErrUnsupported)
	}
	far := append([]r3.Vec(nil), verts[1:]...)
	boundaryGroups := map[Edge]int{}
	for i, tri := range tris {
		if tri[0] == 0 {
			boundaryGroups[OrderedEdge(tri[1]-1, tri[2]-1)] = src[i]
		}
	}
	var caps [][3]int
	var sides []Side
	for i, tri := range tris {
		if tri[0] == 0 || tri[1] == 0 || tri[2] == 0 {
			continue
		}
		t := [3]int{tri[0] - 1, tri[1] - 1, tri[2] - 1}
		caps = append(caps, t)
		for j := range 3 {
			u, v := t[j], t[(j+1)%3]
			if group, ok := boundaryGroups[OrderedEdge(u, v)]; ok {
				sides = append(sides, Side{u, v, group})
			}
		}
		_ = i
	}
	totalRound := 0.0
	for range rounds {
		levelRound := 0.0
		midpoints := map[Edge]int{}
		midpoint := func(u, v int) int {
			key := OrderedEdge(u, v)
			if index, ok := midpoints[key]; ok {
				return index
			}
			p, q := far[u], far[v]
			held := p.Scale(0.5).Add(q.Scale(0.5))
			levelRound = max(levelRound, MidpointRound(p, q, held))
			index := len(far)
			far = append(far, held)
			midpoints[key] = index
			return index
		}
		next := make([][3]int, 0, len(caps)*4)
		for _, t := range caps {
			if err := ctx.Err(); err != nil {
				return nil, nil, nil, 0, err
			}
			a, b, c := t[0], t[1], t[2]
			ab, bc, ca := midpoint(a, b), midpoint(b, c), midpoint(c, a)
			next = append(next, [3]int{a, ab, ca}, [3]int{ab, b, bc},
				[3]int{ca, bc, c}, [3]int{ab, bc, ca})
		}
		newSides := make([]Side, 0, len(sides)*2)
		for _, side := range sides {
			m := midpoints[OrderedEdge(side.U, side.V)]
			newSides = append(newSides, Side{side.U, m, side.Group},
				Side{m, side.V, side.Group})
		}
		caps, sides = next, newSides
		totalRound = proofbound.AbsSumUpper(totalRound, levelRound)
	}
	return far, caps, sides, totalRound, nil
}

// SplitAtContact splits the far cap where the cone reaches it (sample Q = 1).
// It returns the outside and inside cap triangles, the outside and all sides,
// the extended far vertices and samples, and the contact rounding bound.
func SplitAtContact(far []r3.Vec, caps [][3]int, sides []Side,
	samples []Sample) ([][3]int, [][3]int, []Side, []Side,
	[]r3.Vec, []Sample, float64, error) {
	contact := map[Edge]int{}
	maxRound := 0.0
	crossingFailure := false
	crossing := func(u, v int) int {
		if samples[u].Q == 1 {
			return u
		}
		if samples[v].Q == 1 {
			return v
		}
		key := OrderedEdge(u, v)
		if index, ok := contact[key]; ok {
			return index
		}
		qu, qv := samples[u].Q, samples[v].Q
		t := (1 - qu) / (qv - qu)
		if !(t > 0 && t < 1) {
			crossingFailure = true
			return u
		}
		p := far[u].Scale(1 - t).Add(far[v].Scale(t))
		tr := new(big.Rat).Quo(new(big.Rat).Sub(big.NewRat(1, 1), proofarith.FloatRat(qu)),
			new(big.Rat).Sub(proofarith.FloatRat(qv), proofarith.FloatRat(qu)))
		oneminust := new(big.Rat).Sub(big.NewRat(1, 1), tr)
		exact := [3]*big.Rat{}
		for axis, pair := range [...][2]float64{{far[u].X, far[v].X}, {far[u].Y, far[v].Y}, {far[u].Z, far[v].Z}} {
			exact[axis] = new(big.Rat).Add(new(big.Rat).Mul(oneminust, proofarith.FloatRat(pair[0])),
				new(big.Rat).Mul(tr, proofarith.FloatRat(pair[1])))
		}
		maxRound = max(maxRound, proofbound.Radius3D(max(
			proofbound.RatAbsDiff(exact[0], p.X),
			proofbound.RatAbsDiff(exact[1], p.Y),
			proofbound.RatAbsDiff(exact[2], p.Z))))
		index := len(far)
		far = append(far, p)
		samples = append(samples, Sample{Q: 1, Alpha: 1})
		contact[key] = index
		return index
	}
	outside := make([][3]int, 0, len(caps))
	inside := make([][3]int, 0, len(caps))
	for _, tri := range caps {
		for _, arm := range []struct {
			keepOutside bool
			output      *[][3]int
		}{{true, &outside}, {false, &inside}} {
			if arm.keepOutside && samples[tri[0]].Q <= 1 && samples[tri[1]].Q <= 1 && samples[tri[2]].Q <= 1 {
				continue
			}
			if !arm.keepOutside && samples[tri[0]].Q >= 1 && samples[tri[1]].Q >= 1 && samples[tri[2]].Q >= 1 {
				continue
			}
			poly := []int{tri[0], tri[1], tri[2]}
			out := make([]int, 0, 4)
			for j := range poly {
				u, v := poly[j], poly[(j+1)%len(poly)]
				uIn, vIn := samples[u].Q >= 1, samples[v].Q >= 1
				if !arm.keepOutside {
					uIn, vIn = samples[u].Q <= 1, samples[v].Q <= 1
				}
				if uIn {
					out = append(out, u)
				}
				if uIn != vIn {
					out = append(out, crossing(u, v))
				}
			}
			poly = poly[:0]
			for _, index := range out {
				if len(poly) == 0 || poly[len(poly)-1] != index {
					poly = append(poly, index)
				}
			}
			if len(poly) > 1 && poly[0] == poly[len(poly)-1] {
				poly = poly[:len(poly)-1]
			}
			for j := 1; j+1 < len(poly); j++ {
				*arm.output = append(*arm.output, [3]int{poly[0], poly[j], poly[j+1]})
			}
		}
	}
	outsideSides := make([]Side, 0, len(sides))
	allSides := make([]Side, 0, len(sides)*2)
	for _, side := range sides {
		u, v := side.U, side.V
		qu, qv := samples[u].Q, samples[v].Q
		if qu == 1 || qv == 1 || (qu > 1) == (qv > 1) {
			allSides = append(allSides, side)
			if qu > 1 || qv > 1 {
				outsideSides = append(outsideSides, side)
			}
			continue
		}
		m := crossing(u, v)
		first, second := Side{u, m, side.Group}, Side{m, v, side.Group}
		allSides = append(allSides, first, second)
		if qu > 1 {
			outsideSides = append(outsideSides, first)
		} else {
			outsideSides = append(outsideSides, second)
		}
	}
	if crossingFailure {
		return nil, nil, nil, nil, nil, nil, 0,
			fmt.Errorf("%w: a point-cone contact rounds onto a cap vertex", decaderr.ErrUnsupported)
	}
	return outside, inside, outsideSides, allSides, far, samples, maxRound, nil
}

// Compact renumbers verts to the vertices tris use, in first-use order.
func Compact(verts []r3.Vec, tris [][3]int) ([]r3.Vec, [][3]int) {
	index := make(map[int]int, len(verts))
	out := make([]r3.Vec, 0, len(verts))
	for i := range tris {
		for j, old := range tris[i] {
			fresh, ok := index[old]
			if !ok {
				fresh = len(out)
				index[old] = fresh
				out = append(out, verts[old])
			}
			tris[i][j] = fresh
		}
	}
	return out, tris
}

func ReachUpper(far []r3.Vec) float64 {
	reach := 0.0
	for _, p := range far {
		reach = max(reach, proofbound.DvLenUpper(proofbound.HeldDelta(p, r3.Vec{})))
	}
	return reach
}

// CapArea is the cap's area upper bound, with no vertex perturbation.
func CapArea(far []r3.Vec, caps [][3]int) float64 {
	return proofbound.PerturbedAreaUpper(far, caps, 0)
}

// MidpointRound is the rounding error of the held midpoint of p and
// q: the largest per-axis distance from the exact midpoint, as a 3-D radius.
func MidpointRound(p, q, held r3.Vec) float64 {
	dp, dq := proofarith.DyVec(p), proofarith.DyVec(q)
	heldAxes := [3]float64{held.X, held.Y, held.Z}
	var e [3]float64
	for axis := range 3 {
		exact := proofarith.DyShift(proofarith.DyAdd(dp[axis], dq[axis]), -1)
		e[axis] = proofarith.DyRoundedFloatError(exact, heldAxes[axis])
	}
	return proofbound.Radius3D(max(e[0], e[1], e[2]))
}

// CellGeom is the cone-independent half of TriangleBound, in millimetres:
// a lower bound on the cell's axial coordinate and on its distance from the
// axis, its greatest reach from the origin and its longest edge.
type CellGeom struct{ xMin, rhoMin, reach, h float64 }

// capParallelMin is the cell count from which CapGeometry and CapBound work
// on several goroutines.
const capParallelMin = 1024

// CapGeometry reads every cap cell's geometry about axis once, so two cones
// about that axis share it. A vertex's reach is the same DvLenUpper of the
// same exact vector in each cell holding it, and is computed once.
func CapGeometry(far []r3.Vec, caps [][3]int, axis coneaxis.Axis) []CellGeom {
	workers := orderedwork.EachWorkers(len(caps), capParallelMin)
	return capGeometry(far, caps, axis, workers)
}

// capGeometry is CapGeometry on up to workers goroutines. It runs the serial
// loop when workers < 2 or a far vertex is not finite.
func capGeometry(far []r3.Vec, caps [][3]int, axis coneaxis.Axis, workers int) []CellGeom {
	reach := make([]float64, len(far))
	out := make([]CellGeom, len(caps))
	if workers >= 2 && !proofbound.AllFinite(far) {
		workers = 1
	}
	if workers < 2 {
		done := make([]bool, len(far))
		for k, tri := range caps {
			for _, v := range tri {
				if done[v] {
					continue
				}
				reach[v] = proofbound.DvLenUpper(proofbound.HeldDelta(far[v], r3.Vec{}))
				done[v] = true
			}
			out[k] = cellGeometryWith([3]r3.Vec{far[tri[0]], far[tri[1]], far[tri[2]]},
				[3]float64{reach[tri[0]], reach[tri[1]], reach[tri[2]]}, axis)
		}
		return out
	}
	used := make([]bool, len(far))
	for _, tri := range caps {
		used[tri[0]], used[tri[1]], used[tri[2]] = true, true, true
	}
	_, _ = orderedwork.Each(len(far), workers, 512, nil, func(v int) error {
		if used[v] {
			reach[v] = proofbound.DvLenUpper(proofbound.HeldDelta(far[v], r3.Vec{}))
		}
		return nil
	})
	_, _ = orderedwork.Each(len(caps), workers, 256, nil, func(k int) error {
		tri := caps[k]
		out[k] = cellGeometryWith([3]r3.Vec{far[tri[0]], far[tri[1]], far[tri[2]]},
			[3]float64{reach[tri[0]], reach[tri[1]], reach[tri[2]]}, axis)
		return nil
	})
	return out
}

// CellGeometry reads one cap cell's geometry about axis.
func CellGeometry(tri [3]r3.Vec, axis coneaxis.Axis) CellGeom {
	var reach [3]float64
	for i, p := range tri {
		reach[i] = proofbound.DvLenUpper(proofbound.HeldDelta(p, r3.Vec{}))
	}
	return cellGeometryWith(tri, reach, axis)
}

// cellGeometryWith reads one cell's geometry from its vertices and their
// reaches. Reach and h are millimetre distances for every axis. For a general
// axis, xMin is the exact smallest s(p) converted down to millimetres, and
// rhoMin is the §3.5 lower bound on σ divided by |A|↑ (agent-docs/
// revolve-axis-design.md §3.4, §3.5). A cell whose conversion fails gets
// rhoMin = 0, which TriangleBound refuses.
func cellGeometryWith(tri [3]r3.Vec, vertexReach [3]float64, axis coneaxis.Axis) CellGeom {
	if !axis.Canonical() {
		return cellGeometryAxis(tri, vertexReach, axis)
	}
	xMin, yMin, yMax, zMin, zMax := math.Inf(1), math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)
	reach, h := 0.0, 0.0
	for i, p := range tri {
		xMin = min(xMin, p.X)
		yMin, yMax = min(yMin, p.Y), max(yMax, p.Y)
		zMin, zMax = min(zMin, p.Z), max(zMax, p.Z)
		reach = max(reach, vertexReach[i])
		h = max(h, proofbound.DvLenUpper(proofbound.HeldDelta(p, tri[(i+1)%3])))
	}
	minAbs := func(lo, hi float64) float64 {
		if lo <= 0 && hi >= 0 {
			return 0
		}
		return min(math.Abs(lo), math.Abs(hi))
	}
	rhoMin := max(minAbs(yMin, yMax), minAbs(zMin, zMax))
	return CellGeom{xMin: xMin, rhoMin: rhoMin, reach: reach, h: h}
}

func cellGeometryAxis(tri [3]r3.Vec, vertexReach [3]float64, axis coneaxis.Axis) CellGeom {
	reach, h := 0.0, 0.0
	sMin := axis.Axial(tri[0])
	for i, p := range tri {
		sMin = proofarith.DyMin(sMin, axis.Axial(p))
		reach = max(reach, vertexReach[i])
		h = max(h, proofbound.DvLenUpper(proofbound.HeldDelta(p, tri[(i+1)%3])))
	}
	geom := CellGeom{reach: reach, h: h}
	xMin, ok := axis.ScaledDown(sMin.Rat())
	if !ok {
		return geom
	}
	sigma, ok := axis.RadialLower(tri)
	if !ok {
		return geom
	}
	rhoMin, ok := axis.ScaledDown(proofarith.FloatRat(sigma))
	if !ok {
		return geom
	}
	geom.xMin, geom.rhoMin = xMin, rhoMin
	return geom
}

// CapBound is the largest cell bound and Lipschitz value of TriangleBound over
// precomputed cell geometry.
func CapBound(geom []CellGeom, c *Consts) (float64, float64, error) {
	return capBound(geom, c, orderedwork.EachWorkers(len(geom), capParallelMin))
}

// capBound is CapBound with the cells' TriangleBound formed on up to workers
// goroutines. It returns the error of the lowest failing cell and otherwise
// folds the cells in order. It runs the serial loop when workers < 2 or a
// cell field is not finite, so no cell the serial loop never reaches can panic.
func capBound(geom []CellGeom, c *Consts, workers int) (float64, float64, error) {
	if workers >= 2 && geomFinite(geom) {
		bounds, lips := make([]float64, len(geom)), make([]float64, len(geom))
		if _, err := orderedwork.Each(len(geom), workers, 256, nil, func(k int) error {
			var err error
			bounds[k], _, lips[k], err = TriangleBound(geom[k], c)
			return err
		}); err != nil {
			return 0, 0, err
		}
		capBound, lip := 0.0, 0.0
		for k := range geom {
			capBound = max(capBound, bounds[k])
			lip = max(lip, lips[k])
		}
		return capBound, lip, nil
	}
	capBound, lip := 0.0, 0.0
	for _, g := range geom {
		cellBound, _, cellLip, err := TriangleBound(g, c)
		if err != nil {
			return 0, 0, err
		}
		capBound = max(capBound, cellBound)
		lip = max(lip, cellLip)
	}
	return capBound, lip, nil
}

// geomFinite reports whether every field of every cell is finite.
func geomFinite(geom []CellGeom) bool {
	for _, g := range geom {
		if !proofbound.FiniteVec(r3.Vec{X: g.xMin, Y: g.rhoMin, Z: g.h}) || !proofbound.FiniteVec(r3.Vec{X: g.reach}) {
			return false
		}
	}
	return true
}

// TriangleBound forms five rationals, each formed as one Dyadic numerator over one
// Dyadic denominator and rounded up once (proofarith.DyQuoFloatUp), so no
// big.Rat is normalised. With a = an/ad, k = kn/kd and G = xMin·kd + kn·rhoMin
// (so gMin = G/kd):
//
//	reciprocal = an(kd+kn)²kd·h² / (ad·G³)
//	radial     = an·kn·kd·h² / (ad·2·rhoMin·G²)
//	product    = an(kd+kn)kd·h² / (ad·G²)
//	alphaMax   = an·kd / (ad·G)
//	lip ratio  = an(kd+kn)kd / (ad·G²)
//
// It works over the cell geometry computed by CellGeometry or CapGeometry.
//
// c is the bound constant set of ConstantsOf. A nil c refuses.
func TriangleBound(geom CellGeom, c *Consts) (float64, float64, float64, error) {
	if c == nil {
		return 0, 0, 0, fmt.Errorf("%w: the point-cone axis has no finite norm bracket", decaderr.ErrUnsupported)
	}
	xMin, rhoMin, reach, h := geom.xMin, geom.rhoMin, geom.reach, geom.h
	if rhoMin <= 0 {
		return 0, 0, 0, fmt.Errorf("%w: the point-cone cap reaches the axis", decaderr.ErrUnsupported)
	}
	x, rho, hd := proofarith.MustDyOf(xMin), proofarith.MustDyOf(rhoMin), proofarith.MustDyOf(h)
	g := proofarith.DyAdd(proofarith.DyMul(x, c.kd), proofarith.DyMul(c.kn, rho))
	if g.Sign() <= 0 {
		return 0, 0, 0, fmt.Errorf("%w: the point-cone cap has no positive ray denominator", decaderr.ErrUnsupported)
	}
	h2 := proofarith.DyMul(hd, hd)
	g2 := proofarith.DyMul(g, g)
	g3 := proofarith.DyMul(g2, g)
	adG2 := proofarith.DyMul(c.ad, g2)
	alphaBound := proofbound.AbsSumUpper(
		proofarith.DyQuoFloatUp(proofarith.DyMul(c.cRec, h2), proofarith.DyMul(c.ad, g3)),
		proofarith.DyQuoFloatUp(proofarith.DyMul(c.cRad, h2),
			proofarith.DyMul(proofarith.DyShift(rho, 1), adG2)))
	capBound := proofbound.AbsSumUpper(
		proofbound.ProductUpper(reach, alphaBound),
		proofarith.DyQuoFloatUp(proofarith.DyMul(c.cProd, h2), adG2))
	alphaMax := proofarith.DyQuoFloatUp(c.cAlpha, proofarith.DyMul(c.ad, g))
	lip := proofbound.AbsSumUpper(alphaMax,
		proofbound.ProductUpper(reach, proofarith.DyQuoFloatUp(c.cProd, adG2)))
	return capBound, alphaBound, lip, nil
}
