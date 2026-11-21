package decad

import (
	"context"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/brepmodify"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type rimPoint [3]*big.Rat

func rimRat(v float64) *big.Rat     { return new(big.Rat).SetFloat64(v) }
func rimSub(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }
func rimAdd(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }
func rimMul(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
func rimVec(p r3.Vec) rimPoint      { return rimPoint{rimRat(p.X), rimRat(p.Y), rimRat(p.Z)} }
func rimRound(p rimPoint) (r3.Vec, float64) {
	var xyz [3]float64
	sum := new(big.Rat)
	for i, q := range p {
		xyz[i], _ = q.Float64()
		e := rimSub(q, rimRat(xyz[i]))
		sum.Add(sum, e.Abs(e))
	}
	return r3.Vec{X: xyz[0], Y: xyz[1], Z: xyz[2]}, proofbound.RatFloatUp(sum)
}
func directUpperRim(ctx context.Context, env topo.Env, b *topo.Body, capFace, wall *topo.Face, center rimPoint,
	hinge *big.Rat, radius, d, height float64, budget *proofbound.ReplayBudget) (brepmodify.Result, bool, error) {
	return directUpperRimAt(ctx, env, b, capFace, wall, center, hinge, radius, d, height, budget, nil)
}

func directUpperRimAt(ctx context.Context, env topo.Env, b *topo.Body, capFace, wall *topo.Face, center rimPoint,
	trueHinge *big.Rat, radius, d, height float64, budget *proofbound.ReplayBudget, foot *alignedRimFoot) (brepmodify.Result, bool, error) {
	hinge := trueHinge
	var mapOffset, footGap float64
	if foot != nil {
		hinge = foot.held
		// The taper vanishes outside 9R/8. Throughout its support a
		// change in the hinge moves the radial map by at most 9/8 of it.
		mapOffset = proofbound.ProductUpper(1.125, foot.displacement)
		footGap = foot.gap
	}
	var none brepmodify.Result
	old, ok := b.Payload.(facetedrecord.Payload)
	if !ok || len(old.Verts) > 65536 || len(old.Groups) > 65536 || len(old.Tris) > 65536 || len(old.Tris) != len(old.FaceOf) || len(old.Src) != len(old.Tris) || len(old.Verts) != len(old.VertexBound) || !(d > 0 && d <= radius/8) {
		return none, false, nil
	}
	if err := budget.ReserveBundle(ctx, uint64(len(old.Tris)*8), uint64(len(old.Verts)+len(old.Tris)), uint64(len(old.Verts)*2048+len(old.Tris)*2048+len(old.Groups)*1024)); err != nil {
		return none, false, err
	}
	// Bound all dyadic input exponents before allocating chart rationals.
	for _, q := range append(center[:], hinge) {
		if q == nil || q.Num().BitLen() > 192 || q.Denom().BitLen() > 192 {
			return none, false, nil
		}
	}
	finite := func(x float64) bool {
		_, e := math.Frexp(x)
		return !math.IsNaN(x) && !math.IsInf(x, 0) && (x == 0 || e >= -96 && e <= 96)
	}
	if !finite(radius) || !finite(d) {
		return none, false, nil
	}
	for i, v := range old.Verts {
		if !finite(v.X) || !finite(v.Y) || !finite(v.Z) {
			return none, false, nil
		}
		if i%256 == 0 && ctx.Err() != nil {
			return none, false, ctx.Err()
		}
	}
	faces := topobuild.Faces(b)
	capIndex, wallIndex := slices.Index(faces, capFace), slices.Index(faces, wall)
	if capIndex < 0 || wallIndex < 0 {
		return none, false, nil
	}
	p := facetedrecord.Payload{Verts: slices.Clone(old.Verts), VertexBound: slices.Clone(old.VertexBound), Groups: slices.Clone(old.Groups), Xform: r3.Identity(), DPair: old.DPair}
	blend := len(p.Groups)
	p.Groups = append(p.Groups, facetedrecord.Group{Origins: []provenance.Ref{{Producer: env.Ref, Role: "chamfer(0)"}}, Source: facetedrecord.BlendSource(env.Ref, 0)})
	R := rimRat(radius)
	r0 := rimMul(R, big.NewRat(33, 32))
	r1 := rimMul(R, big.NewRat(9, 8))
	rr := rimMul(r0, r0)
	maxDepth := rimAdd(rimRat(d), rimMul(R, big.NewRat(1, 128)))
	// Extend the map to the empty annulus between 33R/32 and 9R/8, linearly
	// tapering the radial displacement to zero. At each y it is increasing
	// in radius. Its radial/axial Jacobian has norm below three for d<=R/8.
	cache := map[int]rimPoint{}
	cuts := map[[2]int]int{}
	moved := map[int]bool{}
	get := func(i int) rimPoint {
		q, yes := cache[i]
		if !yes {
			q = rimVec(old.Verts[i])
			cache[i] = q
		}
		return q
	}
	rounding := 0.
	put := func(i int, q rimPoint, move bool) {
		f, e := rimRound(q)
		p.Verts[i] = f
		rounding = max(rounding, e)
		if move {
			moved[i] = true
		}
	}
	lo := r3.Vec{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
	hi := lo.Scale(-1)
	growBox := func(v r3.Vec) {
		lo = r3.Vec{X: min(v.X, lo.X), Y: min(v.Y, lo.Y), Z: min(v.Z, lo.Z)}
		hi = r3.Vec{X: max(v.X, hi.X), Y: max(v.Y, hi.Y), Z: max(v.Z, hi.Z)}
	}
	caps, lower := map[int]bool{}, map[int]bool{}
	for i, g := range old.Src {
		if old.FaceOf[i] == capIndex {
			caps[g] = true
		}
		if old.FaceOf[i] == wallIndex {
			lower[g] = true
		}
	}
	var cheekGroups map[int]bool
	if foot != nil {
		if e := budget.ReserveBundle(ctx, uint64(len(old.Tris)*4), uint64(len(old.Groups)*2), uint64(len(old.Groups)*128)); e != nil {
			return none, false, e
		}
		cheekGroups = map[int]bool{}
		foreign := map[int]bool{}
		for i, g := range old.Src {
			fi := old.FaceOf[i]
			if fi < 0 || fi >= len(faces) {
				return none, false, nil
			}
			if _, ok := foot.cheek[faces[fi]]; ok {
				cheekGroups[g] = true
			} else {
				foreign[g] = true
			}
		}
		for g := range foreign {
			delete(cheekGroups, g)
		}
	}
	var maxInterp float64
	for index, t := range old.Tris {
		if len(p.Tris)+3 > loftmesh.MaxLoftAuditTriangles || len(p.Verts) > 65536 {
			return none, false, nil
		}
		if index%128 == 0 {
			if err := ctx.Err(); err != nil {
				return none, false, err
			}
		}
		points := [3]r3.Vec{old.Verts[t[0]], old.Verts[t[1]], old.Verts[t[2]]}
		// An unchanged triangle entirely below the hinge is its own image.
		h, _ := hinge.Float64()
		if max(points[0].Y, points[1].Y, points[2].Y) < math.Nextafter(h, math.Inf(-1)) {
			p.Tris = append(p.Tris, t)
			p.Src = append(p.Src, old.Src[index])
			continue
		}
		// Outside a cardinal support halfspace the map is identity.
		outer := false
		for _, axis := range []int{0, 2} {
			mn := rimAdd(center[axis], r1)
			mx := rimSub(center[axis], r1)
			plus, minus := true, true
			for _, v := range t {
				q := get(v)[axis]
				plus = plus && q.Cmp(mn) >= 0
				minus = minus && q.Cmp(mx) <= 0
			}
			outer = outer || plus || minus
		}
		if outer {
			p.Tris = append(p.Tris, t)
			p.Src = append(p.Src, old.Src[index])
			continue
		}
		var signs [3]int
		positive := false
		for k, v := range t {
			q := get(v)
			signs[k] = q[1].Cmp(hinge)
			positive = positive || signs[k] > 0
		}
		if !positive {
			p.Tris = append(p.Tris, t)
			p.Src = append(p.Src, old.Src[index])
			continue
		}
		if err := budget.ReserveBundle(ctx, 1024, 128, 262144); err != nil {
			return none, false, err
		}
		for _, v := range points {
			growBox(v)
		}
		fi := old.FaceOf[index]
		if fi != wallIndex && fi != capIndex {
			return none, false, nil
		}
		for _, v := range t {
			q := get(v)
			x, z := rimSub(q[0], center[0]), rimSub(q[2], center[2])
			depth := rimSub(q[1], hinge)
			if rimAdd(rimMul(x, x), rimMul(z, z)).Cmp(rr) > 0 || depth.Cmp(maxDepth) > 0 {
				return none, false, nil
			}
			if depth.Sign() > 0 && !moved[v] {
				k := new(big.Rat).Quo(depth, R)
				put(v, rimPoint{rimSub(q[0], rimMul(k, x)), q[1], rimSub(q[2], rimMul(k, z))}, true)
			}
		}
		// Subdivide only a crossing nose triangle. Each edge's shared rational
		// intersection is rounded once; neighboring facets use the same index.
		for _, side := range []int{-1, 1} {
			polygon := make([]int, 0, 4)
			for k, a := range t {
				l := (k + 1) % 3
				b := t[l]
				if signs[k]*side >= 0 {
					polygon = append(polygon, a)
				}
				if signs[k]*signs[l] >= 0 {
					continue
				}
				key := [2]int{min(a, b), max(a, b)}
				id, yes := cuts[key]
				if !yes {
					va, vb := get(a), get(b)
					u := new(big.Rat).Quo(rimSub(hinge, va[1]), rimSub(vb[1], va[1]))
					q := rimPoint{rimAdd(va[0], rimMul(u, rimSub(vb[0], va[0]))), hinge, rimAdd(va[2], rimMul(u, rimSub(vb[2], va[2])))}
					id = len(p.Verts)
					cuts[key] = id
					cache[id] = q
					p.Verts = append(p.Verts, r3.Vec{})
					p.VertexBound = append(p.VertexBound, old.MeshBound)
					put(id, q, false)
				}
				polygon = append(polygon, id)
			}
			for k := 1; k+1 < len(polygon); k++ {
				out := [3]int{polygon[0], polygon[k], polygon[k+1]}
				p.Tris = append(p.Tris, out)
				for _, v := range out {
					growBox(p.Verts[v])
				}
				group := old.Src[index]
				if side > 0 && fi == wallIndex {
					group = blend
				}
				p.Src = append(p.Src, group)
				if side > 0 {
					var span [3]*big.Rat
					for a := range 3 {
						lo, hi := get(out[0])[a], get(out[0])[a]
						for _, v := range out[1:] {
							q := get(v)[a]
							if q.Cmp(lo) < 0 {
								lo = q
							}
							if q.Cmp(hi) > 0 {
								hi = q
							}
						}
						span[a] = rimSub(hi, lo)
					}
					// Cauchy-Schwarz and the variance bound range²/4 for each coordinate.
					spanXZ := rimAdd(span[0], span[2])
					e := new(big.Rat).Quo(rimMul(span[1], spanXZ), rimMul(big.NewRat(4, 1), R))
					maxInterp = max(maxInterp, proofbound.RatFloatUp(e))
				}
			}
		}
	}
	if e := budget.ReserveBundle(ctx, uint64(len(p.Tris)*32), uint64(8*len(p.Tris)), uint64(2048*len(p.Tris))); e != nil {
		return none, false, e
	}
	chart, err := rimBandChart(ctx, &p, blend, center, budget)
	if err != nil {
		return none, false, err
	}
	if !chart || !rimCapChartAt(ctx, &p, blend, caps, lower, cheekGroups, hinge) {
		if ctx.Err() != nil {
			return none, false, ctx.Err()
		}
		return none, false, nil
	}
	if len(moved) == 0 {
		return none, false, nil
	}
	chartBound, ok := rimBandError(ctx, &p, blend, center, trueHinge, radius, d)
	if !ok {
		if ctx.Err() != nil {
			return none, false, ctx.Err()
		}
		return none, false, nil
	}
	p.MeshBound = old.MeshBound
	// A source counterpart can cross the hinge by at most its old facet
	// bound. Its projection back to the common circle moves by at most twice
	// that amount (one axial and one radial setback). Outside the support
	// expanded by its facet bound, both surfaces retain the identity map.
	chartBound = proofbound.AbsSumUpper(chartBound, footGap)
	localMax := chartBound
	hingeLo, _ := hinge.Float64()
	xLo, xHi := rimSupportInterval(center[0], r1)
	zLo, zHi := rimSupportInterval(center[2], r1)
	for i, t := range old.Tris {
		if i%256 == 0 {
			if e := ctx.Err(); e != nil {
				return none, false, e
			}
		}
		v, w, z := old.Verts[t[0]], old.Verts[t[1]], old.Verts[t[2]]
		beta := max(old.VertexBound[t[0]], old.VertexBound[t[1]], old.VertexBound[t[2]])
		// The annulus proof excludes all faces except the cap, wall and
		// authenticated cheek patches from the true bevel support.
		f := faces[old.FaceOf[i]]
		p.Groups[old.Src[i]].RestrictMeshPlaneSlab(f, foot == nil || f == capFace || f == wall || foot.cheek[f] != nil)

		if math.Nextafter(max(v.Y, w.Y, z.Y)+beta, math.Inf(1)) < math.Nextafter(hingeLo, math.Inf(-1)) {
			continue
		}
		if min(v.X, w.X, z.X) > math.Nextafter(xHi+beta, math.Inf(1)) || max(v.X, w.X, z.X) < math.Nextafter(xLo-beta, math.Inf(-1)) || min(v.Z, w.Z, z.Z) > math.Nextafter(zHi+beta, math.Inf(1)) || max(v.Z, w.Z, z.Z) < math.Nextafter(zLo-beta, math.Inf(-1)) {
			continue
		}
		local := max(chartBound, proofbound.AbsSumUpper(proofbound.ProductUpper(5, beta), maxInterp, rounding, mapOffset))
		localMax = max(localMax, local)
		for _, j := range t {
			p.VertexBound[j] = max(p.VertexBound[j], local)
		}
	}
	for j := len(old.Verts); j < len(p.Verts); j++ {
		p.VertexBound[j] = localMax
	}
	for _, bound := range p.VertexBound {
		p.MeshBound = max(p.MeshBound, bound)
	}
	// Both occupied sets can change only in the cylinder's bounding prism.
	// Any changed held triangle may extend slightly below its true hinge;
	// the previous mesh bound encloses that discrepancy. Bound the entire
	// held swept region independently from the true removed wedge.
	width := math.Nextafter(hi.X-lo.X, math.Inf(1))
	heightY := math.Nextafter(hi.Y-lo.Y, math.Inf(1))
	depth := math.Nextafter(hi.Z-lo.Z, math.Inf(1))
	// Differences between embedded old and new held solids are contained in
	// the bounding box of all changed triangles: outside their swept box,
	// the oriented boundary has unchanged winding number.
	boxVol := proofbound.ProductUpper(width, proofbound.ProductUpper(heightY, depth))
	trueRemoved := proofbound.ProductUpper(4, proofbound.ProductUpper(radius, proofbound.ProductUpper(d, d)))
	p.VolSymDiff = proofbound.AbsSumUpper(old.VolSymDiff, boxVol, trueRemoved)
	// The former and new cap and side groups derive solely from one complete
	// source cylinder. Their possible true areas fit two disks plus one
	// cylinder and one sloping band. Other groups continue the original proof.
	oneDisk := proofbound.ProductUpper(4, proofbound.ProductUpper(radius, radius))
	lateral := proofbound.ProductUpper(8, proofbound.ProductUpper(radius, proofbound.AbsSumUpper(height, 2*d)))
	p.AreaSlack = proofbound.AbsSumUpper(old.AreaSlack, 4*oneDisk, 2*lateral)
	// Charge every changed group's held area too. The original and new group
	// partitions can differ at the hinge; this bound is valid for each part.
	for i, t := range p.Tris {
		n := p.Src[i]
		if n != blend && !slices.ContainsFunc(p.Groups[n].Origins, func(o provenance.Ref) bool {
			return slices.Contains(capFace.Origins, o) || slices.Contains(wall.Origins, o)
		}) {
			continue
		}
		a, c, e := p.Verts[t[0]], p.Verts[t[1]], p.Verts[t[2]]
		p.AreaSlack = proofbound.AbsSumUpper(p.AreaSlack, proofbound.ProductUpper(proofbound.DvLenUpper(proofbound.HeldDelta(c, a)), proofbound.DvLenUpper(proofbound.HeldDelta(e, a)))/2)
	}
	for i := range p.Groups {
		p.Groups[i].HasCentroid = false
	}
	p.DPair = proofbound.AbsSumUpper(old.DPair, 2*p.MeshBound, 2*d)
	if !old.BooleanSimilaritySource || !b.Solid || old.Xform != r3.Identity() || !facetedrecord.CertifiedFacetedSimilaritySource(old, len(faces)) {
		return none, false, nil
	}
	local, ok, err := rimLocalTriangles(ctx, old, p, lo, hi, budget)
	if !ok || err != nil {
		return none, false, err
	}
	return publishRim(ctx, env, p, local, budget)
}

func publishRim(ctx context.Context, env topo.Env, p facetedrecord.Payload, local [][3]int, budget *proofbound.ReplayBudget) (brepmodify.Result, bool, error) {
	var none brepmodify.Result
	if len(p.Tris) > loftmesh.MaxLoftAuditTriangles {
		return none, false, nil
	}
	work := budget.WorkBudget(ctx)
	if err := loftmesh.LoftCrossingAuditBounded(work, p.Verts, local, loftmesh.MaxLoftAuditTriangles, solidFromMeshScans); err != nil {
		return none, false, err
	}
	admitted, err := budget.BeginRetainedRimPublication(ctx, len(p.Verts), len(p.Tris))
	if !admitted || err != nil {
		return none, false, err
	}
	p.ContactAudited = true
	audit, ok, err := facetproof.AuditFacetedMeshReplay(ctx, work, p.Verts, p.Tris)
	if !ok || err != nil {
		return none, false, err
	}
	v, b, rat, ok, err := facetproof.VolumeFromAuditReplay(ctx, work, audit, p.VolSymDiff)
	if !ok || err != nil {
		return none, false, err
	}
	vol := measurement.Measurement{Value: units.CubicMillimeters(v), Bound: units.CubicMillimeters(b), Exactness: measurement.ExactnessOf(b)}
	out, ok, err := facetedrecord.BuildWithProofReplay(ctx, work, env, p, audit, vol, rat, true)
	if !ok || err != nil {
		return none, false, err
	}

	return brepmodify.Result{Body: out}, true, nil
}

func rimSupportInterval(center, reach *big.Rat) (float64, float64) {
	return proofbound.RatFloatDown(rimSub(center, reach)), proofbound.RatFloatUp(rimAdd(center, reach))
}

func rimLocalTriangles(ctx context.Context, old, p facetedrecord.Payload, lo, hi r3.Vec, budget *proofbound.ReplayBudget) ([][3]int, bool, error) {
	if err := budget.ReserveBundle(ctx, uint64(len(old.Tris)+len(p.Tris)), uint64(len(old.Tris)+len(p.Tris)), uint64(96*(len(old.Tris)+len(p.Tris)))); err != nil {
		return nil, false, err
	}
	oldFaces := make(map[[3]int]bool, len(old.Tris))
	for _, t := range old.Tris {
		oldFaces[t] = true
	}
	seen := map[[3]int]bool{}
	var local [][3]int
	for _, t := range p.Tris {
		a, b, c := p.Verts[t[0]], p.Verts[t[1]], p.Verts[t[2]]
		seen[t] = true
		outside := min(a.X, b.X, c.X) > hi.X || max(a.X, b.X, c.X) < lo.X || min(a.Y, b.Y, c.Y) > hi.Y || max(a.Y, b.Y, c.Y) < lo.Y || min(a.Z, b.Z, c.Z) > hi.Z || max(a.Z, b.Z, c.Z) < lo.Z
		if outside {
			if !oldFaces[t] || old.Verts[t[0]] != a || old.Verts[t[1]] != b || old.Verts[t[2]] != c {
				return nil, false, nil
			}
			continue
		}
		if !oldFaces[t] || old.Verts[t[0]] != a || old.Verts[t[1]] != b || old.Verts[t[2]] != c {
			for _, v := range [...]r3.Vec{a, b, c} {
				if v.X < lo.X || v.X > hi.X || v.Y < lo.Y || v.Y > hi.Y || v.Z < lo.Z || v.Z > hi.Z {
					return nil, false, nil
				}
			}
		}
		local = append(local, t)
	}
	for _, t := range old.Tris {
		if seen[t] {
			continue
		}
		for _, i := range t {
			v := old.Verts[i]
			if v.X < lo.X || v.X > hi.X || v.Y < lo.Y || v.Y > hi.Y || v.Z < lo.Z || v.Z > hi.Z {
				return nil, false, nil
			}
		}
	}
	return local, len(local) > 0, nil
}
