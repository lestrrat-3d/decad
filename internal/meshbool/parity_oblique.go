package meshbool

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// obliqueRays are the directions the parity scan falls back to when every
// axis ray is ambiguous. A query on a mesh's symmetry lines — the equator and
// a meridian of a torus, say — projects onto facet edges along all six axis
// rays; a ray off every coordinate plane passes through no such line. Each
// direction is integral, so the exact sign tests stay integer products.
var obliqueRays = [...][3]int64{{3, 5, 7}, {-5, 7, 3}, {7, -3, 5}, {-3, -7, 5}, {5, 3, -7}}

// obliqueParity answers the parity query along the oblique rays in order:
// inside, onBoundary, and whether some ray decided it. facet(i) names the
// i-th facet of the scan, count of them. A ray that meets a facet's edge or
// vertex, by an exact zero sign, is ambiguous and the next is tried.
//
// For q = p + d the line p q crosses facet (a, b, c) through its interior
// exactly when the three volumes ((a−p)×(b−p))·d, ((b−p)×(c−p))·d and
// ((c−p)×(a−p))·d share one nonzero sign; their sum is (b−a)×(c−a)·d, so a
// line parallel to the facet never does. The crossing lies on the ray when
// (a−p)·n and d·n, n = (b−a)×(c−a), share a sign, and p lies on the facet
// when the first is zero.
func (pm *ParityMesh) obliqueParity(ctx context.Context, p proof.Xpt, count int, facet func(int) int) (bool, bool, bool, error) {
	pf := p.Vec()
	finite := !math.IsInf(pf.X, 0) && !math.IsInf(pf.Y, 0) && !math.IsInf(pf.Z, 0) &&
		!math.IsNaN(pf.X) && !math.IsNaN(pf.Y) && !math.IsNaN(pf.Z)
	for _, dir := range obliqueRays {
		d := proof.Xpt{X: big.NewInt(dir[0]), Y: big.NewInt(dir[1]), Z: big.NewInt(dir[2]), W: big.NewInt(1)}
		df := [3]float64{float64(dir[0]), float64(dir[1]), float64(dir[2])}
		crossings := 0
		ambiguous, onBoundary := false, false
		for i := range count {
			if i%256 == 0 {
				if err := ctx.Err(); err != nil {
					return false, false, false, err
				}
			}
			ti := facet(i)
			tri := pm.Tris[ti]
			if finite && rayMissesBox(pf, df, pm.Verts[tri[0]], pm.Verts[tri[1]], pm.Verts[tri[2]]) {
				continue
			}
			xa, xb, xc := pm.exactVertex(tri[0]), pm.exactVertex(tri[1]), pm.exactVertex(tri[2])
			ua, ub, uc := proof.Xsub(xa, p), proof.Xsub(xb, p), proof.Xsub(xc, p)
			s1 := proof.XdotSign(proof.Xcross(ua, ub), d)
			s2 := proof.XdotSign(proof.Xcross(ub, uc), d)
			s3 := proof.XdotSign(proof.Xcross(uc, ua), d)
			neg := s1 < 0 || s2 < 0 || s3 < 0
			pos := s1 > 0 || s2 > 0 || s3 > 0
			if neg && pos {
				continue
			}
			if s1 == 0 || s2 == 0 || s3 == 0 {
				ambiguous = true
				break
			}
			n := proof.Xcross(proof.Xsub(xb, xa), proof.Xsub(xc, xa))
			t := proof.XdotSign(ua, n)
			if t == 0 {
				onBoundary = true
				break
			}
			if t*proof.XdotSign(n, d) > 0 {
				crossings++
			}
		}
		if onBoundary {
			return false, true, true, nil
		}
		if ambiguous {
			continue
		}
		return crossings%2 == 1, false, true, nil
	}
	return false, false, false, nil
}

// exactVertex is vertex vi's exact point: the borrowed Boolean geometry when
// present, else the float vertex lifted exactly.
func (pm *ParityMesh) exactVertex(vi int) proof.Xpt {
	if len(pm.exactVerts) == len(pm.Verts) {
		return pm.exactVerts[vi]
	}
	return proof.XptOf(pm.Verts[vi])
}

// rayMissesBox reports that the ray p + t·d, t ≥ 0, provably misses the
// facet's float box widened by a margin far past every rounding the float
// slab test commits, so a skipped facet is one the exact test would also find
// the ray misses. It decides no sign.
func rayMissesBox(p r3.Vec, d [3]float64, a, b, c r3.Vec) bool {
	pc := [3]float64{p.X, p.Y, p.Z}
	lo := [3]float64{min(a.X, b.X, c.X), min(a.Y, b.Y, c.Y), min(a.Z, b.Z, c.Z)}
	hi := [3]float64{max(a.X, b.X, c.X), max(a.Y, b.Y, c.Y), max(a.Z, b.Z, c.Z)}
	scale := 1.0
	for k := range 3 {
		scale = max(scale, math.Abs(lo[k]), math.Abs(hi[k]), math.Abs(pc[k]))
	}
	if math.IsInf(scale, 0) || scale > 0x1p500 {
		return false
	}
	pad := 1e-9 * scale
	tmin, tmax := 0.0, math.Inf(1)
	for k := range 3 {
		t1 := (lo[k] - pad - pc[k]) / d[k]
		t2 := (hi[k] + pad - pc[k]) / d[k]
		if t1 > t2 {
			t1, t2 = t2, t1
		}
		tmin, tmax = max(tmin, t1), min(tmax, t2)
	}
	return tmin > tmax
}
