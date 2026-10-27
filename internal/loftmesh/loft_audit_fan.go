package loftmesh

import (
	"slices"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
)

// loftFanMin is the incident flat triangle count from which a vertex gets an
// exact angle table. Smaller fans are cheaper to certify pair by pair.
const loftFanMin = 32

// loftAuditFans is an exact angular ranking of the rim of every large flat
// fan: a vertex s with at least loftFanMin incident triangles whose corners
// all lie at one z and whose projections all drop the same axis. It is built
// with the audit data before any pair runs and is never written afterwards, so
// concurrent pair workers may read it.
//
// For a fan, the distinct directions of the rim vertices around s are ranked
// exactly (half-plane by exact U/V comparison against s, then
// meshbool.Cross2xSign within a half-plane), and equal directions form one
// class. For every class c the table stores the first class, counted cyclically
// from c, that is not strictly inside the half-turn after c, and whether that
// class is exactly antipodal to c. From these, sign((p-s) x (q-s)) for two rim
// vertices p, q is read without any arithmetic: the same class gives 0, a class
// strictly within the half-turn after p's gives +1, the antipodal class gives 0
// and every other class gives -1. These are exactly the values
// meshbool.Cross2xSign returns for the same points, because they are decided by
// the exact ranking alone; no float angle and no tolerance is involved.
type loftAuditFans struct {
	// fan and cls are per triangle and corner: the id of the fan centred at
	// that corner (-1 when none) and the classes of the other two corners, in
	// corner order.
	fan  [][3]int32
	cls  [][3][2]int32
	fans []loftFanInfo
	tris [][3]int
}

type loftFanInfo struct {
	k    int32   // number of direction classes
	end  []int32 // end[c]: first cyclic offset (1..k) from c outside the open half-turn after c
	anti []bool  // anti[c]: the class at offset end[c] is exactly antipodal to c
}

// newLoftAuditFans builds the fan tables of d over tris, or returns nil when no
// vertex qualifies. d must hold the projections of every z-flat triangle,
// including in a selected table, and its exact lifts must be the float vertices'
// own lifts: a triangle whose corner z values are equal floats is then exactly
// flat at that z. NewLoftAuditDataWithExact data must not be passed.
func newLoftAuditFans(d *LoftAuditData, tris [][3]int) *loftAuditFans {
	nv := len(d.Xverts)
	flat := make([]bool, len(tris))
	counts := make([]int32, nv)
	for t, tri := range tris {
		if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] {
			continue
		}
		c := d.corners(t)
		if c[1].Z-c[0].Z != 0 || c[2].Z-c[0].Z != 0 {
			continue
		}
		flat[t] = true
		for _, v := range tri {
			counts[v]++
		}
	}
	hotID := make([]int32, nv)
	var hot []int32
	for v, n := range counts {
		hotID[v] = -1
		if n >= loftFanMin {
			hotID[v] = int32(len(hot))
			hot = append(hot, int32(v))
		}
	}
	if len(hot) == 0 {
		return nil
	}
	incident := make([][]int32, len(hot))
	for t, tri := range tris {
		if !flat[t] {
			continue
		}
		for _, v := range tri {
			if h := hotID[v]; h >= 0 {
				incident[h] = append(incident[h], int32(t))
			}
		}
	}
	f := &loftAuditFans{
		fan:  make([][3]int32, len(tris)),
		cls:  make([][3][2]int32, len(tris)),
		tris: tris,
	}
	for t := range f.fan {
		f.fan[t] = [3]int32{-1, -1, -1}
	}
	classOf := make([]int32, nv)
	for h, s := range hot {
		f.addFan(d, tris, int(s), incident[h], classOf)
	}
	if len(f.fans) == 0 {
		return nil
	}
	return f
}

type loftFanRim struct {
	vertex int32
	p      meshbool.Xp2
}

// loftFanHalf is 0 for the directions in the half-open upper half-plane around
// v (above it, or level and to its right) and 1 for the others.
func loftFanHalf(v, p meshbool.Xp2) int {
	dv := proof.CmpRat(p.V, v.V)
	if dv > 0 || dv == 0 && proof.CmpRat(p.U, v.U) > 0 {
		return 0
	}
	return 1
}

// addFan ranks the rim of the fan centred at vertex s whose incident flat
// triangles are ts. It records nothing when the triangles do not share one
// projection axis pair or when a rim vertex projects onto s itself.
func (f *loftAuditFans) addFan(d *LoftAuditData, tris [][3]int, s int, ts []int32, classOf []int32) {
	if len(d.certAxes) == 0 {
		return
	}
	axis := d.certAxes[ts[0]]
	for _, t := range ts {
		if d.certAxes[t] != axis {
			return
		}
	}
	var center meshbool.Xp2
	var rim []loftFanRim
	seen := map[int32]bool{}
	for _, t := range ts {
		for pos, q := range tris[t] {
			p := d.projection(int(t), pos)
			if q == s {
				center = p
				continue
			}
			if !seen[int32(q)] {
				seen[int32(q)] = true
				rim = append(rim, loftFanRim{vertex: int32(q), p: p})
			}
		}
	}
	for _, r := range rim {
		if r.p.U.Cmp(center.U) == 0 && r.p.V.Cmp(center.V) == 0 {
			return
		}
	}
	cmp := func(a, b loftFanRim) int {
		ha, hb := loftFanHalf(center, a.p), loftFanHalf(center, b.p)
		if ha != hb {
			return ha - hb
		}
		// Same half-plane: a sorts first when b lies counter-clockwise of it.
		return -meshbool.Cross2xSign(center, a.p, b.p)
	}
	slices.SortStableFunc(rim, cmp)
	var reps []meshbool.Xp2
	for i, r := range rim {
		if i == 0 || cmp(rim[i-1], r) != 0 {
			reps = append(reps, r.p)
		}
		classOf[r.vertex] = int32(len(reps) - 1)
	}
	k := int32(len(reps))
	info := loftFanInfo{k: k, end: make([]int32, k), anti: make([]bool, k)}
	for c := range k {
		// Offsets 1..k-1 read +1 while within the half-turn after c, then 0
		// once at the antipode, then -1: the first offset reading <= 0 is found
		// by binary search.
		lo, hi := int32(1), k
		for lo < hi {
			mid := (lo + hi) / 2
			if meshbool.Cross2xSign(center, reps[c], reps[(c+mid)%k]) > 0 {
				lo = mid + 1
			} else {
				hi = mid
			}
		}
		info.end[c] = lo
		info.anti[c] = lo < k && meshbool.Cross2xSign(center, reps[c], reps[(c+lo)%k]) == 0
	}
	id := int32(len(f.fans))
	f.fans = append(f.fans, info)
	for _, t := range ts {
		pos := slices.Index(tris[t][:], s)
		f.fan[t][pos] = id
		n := 0
		for _, q := range tris[t] {
			if q != s {
				f.cls[t][pos][n] = classOf[q]
				n++
			}
		}
	}
}

// sign is sign((p-s) x (q-s)) for rim classes cp and cq of the fan.
func (fi *loftFanInfo) sign(cp, cq int32) int {
	if cp == cq {
		return 0
	}
	off := (cq - cp + fi.k) % fi.k
	end := fi.end[cp]
	switch {
	case off < end:
		return 1
	case off == end && fi.anti[cp]:
		return 0
	}
	return -1
}

// vertexCertificate is CoplanarIsolatedSharedVertex for triangles i and j,
// which share exactly the vertex shared. The second result is false when
// the pair does not lie in one tabulated fan at shared, and the caller must
// use the unchanged path. When it is true, the first result is the exact value
// CoplanarIsolatedSharedVertex returns, and the two triangles are exactly
// coplanar: both are flat at the z of the common vertex.
func (f *loftAuditFans) vertexCertificate(i, j, shared int) (bool, bool) {
	pi := slices.Index(f.tris[i][:], shared)
	pj := slices.Index(f.tris[j][:], shared)
	if pi < 0 || pj < 0 {
		return false, false
	}
	id := f.fan[i][pi]
	if id < 0 || id != f.fan[j][pj] {
		return false, false
	}
	fi := &f.fans[id]
	ac, bc := f.cls[i][pi], f.cls[j][pj]
	for k := range 2 {
		side := fi.sign(ac[k], ac[1-k])
		if side == 0 {
			continue
		}
		if fi.sign(ac[k], bc[0]) == -side && fi.sign(ac[k], bc[1]) == -side {
			return true, true
		}
	}
	return false, true
}
