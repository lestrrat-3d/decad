// Package meshplane groups a mesh's triangles by the exact plane through their
// float vertices and reads each plane group's exact area centroid, for
// SolidFromMesh (agent-docs/mesh-solid-design.md §7).
package meshplane

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Group is one source group of a mesh: the triangles that lie on one oriented
// plane, or a single triangle of no area.
type Group struct {
	// Planar reports a group of triangles with area. A triangle of no area is a
	// group of its own and is not Planar.
	Planar bool
	// First is the index of the group's first triangle.
	First int
	// Normal is the exact normal (b−a)×(c−a) of the group's first triangle abc;
	// it is zero for a group that is not Planar.
	Normal proof.DyV3
	// CentroidNum/CentroidDen is the exact area centroid of a Planar group: the
	// sum over its triangles abc of w·(a+b+c) over three times the sum of w, with
	// w the dot product of each triangle's normal and Normal. Coplanar triangles
	// of one orientation have areas in the ratio of those dot products, so the
	// weights are exact. A group of one triangle holds a+b+c over 3, the same
	// rational with w cancelled. Both are zero for a group that is not Planar.
	CentroidNum proof.DyV3
	CentroidDen proof.Dyadic
	// Centroid is that rational rounded to the nearest float per coordinate,
	// with the largest rounding error, rounded upward, as its bound. HasCentroid
	// reports whether it was read.
	Centroid    measurement.VecMeasurement
	HasCentroid bool
}

// Groups assigns each triangle of the mesh a group index and returns the
// groups in the order their first triangles appear. Triangles share a group
// exactly when their normals n = (b−a)×(c−a) and offsets n·a, each divided by
// the magnitude of the normal's first non-zero part, are equal as rationals;
// so a plane and its triangles' orientation have one group. A triangle of no
// area is a group of its own. The vertices MUST be finite.
//
// All arithmetic is exact over the vertices' own dyadic values. The plane of a
// triangle is the tuple (n, n·a); two triangles share it exactly when the
// tuples are positive multiples of each other, and samePlane decides that by
// cross-multiplying with the two normals' first non-zero magnitudes, which is
// the rational equality above with both sides multiplied by those two
// positive numbers. A float key only buckets the planes: it is the downward
// rounding of each rational n_i/|n_k| and n·a/|n_k|, a function of the plane
// alone, so triangles of one plane always share a bucket, and only samePlane
// joins two triangles.
func Groups(verts []r3.Vec, tris [][3]int) ([]int, []Group) {
	exact := make([]proof.DyV3, len(verts))
	for i, v := range verts {
		exact[i] = proof.DyVec(v)
	}
	src := make([]int, len(tris))
	normals := make([]proof.DyV3, len(tris))
	groups := make([]Group, 0, len(tris))
	planes := make([]plane, 0, len(tris))
	count := make([]int, 0, len(tris))
	heads := make(map[planeKey]int, len(tris))
	for ti, t := range tris {
		a, b, c := exact[t[0]], exact[t[1]], exact[t[2]]
		n := proof.DvCross(proof.DvSub(b, a), proof.DvSub(c, a))
		normals[ti] = n
		p, planar := planeOf(n, a)
		if !planar {
			src[ti] = len(groups)
			groups = append(groups, Group{First: ti})
			planes = append(planes, plane{next: -1})
			count = append(count, 1)
			continue
		}
		key := p.key()
		gi, ok := heads[key]
		for ok && !samePlane(planes[gi], p) {
			gi = planes[gi].next
			ok = gi >= 0
		}
		if !ok {
			gi = len(groups)
			// The new group heads its bucket, and the bucket's earlier groups follow it.
			p.next = -1
			if head, found := heads[key]; found {
				p.next = head
			}
			heads[key] = gi
			groups = append(groups, Group{Planar: true, First: ti, Normal: n})
			planes = append(planes, p)
			count = append(count, 0)
		}
		src[ti] = gi
		count[gi]++
	}
	// A group of several triangles sums its weights and moments; slot maps it to its accumulator.
	slot := make([]int, len(groups))
	var accs []groupSums
	for gi, g := range groups {
		slot[gi] = -1
		if g.Planar && count[gi] > 1 {
			slot[gi] = len(accs)
			accs = append(accs, groupSums{})
		}
	}
	for ti, gi := range src {
		if k := slot[gi]; k >= 0 {
			t := tris[ti]
			accs[k].add(normals[ti], groups[gi].Normal, exact[t[0]], exact[t[1]], exact[t[2]])
		}
	}
	three := proof.DyInt(3)
	for gi := range groups {
		g := &groups[gi]
		if !g.Planar {
			continue
		}
		if k := slot[gi]; k >= 0 {
			s := &accs[k]
			g.CentroidNum = proof.DyV3{s.moment[0].Dyadic(), s.moment[1].Dyadic(), s.moment[2].Dyadic()}
			g.CentroidDen = proof.DyMul(s.weight.Dyadic(), three)
		} else {
			t := tris[g.First]
			g.CentroidNum = cornerSum(exact[t[0]], exact[t[1]], exact[t[2]])
			g.CentroidDen = three
		}
		g.Centroid, g.HasCentroid = centroid(g.CentroidNum, g.CentroidDen)
	}
	return src, groups
}

// plane is a triangle's plane (n, offset) with scale the magnitude of n's
// first non-zero part, and next the group of the same key read before it, or −1.
type plane struct {
	n      proof.DyV3
	offset proof.Dyadic
	scale  proof.Dyadic
	next   int
}

// planeKey is the bucket of a plane.
type planeKey [4]float64

// planeOf returns the plane of the triangle with normal n through a, or false
// for a zero normal.
func planeOf(n proof.DyV3, a proof.DyV3) (plane, bool) {
	n0, n1, n2 := n[0], n[1], n[2]
	var scale proof.Dyadic
	switch {
	case !n0.IsZero():
		scale = proof.DyAbs(n0)
	case !n1.IsZero():
		scale = proof.DyAbs(n1)
	case !n2.IsZero():
		scale = proof.DyAbs(n2)
	default:
		return plane{}, false
	}
	return plane{n: n, offset: proof.DvDot(n, a), scale: scale}, true
}

// key rounds each part of the plane divided by its scale downward. The
// rounding is a function of the exact rational quotient, and positive
// multiples of one plane have equal quotients, so they have one key.
func (p plane) key() planeKey {
	n0, n1, n2 := p.n[0], p.n[1], p.n[2]
	return planeKey{
		keyPart(n0, p.scale), keyPart(n1, p.scale), keyPart(n2, p.scale), keyPart(p.offset, p.scale),
	}
}

// keyPart is DyQuoFloatDown(x, scale). A finite quotient never rounds to NaN;
// mapping NaN to zero still keeps every key equal to itself.
func keyPart(x, scale proof.Dyadic) float64 {
	f := proof.DyQuoFloatDown(x, scale)
	if f != f {
		return 0
	}
	return f
}

// samePlane reports whether p and q are one plane: p.n/p.scale = q.n/q.scale and
// p.offset/p.scale = q.offset/q.scale, compared as x·q.scale = y·p.scale with
// both scales positive.
func samePlane(p, q plane) bool {
	same := func(x, y proof.Dyadic) bool {
		return proof.DyCmp(proof.DyMul(x, q.scale), proof.DyMul(y, p.scale)) == 0
	}
	p0, p1, p2 := p.n[0], p.n[1], p.n[2]
	q0, q1, q2 := q.n[0], q.n[1], q.n[2]
	return same(p0, q0) && same(p1, q1) && same(p2, q2) && same(p.offset, q.offset)
}

// groupSums accumulates, for the triangles of one plane group, the exact weighted sum of their corner sums.
type groupSums struct {
	weight proof.DyAcc
	moment [3]proof.DyAcc
}

// add counts the triangle abc of normal n, its weight the dot product of n and the group's first normal.
func (s *groupSums) add(n, first, a, b, c proof.DyV3) {
	w := proof.DvDot(n, first)
	s.weight.Add(w)
	sum := cornerSum(a, b, c)
	s0, s1, s2 := sum[0], sum[1], sum[2]
	s.moment[0].AddProduct(s0, w)
	s.moment[1].AddProduct(s1, w)
	s.moment[2].AddProduct(s2, w)
}

// cornerSum is a+b+c.
func cornerSum(a, b, c proof.DyV3) proof.DyV3 {
	return proof.DvAdd(proof.DvAdd(a, b), c)
}

// centroid is the area centroid num/den, den positive, the exact rational rounded to the nearest float per
// coordinate, with the largest rounding error as its bound, rounded upward. The vertices are held exactly, so
// that rounding is the whole error. DyQuoFloat64 and DyQuoFloatError answer RatFloat64 and RationalFloatError
// of the quotient's value bit for bit, so no rational is formed.
func centroid(num proof.DyV3, den proof.Dyadic) (measurement.VecMeasurement, bool) {
	if den.Sign() <= 0 {
		return measurement.VecMeasurement{}, false
	}
	var coords [3]float64
	var bound float64
	n0, n1, n2 := num[0], num[1], num[2]
	for i, m := range [3]proof.Dyadic{n0, n1, n2} {
		f := proof.DyQuoFloat64(m, den)
		coords[i] = f
		bound = math.Max(bound, proof.DyQuoFloatError(m, den, f))
	}
	if math.IsInf(bound, 0) || math.IsNaN(bound) {
		return measurement.VecMeasurement{}, false
	}
	return measurement.VecMeasurement{
		Value:     r3.NewVec(coords[0], coords[1], coords[2]),
		Exactness: measurement.ExactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, true
}
