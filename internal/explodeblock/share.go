package explodeblock

import (
	"hash/maphash"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// shareTolerance is the largest distance in millimetres a vertex may sit from
// its translated counterpart for two meshes to count as the same shape. Floating
// point translation moves a coordinate by a few ulps, far below this. The
// deviation actually seen is added to the shared bound, so it never makes a
// verdict less cautious.
const shareTolerance = 1e-9

// Shared says how one mesh relates to the representative of its class.
type Shared struct {
	// Class is the index into the classes [Group] returned.
	Class int
	// Offset is the world translation that takes the class mesh onto this
	// mesh. It is zero for the representative.
	Offset r3.Vec
}

// Group sorts meshes into classes of the same shape at different positions and
// returns, per mesh, its class and offset, and per class the mesh to project.
// Two meshes share a class when they have the same triangles over the same
// vertex numbering and each vertex of one lies within 1e-9 mm of the other's
// translated by one common vector. A class mesh carries the largest Bound of
// its members plus the largest deviation seen, so a projection of it, moved
// by an offset, bounds every member.
//
// A cheap hash of the triangle numbering picks the candidates; only those are
// compared vertex by vertex. Meshes that share no class with an earlier mesh
// start their own, so Group never merges meshes that differ.
func Group(meshes []Mesh) ([]Shared, []Mesh) {
	seed := maphash.MakeSeed()
	shares := make([]Shared, len(meshes))
	var classes []Mesh
	var reps []int // class -> index of its representative in meshes
	byHash := map[uint64][]int{}
	for i, m := range meshes {
		h := signature(seed, m)
		found := false
		for _, c := range byHash[h] {
			off, dev, ok := translation(meshes[reps[c]], m)
			if !ok {
				continue
			}
			shares[i] = Shared{Class: c, Offset: off}
			classes[c].Bound = units.Millimeters(math.Max(classes[c].Bound.Base(), m.Bound.Base()+dev))
			found = true
			break
		}
		if found {
			continue
		}
		c := len(classes)
		byHash[h] = append(byHash[h], c)
		classes = append(classes, m)
		reps = append(reps, i)
		shares[i] = Shared{Class: c}
	}
	return shares, classes
}

// signature hashes what a translation cannot change: the counts and the
// triangle numbering.
func signature(seed maphash.Seed, m Mesh) uint64 {
	var h maphash.Hash
	h.SetSeed(seed)
	var buf [8]byte
	put := func(v int) {
		for k := range buf {
			buf[k] = byte(v >> (8 * k))
		}
		_, _ = h.Write(buf[:])
	}
	put(len(m.Vertices))
	put(len(m.Triangles))
	for _, t := range m.Triangles {
		put(t[0])
		put(t[1])
		put(t[2])
	}
	return h.Sum64()
}

// translation reports whether b is a translate of a, and by what vector and
// with what largest vertex deviation.
func translation(a, b Mesh) (r3.Vec, float64, bool) {
	if len(a.Vertices) != len(b.Vertices) || len(a.Triangles) != len(b.Triangles) {
		return r3.Vec{}, 0, false
	}
	for i, t := range a.Triangles {
		if b.Triangles[i] != t {
			return r3.Vec{}, 0, false
		}
	}
	if len(a.Vertices) == 0 {
		return r3.Vec{}, 0, true
	}
	off := b.Vertices[0].Sub(a.Vertices[0])
	if math.IsNaN(off.X+off.Y+off.Z) || math.IsInf(off.X+off.Y+off.Z, 0) {
		return r3.Vec{}, 0, false
	}
	dev := 0.0
	for i, v := range a.Vertices {
		d := b.Vertices[i].Sub(v.Add(off))
		l := math.Sqrt(d.X*d.X + d.Y*d.Y + d.Z*d.Z)
		if !(l <= shareTolerance) {
			return r3.Vec{}, 0, false
		}
		dev = math.Max(dev, l)
	}
	return off, dev, true
}
