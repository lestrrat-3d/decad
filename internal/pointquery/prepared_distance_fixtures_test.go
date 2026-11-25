package pointquery

import (
	"math"
	"math/rand/v2"

	"github.com/lestrrat-3d/r3"
)

// distanceFixture is a named mesh for the prepared distance tests.
type distanceFixture struct {
	name  string
	verts []r3.Vec
	tris  [][3]int
}

// boxFixture is the outward-oriented box with corners at the origin and size.
func boxFixture(name string, size r3.Vec) distanceFixture {
	verts := []r3.Vec{
		{X: 0, Y: 0, Z: 0}, {X: size.X, Y: 0, Z: 0}, {X: size.X, Y: size.Y, Z: 0}, {X: 0, Y: size.Y, Z: 0},
		{X: 0, Y: 0, Z: size.Z}, {X: size.X, Y: 0, Z: size.Z},
		{X: size.X, Y: size.Y, Z: size.Z}, {X: 0, Y: size.Y, Z: size.Z},
	}
	tris := [][3]int{
		{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
		{0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7},
	}
	return distanceFixture{name: name, verts: verts, tris: tris}
}

// sphereFixture is an octahedron subdivided level times and projected to a
// sphere of the given radius around centre. It has 8*4^level triangles.
func sphereFixture(name string, level int, radius float64, centre r3.Vec) distanceFixture {
	verts := []r3.Vec{{X: 1}, {X: -1}, {Y: 1}, {Y: -1}, {Z: 1}, {Z: -1}}
	tris := [][3]int{
		{0, 2, 4}, {2, 1, 4}, {1, 3, 4}, {3, 0, 4}, {2, 0, 5}, {1, 2, 5}, {3, 1, 5}, {0, 3, 5},
	}
	for range level {
		midpoint := map[[2]int]int{}
		mid := func(a, b int) int {
			key := [2]int{min(a, b), max(a, b)}
			if i, ok := midpoint[key]; ok {
				return i
			}
			m := verts[a].Add(verts[b]).Scale(0.5)
			verts = append(verts, m.Scale(1/math.Sqrt(m.X*m.X+m.Y*m.Y+m.Z*m.Z)))
			midpoint[key] = len(verts) - 1
			return len(verts) - 1
		}
		next := make([][3]int, 0, 4*len(tris))
		for _, t := range tris {
			ab, bc, ca := mid(t[0], t[1]), mid(t[1], t[2]), mid(t[2], t[0])
			next = append(next, [3]int{t[0], ab, ca}, [3]int{ab, t[1], bc}, [3]int{ca, bc, t[2]}, [3]int{ab, bc, ca})
		}
		tris = next
	}
	out := make([]r3.Vec, len(verts))
	for i, v := range verts {
		out[i] = v.Scale(radius).Add(centre)
	}
	return distanceFixture{name: name, verts: out, tris: tris}
}

// ringFixture is a closed tube of rings along z with a bumpy wall and two
// fan caps: many long, thin triangles of unequal size.
func ringFixture(name string, around, rings int, rng *rand.Rand) distanceFixture {
	var verts []r3.Vec
	for i := range rings {
		for j := range around {
			a := 2 * math.Pi * float64(j) / float64(around)
			r := 3 + 0.2*rng.Float64()
			verts = append(verts, r3.NewVec(r*math.Cos(a), r*math.Sin(a), 0.37*float64(i)+0.01*rng.Float64()))
		}
	}
	var tris [][3]int
	for i := 0; i+1 < rings; i++ {
		for j := range around {
			a, b := i*around+j, i*around+(j+1)%around
			tris = append(tris, [3]int{a, b, a + around}, [3]int{b, b + around, a + around})
		}
	}
	bottom, top := len(verts), len(verts)+1
	verts = append(verts, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 0.37*float64(rings-1)))
	for j := range around {
		k := (j + 1) % around
		tris = append(tris, [3]int{bottom, k, j}, [3]int{top, (rings-1)*around + j, (rings-1)*around + k})
	}
	return distanceFixture{name: name, verts: verts, tris: tris}
}

// slabFixture is a closed slab whose thickness is far below its width, so
// its side triangles are slivers.
func slabFixture(name string, thickness float64) distanceFixture {
	f := boxFixture(name, r3.NewVec(4, 4, thickness))
	return f
}

// withTriangle returns a copy of f with one more triangle.
func withTriangle(name string, f distanceFixture, tri [3]int) distanceFixture {
	tris := append(append([][3]int(nil), f.tris...), tri)
	return distanceFixture{name: name, verts: f.verts, tris: tris}
}

// withFirstTriangle returns a copy of f with one more triangle at index 0, so
// an index-order scan reaches it before it has a minimum to skip it with.
func withFirstTriangle(name string, f distanceFixture, tri [3]int) distanceFixture {
	tris := append([][3]int{tri}, f.tris...)
	return distanceFixture{name: name, verts: f.verts, tris: tris}
}

// withVertex returns a copy of f whose vertex i is replaced by v.
func withVertex(name string, f distanceFixture, i int, v r3.Vec) distanceFixture {
	verts := append([]r3.Vec(nil), f.verts...)
	verts[i] = v
	return distanceFixture{name: name, verts: verts, tris: f.tris}
}

// cleanDistanceFixtures are meshes the distance index accepts.
func cleanDistanceFixtures() []distanceFixture {
	rng := rand.New(rand.NewPCG(11, 12))
	return []distanceFixture{
		boxFixture("cube", r3.NewVec(10, 10, 10)),
		sphereFixture("sphere-small", 1, 4, r3.NewVec(1, -2, 3)),
		sphereFixture("sphere-large", 5, 4, r3.NewVec(0.5, 0.25, -1)),
		ringFixture("ring", 40, 30, rng),
		slabFixture("slab-sliver", 1e-7),
		boxFixture("box-huge", r3.NewVec(1e9, 2e9, 3e9)),
	}
}

// uncleanDistanceFixtures are meshes the distance index rejects, so queries
// keep MeshDistanceSquared and its errors.
func uncleanDistanceFixtures() []distanceFixture {
	cube := boxFixture("cube", r3.NewVec(10, 10, 10))
	collinear := len(cube.verts)
	cubeWithMid := distanceFixture{name: "cube", verts: append(append([]r3.Vec(nil), cube.verts...), r3.NewVec(5, 0, 0)), tris: cube.tris}
	return []distanceFixture{
		withTriangle("degenerate-collinear", cubeWithMid, [3]int{0, 1, collinear}),
		withTriangle("degenerate-repeated-vertex", cube, [3]int{0, 0, 1}),
		withFirstTriangle("degenerate-first", cube, [3]int{2, 2, 5}),
		withTriangle("index-out-of-range", cube, [3]int{0, 1, 99}),
		withTriangle("index-negative", cube, [3]int{0, -1, 2}),
		withVertex("vertex-nan", cube, 3, r3.NewVec(math.NaN(), 0, 0)),
		withVertex("vertex-inf", cube, 5, r3.NewVec(math.Inf(1), 1, 1)),
		{name: "no-triangles", verts: cube.verts},
	}
}

// distancePoints returns a spread of query points for f: random, far away,
// every vertex, edge midpoints, triangle centroids, and the vertices nudged
// by one float step.
func distancePoints(f distanceFixture, count int, rng *rand.Rand) []r3.Vec {
	lo, hi := f.verts[0], f.verts[0]
	for _, v := range f.verts {
		if math.IsNaN(v.X+v.Y+v.Z) || math.IsInf(v.X+v.Y+v.Z, 0) {
			continue
		}
		lo = r3.NewVec(min(lo.X, v.X), min(lo.Y, v.Y), min(lo.Z, v.Z))
		hi = r3.NewVec(max(hi.X, v.X), max(hi.Y, v.Y), max(hi.Z, v.Z))
	}
	size := hi.Sub(lo)
	var points []r3.Vec
	for range count {
		points = append(points, r3.NewVec(
			lo.X-0.25*size.X+1.5*size.X*rng.Float64(),
			lo.Y-0.25*size.Y+1.5*size.Y*rng.Float64(),
			lo.Z-0.25*size.Z+1.5*size.Z*rng.Float64()))
	}
	points = append(points,
		r3.NewVec(1e7, 1e7, 1e7), r3.NewVec(-1e12, 3, 4), r3.NewVec(1e-300, 0, 0), r3.NewVec(0, 0, 0), lo, hi,
		lo.Add(hi).Scale(0.5))
	step := 0
	for _, tri := range f.tris {
		step++
		if step%max(1, len(f.tris)/200) != 0 {
			continue
		}
		var corner [3]r3.Vec
		ok := true
		for k, i := range tri {
			if i < 0 || i >= len(f.verts) {
				ok = false
				break
			}
			corner[k] = f.verts[i]
		}
		if !ok {
			continue
		}
		points = append(points, corner[0], corner[0].Add(corner[1]).Scale(0.5),
			corner[0].Add(corner[1]).Add(corner[2]).Scale(1.0/3),
			r3.NewVec(math.Nextafter(corner[1].X, math.Inf(1)), corner[1].Y, corner[1].Z))
	}
	finite := points[:0]
	for _, p := range points {
		if !math.IsNaN(p.X+p.Y+p.Z) && !math.IsInf(p.X+p.Y+p.Z, 0) {
			finite = append(finite, p)
		}
	}
	return finite
}

// parityAcceptedUncleanFixtures are the unclean meshes the parity scan
// itself accepts: it panics on an out-of-range index or a non-finite vertex,
// on the unprepared path too.
func parityAcceptedUncleanFixtures() []distanceFixture {
	var kept []distanceFixture
	for _, f := range uncleanDistanceFixtures() {
		switch f.name {
		case "degenerate-collinear", "degenerate-repeated-vertex", "degenerate-first", "no-triangles":
			kept = append(kept, f)
		}
	}
	return kept
}
