// Package meshrepair mends an indexed triangle mesh: it welds close vertices,
// drops triangles that carry no surface, winds shells consistently and closes
// simple holes (agent-docs/mesh-repair-design.md). It works on plain
// millimetre data and touches no body or document.
package meshrepair

import (
	"context"
	"math"

	"github.com/lestrrat-3d/r3"
)

// Options selects the repairs. Dropping degenerate and repeated triangles is
// always done.
type Options struct {
	// Weld merges vertices within Distance of one another when Enabled.
	Weld struct {
		Enabled  bool
		Distance float64
	}
	// Orient winds every shell consistently and turns an inside-out closed
	// shell right way out.
	Orient bool
	// FillHoles closes each simple boundary loop.
	FillHoles bool
}

// Stats counts what Repair changed.
type Stats struct {
	// WeldedVertices counts vertices merged into another.
	WeldedVertices int
	// DroppedTriangles counts triangles removed: those with a repeated vertex
	// after welding, those of zero area, and repeats of an earlier triangle.
	DroppedTriangles int
	// FlippedTriangles counts triangles whose winding was reversed.
	FlippedTriangles int
	// FilledLoops counts the boundary loops that were closed, and SkippedLoops
	// the loops left open because a vertex on them had several ways out.
	FilledLoops, SkippedLoops int
	// AddedTriangles counts the triangles the fills added.
	AddedTriangles int
}

// Result is a repaired mesh.
type Result struct {
	Vertices  []r3.Vec
	Triangles [][3]int
	Stats     Stats
}

// Repair returns the mesh mended as opts says. The input is left as it was.
// Triangle indices must lie within vertices. A canceled context returns its
// error.
func Repair(ctx context.Context, vertices []r3.Vec, triangles [][3]int, opts Options) (Result, error) {
	var st Stats
	verts := append([]r3.Vec(nil), vertices...)
	tris := append([][3]int(nil), triangles...)
	if opts.Weld.Enabled {
		remap, merged := weld(verts, opts.Weld.Distance)
		st.WeldedVertices = merged
		for i := range tris {
			for k := range tris[i] {
				tris[i][k] = remap[tris[i][k]]
			}
		}
	}
	if err := ctx.Err(); err != nil {
		return Result{}, err
	}
	tris, st.DroppedTriangles = dropUseless(verts, tris)
	if opts.Orient {
		st.FlippedTriangles = orient(tris, verts)
	}
	if err := ctx.Err(); err != nil {
		return Result{}, err
	}
	if opts.FillHoles {
		verts, tris = fillHoles(verts, tris, &st)
	}
	verts, tris = compact(verts, tris)
	return Result{Vertices: verts, Triangles: tris, Stats: st}, nil
}

// weld maps each vertex to the first earlier vertex within distance of it, or
// to itself, and returns the map and how many vertices were merged away.
func weld(verts []r3.Vec, distance float64) ([]int, int) {
	remap := make([]int, len(verts))
	cell := math.Max(distance, 1e-12)
	type key [3]int64
	at := func(v r3.Vec) key {
		return key{int64(math.Floor(v.X / cell)), int64(math.Floor(v.Y / cell)), int64(math.Floor(v.Z / cell))}
	}
	grid := map[key][]int{}
	merged := 0
	for i, v := range verts {
		k := at(v)
		rep := -1
	search:
		for dx := int64(-1); dx <= 1; dx++ {
			for dy := int64(-1); dy <= 1; dy++ {
				for dz := int64(-1); dz <= 1; dz++ {
					for _, j := range grid[key{k[0] + dx, k[1] + dy, k[2] + dz}] {
						if verts[j].Sub(v).Len() <= distance {
							rep = j
							break search
						}
					}
				}
			}
		}
		if rep >= 0 {
			remap[i] = rep
			merged++
			continue
		}
		remap[i] = i
		grid[k] = append(grid[k], i)
	}
	return remap, merged
}

// dropUseless removes triangles that name a vertex twice, have zero area or
// repeat an earlier triangle's vertex set.
func dropUseless(verts []r3.Vec, tris [][3]int) ([][3]int, int) {
	seen := map[[3]int]bool{}
	out := tris[:0:0]
	dropped := 0
	for _, t := range tris {
		if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
			dropped++
			continue
		}
		if verts[t[1]].Sub(verts[t[0]]).Cross(verts[t[2]].Sub(verts[t[0]])).Len() == 0 {
			dropped++
			continue
		}
		key := sorted3(t)
		if seen[key] {
			dropped++
			continue
		}
		seen[key] = true
		out = append(out, t)
	}
	return out, dropped
}

func sorted3(t [3]int) [3]int {
	if t[0] > t[1] {
		t[0], t[1] = t[1], t[0]
	}
	if t[1] > t[2] {
		t[1], t[2] = t[2], t[1]
	}
	if t[0] > t[1] {
		t[0], t[1] = t[1], t[0]
	}
	return t
}

type edge [2]int

func undirected(a, b int) edge {
	if a > b {
		a, b = b, a
	}
	return edge{a, b}
}

// use is one triangle's use of an edge: which triangle, and whether it runs
// from the smaller vertex index to the larger.
type use struct {
	tri     int
	forward bool
}

func edgeUses(tris [][3]int) map[edge][]use {
	uses := map[edge][]use{}
	for i, t := range tris {
		for k := range t {
			a, b := t[k], t[(k+1)%3]
			uses[undirected(a, b)] = append(uses[undirected(a, b)], use{i, a < b})
		}
	}
	return uses
}

// orient flips triangles so that every edge two triangles share runs in
// opposite directions in them, shell by shell over edges used exactly twice,
// then turns each closed shell of negative volume right way out. It returns the
// number of triangles flipped.
func orient(tris [][3]int, verts []r3.Vec) int {
	uses := edgeUses(tris)
	flipped := make([]bool, len(tris))
	visited := make([]bool, len(tris))
	comp := make([]int, len(tris))
	components := 0
	for start := range tris {
		if visited[start] {
			continue
		}
		visited[start] = true
		comp[start] = components
		queue := []int{start}
		for len(queue) > 0 {
			cur := queue[0]
			queue = queue[1:]
			t := tris[cur]
			for k := range t {
				a, b := t[k], t[(k+1)%3]
				us := uses[undirected(a, b)]
				if len(us) != 2 {
					continue
				}
				other, me := us[0], us[1]
				if us[0].tri == cur {
					other, me = us[1], us[0]
				}
				if visited[other.tri] {
					continue
				}
				visited[other.tri] = true
				comp[other.tri] = components
				// The shared edge must run opposite ways in the two triangles.
				same := (me.forward != flipped[cur]) == other.forward
				flipped[other.tri] = same
				queue = append(queue, other.tri)
			}
		}
		components++
	}
	// A closed shell whose volume is negative is inside out: flip all of it.
	closed := make([]bool, components)
	for i := range closed {
		closed[i] = true
	}
	for _, us := range uses {
		if len(us) != 2 {
			for _, u := range us {
				closed[comp[u.tri]] = false
			}
		}
	}
	volume := make([]float64, components)
	for i, t := range tris {
		a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
		if flipped[i] {
			b, c = c, b
		}
		volume[comp[i]] += a.Dot(b.Cross(c)) / 6
	}
	for i := range tris {
		if closed[comp[i]] && volume[comp[i]] < 0 {
			flipped[i] = !flipped[i]
		}
	}
	count := 0
	for i, f := range flipped {
		if f {
			tris[i][1], tris[i][2] = tris[i][2], tris[i][1]
			count++
		}
	}
	return count
}

// fillHoles closes each boundary loop with a fan of triangles from a new vertex
// at the loop's centroid, or with one triangle for a loop of three.
func fillHoles(verts []r3.Vec, tris [][3]int, st *Stats) ([]r3.Vec, [][3]int) {
	uses := edgeUses(tris)
	next := map[int][]int{}
	var starts []int
	for _, t := range tris {
		for k := range t {
			a, b := t[k], t[(k+1)%3]
			if len(uses[undirected(a, b)]) == 1 {
				if len(next[a]) == 0 {
					starts = append(starts, a)
				}
				next[a] = append(next[a], b)
			}
		}
	}
	done := map[int]bool{}
	for _, s := range starts {
		if done[s] {
			continue
		}
		loop, ok := []int{s}, true
		for cur := s; ; {
			outs := next[cur]
			if len(outs) != 1 {
				ok = false
				break
			}
			cur = outs[0]
			if cur == s {
				break
			}
			if done[cur] || len(loop) > len(next) {
				ok = false
				break
			}
			loop = append(loop, cur)
		}
		if !ok || len(loop) < 3 {
			// Skip the whole connected run of boundary, once.
			for _, v := range reach(next, s) {
				done[v] = true
			}
			st.SkippedLoops++
			continue
		}
		for _, v := range loop {
			done[v] = true
		}
		st.FilledLoops++
		if len(loop) == 3 {
			tris = append(tris, [3]int{loop[0], loop[2], loop[1]})
			st.AddedTriangles++
			continue
		}
		var centre r3.Vec
		for _, v := range loop {
			centre = centre.Add(verts[v])
		}
		centre = centre.Scale(1 / float64(len(loop)))
		c := len(verts)
		verts = append(verts, centre)
		for k, a := range loop {
			b := loop[(k+1)%len(loop)]
			tris = append(tris, [3]int{b, a, c})
			st.AddedTriangles++
		}
	}
	return verts, tris
}

// reach returns the vertices joined to from by boundary edges, in either
// direction.
func reach(next map[int][]int, from int) []int {
	back := map[int][]int{}
	for a, outs := range next {
		for _, b := range outs {
			back[b] = append(back[b], a)
		}
	}
	seen := map[int]bool{from: true}
	order := []int{from}
	for i := 0; i < len(order); i++ {
		for _, list := range [2][]int{next[order[i]], back[order[i]]} {
			for _, w := range list {
				if !seen[w] {
					seen[w] = true
					order = append(order, w)
				}
			}
		}
	}
	return order
}

// compact drops the vertices no triangle uses and renumbers the rest.
func compact(verts []r3.Vec, tris [][3]int) ([]r3.Vec, [][3]int) {
	index := make([]int, len(verts))
	for i := range index {
		index[i] = -1
	}
	var out []r3.Vec
	renamed := make([][3]int, len(tris))
	for i, t := range tris {
		for k, v := range t {
			if index[v] < 0 {
				index[v] = len(out)
				out = append(out, verts[v])
			}
			renamed[i][k] = index[v]
		}
	}
	return out, renamed
}
