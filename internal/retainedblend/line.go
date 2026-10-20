package retainedblend

import (
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// minSine is the smallest sine of an angle between two directions that the
// straight-edge construction accepts as a real angle.
const minSine = 1e-3

// plane is an exact plane of a retained tag: outward unit normal n through o.
type plane struct{ n, o r3.Vec }

func (p plane) offset(x r3.Vec) float64 { return p.n.Dot(x.Sub(p.o)) }

// lineEnd is one end of a straight edge: the corner where the edge meets the
// end face G.
type lineEnd struct {
	face  *topo.Face
	g     plane
	exact r3.Vec // the meeting of the planes A, B and G
	v     int    // the held vertex
}

// line is a straight edge between two retained planes, read from the tags
// (§4.1).
type line struct {
	faceA, faceB *topo.Face
	a, b         plane
	d            r3.Vec // edge direction from end 0 to end 1
	ends         [2]lineEnd
	phi          float64 // angle between the outward normals
	edgeConvex   bool
	scale        float64
}

func (l line) slack() float64 { return 64 * ulp * l.scale }

// lineKey identifies a straight edge across rewrites by its two corners.
type lineKey struct{ p0, p1 r3.Vec }

func (l line) key() lineKey { return lineKey{l.ends[0].exact, l.ends[1].exact} }

func retainedPlane(f *topo.Face) (plane, bool) {
	if f == nil || !f.RetainedTag || f.TagDeparture != 0 || f.Denoted != nil {
		return plane{}, false
	}
	pl, ok := f.Surface.(surfacegeom.Plane)
	if !ok {
		return plane{}, false
	}
	n, ok := pl.Frame.N().Normalize()
	if !ok {
		return plane{}, false
	}
	if f.Reversed {
		n = n.Scale(-1)
	}
	return plane{n: n, o: pl.Frame.Origin()}, true
}

// looksLikeLine reports whether edge is a candidate for the straight-edge
// route: a retained Line3 between two retained planes, with two end corners.
func looksLikeLine(edge *topo.Edge) bool {
	if edge == nil || edge.Start == nil || edge.End == nil || edge.Start == edge.End ||
		!edge.RetainedCurve || len(edge.Faces) != 2 {
		return false
	}
	if _, ok := edge.Curve.(surfacegeom.Line3); !ok {
		return false
	}
	for _, f := range edge.Faces {
		if _, ok := retainedPlane(f); !ok {
			return false
		}
	}
	return true
}

// intersect3 is the point common to three planes, and false when their normals
// are nearly dependent.
func intersect3(p, q, s plane) (r3.Vec, bool) {
	det := p.n.Dot(q.n.Cross(s.n))
	if math.Abs(det) < minSine {
		return r3.Vec{}, false
	}
	hp, hq, hs := p.n.Dot(p.o), q.n.Dot(q.o), s.n.Dot(s.o)
	x := q.n.Cross(s.n).Scale(hp).Add(s.n.Cross(p.n).Scale(hq)).Add(p.n.Cross(q.n).Scale(hs))
	return x.Scale(1 / det), true
}

// readLine reads the straight edge from its tags and the held mesh. It refuses
// an edge whose tags and held mesh do not describe one corner at each end.
func readLine(edge *topo.Edge, fp facetedrecord.Payload, faceIndex map[*topo.Face]int) (line, error) {
	var l line
	l.faceA, l.faceB = edge.Faces[0], edge.Faces[1]
	l.a, _ = retainedPlane(l.faceA)
	l.b, _ = retainedPlane(l.faceB)
	l.edgeConvex = edge.Convex
	l.scale = 1
	for _, v := range fp.Verts {
		l.scale = max(l.scale, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	cross := l.a.n.Cross(l.b.n)
	sin := cross.Len()
	if sin < minSine {
		return line{}, fmt.Errorf("%w: the edge's two planes are nearly parallel", decaderr.ErrDegenerate)
	}
	l.d = cross.Scale(1 / sin)
	l.phi = math.Atan2(sin, l.a.n.Dot(l.b.n))

	fa, fb := faceIndex[l.faceA], faceIndex[l.faceB]
	for i, vertex := range []*topo.Vertex{edge.Start, edge.End} {
		end, err := l.readEnd(vertex, fp, faceIndex, fa, fb)
		if err != nil {
			return line{}, err
		}
		l.ends[i] = end
	}
	if l.d.Dot(l.ends[1].exact.Sub(l.ends[0].exact)) < 0 {
		l.d = l.d.Scale(-1)
	}
	if l.ends[0].face == l.ends[1].face {
		return line{}, fmt.Errorf("%w: both ends of the edge meet one face", decaderr.ErrUnsupported)
	}
	return l, nil
}

func (l *line) readEnd(vertex *topo.Vertex, fp facetedrecord.Payload, faceIndex map[*topo.Face]int,
	fa, fb int) (lineEnd, error) {
	var end lineEnd
	end.v = -1
	for i, p := range fp.Verts {
		if p == vertex.Position {
			end.v = i
			break
		}
	}
	if end.v < 0 {
		return lineEnd{}, fmt.Errorf("%w: an edge end is not a vertex of the held mesh", decaderr.ErrUnsupported)
	}
	faces := map[int]bool{}
	for i, t := range fp.Tris {
		if t[0] == end.v || t[1] == end.v || t[2] == end.v {
			faces[fp.FaceOf[i]] = true
		}
	}
	if len(faces) != 3 || !faces[fa] || !faces[fb] {
		return lineEnd{}, fmt.Errorf("%w: an edge end is not a corner of exactly three faces", decaderr.ErrUnsupported)
	}
	for face, idx := range faceIndex {
		if faces[idx] && idx != fa && idx != fb {
			end.face = face
		}
	}
	g, ok := retainedPlane(end.face)
	if !ok {
		return lineEnd{}, fmt.Errorf("%w: an edge end meets a face that is not a retained plane", decaderr.ErrUnsupported)
	}
	end.g = g
	exact, ok := intersect3(l.a, l.b, g)
	if !ok {
		return lineEnd{}, fmt.Errorf("%w: an edge end meets its end face at a grazing angle", decaderr.ErrUnsupported)
	}
	end.exact = exact
	if exact.Sub(fp.Verts[end.v]).Len() > fp.VertexBound[end.v]+l.slack() {
		return lineEnd{}, fmt.Errorf("%w: an edge end is not where its three tags meet", decaderr.ErrUnsupported)
	}
	if math.Abs(l.d.Dot(g.n)) < minSine || l.a.n.Cross(g.n).Len() < minSine || l.b.n.Cross(g.n).Len() < minSine {
		return lineEnd{}, fmt.Errorf("%w: an edge end meets its end face at a grazing angle", decaderr.ErrUnsupported)
	}
	return end, nil
}

// distToLine is the distance of p from the line through a with unit direction d.
func distToLine(p, a, d r3.Vec) float64 {
	q := p.Sub(a)
	return q.Sub(d.Scale(q.Dot(d))).Len()
}

// distToSegment is the distance of p from the segment ab.
func distToSegment(p, a, b r3.Vec) float64 {
	ab := b.Sub(a)
	t := 0.0
	if n := ab.Dot(ab); n > 0 {
		t = math.Min(1, math.Max(0, p.Sub(a).Dot(ab)/n))
	}
	return p.Sub(a.Add(ab.Scale(t))).Len()
}

// faceLoops is a face's facets and boundary loops.
type faceLoops struct {
	index  int
	facets []int
	group  int
	loops  [][]int
}

func (w *work) faceLoops(fi int) (faceLoops, error) {
	f := faceLoops{index: fi, facets: facetsOfFace(w.fp.FaceOf, fi)}
	var err error
	if f.group, err = w.groupOf(f.facets); err != nil {
		return f, err
	}
	if f.loops, err = boundaryLoops(w.fp.Tris, f.facets); err != nil {
		return f, err
	}
	in := map[int]bool{}
	for _, loop := range f.loops {
		for _, v := range loop {
			in[v] = true
		}
	}
	for _, facet := range f.facets {
		for _, v := range w.fp.Tris[facet] {
			if !in[v] {
				return f, fmt.Errorf("%w: a face holds a vertex off its boundary", decaderr.ErrUnsupported)
			}
		}
	}
	return f, nil
}

// loopWith returns the position of the loop that holds all the vertices, or
// false.
func (f faceLoops) loopWith(vs ...int) (int, bool) {
	for li, loop := range f.loops {
		found := 0
		for _, v := range loop {
			if slices.Contains(vs, v) {
				found++
			}
		}
		if found == len(vs) {
			return li, true
		}
	}
	return 0, false
}

// indexIn is the position of v in loop, or -1.
func indexIn(loop []int, v int) int {
	for i, x := range loop {
		if x == v {
			return i
		}
	}
	return -1
}

// walk collects the loop vertices from position at, stepping by step, that
// satisfy ok, stopping at the first that does not (or at the start again).
func walk(loop []int, at, step int, ok func(v int) bool) []int {
	var out []int
	n := len(loop)
	for k := 1; k < n; k++ {
		v := loop[((at+step*k)%n+n)%n]
		if !ok(v) {
			break
		}
		out = append(out, v)
	}
	return out
}

// splice cuts the vertices in run out of the loop, which must hold them as one
// contiguous cyclic run. It returns the vertices that remain, starting just
// after the run, and the run in loop order.
func splice(loop []int, run map[int]bool) (outside, inOrder []int, err error) {
	n := len(loop)
	start := -1
	for i, v := range loop {
		if run[v] && !run[loop[(i+n-1)%n]] {
			if start >= 0 {
				return nil, nil, fmt.Errorf("%w: a face boundary holds two runs to replace", decaderr.ErrUnsupported)
			}
			start = i
		}
	}
	if start < 0 {
		return nil, nil, fmt.Errorf("%w: a face boundary holds no run to replace", decaderr.ErrUnsupported)
	}
	k := 0
	for k < n && run[loop[(start+k)%n]] {
		inOrder = append(inOrder, loop[(start+k)%n])
		k++
	}
	if len(inOrder) != len(run) {
		return nil, nil, fmt.Errorf("%w: a face boundary breaks a run into pieces", decaderr.ErrUnsupported)
	}
	for j := k; j < n; j++ {
		outside = append(outside, loop[(start+j)%n])
	}
	return outside, inOrder, nil
}

var _ = units.Millimeter
