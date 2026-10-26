package stepbrep

import (
	"fmt"
	"math"
	"sort"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/stepread"
	"github.com/lestrrat-3d/r3"
)

// cylinderFace triangulates an ADVANCED_FACE on a CYLINDRICAL_SURFACE. Its
// boundary has been cut into chords already (edgeSamples). A wall with one bound
// of arcs at two heights and lines along the axis is a ladder of triangles
// between its two arcs, and a wall that goes all the way round, with a circular
// loop at each end and no seam, is a strip between the two rings. Any other wall
// is refused.
func (r *reader) cylinderFace(f, surface stepread.Entity, flipped bool) error {
	if len(surface.Params) < 3 {
		return fmt.Errorf("%w: #%d is not a CYLINDRICAL_SURFACE", ErrMalformed, surface.ID)
	}
	frame, err := r.placement(surface.Params[1])
	if err != nil {
		return err
	}
	rad, ok := surface.Params[2].(stepread.Number)
	if !ok || !(float64(rad) > 0) {
		return fmt.Errorf("%w: a cylinder has no positive radius", ErrMalformed)
	}
	_ = float64(rad) * r.scale // the radius is the points' own; the frame and the rings carry it
	bounds, _ := f.Params[1].(stepread.List)
	if len(bounds) == 0 {
		return fmt.Errorf("%w: face #%d has no bound", ErrMalformed, f.ID)
	}
	var rings [][]r3.Vec
	for _, bv := range bounds {
		ring, _, err := r.ring(bv)
		if err != nil {
			return err
		}
		if len(ring) < 3 {
			return fmt.Errorf("%w: a bound of face #%d has %d points", ErrMalformed, f.ID, len(ring))
		}
		rings = append(rings, ring)
	}
	outward := 1.0
	if sense, _ := f.Params[3].(stepread.Enum); sense == "F" {
		outward = -1
	}
	// The unwrapped angle of each ring's points, and how many turns the ring makes.
	angles := make([][]float64, len(rings))
	windings := make([]int, len(rings))
	for i, ring := range rings {
		angles[i], windings[i] = unwrap(frame, ring)
	}
	var triangles [][3]r3.Vec
	if turns(windings) > 0 {
		if len(rings) != 2 || windings[0] == 0 || windings[1] == 0 {
			return fmt.Errorf("%w: face #%d goes round its cylinder with a boundary other than two circular rings", ErrMalformed, f.ID)
		}
		triangles = strip(frame, rings[0], rings[1])
	} else {
		if len(rings) != 1 {
			return fmt.Errorf("%w: face #%d is a cylinder wall with %d bounds, and only a wall with one bound or two circular rings is read",
				decaderr.ErrUnsupported, f.ID, len(rings))
		}
		if triangles, err = band(frame, rings[0], angles[0]); err != nil {
			return fmt.Errorf("%w: face #%d: %s", decaderr.ErrUnsupported, f.ID, err)
		}
	}
	for _, t := range triangles {
		if len(r.tris) >= r.limit {
			return fmt.Errorf("%w: more than %d triangles", ErrTooLarge, r.limit)
		}
		// The triangle faces the way the surface does: along the radius, or against
		// it when the face's sense is reversed.
		centre := t[0].Add(t[1]).Add(t[2]).Scale(1.0 / 3)
		radial := centre.Sub(frame.o)
		radial = radial.Sub(frame.z.Scale(radial.Dot(frame.z)))
		normal := t[1].Sub(t[0]).Cross(t[2].Sub(t[0]))
		a, b, c := r.weld(t[0]), r.weld(t[1]), r.weld(t[2])
		if normal.Dot(radial)*outward < 0 {
			b, c = c, b
		}
		if flipped {
			b, c = c, b
		}
		r.tris = append(r.tris, [3]int{a, b, c})
	}
	return nil
}

func turns(windings []int) int {
	n := 0
	for _, w := range windings {
		if w != 0 {
			n++
		}
	}
	return n
}

// unwrap returns the angle of each ring point about the axis, continuous along
// the ring from the first, and how many full turns the ring makes closing back
// on the first.
func unwrap(f frame, ring []r3.Vec) ([]float64, int) {
	out := make([]float64, len(ring))
	out[0] = f.angle(ring[0])
	step := func(from, to float64) float64 {
		d := math.Mod(to-from+3*math.Pi, 2*math.Pi) - math.Pi
		return d
	}
	for i := 1; i < len(ring); i++ {
		out[i] = out[i-1] + step(out[i-1], f.angle(ring[i]))
	}
	closing := step(out[len(out)-1], f.angle(ring[0]))
	total := out[len(out)-1] + closing - out[0]
	return out, int(math.Round(total / (2 * math.Pi)))
}

// band cuts a wall bounded by arcs at two heights and lines along the axis into
// a ladder of triangles between the two arcs. Every ring point lies at the lower
// height or the upper one, which an arc of constant height and the ends of the
// lines do, and the two lists of points, each in order of angle, are merged: a
// triangle spans only neighbouring points, so it lies on the cylinder to within
// the chord tolerance. A boundary that leaves those two heights, as a wall cut by
// a slanted plane does, is refused.
func band(f frame, ring []r3.Vec, angles []float64) ([][3]r3.Vec, error) {
	type item struct {
		angle float64
		p     r3.Vec
	}
	lo, hi := math.Inf(1), math.Inf(-1)
	heights := make([]float64, len(ring))
	for i, p := range ring {
		heights[i] = p.Sub(f.o).Dot(f.z)
		lo, hi = math.Min(lo, heights[i]), math.Max(hi, heights[i])
	}
	if !(hi-lo > 1e-9) {
		return nil, fmt.Errorf("the wall has no height")
	}
	eps := 1e-7 * math.Max(1, hi-lo)
	var bottom, top []item
	for i, p := range ring {
		switch {
		case math.Abs(heights[i]-lo) <= eps:
			bottom = append(bottom, item{angles[i], p})
		case math.Abs(heights[i]-hi) <= eps:
			top = append(top, item{angles[i], p})
		default:
			return nil, fmt.Errorf("a boundary point lies between the wall's two end heights, so the wall is not bounded by arcs and lines along its axis")
		}
	}
	if len(bottom) < 2 || len(top) < 2 {
		return nil, fmt.Errorf("the wall has fewer than two points at an end")
	}
	byAngle := func(v []item) { sort.SliceStable(v, func(a, b int) bool { return v[a].angle < v[b].angle }) }
	byAngle(bottom)
	byAngle(top)
	var out [][3]r3.Vec
	i, j := 0, 0
	for i+1 < len(bottom) || j+1 < len(top) {
		switch {
		case i+1 >= len(bottom):
			out = append(out, [3]r3.Vec{bottom[i].p, top[j+1].p, top[j].p})
			j++
		case j+1 >= len(top):
			out = append(out, [3]r3.Vec{bottom[i].p, bottom[i+1].p, top[j].p})
			i++
		case bottom[i+1].angle <= top[j+1].angle:
			out = append(out, [3]r3.Vec{bottom[i].p, bottom[i+1].p, top[j].p})
			i++
		default:
			out = append(out, [3]r3.Vec{bottom[i].p, top[j+1].p, top[j].p})
			j++
		}
	}
	return out, nil
}

// strip joins two circular rings around one axis into a band of triangles, each
// ring's points in the order of their angle, the two lists merged by angle.
func strip(f frame, a, b []r3.Vec) [][3]r3.Vec {
	type item struct {
		angle float64
		p     r3.Vec
	}
	order := func(ring []r3.Vec) []item {
		out := make([]item, len(ring))
		for i, p := range ring {
			angle := math.Mod(f.angle(p)+2*math.Pi, 2*math.Pi)
			out[i] = item{angle, p}
		}
		sort.SliceStable(out, func(x, y int) bool { return out[x].angle < out[y].angle })
		return out
	}
	A, B := order(a), order(b)
	// Start at A's first point and the B point at or just before its angle.
	j := len(B) - 1
	for k, it := range B {
		if it.angle <= A[0].angle {
			j = k
		}
	}
	var out [][3]r3.Vec
	ai := 0
	// Take one step at a time round the cycle: the next point of A or of B, whichever
	// has the smaller angle ahead of the current pair.
	next := func(list []item, k int) item { return list[(k+1)%len(list)] }
	ahead := func(from, to float64) float64 { return math.Mod(to-from+4*math.Pi, 2*math.Pi) }
	cur := A[0].angle
	for range len(A) + len(B) {
		na, nb := next(A, ai), next(B, j)
		da, db := ahead(cur, na.angle), ahead(cur, nb.angle)
		if da <= db {
			out = append(out, [3]r3.Vec{A[ai].p, na.p, B[j].p})
			ai = (ai + 1) % len(A)
			cur = A[ai].angle
		} else {
			out = append(out, [3]r3.Vec{A[ai].p, nb.p, B[j].p})
			j = (j + 1) % len(B)
			cur = B[j].angle
		}
	}
	return out
}
