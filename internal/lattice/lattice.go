// Package lattice builds the triangle mesh of a gyroid, Schwarz-P, diamond, Neovius, IWP, Fischer-Koch S or Lidinoid lattice inside a box
// (agent-docs/lattice-design.md). It works on plain millimetre data and touches
// no body, document or sketch.
package lattice

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
)

// ErrDegenerate is returned for a box, cell, level or resolution that names no
// lattice, or one too large to build.
var ErrDegenerate = errors.New("lattice: degenerate lattice")

// maxNodes bounds the grid a call may sample, and maxTriangles the mesh it may
// return.
const (
	maxNodes     = 8_000_000
	maxTriangles = 2_000_000
)

// MaxLevel is the largest value |g| reaches for the gyroid field, 3/2; a level at
// or above it fills the whole box. [Kind.MaxLevel] is the largest of any kind.
const MaxLevel = 1.5

// Kind is a triply periodic surface, the zero set of a field g whose solid is
// where |g| ≤ level.
type Kind int

const (
	// KindGyroid is sin kx·cos ky + sin ky·cos kz + sin kz·cos kx, which runs from
	// −3/2 to 3/2.
	KindGyroid Kind = iota
	// KindSchwarzP is cos kx + cos ky + cos kz, which runs from −3 to 3.
	KindSchwarzP
	// KindDiamond is sin kx·sin ky·sin kz + sin kx·cos ky·cos kz +
	// cos kx·sin ky·cos kz + cos kx·cos ky·sin kz, which runs from −√2 to √2.
	KindDiamond
	// KindNeovius is 3(cos kx + cos ky + cos kz) + 4 cos kx·cos ky·cos kz, which runs from
	// −13 to 13.
	KindNeovius
	// KindIWP is 2(cos kx·cos ky + cos ky·cos kz + cos kz·cos kx) − (cos 2kx + cos 2ky + cos 2kz),
	// which runs from −5 to 3, so the largest |g| is 5.
	KindIWP
	// KindFischerKochS is cos 2kx·sin ky·cos kz + cos kx·cos 2ky·sin kz + sin kx·cos ky·cos 2kz, which runs from
	// −√2 to √2.
	KindFischerKochS
	// KindLidinoid is ½(sin 2kx·cos ky·sin kz + sin 2ky·cos kz·sin kx + sin 2kz·cos kx·sin ky) −
	// ½(cos 2kx·cos 2ky + cos 2ky·cos 2kz + cos 2kz·cos 2kx) + 0.15, which runs from −1.35 to 1.15, so the
	// largest |g| is 1.35.
	KindLidinoid
)

// MaxLevel is the largest value |g| reaches for the kind; a level at or above it
// fills the whole box.
func (k Kind) MaxLevel() float64 {
	switch k {
	case KindSchwarzP:
		return 3
	case KindDiamond:
		return math.Sqrt2
	case KindNeovius:
		return 13
	case KindIWP:
		return 5
	case KindFischerKochS:
		return math.Sqrt2
	case KindLidinoid:
		return 1.35
	default:
		return MaxLevel
	}
}

// field is g at the point whose phases kx, ky and kz are given.
func (k Kind) field(x, y, z float64) float64 {
	sx, cx := math.Sincos(x)
	sy, cy := math.Sincos(y)
	sz, cz := math.Sincos(z)
	switch k {
	case KindSchwarzP:
		return cx + cy + cz
	case KindDiamond:
		return sx*sy*sz + sx*cy*cz + cx*sy*cz + cx*cy*sz
	case KindNeovius:
		return 3*(cx+cy+cz) + 4*cx*cy*cz
	case KindIWP:
		return 2*(cx*cy+cy*cz+cz*cx) - (math.Cos(2*x) + math.Cos(2*y) + math.Cos(2*z))
	case KindFischerKochS:
		return math.Cos(2*x)*sy*cz + cx*math.Cos(2*y)*sz + sx*cy*math.Cos(2*z)
	case KindLidinoid:
		return 0.5*(math.Sin(2*x)*cy*sz+math.Sin(2*y)*cz*sx+math.Sin(2*z)*cx*sy) -
			0.5*(math.Cos(2*x)*math.Cos(2*y)+math.Cos(2*y)*math.Cos(2*z)+math.Cos(2*z)*math.Cos(2*x)) + 0.15
	default:
		return sx*cy + sy*cz + sz*cx
	}
}

// Gyroid returns the mesh of the solid {|g(p)| ≤ level} inside the box from min
// to max, where g(p) = sin kx·cos ky + sin ky·cos kz + sin kz·cos kx and
// k = 2π/cell. The mesh is the zero set of F(p) = min(level − |g(p)|, d(p)), d
// being the distance from p to the nearest face of the box, sampled on a grid of
// res nodes to a cell, offset half a step from the box so that no node lies on a
// face, and cut by marching tetrahedra, so it is closed and its
// triangles wind counter-clockwise seen from outside the solid.
//
// Every length is in millimetres. min is below max on every axis, cell is
// positive, level lies strictly between 0 and MaxLevel and res is at least 4.
func Gyroid(ctx context.Context, lo, hi r3.Vec, cell, level float64, res int) (vertices []r3.Vec, triangles [][3]int, err error) {
	return Surface(ctx, KindGyroid, lo, hi, cell, level, res)
}

// Surface is [Gyroid] for any [Kind]: the mesh of {|g(p)| ≤ level} inside the box
// from lo to hi, with g the kind's field and level strictly between 0 and the
// kind's MaxLevel.
func Surface(ctx context.Context, kind Kind, lo, hi r3.Vec, cell, level float64, res int) (vertices []r3.Vec, triangles [][3]int, err error) {
	if !(cell > 0) || math.IsInf(cell, 0) {
		return nil, nil, fmt.Errorf("%w: the cell must be positive and finite", ErrDegenerate)
	}
	if !(level > 0 && level < kind.MaxLevel()) {
		return nil, nil, fmt.Errorf("%w: the level must lie between 0 and %g", ErrDegenerate, kind.MaxLevel())
	}
	if res < 4 {
		return nil, nil, fmt.Errorf("%w: at least 4 samples to a cell", ErrDegenerate)
	}
	size := hi.Sub(lo)
	if !(size.X > 0 && size.Y > 0 && size.Z > 0) || math.IsInf(size.X+size.Y+size.Z, 0) {
		return nil, nil, fmt.Errorf("%w: the box has no volume", ErrDegenerate)
	}
	h := cell / float64(res)
	// The nodes lie at lo + (i − 1/2)·h, so the first is half a step outside the box and the
	// box's own face is the zero of d between it and the next, a half step from either; the
	// last node lies beyond the far face the same way.
	count := func(extent float64) int { return int(math.Floor(extent/h+0.5)) + 3 }
	n := [3]int{count(size.X), count(size.Y), count(size.Z)}
	if float64(n[0])*float64(n[1])*float64(n[2]) > maxNodes {
		return nil, nil, fmt.Errorf("%w: more than %d samples; use a coarser resolution or a smaller box", ErrDegenerate, maxNodes)
	}
	k := 2 * math.Pi / cell
	at := func(i, j, l int) r3.Vec {
		return r3.Vec{X: lo.X + (float64(i)-0.5)*h, Y: lo.Y + (float64(j)-0.5)*h, Z: lo.Z + (float64(l)-0.5)*h}
	}
	index := func(i, j, l int) int { return (l*n[1]+j)*n[0] + i }
	field := make([]float64, n[0]*n[1]*n[2])
	for l := range n[2] {
		if err := ctx.Err(); err != nil {
			return nil, nil, err
		}
		for j := range n[1] {
			for i := range n[0] {
				p := at(i, j, l)
				g := kind.field(k*p.X, k*p.Y, k*p.Z)
				d := math.Min(math.Min(p.X-lo.X, hi.X-p.X), math.Min(math.Min(p.Y-lo.Y, hi.Y-p.Y), math.Min(p.Z-lo.Z, hi.Z-p.Z)))
				f := math.Min(level-math.Abs(g), d)
				if f == 0 {
					f = 1e-12 // a node exactly on the surface counts as inside
				}
				field[index(i, j, l)] = f
			}
		}
	}
	b := &builder{field: field, vertexAt: map[[2]int]int{}}
	// The six tetrahedra of a cube, all about its diagonal from corner 0 to corner 6, so
	// neighbouring cubes agree on the faces they share.
	tets := [6][4]int{{0, 5, 1, 6}, {0, 1, 2, 6}, {0, 2, 3, 6}, {0, 3, 7, 6}, {0, 7, 4, 6}, {0, 4, 5, 6}}
	corner := [8][3]int{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}}
	for l := 0; l+1 < n[2]; l++ {
		if err := ctx.Err(); err != nil {
			return nil, nil, err
		}
		for j := 0; j+1 < n[1]; j++ {
			for i := 0; i+1 < n[0]; i++ {
				var ids [8]int
				for c, o := range corner {
					ids[c] = index(i+o[0], j+o[1], l+o[2])
				}
				for _, t := range tets {
					b.tet([4]int{ids[t[0]], ids[t[1]], ids[t[2]], ids[t[3]]}, func(id int) r3.Vec {
						return at(id%n[0], (id/n[0])%n[1], id/(n[0]*n[1]))
					})
				}
				if len(b.triangles) > maxTriangles {
					return nil, nil, fmt.Errorf("%w: more than %d triangles; use a coarser resolution or a smaller box", ErrDegenerate, maxTriangles)
				}
			}
		}
	}
	if len(b.triangles) == 0 {
		return nil, nil, fmt.Errorf("%w: the level leaves no solid in the box", ErrDegenerate)
	}
	return b.vertices, b.triangles, nil
}

// builder collects the mesh: one vertex for each grid edge the surface crosses,
// shared by every triangle that meets it.
type builder struct {
	field     []float64
	vertexAt  map[[2]int]int
	vertices  []r3.Vec
	triangles [][3]int
}

// crossing is the vertex where the surface crosses the grid edge between nodes a
// and b, one inside (positive field) and one not, found by linear interpolation
// of the field.
func (b *builder) crossing(a, c int, pos func(int) r3.Vec) int {
	if a > c {
		a, c = c, a
	}
	key := [2]int{a, c}
	if v, ok := b.vertexAt[key]; ok {
		return v
	}
	fa, fc := b.field[a], b.field[c]
	t := fa / (fa - fc)
	pa, pc := pos(a), pos(c)
	v := len(b.vertices)
	b.vertices = append(b.vertices, pa.Add(pc.Sub(pa).Scale(t)))
	b.vertexAt[key] = v
	return v
}

// tet cuts one tetrahedron of grid nodes. A node is inside when its field is
// positive. The triangles it adds wind so that their normal points from the
// inside nodes toward the outside ones.
func (b *builder) tet(ids [4]int, pos func(int) r3.Vec) {
	var in, out []int
	for _, id := range ids {
		if b.field[id] > 0 {
			in = append(in, id)
		} else {
			out = append(out, id)
		}
	}
	if len(in) == 0 || len(out) == 0 {
		return
	}
	centre := func(nodes []int) r3.Vec {
		var s r3.Vec
		for _, id := range nodes {
			s = s.Add(pos(id))
		}
		return s.Scale(1 / float64(len(nodes)))
	}
	toward := centre(out).Sub(centre(in))
	emit := func(x, y, z int) {
		n := b.vertices[y].Sub(b.vertices[x]).Cross(b.vertices[z].Sub(b.vertices[x]))
		if n.Dot(toward) < 0 {
			y, z = z, y
		}
		b.triangles = append(b.triangles, [3]int{x, y, z})
	}
	switch {
	case len(in) == 1:
		emit(b.crossing(in[0], out[0], pos), b.crossing(in[0], out[1], pos), b.crossing(in[0], out[2], pos))
	case len(out) == 1:
		emit(b.crossing(in[0], out[0], pos), b.crossing(in[1], out[0], pos), b.crossing(in[2], out[0], pos))
	default:
		// Two inside and two outside: a quad over the four edges that cross.
		a, c := in[0], in[1]
		d, e := out[0], out[1]
		v0, v1 := b.crossing(a, d, pos), b.crossing(a, e, pos)
		v2, v3 := b.crossing(c, e, pos), b.crossing(c, d, pos)
		emit(v0, v1, v2)
		emit(v0, v2, v3)
	}
}
