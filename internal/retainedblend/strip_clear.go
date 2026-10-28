package retainedblend

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// The held audit checks that the rewritten mesh closes and that its shells
// nest with consistent signs, reading one probe point per shell; it does not
// test facets for intersection. A shell that reaches into the blend region K
// from outside it would cross the new strip and still pass. checkStripClear
// closes that gap for every route (agent-docs/retained-modify-reach-design.md
// §5.6): no facet of the rest of the body may meet a strip facet, decided by
// meshbool's exact triangle/triangle classification on the held floats. A
// pair that shares a vertex is the construction's own seam, along the base's
// setback contour, a wall's cut or an end face's trim, and is not tested. The
// test only refuses. An end face that is reflex at an open chain's corner
// takes the blend's end cap, which closes the region K on that side: its
// rebuilt facets are new surface too, and are tested the same way against
// every facet the result keeps outside them.

// facetBox is a facet's float bounding box. Its corners are exact
// coordinates, so two boxes that miss prove the facets miss.
type facetBox struct{ lo, hi r3.Vec }

func boxOf(verts []r3.Vec, t [3]int) facetBox {
	a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
	return facetBox{
		lo: r3.NewVec(min(a.X, b.X, c.X), min(a.Y, b.Y, c.Y), min(a.Z, b.Z, c.Z)),
		hi: r3.NewVec(max(a.X, b.X, c.X), max(a.Y, b.Y, c.Y), max(a.Z, b.Z, c.Z)),
	}
}

func (b facetBox) meets(o facetBox) bool {
	return b.lo.X <= o.hi.X && o.lo.X <= b.hi.X && b.lo.Y <= o.hi.Y && o.lo.Y <= b.hi.Y &&
		b.lo.Z <= o.hi.Z && o.lo.Z <= b.hi.Z
}

func (b facetBox) join(o facetBox) facetBox {
	return facetBox{
		lo: r3.NewVec(min(b.lo.X, o.lo.X), min(b.lo.Y, o.lo.Y), min(b.lo.Z, o.lo.Z)),
		hi: r3.NewVec(max(b.hi.X, o.hi.X), max(b.hi.Y, o.hi.Y), max(b.hi.Z, o.hi.Z)),
	}
}

// exactFacet is a facet's float corners with their exact lifts and normal.
// flat is set when the corners are collinear and the normal is zero.
type exactFacet struct {
	f        [3]r3.Vec
	x        [3]proof.Xpt
	n        proof.Xpt
	ok, flat bool
}

// buildClear is build after checkStripClear, for the routes that blend one
// edge or a mitre run. A chain runs the check itself before its build.
func (w *work) buildClear(ctx context.Context, env topo.Env, groups []facetedrecord.Group, stripFrom int,
	tag blendTag, faceIndex map[*topo.Face]int) (*topo.Body, error) {
	if err := w.checkStripClear(ctx, stripFrom, nil); err != nil {
		return nil, err
	}
	return w.build(ctx, env, groups, stripFrom, tag, faceIndex)
}

// checkStripClear refuses the rewrite when a facet the result keeps outside
// the strip w.out[stripFrom:] meets a strip facet it shares no vertex with,
// and when a facet the result keeps outside an end face that takes a cap,
// whose rebuilt facets fill the range taken of w.out, meets one of them.
func (w *work) checkStripClear(ctx context.Context, stripFrom int, taken [][2]int) error {
	gone := make(map[int]struct{}, len(w.removed))
	for _, f := range w.removed {
		gone[f] = struct{}{}
	}
	held := make([][3]int, 0, len(w.fp.Tris)-len(gone))
	for i, t := range w.fp.Tris {
		if _, ok := gone[i]; !ok {
			held = append(held, t)
		}
	}
	rest := append(append(make([][3]int, 0, len(held)+stripFrom), held...), w.out[:stripFrom]...)
	if err := w.clearOf(ctx, w.out[stripFrom:], rest, "the blend", "the blend's", nil); err != nil {
		return err
	}
	// A rebuilt end face keeps its held boundary vertices, and ear clipping
	// can join one of them to the cap. A facet of another face at such a
	// vertex may meet the end face only where they share: there, and not
	// across the cap. A vertex the construction made (the end column, a
	// setback point, a wall's cut) is shared only with the strip and with the
	// rebuilt base and walls, whose planes meet G's along the cap's own edges.
	made := func(v int) bool { return v >= len(w.fp.Verts) }
	for _, r := range taken {
		rest := append(append(append(make([][3]int, 0, len(held)+stripFrom), held...), w.out[:r[0]]...),
			w.out[r[1]:stripFrom]...)
		if err := w.clearOf(ctx, w.out[r[0]:r[1]], rest, "the cap an end face takes", "the taken cap's",
			made); err != nil {
			return err
		}
	}
	return nil
}

// clearOf refuses when a facet of rest meets a facet of strip. A pair that
// shares a vertex is not tested when seam is nil or seam holds for a shared
// vertex; any other pair that shares vertices must meet only at a shared
// vertex or along the edge between two. what and whose name strip in the
// refusal.
func (w *work) clearOf(ctx context.Context, strip, rest [][3]int, what, whose string, seam func(int) bool) error {
	if len(strip) == 0 {
		return nil
	}

	boxes := make([]facetBox, len(strip))
	for i, t := range strip {
		boxes[i] = boxOf(w.verts, t)
	}
	grid := newBoxGrid(boxes)
	exact := make([]exactFacet, len(strip))
	lift := func(t [3]int) exactFacet {
		var e exactFacet
		for k, v := range t {
			e.f[k] = w.verts[v]
			e.x[k] = proof.XptOf(w.verts[v])
		}
		e.n = proof.XtriangleNormal(e.x[0], e.x[1], e.x[2])
		e.ok = true
		e.flat = e.n.X.Sign() == 0 && e.n.Y.Sign() == 0 && e.n.Z.Sign() == 0
		return e
	}
	budget := proofbound.NewWorkBudget(ctx)
	var near []int
	for _, r := range rest {
		rb := boxOf(w.verts, r)
		near = grid.near(rb, near[:0])
		var re exactFacet
		for _, i := range near {
			s := strip[i]
			if !rb.meets(boxes[i]) {
				continue
			}
			if err := budget.Step(); err != nil {
				return err
			}
			shared := sharedVerts(r, s)
			if len(shared) > 0 && (seam == nil || slices.ContainsFunc(shared, seam)) {
				continue
			}
			if !re.ok {
				re = lift(r)
			}
			if !exact[i].ok {
				exact[i] = lift(s)
			}
			if exact[i].flat || re.flat {
				// The classification needs a plane for each facet.
				return fmt.Errorf("%w: %s clearance from a flat facet is undecided", decaderr.ErrUnsupported, whose)
			}
			contact, err := meshbool.TriTriClassify(exact[i].f, re.f, exact[i].x, re.x, exact[i].n, re.n)
			if err != nil {
				return fmt.Errorf("%w: %s clearance from the rest of the body is undecided: %w",
					decaderr.ErrUnsupported, whose, err)
			}
			if contact.Kind != meshbool.ContactNone && !w.atShared(contact, shared) {
				return fmt.Errorf("%w: %s meets another facet of the body at %v", decaderr.ErrUnsupported, what,
					contactAt(contact, re.f))
			}
		}
	}
	return nil
}

// contactAt is a point of a contact between a strip facet and the rest facet
// with corners rest. A point or segment contact carries its exact end point;
// a coplanar region contact carries none, so the rest facet's first corner
// stands for it.
func contactAt(c meshbool.TriContact, rest [3]r3.Vec) r3.Vec {
	if c.Kind == meshbool.ContactPoint || c.Kind == meshbool.ContactSegment {
		return c.P0.Vec()
	}
	return rest[0]
}

// boxGrid buckets the strip facets' boxes in a uniform grid over their joint
// box, so a facet of the rest of the body is tested only against the strip
// facets in the cells its own box covers. cell is monotone in each
// coordinate, so two boxes that meet cover a common cell: the grid never
// drops a pair the exact test would have to see.
type boxGrid struct {
	all      facetBox
	lo, size r3.Vec
	n        [3]int
	cells    [][]int
	stamp    []int
	round    int
}

func newBoxGrid(boxes []facetBox) *boxGrid {
	all := boxes[0]
	for _, b := range boxes[1:] {
		all = all.join(b)
	}
	k := max(1, min(64, int(math.Cbrt(float64(len(boxes))))))
	g := &boxGrid{all: all, lo: all.lo, n: [3]int{k, k, k}, stamp: make([]int, len(boxes))}
	span := all.hi.Sub(all.lo)
	g.size = r3.NewVec(span.X/float64(k), span.Y/float64(k), span.Z/float64(k))
	g.cells = make([][]int, k*k*k)
	for i, b := range boxes {
		g.visit(b, func(c int) { g.cells[c] = append(g.cells[c], i) })
	}
	return g
}

// cell is the grid index of coordinate x along an axis, clamped to the grid.
func cell(x, lo, size float64, n int) int {
	if !(size > 0) {
		return 0
	}
	// Clamp before converting: a float outside int's range converts to an
	// implementation-defined value, which would break the monotone order.
	q := math.Floor((x - lo) / size)
	if q >= float64(n-1) {
		return n - 1
	}
	if !(q > 0) {
		return 0
	}
	return int(q)
}

func (g *boxGrid) visit(b facetBox, f func(int)) {
	x0, x1 := cell(b.lo.X, g.lo.X, g.size.X, g.n[0]), cell(b.hi.X, g.lo.X, g.size.X, g.n[0])
	y0, y1 := cell(b.lo.Y, g.lo.Y, g.size.Y, g.n[1]), cell(b.hi.Y, g.lo.Y, g.size.Y, g.n[1])
	z0, z1 := cell(b.lo.Z, g.lo.Z, g.size.Z, g.n[2]), cell(b.hi.Z, g.lo.Z, g.size.Z, g.n[2])
	for x := x0; x <= x1; x++ {
		for y := y0; y <= y1; y++ {
			for z := z0; z <= z1; z++ {
				f((x*g.n[1]+y)*g.n[2] + z)
			}
		}
	}
}

// near appends to out, in ascending order and once each, the strip facets
// in the cells b covers. A box that misses the strip's joint box gets none.
func (g *boxGrid) near(b facetBox, out []int) []int {
	if !b.meets(g.all) {
		return out
	}
	g.round++
	g.visit(b, func(c int) {
		for _, i := range g.cells[c] {
			if g.stamp[i] != g.round {
				g.stamp[i] = g.round
				out = append(out, i)
			}
		}
	})
	slices.Sort(out)
	return out
}

// sharedVerts are the vertices two facets share.
func sharedVerts(a, b [3]int) []int {
	var out []int
	for _, u := range a {
		if slices.Contains(b[:], u) {
			out = append(out, u)
		}
	}
	return out
}

// atShared reports whether a contact lies wholly where its two facets share:
// a point at a shared vertex, or a segment between two shared vertices,
// decided on exact points.
func (w *work) atShared(c meshbool.TriContact, shared []int) bool {
	at := func(p proof.Xpt) bool {
		k := p.Key()
		return slices.ContainsFunc(shared, func(v int) bool { return proof.XptOf(w.verts[v]).Key() == k })
	}
	switch c.Kind {
	case meshbool.ContactPoint:
		return at(c.P0)
	case meshbool.ContactSegment:
		return at(c.P0) && at(c.P1)
	}
	return false
}
