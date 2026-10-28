package retainedblend

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// The held audit checks that the rewritten mesh closes and that its shells
// nest with consistent signs, reading one probe point per shell; it does not
// test facets for intersection. A shell that reaches into the blend region K
// from outside it would cross the new strip and still pass. checkStripClear
// closes that gap for a chain (agent-docs/retained-modify-reach-design.md
// §5.6): no facet of the rest of the body may meet a strip facet, decided by
// meshbool's exact triangle/triangle classification on the held floats. A
// pair that shares a vertex is the construction's own seam, along the base's
// setback contour or a wall's cut, and is not tested. The test only refuses.

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

// checkStripClear refuses the rewrite when a facet the result keeps outside
// the strip w.out[stripFrom:] meets a strip facet it shares no vertex with.
func (w *work) checkStripClear(ctx context.Context, stripFrom int) error {
	strip := w.out[stripFrom:]
	if len(strip) == 0 {
		return nil
	}
	gone := make(map[int]struct{}, len(w.removed))
	for _, f := range w.removed {
		gone[f] = struct{}{}
	}
	rest := make([][3]int, 0, len(w.fp.Tris)-len(gone)+stripFrom)
	for i, t := range w.fp.Tris {
		if _, ok := gone[i]; !ok {
			rest = append(rest, t)
		}
	}
	rest = append(rest, w.out[:stripFrom]...)

	boxes := make([]facetBox, len(strip))
	all := boxOf(w.verts, strip[0])
	for i, t := range strip {
		boxes[i] = boxOf(w.verts, t)
		all = all.join(boxes[i])
	}
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
	for _, r := range rest {
		rb := boxOf(w.verts, r)
		if !rb.meets(all) {
			continue
		}
		var re exactFacet
		for i, s := range strip {
			if err := budget.Step(); err != nil {
				return err
			}
			if !rb.meets(boxes[i]) || sharesVertex(r, s) {
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
				return fmt.Errorf("%w: the blend's clearance from a flat facet is undecided", decaderr.ErrUnsupported)
			}
			contact, err := meshbool.TriTriClassify(exact[i].f, re.f, exact[i].x, re.x, exact[i].n, re.n)
			if err != nil {
				return fmt.Errorf("%w: the blend's clearance from the rest of the body is undecided: %w",
					decaderr.ErrUnsupported, err)
			}
			if contact.Kind != meshbool.ContactNone {
				return fmt.Errorf("%w: the blend meets another facet of the body near %v", decaderr.ErrUnsupported,
					w.verts[s[0]])
			}
		}
	}
	return nil
}

func sharesVertex(a, b [3]int) bool {
	for _, u := range a {
		for _, v := range b {
			if u == v {
				return true
			}
		}
	}
	return false
}
