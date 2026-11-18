package brepmodify

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// checkCutterClear refuses a run whose cutter reaches a held facet of a face the chamfer must not touch. On a mesh
// Boolean result the loop's polygon holds the face's own material alone, so it cannot see a feature that stands in the
// cutter's band from another face (a lug through a wall, a shell floating in it). Every held facet of a face outside
// the run's own faces must therefore miss the cutter. The run's own faces are the face of the loop, both faces of every
// selected edge, and both faces of the edge before and after the run, where the bevel runs out onto the next wall or
// stops on its plane.
//
// The test refuses and never admits. No facet may touch a cutter facet, which meshbool's exact triangle classification
// decides on the held floats; a classification that cannot be decided, or a facet with collinear corners where the
// classification needs a plane, refuses. A facet within its vertices' bounds plus the cutter's bound of a cutter facet
// is too near to tell apart and refuses. A facet that crosses nothing may still lie wholly inside the cutter, so one
// corner of every such facet is placed by the exact parity of a ray through the cutter's mesh: inside, on its boundary
// or undecided refuses. Inflating by the bounds only refuses more.
//
// cutter is the body the run's cut takes, after any end clip, so the sector a clip removes is not tested.
func (r chamferRun) checkCutterClear(ctx context.Context, body, cutter *topo.Body, bound float64) error {
	return checkCutterClearOf(ctx, body, cutter, bound, r.ownFaces(body), "partial chamfer")
}

// checkCutterClearOf is checkCutterClear for any cutter: every held facet of a face outside allowed (indices in
// topobuild.Faces order) must miss cutter. what names the chamfer in a refusal.
func checkCutterClearOf(ctx context.Context, body, cutter *topo.Body, bound float64, allowed map[int]struct{},
	what string) error {
	fp, ok := body.Payload.(facetedrecord.Payload)
	if !ok {
		return nil
	}
	cp, ok := cutter.Payload.(facetedrecord.Payload)
	if !ok || len(cp.Tris) == 0 {
		return fmt.Errorf("%w: the %s's cutter has no held mesh to test", decaderr.ErrUnsupported, what)
	}
	if len(fp.VertexBound) != len(fp.Verts) || len(fp.FaceOf) != len(fp.Tris) {
		return fmt.Errorf("%w: the receiver's held mesh has no vertex bounds to test the cutter against",
			decaderr.ErrUnsupported)
	}
	for _, b := range cp.VertexBound {
		bound = max(bound, b)
	}

	boxes := make([]facetBox, len(cp.Tris))
	for i, t := range cp.Tris {
		boxes[i] = boxOfFacet(cp.Verts, t)
	}
	grid := newFacetGrid(boxes)
	whole := boxes[0]
	for _, b := range boxes[1:] {
		whole = whole.join(b)
	}
	cutterFacets := make([]*liftedFacet, len(cp.Tris))
	lift := func(verts []r3.Vec, t [3]int) *liftedFacet {
		var f liftedFacet
		for k, v := range t {
			f.f[k] = verts[v]
			f.x[k] = proof.XptOf(verts[v])
		}
		f.n = proof.XtriangleNormal(f.x[0], f.x[1], f.x[2])
		f.flat = f.n.X.Sign() == 0 && f.n.Y.Sign() == 0 && f.n.Z.Sign() == 0
		return &f
	}
	budget := proofbound.NewWorkBudget(ctx)
	parity := meshbool.NewParityMesh(cp.Verts, cp.Tris)
	placed := map[int]struct{}{}
	var near []int
	for ti, t := range fp.Tris {
		if _, ok := allowed[fp.FaceOf[ti]]; ok {
			continue
		}
		infl := bound
		for _, v := range t {
			infl = max(infl, bound+fp.VertexBound[v])
		}
		infl += 1e-9
		fb := boxOfFacet(fp.Verts, t)
		grown := facetBox{lo: fb.lo.Sub(r3.Vec{X: infl, Y: infl, Z: infl}), hi: fb.hi.Add(r3.Vec{X: infl, Y: infl, Z: infl})}
		if !grown.meets(whole) {
			continue
		}
		f := lift(fp.Verts, t)
		near = grid.near(grown, near[:0])
		for _, ci := range near {
			if !grown.meets(boxes[ci]) {
				continue
			}
			if err := budget.Step(); err != nil {
				return err
			}
			if cutterFacets[ci] == nil {
				cutterFacets[ci] = lift(cp.Verts, cp.Tris[ci])
			}
			c := cutterFacets[ci]
			if f.flat || c.flat {
				// The classification needs a plane for each facet.
				return fmt.Errorf("%w: a flat facet beside the %s's cutter leaves its clearance undecided",
					decaderr.ErrUnsupported, what)
			}
			contact, err := meshbool.TriTriClassify(c.f, f.f, c.x, f.x, c.n, f.n)
			if err != nil {
				return fmt.Errorf("%w: the %s's clearance from the rest of the body is undecided: %w",
					decaderr.ErrUnsupported, what, err)
			}
			if contact.Kind != meshbool.ContactNone {
				return refuseOtherFace(what, contactAt3(contact, f.f))
			}
			if d := meshbool.TriTriDistance(c.f, f.f); d <= infl {
				return refuseOtherFace(what, f.f[0])
			}
		}
		// A facet that crosses nothing lies wholly inside the cutter or wholly outside it.
		if !fb.meets(whole) {
			continue
		}
		if _, done := placed[t[0]]; done {
			continue
		}
		placed[t[0]] = struct{}{}
		inside, on, err := meshbool.MeshParityAllContext(ctx, proof.XptOf(fp.Verts[t[0]]), parity)
		if err != nil || inside || on {
			return refuseOtherFace(what, fp.Verts[t[0]])
		}
	}
	return nil
}

func refuseOtherFace(what string, p r3.Vec) error {
	return fmt.Errorf(`%w: a %s's cutter meets or comes within bounds of another face of the body near %.4g,%.4g,%.4g`,
		decaderr.ErrUnsupported, what, p.X, p.Y, p.Z)
}

func contactAt3(c meshbool.TriContact, f [3]r3.Vec) r3.Vec {
	if c.Kind == meshbool.ContactPoint || c.Kind == meshbool.ContactSegment {
		return c.P0.Vec()
	}
	return f[0]
}

// ownFaces are the indices, in topobuild.Faces order, of the faces the run's cutter may meet: the loop's face, both
// faces of each selected edge, and both faces of the edge before and after the run.
func (r chamferRun) ownFaces(body *topo.Body) map[int]struct{} {
	faces := topobuild.Faces(body)
	out := map[int]struct{}{}
	add := func(f *topo.Face) {
		if i := slices.Index(faces, f); i >= 0 {
			out[i] = struct{}{}
		}
	}
	add(r.loop.face)
	n := len(r.loop.edges)
	for i := -1; i <= r.count; i++ {
		for _, f := range r.loop.edges[((r.first+i)%n+n)%n].Faces {
			add(f)
		}
	}
	return out
}

// facetBox is a facet's float bounding box. Its corners are exact coordinates, so two boxes that miss prove the facets
// miss.
type facetBox struct{ lo, hi r3.Vec }

func boxOfFacet(verts []r3.Vec, t [3]int) facetBox {
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

// liftedFacet is a facet's float corners with their exact lifts and normal. flat is set when the corners are collinear.
type liftedFacet struct {
	f    [3]r3.Vec
	x    [3]proof.Xpt
	n    proof.Xpt
	flat bool
}

// facetGrid buckets facet boxes in a uniform grid over their joint box, so a query is tested only against the facets in
// the cells its box covers. A cell index is monotone in each coordinate, so two boxes that meet cover a common cell: the
// grid never drops a pair the exact test would have to see.
type facetGrid struct {
	all      facetBox
	lo, size r3.Vec
	n        int
	cells    [][]int
	stamp    []int
	round    int
}

func newFacetGrid(boxes []facetBox) *facetGrid {
	all := boxes[0]
	for _, b := range boxes[1:] {
		all = all.join(b)
	}
	k := max(1, min(32, int(math.Cbrt(float64(len(boxes))))))
	g := &facetGrid{all: all, lo: all.lo, n: k, stamp: make([]int, len(boxes))}
	span := all.hi.Sub(all.lo)
	g.size = r3.NewVec(span.X/float64(k), span.Y/float64(k), span.Z/float64(k))
	g.cells = make([][]int, k*k*k)
	for i, b := range boxes {
		g.visit(b, func(c int) { g.cells[c] = append(g.cells[c], i) })
	}
	return g
}

// gridCell is the grid index of coordinate x along an axis, clamped to the grid. The clamp comes before the conversion
// to an int: a float outside int's range converts to an implementation-defined value that would break the order.
func gridCell(x, lo, size float64, n int) int {
	if !(size > 0) {
		return 0
	}
	q := math.Floor((x - lo) / size)
	if q >= float64(n-1) {
		return n - 1
	}
	if !(q > 0) {
		return 0
	}
	return int(q)
}

func (g *facetGrid) visit(b facetBox, f func(int)) {
	x0, x1 := gridCell(b.lo.X, g.lo.X, g.size.X, g.n), gridCell(b.hi.X, g.lo.X, g.size.X, g.n)
	y0, y1 := gridCell(b.lo.Y, g.lo.Y, g.size.Y, g.n), gridCell(b.hi.Y, g.lo.Y, g.size.Y, g.n)
	z0, z1 := gridCell(b.lo.Z, g.lo.Z, g.size.Z, g.n), gridCell(b.hi.Z, g.lo.Z, g.size.Z, g.n)
	for x := x0; x <= x1; x++ {
		for y := y0; y <= y1; y++ {
			for z := z0; z <= z1; z++ {
				f((x*g.n+y)*g.n + z)
			}
		}
	}
}

// near appends to out, in ascending order and once each, the facets in the cells b covers. A box that misses the joint
// box gets none.
func (g *facetGrid) near(b facetBox, out []int) []int {
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
