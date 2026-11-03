package occupancy

import (
	"fmt"
	"math"
	"math/big"
	"sync"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Leaf is one leaf of a Union tree (agent-docs/body-refine-design.md §4): a
// true solid C, held as outer convex pieces that together contain it, the
// shape that bounds how far a held facet can poke out of it, and Delta, the
// published two-sided bound of the held mesh the Boolean used (C ⊂ A'^{+Delta}).
type Leaf struct {
	Delta float64
	shape shape
	box   [2]r3.Vec
	// cells are float boxes, each containing one outer piece (and so the
	// part of the true solid that piece covers), for nearness readings.
	cells [][2]r3.Vec
	// pieces builds the exact outer pieces on first use: only the certificate
	// reads them, so a Boolean that only composes the record never pays for
	// their rational construction.
	pieces []*lazyPiece
	// inner builds exact convex pieces inside the true solid, for a leaf a Cut
	// may use as its tool; nil for a leaf that offers none.
	inner *lazyPieces
	// key names the true solid: two leaves read from one record, through
	// any number of reads, share it.
	key string
	// operand names the one held mesh this leaf came in with: leaves of one
	// operand (a composite sweep's spans) share it, and their joins are not
	// crossings of two Boolean operands.
	operand *int
}

// shape bounds, for a held triangle, how far any of its points lies outside
// the leaf's true solid: +Inf when it cannot say.
type shape interface {
	poke(tri [3]r3.Vec) float64
	// boundaryFar reports whether every point of the box (lo, hi) is proven
	// farther than pad from the true solid's boundary; false when it cannot
	// say.
	boundaryFar(lo, hi r3.Vec, pad float64) bool
}

// nearHoleWall reports whether the box (lo, hi) may come within pad of the
// hole's wall region: within pad + 2·Delta of the true tool's boundary, which
// holds every point between the true tool and its held mesh.
func (l Leaf) nearHoleWall(lo, hi r3.Vec, pad float64) bool {
	if !l.near(lo, hi, pad+l.Delta) {
		return false
	}
	return !l.shape.boundaryFar(lo, hi, proofbound.AbsSumUpper(pad, l.Delta, l.Delta))
}

// Record is the occupancy record of a held Union result X' of the true union
// X of its leaves (§5). The union of the held leaf meshes lies within Drift of
// X' (∪ A'_i ⊂ X'^{+Drift}), and Cross bounds how deep inside X' a point
// outside X can sit near a crossing of two held operands, accumulated over
// every step. A Record is immutable once built.
type Record struct {
	Leaves []Leaf
	// Holes are the tool leaves Cut removed: X is the union of Leaves less
	// the union of Holes, which holds for any tree of Unions followed by Cuts.
	Holes []Leaf
	Drift float64
	Cross float64
	// Sites are float boxes holding every crossing line an earlier step
	// made; Cross applies only near them.
	Sites [][2]r3.Vec
	grid  *siteGrid
	// Mesh is the held mesh the record was proven for; a leaf's own record
	// leaves it zero, since the Boolean that composes it binds the result.
	Mesh Binding
}

// LeafRecord is the record of a set of leaves whose union one held mesh
// stands for, as a composite sweep's spans do.
func LeafRecord(leaves ...Leaf) *Record {
	operand := new(int)
	out := &Record{Leaves: append([]Leaf(nil), leaves...), grid: &siteGrid{}}
	for i := range out.Leaves {
		out.Leaves[i].operand = operand
	}
	return out
}

// Bulges returns λ(t) for every triangle of a held mesh of r's union
// (§5): every point of X' outside X lies within λ(t) of some facet t. A
// facet's bound is the smallest of its own published two-sided bound delta[t],
// which already says that every held point lies within it of the true
// boundary, and of the largest of two readings of the leaves: how far the
// facet pokes out of the nearest leaf (the least over the leaves near it),
// and, for a facet near an earlier crossing site, the accumulated crossing
// depth; a facet near a hole also reads that hole's bound.
func (r *Record) Bulges(verts []r3.Vec, tris [][3]int, delta []float64) []float64 {
	lam, _, _ := r.bulgeParts(verts, tris, delta)
	return lam
}

// BulgesPokes is Bulges together with each facet's least poke over the
// leaves near it, the reading LocalBulgeAt takes in place of the facet.
func (r *Record) BulgesPokes(verts []r3.Vec, tris [][3]int, delta []float64) ([]float64, []float64) {
	lam, _, pokes := r.bulgeParts(verts, tris, delta)
	return lam, pokes
}

// BulgeParts is Bulges together with each facet's own reach outside X: how
// far any point of the facet lies outside the true solid, its poke out of the
// nearest leaf or, near a hole's wall, that hole's bound, capped by the
// facet's own bound. An earlier crossing raises a facet's bulge but not its
// reach: the crossing line lies on held facets.
func (r *Record) BulgeParts(verts []r3.Vec, tris [][3]int, delta []float64) ([]float64, []float64) {
	out, reach, _ := r.bulgeParts(verts, tris, delta)
	return out, reach
}

// bulgeParts is Bulges, the reaches of BulgeParts and each facet's least poke.
func (r *Record) bulgeParts(verts []r3.Vec, tris [][3]int, delta []float64) ([]float64, []float64, []float64) {
	out := make([]float64, len(tris))
	reach := make([]float64, len(tris))
	pokes := r.meshPokes(verts, tris)
	for t, tri := range tris {
		tv := [3]r3.Vec{verts[tri[0]], verts[tri[1]], verts[tri[2]]}
		lo, hi := triBox(tv)
		lam := pokes[t]
		for _, l := range r.Holes {
			if l.nearHoleWall(lo, hi, r.Cross) {
				// The true hole reaches past its held wall by at most the tool
				// mesh's own two-sided bound, and those points are held
				// material outside X.
				lam = math.Max(lam, l.Delta)
			}
		}
		reach[t] = math.Min(delta[t], lam)
		if r.Cross > 0 && r.sites().near(lo, hi, r.Cross) {
			lam = math.Max(lam, r.Cross)
		}
		out[t] = math.Min(delta[t], lam)
	}
	return out, reach, pokes
}

// meshPokes is, per facet, the least poke over the leaves near it, each leaf
// read in order. A cylinder leaf's poke is fixed by its corners' readings, so
// each shared vertex is read once per leaf instead of once per facet.
func (r *Record) meshPokes(verts []r3.Vec, tris [][3]int) []float64 {
	pokes := make([]float64, len(tris))
	for t := range pokes {
		pokes[t] = math.Inf(1)
	}
	var corner []float64
	for _, l := range r.Leaves {
		cyl, isCyl := l.shape.(cylinderShape)
		var rr, twoR *big.Rat
		if isCyl {
			if corner == nil {
				corner = make([]float64, len(verts))
			}
			for v := range corner {
				corner[v] = math.NaN()
			}
			rr, twoR = cyl.radii()
		}
		for t, tri := range tris {
			tv := [3]r3.Vec{verts[tri[0]], verts[tri[1]], verts[tri[2]]}
			lo, hi := triBox(tv)
			if !l.near(lo, hi, r.Cross+l.Delta) {
				continue
			}
			if !isCyl {
				pokes[t] = math.Min(pokes[t], l.shape.poke(tv))
				continue
			}
			worst := 0.0
			for _, v := range tri {
				if math.IsNaN(corner[v]) {
					corner[v] = cyl.cornerPoke(verts[v], rr, twoR)
				}
				worst = max(worst, corner[v])
			}
			pokes[t] = math.Min(pokes[t], cyl.pokeOf(worst))
		}
	}
	return pokes
}

// SiteReach is how far from a contact segment a bulge source (an earlier
// crossing site, a hole, a facet's own poke) is read when bounding that
// contact's deep term; a deep term at or past it is not proven.
const SiteReach = 1.0

// LocalBulge is λ for facet tri of a held mesh of r's union read at one
// contact segment held in site: the facet's poke out of the nearest leaf,
// and, within SiteReach of the segment, each hole's bound and the crossing
// depth near an earlier site. A point of X' outside X near the segment lies
// within that of the facet, since every source farther than SiteReach from
// the segment only places points farther than any proven deep term.
func (r *Record) LocalBulge(tri [3]r3.Vec, delta float64, site [2]r3.Vec) float64 {
	lo, hi := triBox(tri)
	poke := math.Inf(1)
	for _, l := range r.Leaves {
		if l.near(lo, hi, r.Cross+l.Delta) {
			poke = math.Min(poke, l.shape.poke(tri))
		}
	}
	return r.LocalBulgeAt(poke, delta, site)
}

// LocalBulgeAt is LocalBulge for a facet whose least poke over the leaves near
// it is poke, as BulgesPokes reads it.
func (r *Record) LocalBulgeAt(poke, delta float64, site [2]r3.Vec) float64 {
	lam := poke
	for _, l := range r.Holes {
		if l.nearHoleWall(site[0], site[1], SiteReach) {
			lam = math.Max(lam, l.Delta)
		}
	}
	if r.Cross > 0 && r.sites().near(site[0], site[1], SiteReach) {
		lam = math.Max(lam, r.Cross)
	}
	return math.Min(delta, lam)
}

// Compose is the record of round(X'_a ∪ X'_b) for one mesh Union step.
// weld is the step's largest vertex rounding. deepTerm is the largest, over
// the step's contact segments, of 2·max(λ_a(t), λ_b(u))/sin θ(t, u): a point
// shallow in both held operands but deep in their union lies within that of a
// crossing line on the union's boundary. That holds only when every pair of
// held facets that come that close without a proven crossing was refused,
// which the caller checks against the hidden-tangency gate before calling.
// Compose answers nil when deepTerm is not finite.
func Compose(a, b *Record, weld, deepTerm float64, sites [][2]r3.Vec) *Record {
	if a == nil || b == nil || proofbound.IsNonFinite(weld) || weld < 0 ||
		proofbound.IsNonFinite(deepTerm) || deepTerm < 0 || !sameHoles(a.Holes, b.Holes) {
		return nil
	}
	// (A ∖ H) ∪ (C ∖ H) = (A ∪ C) ∖ H, so two operands cut by the very same
	// tools join under those tools; any other mix of holes is not a union
	// less holes and has no record.
	out := &Record{
		Leaves: append(append(make([]Leaf, 0, len(a.Leaves)+len(b.Leaves)), a.Leaves...), b.Leaves...),
		Holes:  joinHoles(a.Holes, b.Holes),
		Drift:  proofbound.AbsSumUpper(max(a.Drift, b.Drift), weld),
		Cross:  proofbound.AbsSumUpper(max(a.Cross, b.Cross, deepTerm), weld),
		Sites:  joinSites(a.Sites, b.Sites, sites),
	}
	out.grid = &siteGrid{boxes: out.Sites}
	return out
}

// sameHoles reports whether two hole lists name the same tool leaves in the
// same order, by the operand each came in with and its place in it.
func sameHoles(a, b []Leaf) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i].key != b[i].key {
			return false
		}
	}
	return true
}

// joinHoles is one list of shared holes, each keeping the larger of the two
// held meshes' bounds.
func joinHoles(a, b []Leaf) []Leaf {
	out := append([]Leaf(nil), a...)
	for i := range out {
		out[i].Delta = max(out[i].Delta, b[i].Delta)
	}
	return out
}

// CutCompose is the record of round(X'_a ∖ X'_b) for one mesh Cut step, with
// Compose's weld and deepTerm. Here a tool facet's bulge is the tool mesh's
// own two-sided bound: the true tool reaches past its held wall by at most
// that, and the points between are held material outside X. The tool must be
// a record of leaves that each offer inner pieces, with no holes of its own;
// otherwise CutCompose answers nil.
func CutCompose(a, b *Record, weld, deepTerm float64, sites [][2]r3.Vec) *Record {
	if a == nil || b == nil || proofbound.IsNonFinite(weld) || weld < 0 ||
		proofbound.IsNonFinite(deepTerm) || deepTerm < 0 || len(b.Holes) != 0 {
		return nil
	}
	for _, l := range b.Leaves {
		if l.inner == nil {
			return nil
		}
	}
	out := &Record{
		Leaves: append([]Leaf(nil), a.Leaves...),
		Holes:  append(append(make([]Leaf, 0, len(a.Holes)+len(b.Leaves)), a.Holes...), b.Leaves...),
		Drift:  proofbound.AbsSumUpper(max(a.Drift, b.Drift), weld),
		Cross:  proofbound.AbsSumUpper(max(a.Cross, b.Cross, deepTerm), weld),
		Sites:  joinSites(a.Sites, b.Sites, sites),
	}
	out.grid = &siteGrid{boxes: out.Sites}
	return out
}

// Shifted is r for a copy of its held mesh moved by at most d: every leaf's and
// hole's two-sided bound and the drift grow by d, and the true solid is r's
// own. A non-finite or negative d answers nil.
func (r *Record) Shifted(d float64) *Record {
	if proofbound.IsNonFinite(d) || d < 0 {
		return nil
	}
	out := *r
	out.Leaves = append([]Leaf(nil), r.Leaves...)
	for i := range out.Leaves {
		out.Leaves[i].Delta = proofbound.AbsSumUpper(out.Leaves[i].Delta, d)
	}
	out.Holes = append([]Leaf(nil), r.Holes...)
	for i := range out.Holes {
		out.Holes[i].Delta = proofbound.AbsSumUpper(out.Holes[i].Delta, d)
	}
	out.Drift = proofbound.AbsSumUpper(r.Drift, d)
	out.Mesh = Binding{}
	return &out
}

// Inner returns the hole leaves' inner pieces, building them on first use. A
// hole too thin to hold one offers none: a moved facet piece is then never
// certified by containment in it, which only costs the search.
func (r *Record) Inner() ([]*Piece, error) {
	var out []*Piece
	for _, l := range r.Holes {
		if l.inner == nil {
			continue
		}
		p, err := l.inner.get()
		if err != nil {
			return nil, err
		}
		out = append(out, p...)
	}
	return out, nil
}

func joinSites(lists ...[][2]r3.Vec) [][2]r3.Vec {
	var out [][2]r3.Vec
	for _, l := range lists {
		out = append(out, l...)
	}
	return out
}

// siteCell is the edge of the uniform grid crossing sites are binned on.
const siteCell = 2.0

type siteGrid struct {
	once  sync.Once
	cells map[[3]int64][]int
	// wide are boxes too large to bin, checked against every query.
	wide  []int
	boxes [][2]r3.Vec
}

// sites returns the record's site index. Every constructor sets grid, so
// concurrent readers share one index and its one build.
func (r *Record) sites() *siteGrid {
	if r.grid == nil {
		return &siteGrid{}
	}
	return r.grid
}

// index bins the boxes on the grid, once.
func (g *siteGrid) index() {
	g.once.Do(func() {
		g.cells = map[[3]int64][]int{}
		for i, b := range g.boxes {
			lo, hi := cellOf(b[0]), cellOf(b[1])
			if (hi[0]-lo[0]+1)*(hi[1]-lo[1]+1)*(hi[2]-lo[2]+1) > 1<<12 {
				g.wide = append(g.wide, i)
				continue
			}
			for x := lo[0]; x <= hi[0]; x++ {
				for y := lo[1]; y <= hi[1]; y++ {
					for z := lo[2]; z <= hi[2]; z++ {
						k := [3]int64{x, y, z}
						g.cells[k] = append(g.cells[k], i)
					}
				}
			}
		}
	})
}

func cellOf(v r3.Vec) [3]int64 {
	return [3]int64{int64(math.Floor(v.X / siteCell)), int64(math.Floor(v.Y / siteCell)), int64(math.Floor(v.Z / siteCell))}
}

// near reports whether the box (lo, hi) comes within pad of a site.
func (g *siteGrid) near(lo, hi r3.Vec, pad float64) bool {
	g.index()
	for _, i := range g.wide {
		if boxesNear(lo, hi, g.boxes[i], pad) {
			return true
		}
	}
	a := cellOf(r3.Vec{X: lo.X - pad, Y: lo.Y - pad, Z: lo.Z - pad})
	b := cellOf(r3.Vec{X: hi.X + pad, Y: hi.Y + pad, Z: hi.Z + pad})
	if (b[0]-a[0]+1)*(b[1]-a[1]+1)*(b[2]-a[2]+1) > 1<<16 {
		for _, s := range g.boxes {
			if boxesNear(lo, hi, s, pad) {
				return true
			}
		}
		return false
	}
	for x := a[0]; x <= b[0]; x++ {
		for y := a[1]; y <= b[1]; y++ {
			for z := a[2]; z <= b[2]; z++ {
				for _, i := range g.cells[[3]int64{x, y, z}] {
					if boxesNear(lo, hi, g.boxes[i], pad) {
						return true
					}
				}
			}
		}
	}
	return false
}

// Bounds are a record's figures over one held mesh of its union: every held
// point lies within Excess of X, and every true boundary point within
// TrueToHeld of the held boundary (§5 (i), (ii)).
type Bounds struct {
	Excess, TrueToHeld float64
}

// MeshBounds reads Bounds from the bulges and reaches (BulgeParts) of the
// record's own held mesh. A point of X' outside X lies within λ(t) of a facet
// t whose points lie within its reach of X, so Excess is the largest sum of
// the two; a true boundary point inside X' has a point of X' outside X
// arbitrarily near, so it lies within the largest bulge of the held boundary.
func (r *Record) MeshBounds(bulges, reaches []float64) Bounds {
	lam, d, excess := 0.0, 0.0, 0.0
	for t, b := range bulges {
		lam = max(lam, b)
		excess = max(excess, proofbound.AbsSumUpper(b, reaches[t]))
	}
	for _, l := range append(append([]Leaf(nil), r.Leaves...), r.Holes...) {
		d = max(d, l.Delta)
	}
	near := proofbound.AbsSumUpper(d, r.Drift)
	if len(r.Holes) != 0 {
		// A true point a Cut exposed may sit near a held rim, which the
		// crossing depth covers.
		near = proofbound.AbsSumUpper(near, r.Cross)
	}
	return Bounds{
		Excess:     excess,
		TrueToHeld: max(near, lam),
	}
}

// Pieces returns every leaf's outer pieces, building them on first use.
func (r *Record) Pieces() ([]*Piece, error) {
	var out []*Piece
	for _, l := range r.Leaves {
		p, err := l.Pieces()
		if err != nil {
			return nil, err
		}
		out = append(out, p...)
	}
	return out, nil
}

// PiecesNear returns the outer pieces whose float boxes come within pad of
// one of the query boxes, building only those. A facet search over facets in
// the query boxes with translations up to pad never meets any other piece.
func (r *Record) PiecesNear(queries [][2]r3.Vec, pad float64) ([]*Piece, error) {
	g := &siteGrid{boxes: queries}
	var out []*Piece
	for _, l := range r.Leaves {
		for _, part := range l.pieces {
			if !g.near(part.box[0], part.box[1], pad) {
				continue
			}
			p, err := part.get()
			if err != nil {
				return nil, err
			}
			out = append(out, p)
		}
	}
	return out, nil
}

// Pieces returns the leaf's outer pieces, building them on first use.
func (l Leaf) Pieces() ([]*Piece, error) {
	out := make([]*Piece, 0, len(l.pieces))
	for _, part := range l.pieces {
		p, err := part.get()
		if err != nil {
			return nil, err
		}
		out = append(out, p)
	}
	return out, nil
}

// lazyPiece is one outer piece, built on first use, with a float box that
// holds it.
type lazyPiece struct {
	box   [2]r3.Vec
	once  sync.Once
	build func() (*Piece, error)
	out   *Piece
	err   error
}

func (z *lazyPiece) get() (*Piece, error) {
	z.once.Do(func() { z.out, z.err = z.build() })
	return z.out, z.err
}

type lazyPieces struct {
	once  sync.Once
	build func() ([]*Piece, error)
	out   []*Piece
	err   error
}

func (z *lazyPieces) get() ([]*Piece, error) {
	z.once.Do(func() { z.out, z.err = z.build() })
	return z.out, z.err
}

// near reports whether the box (lo, hi) comes within pad of one of the leaf's
// pieces: the leaf's true solid lies inside them, so a facet farther from
// every piece is farther than pad from the leaf.
func (l Leaf) near(lo, hi r3.Vec, pad float64) bool {
	if !boxesNear(lo, hi, l.box, pad) {
		return false
	}
	for _, c := range l.cells {
		if boxesNear(lo, hi, c, pad) {
			return true
		}
	}
	return false
}

// newLeaf builds a leaf from its nearness cells and its outer pieces, given as
// float boxes and builders.
func newLeaf(cells [][2]r3.Vec, pieces []*lazyPiece, delta float64, s shape) Leaf {
	return Leaf{Delta: delta, shape: s, box: unionBox(cells), cells: cells, pieces: pieces}
}

func unionBox(boxes [][2]r3.Vec) [2]r3.Vec {
	box := boxes[0]
	for _, c := range boxes[1:] {
		box = [2]r3.Vec{
			{X: math.Min(box[0].X, c[0].X), Y: math.Min(box[0].Y, c[0].Y), Z: math.Min(box[0].Z, c[0].Z)},
			{X: math.Max(box[1].X, c[1].X), Y: math.Max(box[1].Y, c[1].Y), Z: math.Max(box[1].Z, c[1].Z)},
		}
	}
	return box
}

// cellBox is a float box around the points within r of the segment a→b,
// widened by relSlack of its own size so it holds the exact piece built about
// that segment.
func cellBox(a, b r3.Vec, r float64) [2]r3.Vec {
	pad := r + relSlack*(a.Len()+b.Len()+r)
	return [2]r3.Vec{
		{X: math.Min(a.X, b.X) - pad, Y: math.Min(a.Y, b.Y) - pad, Z: math.Min(a.Z, b.Z) - pad},
		{X: math.Max(a.X, b.X) + pad, Y: math.Max(a.Y, b.Y) + pad, Z: math.Max(a.Z, b.Z) + pad},
	}
}

// CircleLeaf builds the leaf of the solid cylinder that place maps from
// (u−cu)² + (v−cv)² ≤ r², z0 ≤ z ≤ z1, with the record's section
// displacement sectionDelta and level displacements dz0, dz1 widening it.
// delta is the published bound of the held mesh the Boolean used. slack
// widens the piece past any rounding of the record's own maps beyond the
// exact image of its floats.
func CircleLeaf(place Chain, cu, cv, r, z0, z1, sectionDelta, dz0, dz1, slack float64, sides int,
	delta float64) (Leaf, error) {
	if proofbound.IsNonFinite(sectionDelta) || proofbound.IsNonFinite(dz0) || proofbound.IsNonFinite(dz1) ||
		proofbound.IsNonFinite(slack) || proofbound.IsNonFinite(delta) || sectionDelta < 0 || dz0 < 0 || dz1 < 0 || slack < 0 || delta < 0 {
		return Leaf{}, fmt.Errorf("%w: circle leaf displacement", ErrInvalid)
	}
	shifted := append(Chain{{M: [3][3]float64{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, T: [3]float64{cu, cv, 0}}}, place...)
	outer := proofbound.AbsSumUpper(r, sectionDelta, slack)
	lo, hi := z0-proofbound.AbsSumUpper(dz0, slack), z1+proofbound.AbsSumUpper(dz1, slack)
	cyl, err := newCylinderShape(shifted, r, z0, z1, proofbound.AbsSumUpper(sectionDelta, dz0, dz1))
	if err != nil {
		return Leaf{}, err
	}
	// Cells about one radius long, so a long leaf's nearness reading stays
	// local. The circumscribed polygon reaches outer/cos(π/sides) from the
	// axis, which reach covers.
	reach := outer / math.Cos(math.Pi/float64(sides)) * (1 + 1e-9)
	n := int(math.Min(4096, math.Max(1, math.Ceil((hi-lo)/outer))))
	cells := make([][2]r3.Vec, 0, n)
	for k := range n {
		za := lo + (hi-lo)*float64(k)/float64(n)
		zb := lo + (hi-lo)*float64(k+1)/float64(n)
		a, b := shifted.applyFloat(r3.Vec{Z: za}), shifted.applyFloat(r3.Vec{Z: zb})
		cells = append(cells, cellBox(a, b, reach*cyl.stretch))
	}
	piece := &lazyPiece{box: unionBox(cells), build: func() (*Piece, error) {
		return CirclePrism(shifted, outer, lo, hi, sides)
	}}
	l := newLeaf(cells, []*lazyPiece{piece}, delta, cyl)
	l.key = fmt.Sprintf("circle %v %v %v %v %v %v %v %v %v %v", place, cu, cv, r, z0, z1, sectionDelta, dz0, dz1, slack)
	// Inside: the section narrowed by its displacement and the slack, between
	// the levels narrowed the same way.
	innerR := r - proofbound.AbsSumUpper(sectionDelta, slack)
	ilo, ihi := z0+proofbound.AbsSumUpper(dz0, slack), z1-proofbound.AbsSumUpper(dz1, slack)
	if innerR > 0 && ilo < ihi {
		l.inner = &lazyPieces{build: func() ([]*Piece, error) {
			p, err := InscribedCirclePrism(shifted, innerR, ilo, ihi, sides)
			if err != nil {
				return nil, err
			}
			return []*Piece{p}, nil
		}}
	}
	return l, nil
}

func (c Chain) applyFloat(v r3.Vec) r3.Vec {
	for _, a := range c {
		v = a.applyFloat(v)
	}
	return v
}

// cylinderShape reads how far a held triangle pokes out of a placed solid
// cylinder: each corner is taken back to local coordinates exactly, its radial
// and axial excess bounded from above, and the local excess stretched by the
// maps' Frobenius norms. Both excesses are convex functions, so the largest
// corner reading bounds the whole triangle. A corner the float inverse places
// well inside skips the exact reading; the float residue it could carry is
// far below the margin it is tested against.
type cylinderShape struct {
	inv          []exactAffine
	finv         []Affine
	stretch      float64
	invStretch   float64
	r, z0, z1    float64
	displacement float64
}

func newCylinderShape(place Chain, r, z0, z1, displacement float64) (cylinderShape, error) {
	inv := make([]exactAffine, len(place))
	finv := make([]Affine, len(place))
	stretch := 1.0
	for i, a := range place {
		e, err := invert(a)
		if err != nil {
			return cylinderShape{}, err
		}
		inv[len(place)-1-i] = e
		finv[len(place)-1-i] = e.float()
		stretch = proofbound.UpRound(stretch * opNormUpper(a.M))
	}
	invStretch := 1.0
	for _, a := range finv {
		invStretch = proofbound.UpRound(invStretch * opNormUpper(a.M))
	}
	return cylinderShape{inv: inv, finv: finv, stretch: stretch, invStretch: invStretch, r: r, z0: z0, z1: z1, displacement: displacement}, nil
}

// boundaryFar reads the box's centre in local coordinates: its distance to
// the cylinder's boundary there, divided by the inverse maps' stretch, less
// the box's half-diagonal and a float margin, bounds every box point's world
// distance to the boundary from below.
func (c cylinderShape) boundaryFar(lo, hi r3.Vec, pad float64) bool {
	ctr := lo.Add(hi).Scale(0.5)
	half := hi.Sub(lo).Len() / 2
	p := ctr
	for _, a := range c.finv {
		p = a.applyFloat(p)
	}
	rad := math.Hypot(p.X, p.Y)
	var d float64
	if rad <= c.r && p.Z >= c.z0 && p.Z <= c.z1 {
		d = math.Min(c.r-rad, math.Min(p.Z-c.z0, c.z1-p.Z))
	} else {
		d = math.Max(rad-c.r, math.Max(c.z0-p.Z, p.Z-c.z1))
	}
	margin := 1e-9 * (ctr.Len() + p.Len() + c.r + math.Abs(c.z0) + math.Abs(c.z1))
	return d/c.invStretch-half-margin-c.displacement > pad
}

func (c cylinderShape) poke(tri [3]r3.Vec) float64 {
	rr, twoR := c.radii()
	worst := 0.0
	for _, w := range tri {
		worst = max(worst, c.cornerPoke(w, rr, twoR))
	}
	return c.pokeOf(worst)
}

// radii is r² and 2r, exactly, for cornerPoke.
func (c cylinderShape) radii() (*big.Rat, *big.Rat) {
	return new(big.Rat).Mul(ratOf(c.r), ratOf(c.r)), new(big.Rat).Mul(big.NewRat(2, 1), ratOf(c.r))
}

// cornerPoke bounds how far the corner w lies outside the cylinder in local
// coordinates; 0 when the float reading proves it inside. It is never NaN.
func (c cylinderShape) cornerPoke(w r3.Vec, rr, twoR *big.Rat) float64 {
	f := w
	for _, a := range c.finv {
		f = a.applyFloat(f)
	}
	// A corner the float reading puts inside by more than 1e-6 of the
	// section's size cannot be outside: float evaluation errs by far less.
	margin := 1e-6 * (c.r + math.Abs(c.z0) + math.Abs(c.z1) + w.Len())
	if math.Hypot(f.X, f.Y) < c.r-margin && f.Z > c.z0+margin && f.Z < c.z1-margin {
		return 0
	}
	p := xvecOf(w)
	for _, e := range c.inv {
		p = e.apply(p)
	}
	q := new(big.Rat).Mul(p[0], p[0])
	q.Add(q, new(big.Rat).Mul(p[1], p[1]))
	radial := 0.0
	if q.Cmp(rr) > 0 {
		// sqrt(q) − r = (q − r²)/(sqrt(q) + r) ≤ (q − r²)/(2r).
		radial = ratUpper(new(big.Rat).Quo(q.Sub(q, rr), twoR))
	}
	axial := 0.0
	if d := new(big.Rat).Sub(ratOf(c.z0), p[2]); d.Sign() > 0 {
		axial = ratUpper(d)
	}
	if d := new(big.Rat).Sub(p[2], ratOf(c.z1)); d.Sign() > 0 {
		axial = max(axial, ratUpper(d))
	}
	return proofbound.AbsSumUpper(radial, axial)
}

// pokeOf is the facet's poke from its corners' largest cornerPoke.
func (c cylinderShape) pokeOf(worst float64) float64 {
	return proofbound.AbsSumUpper(proofbound.UpRound(worst*c.stretch), c.displacement)
}

func ratUpper(x *big.Rat) float64 {
	f, exact := x.Float64()
	if !exact && new(big.Rat).SetFloat64(f).Cmp(x) < 0 {
		f = math.Nextafter(f, math.Inf(1))
	}
	return f
}

// exactAffine is a rational affine map.
type exactAffine struct {
	m [3][3]*big.Rat
	t xvec
}

func (e exactAffine) apply(p xvec) xvec {
	var out xvec
	// prod is local to this call and never escapes into out. A zero factor
	// adds exactly zero, so the term is skipped. Both signs are read before
	// the skip, so a nil factor panics on the same term as the product would.
	var prod big.Rat
	for i := range 3 {
		s := new(big.Rat).Set(e.t[i])
		for j := range 3 {
			mz, pz := e.m[i][j].Sign() == 0, p[j].Sign() == 0
			if mz || pz {
				continue
			}
			s.Add(s, prod.Mul(e.m[i][j], p[j]))
		}
		out[i] = s
	}
	return out
}

func (e exactAffine) float() Affine {
	var out Affine
	for i := range 3 {
		for j := range 3 {
			out.M[i][j], _ = e.m[i][j].Float64()
		}
		out.T[i], _ = e.t[i].Float64()
	}
	return out
}

func (a Affine) applyFloat(v r3.Vec) r3.Vec {
	p := [3]float64{v.X, v.Y, v.Z}
	var q [3]float64
	for i := range 3 {
		q[i] = a.M[i][0]*p[0] + a.M[i][1]*p[1] + a.M[i][2]*p[2] + a.T[i]
	}
	return r3.Vec{X: q[0], Y: q[1], Z: q[2]}
}

// invert returns the exact inverse of a's map, by the adjugate.
func invert(a Affine) (exactAffine, error) {
	var m [3][3]*big.Rat
	for i := range 3 {
		for j := range 3 {
			m[i][j] = ratOf(a.M[i][j])
		}
	}
	cof := func(i, j int) *big.Rat {
		r0, r1 := (i+1)%3, (i+2)%3
		c0, c1 := (j+1)%3, (j+2)%3
		return new(big.Rat).Sub(new(big.Rat).Mul(m[r0][c0], m[r1][c1]), new(big.Rat).Mul(m[r0][c1], m[r1][c0]))
	}
	det := new(big.Rat)
	for j := range 3 {
		det.Add(det, new(big.Rat).Mul(m[0][j], cof(0, j)))
	}
	if det.Sign() == 0 {
		return exactAffine{}, fmt.Errorf("%w: singular leaf placement", decaderr.ErrDegenerate)
	}
	var out exactAffine
	for i := range 3 {
		for j := range 3 {
			out.m[i][j] = new(big.Rat).Quo(cof(j, i), det)
		}
	}
	for i := range 3 {
		s := new(big.Rat)
		for j := range 3 {
			s.Sub(s, new(big.Rat).Mul(out.m[i][j], ratOf(a.T[j])))
		}
		out.t[i] = s
	}
	return out, nil
}

// Binding ties a Record to the one held mesh it was proven for: the identity
// of that mesh's vertex and triangle arrays. Any producer that changes the
// held geometry (placement, a modify rewrite) allocates new arrays, so a
// record copied along with the rest of a payload no longer matches and is
// never read for it.
type Binding struct {
	verts *r3.Vec
	tris  *[3]int
	nv    int
	nt    int
}

// Bind returns the Binding of a held mesh.
func Bind(verts []r3.Vec, tris [][3]int) Binding {
	if len(verts) == 0 || len(tris) == 0 {
		return Binding{}
	}
	return Binding{&verts[0], &tris[0], len(verts), len(tris)}
}

// Matches reports whether verts and tris are the arrays b was bound to.
func (b Binding) Matches(verts []r3.Vec, tris [][3]int) bool {
	return b.verts != nil && len(verts) == b.nv && len(tris) == b.nt && &verts[0] == b.verts && &tris[0] == b.tris
}

// opNormUpper bounds the spectral norm of m from above: the square root of
// the largest absolute row sum of mᵀm, which bounds mᵀm's largest eigenvalue
// (Gershgorin), with every float step widened. A near-orthonormal map reads
// just above 1, where the Frobenius norm would read about √3.
func opNormUpper(m [3][3]float64) float64 {
	worst := 0.0
	for i := range 3 {
		row := 0.0
		for j := range 3 {
			g := 0.0
			for k := range 3 {
				g += m[k][i] * m[k][j]
			}
			row += math.Abs(g)
		}
		worst = math.Max(worst, row)
	}
	// Each sum of a few float products errs by far less than this widening.
	return proofbound.UpRound(math.Sqrt(proofbound.UpRound(worst*(1+1e-12))) * (1 + 1e-12))
}
