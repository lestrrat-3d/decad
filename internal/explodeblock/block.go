package explodeblock

import (
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Verdict is the outcome of a pairwise blocking test.
type Verdict int

const (
	// Free means no part of the other mesh can stand ahead of the mover,
	// even with both meshes' bounds taken against the test.
	Free Verdict = iota
	// Blocked means the other mesh stands ahead of the mover over a region
	// wider than the sum of the two bounds.
	Blocked
	// Undecided means the other mesh may stand ahead only inside the bounds'
	// uncertainty. A layout reads it as free along the part's own axis.
	Undecided
)

func (v Verdict) String() string {
	switch v {
	case Free:
		return "free"
	case Blocked:
		return "blocked"
	}
	return "undecided"
}

// heightEpsilon is the height difference in millimetres at or below which two
// faces count as touching and not as one standing ahead of the other.
const heightEpsilon = 1e-6

// Result is the verdict for one mover and one other mesh. Witness is valid only
// when Verdict is Blocked.
type Result struct {
	Verdict Verdict
	Witness Witness
}

// Witness is a point on the mover's surface under the blocker and how far the
// blocker's surface stands ahead of it along the direction.
type Witness struct {
	Point  r3.Vec
	Height units.Value
}

// Block tests whether other stands in the way of mover along their shared
// direction (mover moves toward +dir). Both instances must come from
// projections of the same direction. Block only reads its arguments and is safe
// to call from many goroutines.
func Block(mover, other Instance) Result {
	var w worker
	return w.block(mover, other)
}

// Work counts what pairwise tests did. It depends only on the instances
// tested, never on the number of goroutines.
type Work struct {
	// Pairs is the number of ordered pairs that passed the box and position
	// prefilter.
	Pairs int64
	// Cells is the number of grid cells of the other mesh read.
	Cells int64
	// Candidates is the number of triangle pairs read from those cells, each
	// pair once.
	Candidates int64
	// Clipped is the number of triangle pairs whose shadows were clipped
	// against each other.
	Clipped int64
}

func (w *Work) add(o Work) {
	w.Pairs += o.Pairs
	w.Cells += o.Cells
	w.Candidates += o.Candidates
	w.Clipped += o.Clipped
}

// worker is the scratch space of one goroutine testing pairs: the clipping
// buffers, the marks that read each triangle of the other mesh once per
// triangle of the mover, and the work done so far.
type worker struct {
	clip  clipper
	seen  []uint32
	stamp uint32
	work  Work
}

// mark returns a fresh stamp for the n triangles of the other mesh.
func (w *worker) mark(n int) uint32 {
	if len(w.seen) < n {
		w.seen = make([]uint32, n)
		w.stamp = 0
	}
	w.stamp++
	if w.stamp == 0 {
		clear(w.seen)
		w.stamp = 1
	}
	return w.stamp
}

func (w *worker) block(mover, other Instance) Result {
	s := mover.P.bound + other.P.bound
	if !prefilter(mover, other, s) {
		return Result{Verdict: Free}
	}
	w.work.Pairs++
	ua, va, wa := mover.local()
	ub, vb, wb := other.local()
	du, dv, dw := ub-ua, vb-va, wb-wa
	b := other.P
	top := b.topZ + dw // no triangle of other reaches higher
	weak := false
	for i := range mover.P.tris {
		ta := &mover.P.tris[i]
		if top-ta.minZ <= heightEpsilon {
			continue
		}
		x0, x1, y0, y1, ok := b.grid.cells(ta.minX-s-du, ta.maxX+s-du, ta.minY-s-dv, ta.maxY+s-dv)
		if !ok {
			continue
		}
		stamp := w.mark(len(b.tris))
		var planeA affine
		havePlane := false
		for x := x0; x <= x1; x++ {
			for y := y0; y <= y1; y++ {
				w.work.Cells++
				for _, j := range b.grid.at(x, y) {
					if w.seen[j] == stamp {
						continue
					}
					w.seen[j] = stamp
					w.work.Candidates++
					tb := b.tris[j].shifted(du, dv, dw)
					if tb.maxZ-ta.minZ <= heightEpsilon {
						continue
					}
					if tb.maxX < ta.minX-s || tb.minX > ta.maxX+s || tb.maxY < ta.minY-s || tb.minY > ta.maxY+s {
						continue
					}
					if !havePlane {
						planeA, havePlane = ta.plane(), true
					}
					w.work.Clipped++
					if wit, ok := w.strongWitness(ta, &tb, planeA, s); ok {
						wit.Point = world(mover, wit.Point)
						return Result{Verdict: Blocked, Witness: wit}
					}
					if !weak && within(ta, &tb, s) {
						weak = true
					}
				}
			}
		}
	}
	if weak {
		return Result{Verdict: Undecided}
	}
	return Result{Verdict: Free}
}

// prefilter is false when the shadows are further apart than s, or when the
// other mesh lies wholly behind the mover along the direction.
func prefilter(a, b Instance, s float64) bool {
	ua, va, wa := a.local()
	ub, vb, wb := b.local()
	if b.P.minU+ub > a.P.maxU+ua+s || a.P.minU+ua > b.P.maxU+ub+s {
		return false
	}
	if b.P.minV+vb > a.P.maxV+va+s || a.P.minV+va > b.P.maxV+vb+s {
		return false
	}
	return b.P.maxW+wb-(a.P.minW+wa) > heightEpsilon
}

// strongWitness clips the two shadows against each other; where the other
// triangle stands ahead of the mover by more than heightEpsilon, the region
// must survive an erosion by half the bounds' sum, which is exactly a width
// above the sum.
func (w *worker) strongWitness(ta, tb *tri, pa affine, s float64) (Witness, bool) {
	r := w.clip.clipTriangles(ta, tb)
	if r == nil {
		return Witness{}, false
	}
	pb := tb.plane()
	diff := func(q p2) float64 { return pb.at(q) - pa.at(q) - heightEpsilon }
	r = w.clip.clipHalf(r, diff)
	if r == nil {
		return Witness{}, false
	}
	r = w.clip.erode(r, s/2)
	if r == nil {
		return Witness{}, false
	}
	c := centroid(r)
	h := pb.at(c) - pa.at(c)
	return Witness{Point: r3.NewVec(c.x, c.y, pa.at(c)), Height: units.Millimeters(h)}, true
}

// world maps a point in the mover's projection coordinates (u, v, w) to world
// millimetres, including the instance offset.
func world(mover Instance, local r3.Vec) r3.Vec {
	p := mover.P
	return p.u.Scale(local.X).Add(p.v.Scale(local.Y)).Add(p.dir.Scale(local.Z)).Add(mover.Offset)
}
