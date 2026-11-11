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
	s := mover.P.bound + other.P.bound
	if !prefilter(mover, other, s) {
		return Result{Verdict: Free}
	}
	ua, va, wa := mover.local()
	ub, vb, wb := other.local()
	du, dv, dw := ub-ua, vb-va, wb-wa
	b := other.P
	seen := make([]int32, len(b.tris))
	weak := false
	for i := range mover.P.tris {
		ta := &mover.P.tris[i]
		stamp := int32(i + 1)
		x0, x1, y0, y1 := b.cellRange(ta.minX-s-du, ta.maxX+s-du, ta.minY-s-dv, ta.maxY+s-dv)
		var planeA affine
		havePlane := false
		for x := x0; x <= x1; x++ {
			for y := y0; y <= y1; y++ {
				for _, j := range b.grid[[2]int32{x, y}] {
					if seen[j] == stamp {
						continue
					}
					seen[j] = stamp
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
					if w, ok := strongWitness(ta, &tb, planeA, s); ok {
						w.Point = world(mover, w.Point)
						return Result{Verdict: Blocked, Witness: w}
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
func strongWitness(ta, tb *tri, pa affine, s float64) (Witness, bool) {
	r := clipTriangles(ta, tb)
	if r == nil {
		return Witness{}, false
	}
	pb := tb.plane()
	diff := func(q p2) float64 { return pb.at(q) - pa.at(q) - heightEpsilon }
	r = clipHalf(r, diff)
	if r == nil {
		return Witness{}, false
	}
	r = erode(r, s/2)
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
