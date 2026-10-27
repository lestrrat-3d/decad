package meshbool

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// planeSideReference keeps the original exact construction as the sign oracle.
func planeSideReference(a, b, p, n proof.Xpt) int {
	return proof.XdotSign(proof.Xcross(proof.Xsub(b, a), proof.Xsub(p, a)), n)
}

func preparedEdgeSide(a, b, p, n proof.Xpt) int {
	return proof.XdotDiffSign(proof.Xcross(n, proof.Xsub(b, a)), p, a)
}

func clonePlaneSidePoint(p proof.Xpt) proof.Xpt {
	return proof.Xpt{X: new(big.Int).Set(p.X), Y: new(big.Int).Set(p.Y),
		Z: new(big.Int).Set(p.Z), W: new(big.Int).Set(p.W)}
}

func planeSidePoint(x, y, z, w int64) proof.Xpt {
	return proof.Xpt{X: big.NewInt(x), Y: big.NewInt(y), Z: big.NewInt(z), W: big.NewInt(w)}
}

func TestPreparedEdgeSideMatchesConstructedExactSign(t *testing.T) {
	t.Parallel()
	tiny := new(big.Int).Lsh(big.NewInt(1), 900)
	cases := []struct {
		name       string
		a, b, p, n proof.Xpt
	}{
		{"unequal weights", planeSidePoint(1, 2, 3, 3), planeSidePoint(4, 5, 6, 7),
			planeSidePoint(9, 10, 12, 11), planeSidePoint(1, 2, 3, 5)},
		{"edge point", planeSidePoint(0, 0, 0, 1), planeSidePoint(7, 0, 0, 3),
			planeSidePoint(7, 0, 0, 6), planeSidePoint(0, 0, 1, 1)},
		{"near zero positive", planeSidePoint(0, 0, 0, 1), planeSidePoint(1, 0, 0, 1),
			proof.Xpt{X: big.NewInt(0), Y: big.NewInt(1), Z: big.NewInt(1), W: tiny},
			planeSidePoint(0, 0, 1, 1)},
		{"near zero negative", planeSidePoint(0, 0, 0, 1), planeSidePoint(1, 0, 0, 1),
			proof.Xpt{X: big.NewInt(0), Y: big.NewInt(-1), Z: big.NewInt(1), W: tiny},
			planeSidePoint(0, 0, 1, 1)},
		{"collapsed edge", planeSidePoint(1, 2, 3, 3), planeSidePoint(2, 4, 6, 6),
			planeSidePoint(4, 0, 0, 7), planeSidePoint(0, 0, 1, 1)},
		{"zero normal", planeSidePoint(0, 0, 0, 1), planeSidePoint(1, 0, 0, 1),
			planeSidePoint(0, 1, 0, 1), planeSidePoint(0, 0, 0, 17)},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			for _, scale := range []int64{1, 2, 3, 19} {
				a := scaleContactPoint(tc.a, int(scale))
				b := scaleContactPoint(tc.b, int(scale+1))
				p := scaleContactPoint(tc.p, int(scale+2))
				n := scaleContactPoint(tc.n, int(scale+3))
				before := [4]proof.Xpt{clonePlaneSidePoint(a), clonePlaneSidePoint(b),
					clonePlaneSidePoint(p), clonePlaneSidePoint(n)}
				want := planeSideReference(a, b, p, n)
				require.Equal(t, want, preparedEdgeSide(a, b, p, n))
				require.Equal(t, -want, preparedEdgeSide(b, a, p, n))
				require.Equal(t, before, [4]proof.Xpt{a, b, p, n})
			}
		})
	}
	rng := rand.New(rand.NewPCG(17, 29))
	for range 400 {
		point := func() proof.Xpt {
			coord := func() int64 { return int64(rng.IntN(2001) - 1000) }
			return planeSidePoint(coord(), coord(), coord(), int64(1+rng.IntN(99)))
		}
		a, b, p, n := point(), point(), point(), point()
		for _, shift := range []int{0, 4, 90} {
			aa := scaleContactPoint(a, shift)
			bb := scaleContactPoint(b, shift+1)
			pp := scaleContactPoint(p, shift+3)
			nn := scaleContactPoint(n, shift+2)
			require.Equal(t, planeSideReference(aa, bb, pp, nn), preparedEdgeSide(aa, bb, pp, nn))
		}
	}
}

func TestPreparedSegmentBoundaryEdgesMatchesReference(t *testing.T) {
	t.Parallel()
	cases := map[string][3]r3.Vec{
		"interior": {{X: 0.5, Y: -0.5, Z: -1}, {X: 0.5, Y: 1.5, Z: 1}, {X: 0.5, Y: 1.5, Z: -1}},
		"edge":     {{Y: 0, Z: -1}, {X: 2, Y: 0, Z: -1}, {X: 1, Y: 0, Z: 1}},
	}
	for name, tri := range cases {
		t.Run(name, func(t *testing.T) {
			a, b, xa, xb, na, nb := contactMembershipFixture(tri)
			contact, err := TriTriClassify(a, b, xa, xb, na, nb)
			require.NoError(t, err)
			require.Equal(t, ContactSegment, contact.Kind)
			for _, part := range []struct {
				xt [3]proof.Xpt
				n  proof.Xpt
			}{{xa, na}, {xb, nb}} {
				for _, reverse := range []bool{false, true} {
					xt := part.xt
					if reverse {
						xt[0], xt[2] = xt[2], xt[0]
					}
					for _, scale := range []int{0, 80} {
						for i := range xt {
							xt[i] = scaleContactPoint(xt[i], scale+i*3)
						}
						p0 := scaleContactPoint(contact.P0, scale+7)
						p1 := scaleContactPoint(contact.P1, scale+11)
						original := [5]proof.Xpt{clonePlaneSidePoint(xt[0]), clonePlaneSidePoint(xt[1]),
							clonePlaneSidePoint(xt[2]), clonePlaneSidePoint(p0), clonePlaneSidePoint(p1)}
						want0, want1 := OnTriBoundary(p0, xt, part.n), OnTriBoundary(p1, xt, part.n)
						wantEdge := SegAlongEdge(p0, p1, xt, part.n)
						got0, got1, gotEdge := segmentBoundaryEdges(p0, p1, xt, part.n)
						require.Equal(t, [3]any{want0, want1, wantEdge}, [3]any{got0, got1, gotEdge})
						require.Equal(t, original, [5]proof.Xpt{xt[0], xt[1], xt[2], p0, p1})
					}
				}
			}
		})
	}
	// The zero normal and collapsed first edge still use the old closed-range
	// rule after the exact side test returns zero.
	xt := [3]proof.Xpt{planeSidePoint(0, 0, 0, 1), planeSidePoint(0, 0, 0, 3),
		planeSidePoint(2, 0, 0, 1)}
	p0, p1, n := planeSidePoint(0, 0, 0, 5), planeSidePoint(1, 0, 0, 7), planeSidePoint(0, 0, 0, 1)
	want0, want1 := OnTriBoundary(p0, xt, n), OnTriBoundary(p1, xt, n)
	wantEdge := SegAlongEdge(p0, p1, xt, n)
	got0, got1, gotEdge := segmentBoundaryEdges(p0, p1, xt, n)
	require.Equal(t, [3]any{want0, want1, wantEdge}, [3]any{got0, got1, gotEdge})
}

var segmentBoundaryBenchmarkSum int

func BenchmarkSegmentBoundaryEdges(b *testing.B) {
	cases := map[string][3]r3.Vec{
		"interior": {{X: 0.5, Y: -0.5, Z: -1}, {X: 0.5, Y: 1.5, Z: 1}, {X: 0.5, Y: 1.5, Z: -1}},
		"edge":     {{Y: 0, Z: -1}, {X: 2, Y: 0, Z: -1}, {X: 1, Y: 0, Z: 1}},
	}
	for name, tri := range cases {
		b.Run(name, func(b *testing.B) {
			a, c, xa, xc, na, nc := contactMembershipFixture(tri)
			contact, err := TriTriClassify(a, c, xa, xc, na, nc)
			if err != nil || contact.Kind != ContactSegment {
				b.Fatalf("contact fixture = %v, %v", contact.Kind, err)
			}
			sum := 0
			b.ReportAllocs()
			for b.Loop() {
				onA0, onA1, edgeA := segmentBoundaryEdges(contact.P0, contact.P1, xa, na)
				onB0, onB1, edgeB := segmentBoundaryEdges(contact.P0, contact.P1, xc, nc)
				if onA0 {
					sum++
				}
				if onA1 {
					sum++
				}
				if onB0 {
					sum++
				}
				if onB1 {
					sum++
				}
				sum += edgeA + edgeB
			}
			segmentBoundaryBenchmarkSum = sum
		})
	}
}

// These edge-plane predicates independently check the span-derived metadata.
// edgePlanes holds one facet's three in-plane edge tests. For edge k, from
// a = xt[k] to b = xt[(k+1)%3] of a facet with exact normal n, Normal[k] is
// n × (b−a) and Offset[k] is XdotNum(Normal[k], a). The side of a point p on
// the facet's plane is the sign of ((b−a)×(p−a))·n = Normal[k]·(p−a), which
// proof.XdotDiffSignOffset reads from these two values. Axis[k], Lo[k] and
// Hi[k] are the dominant axis of b−a and the edge's two ends in order along
// it, the values PointOnSegment3D(a, b, p) derives before it compares p. All
// of them are functions of the facet's exact corners and normal alone, so
// every pair the facet takes part in may share them.
type edgePlanes struct {
	Normal [3]proof.Xpt
	Offset [3]*big.Int
	Axis   [3]int
	Lo, Hi [3]proof.Xpt
}

func newEdgePlanes(xt [3]proof.Xpt, n proof.Xpt) edgePlanes {
	var out edgePlanes
	var left, right big.Int
	for k := range 3 {
		a, b := xt[k], xt[(k+1)%3]
		delta := proof.Xsub(b, a)
		out.Normal[k] = proof.Xcross(n, delta)
		out.Offset[k] = proof.XdotNum(out.Normal[k], a)
		// PointOnSegment3D's axis and end order, without its allocations.
		axis := 0
		if delta.Y.CmpAbs(delta.X) > 0 {
			axis = 1
		}
		if delta.Z.CmpAbs(Coord(delta, axis)) > 0 {
			axis = 2
		}
		lo, hi := a, b
		if compareCoordScratch(&left, &right, lo, hi, axis) > 0 {
			lo, hi = hi, lo
		}
		out.Axis[k], out.Lo[k], out.Hi[k] = axis, lo, hi
	}
	return out
}

// onSegment is PointOnSegment3D(a, b, p) for edge k's a and b, with scratch
// integers left and right.
func (e *edgePlanes) onSegment(left, right *big.Int, k int, p proof.Xpt) bool {
	return compareCoordScratch(left, right, e.Lo[k], p, e.Axis[k]) <= 0 &&
		compareCoordScratch(left, right, p, e.Hi[k], e.Axis[k]) <= 0
}

// compareCoordScratch is CompareCoord with its two products written to left
// and right.
func compareCoordScratch(left, right *big.Int, a, b proof.Xpt, axis int) int {
	left.Mul(Coord(a, axis), b.W)
	right.Mul(Coord(b, axis), a.W)
	return left.Cmp(right)
}

// segmentBoundaryEdges shares each exact point-on-edge result between the
// boundary flags and the first common edge, retaining their short-circuit order.
func segmentBoundaryEdges(p0, p1 proof.Xpt, xt [3]proof.Xpt, n proof.Xpt) (bool, bool, int) {
	planes := newEdgePlanes(xt, n)
	return segmentBoundaryEdgesPlanes(p0, p1, xt, &planes)
}

// segmentBoundaryEdgesPlanes is segmentBoundaryEdges with the facet's edge
// planes already built.
func segmentBoundaryEdgesPlanes(p0, p1 proof.Xpt, xt [3]proof.Xpt, planes *edgePlanes) (bool, bool, int) {
	points := [2]proof.Xpt{p0, p1}
	var known, on [2]uint8
	var sideScratch [3]big.Int
	onEdge := func(point, edge int) bool {
		bit := uint8(1 << edge)
		if known[point]&bit == 0 {
			known[point] |= bit
			// ((b-a)×(p-a))·n = (n×(b-a))·(p-a).
			if proof.XdotDiffSignOffset(&sideScratch, planes.Normal[edge], points[point], xt[edge], planes.Offset[edge]) == 0 &&
				planes.onSegment(&sideScratch[0], &sideScratch[1], edge, points[point]) {
				on[point] |= bit
			}
		}
		return on[point]&bit != 0
	}
	var on0, on1 bool
	for edge := range 3 {
		if onEdge(0, edge) {
			on0 = true
			break
		}
	}
	for edge := range 3 {
		if onEdge(1, edge) {
			on1 = true
			break
		}
	}
	for edge := range 3 {
		if onEdge(0, edge) && onEdge(1, edge) {
			return on0, on1, edge
		}
	}
	return on0, on1, -1
}
