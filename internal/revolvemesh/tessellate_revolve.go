package revolvemesh

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

// RevMeridian is one meridian sample: either a junction between two
// consecutive walks of one recorded loop or an interior chord station of a
// circular walk, in the axis coordinates the payload's own axisFrame
// re-expressed it into, beside the certified enclosure of the (z, ρ) pair the
// RECORD denotes there and the mesh vertices it owns.
//
// A sample on the axis owns exactly ONE vertex, interned by construction: it is
// the same junction for every angular index and for both partial caps, which is
// how an on-axis line's single geometric edge ends up shared by the two caps
// (docs/tessellation-design.md §9).
type RevMeridian struct {
	Z, Rho     float64
	ZIv, RhoIv proofbound.RatInterval
	OnAxis     bool
	Ring       []int
	// walk is the index, into its loop's resolved walks, of the walk whose
	// OUTGOING chord starts at this sample; sag is that chord's proven meridian
	// sagitta, zero for a straight walk, which chords nothing. arc is the
	// circular chord's own meridian model, nil for a straight one.
	Walk int
	Sag  float64
	Arc  *RevArcCell
	// Freeform bounds the bowed generator cell that starts at this sample.
	Freeform *RevFreeformCell
}

type RevFreeformCell struct {
	ArcUpper float64
	RhoUpper float64
}

// at is the mesh vertex this sample contributes at angular index l. A pole has
// one vertex and answers it for every angle; an off-axis ring of a full turn
// wraps, so index n is index 0 and the seam needs no duplicate.
func (s RevMeridian) At(l int) int {
	if s.OnAxis {
		return s.Ring[0]
	}
	return s.Ring[l%len(s.Ring)]
}

// CoordinateAreaAllow charges the coordinate displacement of each facet at
// its three corners. The combined construction and placement displacement
// covers the ideal, stored unplaced, and placed triangles.
func CoordinateAreaAllow(verts []r3.Vec, tris [][3]int, delta float64) float64 {
	if delta <= 0 {
		return 0
	}
	total := 0.0
	for _, tri := range tris {
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		total = proofbound.AbsSumUpper(total, proofbound.PerturbedTriangleAreaAllow(a, b, c, delta))
	}
	return total
}

// EmitCellTriangles writes one meridian cell's facets across the complete
// angular sequence. With material to the walk's left in (z, rho) and a
// right-handed increasing angle, dX/dt cross dX/dphi points outward, so the
// fixed (meridian, angle) diagonal gives the outward winding. The axis side
// is already in the caller's axis frame; a reflected placement reverses the
// assembled mesh afterward. A ring on the axis emits one fan triangle per
// angular interval.
func EmitCellTriangles(lo, hi RevMeridian, nPhi int, emit func([3]int)) {
	for l := range nPhi {
		a, d := lo.At(l), lo.At(l+1)
		b, c := hi.At(l), hi.At(l+1)
		switch {
		case lo.OnAxis:
			emit([3]int{a, b, c})
		case hi.OnAxis:
			emit([3]int{a, b, d})
		default:
			emit([3]int{a, b, c})
			emit([3]int{a, c, d})
		}
	}
}

// EmitCapTriangles triangulates the meridian region once in the (z, rho)
// plane and maps each triangle onto the two already allocated end rings.
// The end winding follows that plane's normal; the start winding reverses it.
// Pole samples reuse their single interned vertex at both ends.
func EmitCapTriangles(ctx context.Context, samples [][]RevMeridian, pts []sectionrecord.Point2,
	loopIdx [][]int, last int, emit func(end, start [3]int)) error {
	var startV, endV []int
	for _, loop := range samples {
		for _, s := range loop {
			startV = append(startV, s.At(0))
			endV = append(endV, s.At(last))
		}
	}
	tris, err := triangulation.Triangulate(ctx, pts, loopIdx)
	if err != nil {
		return err
	}
	for _, tri := range tris {
		emit([3]int{endV[tri[0]], endV[tri[1]], endV[tri[2]]},
			[3]int{startV[tri[0]], startV[tri[2]], startV[tri[1]]})
	}
	return nil
}
