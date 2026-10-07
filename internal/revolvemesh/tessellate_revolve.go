package revolvemesh

import (
	"github.com/lestrrat-3d/decad/internal/proofbound"
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
