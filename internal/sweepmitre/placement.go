package sweepmitre

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweeparc"
	"github.com/lestrrat-3d/r3"
)

// PlacedVertices keeps the exact placed vertices and each vertex's rounding bound.
type PlacedVertices struct {
	Exact       []sweeparc.RatVec
	LocalExact  []sweeparc.RatVec
	Anchor      sweeparc.RatVec
	Rounded     []r3.Vec
	VertexBound []float64
	Delta       float64
}

// PlaceAndRound applies the accumulated placement exactly and rounds each vertex once.
// needLocal retains the unplaced vertices for a first volume and moment reading.
func PlaceAndRound(c Construction, xform r3.Transform, needLocal bool) (PlacedVertices, error) {
	count := len(c.Sections[0]) * len(c.Sections)
	out := PlacedVertices{Exact: make([]sweeparc.RatVec, 0, count), Anchor: Place(xform, c.Anchor)}
	if needLocal {
		out.LocalExact = make([]sweeparc.RatVec, 0, count)
	}
	for _, section := range c.Sections {
		for _, p := range section {
			if needLocal {
				out.LocalExact = append(out.LocalExact, p)
			}
			out.Exact = append(out.Exact, Place(xform, p))
		}
	}
	out.Rounded = make([]r3.Vec, len(out.Exact))
	out.VertexBound = make([]float64, len(out.Exact))
	for v, p := range out.Exact {
		var coords [3]float64
		worst := new(big.Rat)
		for axis := range 3 {
			f, gap, ok := Round(p[axis])
			if !ok {
				return PlacedVertices{}, fmt.Errorf(`%w: a mitred sweep vertex runs past the representable float64 range`,
					decaderr.ErrUnsupported)
			}
			coords[axis] = f
			if gap.Cmp(worst) > 0 {
				worst = gap
			}
		}
		out.Rounded[v] = r3.NewVec(coords[0], coords[1], coords[2])
		if worst.Sign() != 0 {
			w, _ := worst.Float64()
			out.VertexBound[v] = proofbound.Radius3D(proofbound.ProvenUpRound(w))
			out.Delta = max(out.Delta, out.VertexBound[v])
		}
	}
	return out, nil
}
