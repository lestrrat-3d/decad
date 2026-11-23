package facetproof

import (
	"cmp"
	"context"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// orderedRounding rounds each exact point to a float, keeping on every axis
// the order of the exact coordinates: two points with equal exact coordinates
// on an axis get equal floats on that axis, and two with different ones get
// different floats in the same order. Nearest rounding already keeps that
// order except where it merges different exact coordinates into one float.
// There each later coordinate steps up to the float after the one before it,
// so a coordinate leaves its nearest float only inside such a run. It returns
// the rounded points and whether any differs from nearest rounding. Every
// point must have a finite nearest float. ctx is polled between axes.
func orderedRounding(ctx context.Context, exact []proof.Xpt) ([]r3.Vec, bool, error) {
	out := make([]r3.Vec, len(exact))
	for i, p := range exact {
		out[i] = p.Vec()
	}
	order := make([]int, len(exact))
	rounded := make([]float64, len(exact))
	changed := false
	for axis := range 3 {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		for i := range order {
			order[i] = i
		}
		// Each pass writes only its own axis, so out still holds the nearest
		// float on this one. Nearest rounding never reverses two exact values,
		// so the float order refined by the exact order on ties is the exact
		// order.
		slices.SortStableFunc(order, func(a, b int) int {
			if c := cmp.Compare(vecAxis(out[a], axis), vecAxis(out[b], axis)); c != 0 {
				return c
			}
			return compareXptAxis(exact[a], exact[b], axis)
		})
		for k, i := range order {
			v := vecAxis(out[i], axis)
			if k > 0 {
				switch prev := order[k-1]; {
				case compareXptAxis(exact[prev], exact[i], axis) == 0:
					v = rounded[k-1]
				case v <= rounded[k-1]:
					v = math.Nextafter(rounded[k-1], math.Inf(1))
				}
			}
			rounded[k] = v
		}
		for k, i := range order {
			if rounded[k] != vecAxis(out[i], axis) {
				changed = true
				out[i] = withVecAxis(out[i], axis, rounded[k])
			}
		}
	}
	return out, changed, nil
}

// placeOrdered is the last stage of a placement's embedding repair, run when
// the nearest rounding stays folded after the float-box and gap-tolerance
// searches (foldErr). A vertex chain an earlier repair spread over
// neighbouring floats merges again where a motion takes it to coordinates with
// a coarser ulp, and no float box can hold the chain apart. This stage starts
// every held vertex at its orderedRounding image, which keeps the chain apart
// in its old order, and runs the same exact embedding check again.
//
// It runs only under a positive gap tolerance, and only when every vertex the
// ordered rounding moves stays within that reach of its exact image: it moves
// vertices farther than the float box does. A moved vertex denotes no exact
// point any more, and its vertexAllow entry rises to the measured distance from
// its exact image, so the caller's bounds charge the move. The returned summary
// reports the largest such distance of a final position, so the placement
// carries the same warning a reach move gives. held, denotes and vertexAllow
// are rewritten in place. It returns foldErr unchanged when the stage does not
// apply.
func placeOrdered(ctx context.Context, exact []proof.Xpt, held []r3.Vec, tris [][3]int, axes [3]r3.Vec,
	denotes []bool, vertexAllow []float64, foldErr error) ([]float64, *meshbool.EmbeddingReach, error) {
	reach := gaptol.Reach(ctx)
	if !(reach > 0) {
		return nil, nil, foldErr
	}
	ordered, changed, err := orderedRounding(ctx, exact)
	if err != nil {
		return nil, nil, err
	}
	if !changed {
		return nil, nil, foldErr
	}
	shift := make([]float64, len(ordered))
	for i, p := range ordered {
		if p == exact[i].Vec() {
			continue
		}
		d, _ := meshbool.CoordDistance(exact[i], p).Float64()
		shift[i] = proofbound.Radius3D(proofbound.ProvenUpRound(d))
		if !(shift[i] <= reach) {
			return nil, nil, foldErr
		}
	}
	for i, p := range ordered {
		held[i] = p
		if shift[i] == 0 {
			continue
		}
		denotes[i] = false
		vertexAllow[i] = math.Max(vertexAllow[i], shift[i])
	}
	moved, summary, err := keepPlacedEmbeddedWithAxes(ctx, exact, held, tris, axes, denotes)
	if err != nil {
		return nil, nil, err
	}
	bound := 0.0
	if summary != nil {
		bound = summary.DisplacementBound
	}
	for i, s := range shift {
		if s == 0 {
			continue
		}
		if moved != nil && moved[i] > 0 {
			s = moved[i]
		}
		bound = math.Max(bound, s)
	}
	return moved, &meshbool.EmbeddingReach{DisplacementBound: bound, AllowedReach: reach}, nil
}

// compareXptAxis compares the exact coordinates of p and q on axis.
func compareXptAxis(p, q proof.Xpt, axis int) int {
	// p/pw against q/qw: compare pn*qw with qn*pw, flipped when pw*qw < 0.
	l := new(big.Int).Mul(xptAxis(p, axis), q.W)
	r := new(big.Int).Mul(xptAxis(q, axis), p.W)
	c := l.Cmp(r)
	if p.W.Sign()*q.W.Sign() < 0 {
		c = -c
	}
	return c
}

func xptAxis(p proof.Xpt, axis int) *big.Int {
	switch axis {
	case 0:
		return p.X
	case 1:
		return p.Y
	default:
		return p.Z
	}
}

func vecAxis(v r3.Vec, axis int) float64 {
	switch axis {
	case 0:
		return v.X
	case 1:
		return v.Y
	default:
		return v.Z
	}
}

func withVecAxis(v r3.Vec, axis int, x float64) r3.Vec {
	switch axis {
	case 0:
		v.X = x
	case 1:
		v.Y = x
	default:
		v.Z = x
	}
	return v
}
