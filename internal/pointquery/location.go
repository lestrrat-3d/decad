package pointquery

import (
	"context"
	"errors"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// Location is the proof result over a verified mesh and its true body.
type Location uint8

const (
	Undecided Location = iota
	Outside
	Inside
	OnBoundary
)

// Locate transfers exact held-mesh parity to the denoted body when its
// boundary and occupied-volume bounds exclude a different answer.
func Locate(ctx context.Context, p r3.Vec, verts []r3.Vec, tris [][3]int,
	boundaryBound, symDiffBound float64) (Location, error) {
	if len(tris) == 0 || math.IsNaN(boundaryBound) || math.IsInf(boundaryBound, 0) ||
		math.IsNaN(symDiffBound) || math.IsInf(symDiffBound, 0) ||
		boundaryBound < 0 || symDiffBound < 0 {
		return Undecided, nil
	}
	subset := make([]int, len(tris))
	for i := range subset {
		subset[i] = i
	}
	inside, onBoundary, err := meshbool.MeshParityContext(ctx, proof.XptOf(p), verts, tris, subset)
	if err != nil && !errors.Is(err, decaderr.ErrBooleanFailed) {
		return Undecided, err
	}
	parityErr := err
	if onBoundary {
		if boundaryBound == 0 {
			return OnBoundary, nil
		}
		return Undecided, nil
	}
	distanceSquared, err := MeshDistanceSquared(ctx, p, verts, tris)
	if err != nil {
		return Undecided, err
	}
	if distanceSquared.Sign() == 0 {
		if boundaryBound == 0 {
			return OnBoundary, nil
		}
		return Undecided, nil
	}
	if errors.Is(parityErr, decaderr.ErrBooleanFailed) {
		var resolved bool
		inside, resolved, err = ObliqueParity(ctx, p, verts, tris)
		if err != nil {
			return Undecided, err
		}
		if !resolved {
			return Undecided, nil
		}
	}
	if boundaryBound == 0 && symDiffBound == 0 {
		if inside {
			return Inside, nil
		}
		return Outside, nil
	}
	// Both roundings point downward: SetRat does not exceed the exact squared
	// distance, and Sqrt does not exceed its exact root.
	distance := new(big.Float).SetPrec(256).SetMode(big.ToNegativeInf)
	distance.SetRat(distanceSquared)
	distance.Sqrt(distance)
	lower, _ := distance.Rat(nil)
	radius := new(big.Rat).Sub(lower, proof.FloatRat(boundaryBound))
	if radius.Sign() <= 0 {
		return Undecided, nil
	}
	// A differing answer would make the whole boundary-free ball disagree.
	// Its volume is 4*pi*r^3/3, strictly greater than 4*r^3 since pi > 3.
	ballLowerBound := new(big.Rat).Mul(radius, radius)
	ballLowerBound.Mul(ballLowerBound, radius)
	ballLowerBound.Mul(ballLowerBound, big.NewRat(4, 1))
	if ballLowerBound.Cmp(proof.FloatRat(symDiffBound)) <= 0 {
		return Undecided, nil
	}
	if inside {
		return Inside, nil
	}
	return Outside, nil
}
