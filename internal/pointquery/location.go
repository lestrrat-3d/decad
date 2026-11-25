package pointquery

import (
	"context"
	"errors"
	"math"
	"math/big"
	"sync"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// PreparedMesh keeps the exact parity projections and index for repeated
// queries against one immutable held mesh.
type PreparedMesh struct {
	verts []r3.Vec
	tris  [][3]int
	mu    sync.Mutex
	// parity is guarded by mu.
	parity *meshbool.ParityMesh
	// distanceOnce builds distance on the first query that needs a distance.
	// The index is read-only after that, so queries read it without mu.
	distanceOnce sync.Once
	distance     *distanceIndex
}

// NewPreparedMesh prepares reusable query data for one immutable held mesh.
func NewPreparedMesh(verts []r3.Vec, tris [][3]int) *PreparedMesh {
	return &PreparedMesh{verts: verts, tris: tris, parity: meshbool.NewParityMesh(verts, tris)}
}

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
	return locate(ctx, p, verts, tris, boundaryBound, symDiffBound, nil, nil, false)
}

// LocatePrepared reuses the parity projections and index in prepared across
// repeated queries. Prepared mesh data is immutable, while its lazy parity
// caches are guarded so concurrent queries remain safe.
func LocatePrepared(ctx context.Context, p r3.Vec, prepared *PreparedMesh,
	boundaryBound, symDiffBound float64) (Location, error) {
	if prepared == nil {
		return Undecided, nil
	}
	return locate(ctx, p, prepared.verts, prepared.tris, boundaryBound, symDiffBound, nil, prepared, false)
}

// LocateWithDistance reuses an exact held-mesh distance already computed for
// the same point and triangles. Where the ball proof cannot decide a point the
// mesh puts outside, it also tries a free path (escape.go), which a distance
// query needs near a bore of a large body. The caller must pass MeshDistanceSquared's
// result, or nil to compute it here.
func LocateWithDistance(ctx context.Context, p r3.Vec, verts []r3.Vec, tris [][3]int,
	boundaryBound, symDiffBound float64, distanceSquared *big.Rat) (Location, error) {
	return locate(ctx, p, verts, tris, boundaryBound, symDiffBound, distanceSquared, nil, true)
}

// cleanDistanceIndex returns the distance index, built on first use, or nil
// when the mesh failed its check and distance queries keep MeshDistanceSquared.
func (m *PreparedMesh) cleanDistanceIndex() *distanceIndex {
	m.distanceOnce.Do(func() { m.distance = buildDistanceIndex(m.verts, m.tris) })
	if !m.distance.clean {
		return nil
	}
	return m.distance
}

func locate(ctx context.Context, p r3.Vec, verts []r3.Vec, tris [][3]int,
	boundaryBound, symDiffBound float64, distanceSquared *big.Rat, prepared *PreparedMesh, escape bool) (Location, error) {
	if len(tris) == 0 || math.IsNaN(boundaryBound) || math.IsInf(boundaryBound, 0) ||
		math.IsNaN(symDiffBound) || math.IsInf(symDiffBound, 0) ||
		boundaryBound < 0 || symDiffBound < 0 {
		return Undecided, nil
	}
	var inside, onBoundary bool
	var err error
	if prepared == nil {
		subset := make([]int, len(tris))
		for i := range subset {
			subset[i] = i
		}
		inside, onBoundary, err = meshbool.MeshParityContext(ctx, proof.XptOf(p), verts, tris, subset)
	} else {
		// MeshParityAllContext returns what MeshParityPreparedContext returns
		// over every facet, error text included, and may use a parity index.
		prepared.mu.Lock()
		inside, onBoundary, err = meshbool.MeshParityAllContext(ctx, proof.XptOf(p), prepared.parity)
		prepared.mu.Unlock()
	}
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
	if distanceSquared == nil && prepared != nil {
		if index := prepared.cleanDistanceIndex(); index != nil {
			distanceSquared, err = index.distanceSquared(ctx, p, tris)
			if err != nil {
				return Undecided, err
			}
		}
	}
	if distanceSquared == nil {
		distanceSquared, err = MeshDistanceSquared(ctx, p, verts, tris)
		if err != nil {
			return Undecided, err
		}
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
		// The ball is too small for the body's whole symmetric-difference bound. A point the held mesh puts outside
		// may still reach the outside of both solids by a free path (escape.go), which reads no volume bound.
		if inside || !escape {
			return Undecided, nil
		}
		free, err := escapeOutside(ctx, p, verts, tris, boundaryBound, distanceSquared)
		if err != nil || !free {
			return Undecided, err
		}
		return Outside, nil
	}
	if inside {
		return Inside, nil
	}
	return Outside, nil
}
