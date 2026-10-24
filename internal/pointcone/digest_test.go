package pointcone

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"fmt"
	"hash"
	"math"
	"math/big"
	"os"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// pointConeDigestEnv names the file TestPointConeDigest writes its digests to.
// A change that must leave the +X point-cone arithmetic bit-identical runs this
// test on its base commit and on its head, on one machine, and compares the
// two files byte for byte (agent-docs/revolve-axis-design.md §8.2). Each
// commit runs its own copy of this file, which passes the canonical axis to
// every axis-taking function.
const pointConeDigestEnv = "DECAD_POINTCONE_DIGEST"

type digestCone struct {
	name        string
	apex, slope *big.Rat
}

// digestCones includes the non-positive apex and slope cones that
// facetedrecord.PointConeFactor sends to FactorRat. PointConeSource refuses a
// non-positive apex, so no public fixture reaches that branch.
func digestCones() []digestCone {
	return []digestCone{
		{"toe", big.NewRat(15, 2), big.NewRat(1, 2)},
		{"heel", big.NewRat(12, 1), big.NewRat(3, 4)},
		{"steep", big.NewRat(129600, 1000), big.NewRat(2, 1)},
		{"unit", big.NewRat(1, 5), big.NewRat(1, 1)},
		{"negative slope", big.NewRat(8, 1), big.NewRat(-1, 3)},
		{"zero apex", new(big.Rat), big.NewRat(1, 2)},
		{"negative apex", big.NewRat(-2, 1), big.NewRat(1, 2)},
		{"negative apex and slope", big.NewRat(-2, 1), big.NewRat(-1, 2)},
	}
}

// digestPoints is a fixed pseudo-random point set. Its coordinates come from a
// 64-bit LCG, so the set does not depend on the Go release.
func digestPoints(n int, xLo, xHi, yzHalf float64) []r3.Vec {
	state := uint64(0x9e3779b97f4a7c15)
	next := func() float64 {
		state = state*6364136223846793005 + 1442695040888963407
		return float64(state>>11) / float64(1<<53)
	}
	out := make([]r3.Vec, n)
	for i := range out {
		out[i] = r3.Vec{
			X: xLo + (xHi-xLo)*next(),
			Y: yzHalf * (2*next() - 1),
			Z: yzHalf * (2*next() - 1),
		}
	}
	return out
}

func digestWrite(h hash.Hash, values ...float64) {
	var buf [8]byte
	for _, v := range values {
		binary.LittleEndian.PutUint64(buf[:], math.Float64bits(v))
		h.Write(buf[:])
	}
}

func digestError(h hash.Hash, err error) {
	if err != nil {
		h.Write([]byte(err.Error()))
	}
	h.Write([]byte{0})
}

func digestSample(h hash.Hash, s Sample, err error) {
	digestWrite(h, s.Q, s.Alpha, s.QError, s.PointError)
	digestError(h, err)
}

func digestBool(h hash.Hash, v bool) {
	if v {
		h.Write([]byte{1})
		return
	}
	h.Write([]byte{0})
}

func pointConeDigest() string {
	var report strings.Builder
	points := digestPoints(64, -1, 12, 6)
	// A far cap clear of the axis and of the apex plane: every cell has a
	// positive radial minimum, so CapBound reaches the bound arithmetic.
	far := digestPoints(24, 3, 4, 3)
	for i := range far {
		far[i].Y = math.Copysign(1.2+math.Abs(far[i].Y), 1)
		far[i].Z = math.Copysign(1.2+math.Abs(far[i].Z), 1)
	}
	caps := make([][3]int, 0, len(far)-2)
	for i := 0; i+2 < len(far); i++ {
		caps = append(caps, [3]int{i, i + 1, i + 2})
	}
	fan := append([]r3.Vec{{}}, points[:16]...)
	for i := range fan[1:] {
		fan[i+1].X = math.Abs(fan[i+1].X) + 4
	}
	cones := digestCones()
	for _, cone := range cones {
		h := sha256.New()
		for _, p := range points {
			sample, err := FactorRat(p, coneaxis.X(), cone.apex, cone.slope)
			digestSample(h, sample, err)
			if cone.apex.Sign() > 0 && cone.slope.Sign() > 0 {
				sample, err = FactorDyadic(p, coneaxis.X(), xConsts(cone.apex, cone.slope))
				digestSample(h, sample, err)
			}
		}
		geom := CapGeometry(far, caps, coneaxis.X())
		if cone.apex.Sign() > 0 && cone.slope.Sign() > 0 {
			bound, lip, err := CapBound(geom, xConsts(cone.apex, cone.slope))
			digestWrite(h, bound, lip)
			digestError(h, err)
		}
		digestWrite(h, ReachUpper(far))
		for _, other := range cones {
			digestBool(h, Ordered(cone.apex, cone.slope, other.apex, other.slope, fan, 0, coneaxis.X()))
			digestBool(h, Ordered(cone.apex, cone.slope, other.apex, other.slope, fan, 1e-6, coneaxis.X()))
			digestBool(h, ToeHeelOrdered(cone.apex, cone.slope, other.apex, other.slope, fan, 0, coneaxis.X()))
			digestBool(h, ToeHeelOrdered(cone.apex, cone.slope, other.apex, other.slope, fan, 1e-6, coneaxis.X()))
		}
		for _, bound := range []float64{0, 1e-9, 0.5} {
			for _, farPlane := range []*big.Rat{big.NewRat(-2, 1), big.NewRat(3, 1), new(big.Rat)} {
				digestBool(h, AboveFarCap(fan[1:], bound, farPlane, coneaxis.X()))
			}
		}
		fmt.Fprintf(&report, "%s %s\n", cone.name, hex.EncodeToString(h.Sum(nil)))
	}
	return report.String()
}

// TestPointConeDigest hashes the held crossing samples of FactorDyadic and
// FactorRat, the cap bound, Ordered, ToeHeelOrdered and AboveFarCap over a
// fixed point set and cone list. Two runs in one process must hash equally.
func TestPointConeDigest(t *testing.T) {
	first := pointConeDigest()
	require.Equal(t, first, pointConeDigest())
	require.Contains(t, first, "negative apex ")
	if path := os.Getenv(pointConeDigestEnv); path != "" {
		require.NoError(t, os.WriteFile(path, []byte(first), 0o600))
	}
}
