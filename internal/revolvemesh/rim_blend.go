package revolvemesh

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// The cap-rim blend is sampled at RimBlendStations shared z stations along the
// blend and rimWallStations along the outer circle.
const (
	RimBlendStations = 128
	rimWallStations  = 512
)

// RimKind selects the cap-rim blend profile.
type RimKind uint8

// The cap-rim blend kinds.
const (
	RimFillet RimKind = iota
	RimChamfer
)

// RimSource is the quarter-cylinder cap-rim blend source: its x range, outer
// radius, setback and blend kind.
type RimSource struct {
	XLo, XHi, Radius, Setback float64
	Kind                      RimKind
}

// RimStation is one shared z station of the rim, with its rounded-down x and y
// and the exact upper bounds of both.
type RimStation struct {
	Z, X, Y float64
	XUpper  *big.Rat
	YUpper  *big.Rat
}

// RimStations returns the shared z stations of the rim blend and the outer
// circle, ordered by z.
func RimStations(source RimSource) ([]RimStation, error) {
	n := RimBlendStations + rimWallStations + 1
	stations := make([]RimStation, 0, n)
	for i := 0; i <= RimBlendStations; i++ {
		q := float64(i) / RimBlendStations
		z := source.Setback * q * q
		if i == RimBlendStations {
			z = source.Setback
		}
		station, ok := rimStationAt(source, z)
		if !ok {
			return nil, fmt.Errorf("%w: the cap rim has no certified blend station", decaderr.ErrUnsupported)
		}
		stations = append(stations, station)
	}
	for i := 1; i <= rimWallStations; i++ {
		q := float64(rimWallStations-i) / rimWallStations
		z := source.Radius - (source.Radius-source.Setback)*q*q
		if i == rimWallStations {
			z = source.Radius
		}
		station, ok := rimStationAt(source, z)
		if !ok {
			return nil, fmt.Errorf("%w: the outer circle has no certified station", decaderr.ErrUnsupported)
		}
		stations = append(stations, station)
	}
	for i := 1; i < len(stations); i++ {
		if stations[i].Z <= stations[i-1].Z || stations[i].X < stations[i-1].X ||
			stations[i].Y > stations[i-1].Y {
			return nil, fmt.Errorf("%w: rounded rim stations are not ordered", decaderr.ErrUnsupported)
		}
	}
	return stations, nil
}

// The float station z is read as an exact dyadic coordinate. Directed square
// roots place the fillet's x and both blends' y on or inside their analytic
// circles; the chamfer's x is rounded down from its exact affine value. This
// makes every interpolated station rectangle a subset of the true result:
// both upper-bound curves are concave in z.
func rimStationAt(source RimSource, z float64) (RimStation, bool) {
	qz := proofarith.FloatRat(z)
	if qz == nil || z < 0 || z > source.Radius {
		return RimStation{}, false
	}
	r := proofarith.FloatRat(source.Radius)
	qh := new(big.Rat).Sub(new(big.Rat).Mul(r, r), new(big.Rat).Mul(qz, qz))
	if qh.Sign() < 0 {
		return RimStation{}, false
	}
	y, yu := proofbound.RatSqrtDown(qh), proofbound.RatSqrtUp(qh)
	station := RimStation{Z: z, Y: y, YUpper: proofarith.FloatRat(yu)}
	if z >= source.Setback {
		station.X = source.XHi
		station.XUpper = proofarith.FloatRat(source.XHi)
		return station, true
	}
	qf := proofarith.FloatRat(source.Setback)
	if source.Kind == RimChamfer {
		x := new(big.Rat).Add(new(big.Rat).Sub(proofarith.FloatRat(source.XHi), qf), qz)
		station.X = proofbound.RatFloatDown(x)
		station.XUpper = x
		return station, !proofbound.IsNonFinite(station.X) && !proofbound.IsNonFinite(station.Y)
	}
	qroot := new(big.Rat).Sub(new(big.Rat).Mul(new(big.Rat).Mul(big.NewRat(2, 1), qf), qz),
		new(big.Rat).Mul(qz, qz))
	if qroot.Sign() < 0 {
		return RimStation{}, false
	}
	base := new(big.Rat).Sub(proofarith.FloatRat(source.XHi), qf)
	lo := new(big.Rat).Add(base, proofarith.FloatRat(proofbound.RatSqrtDown(qroot)))
	hi := new(big.Rat).Add(base, proofarith.FloatRat(proofbound.RatSqrtUp(qroot)))
	station.X = proofbound.RatFloatDown(lo)
	station.XUpper = hi
	return station, !proofbound.IsNonFinite(station.X) && !proofbound.IsNonFinite(station.Y)
}

// RimBounds returns the certified boundary bound and the missing-volume bound
// of the inscribed polyhedron over stations.
func RimBounds(source RimSource, stations []RimStation) (float64, float64, error) {
	r := proofarith.FloatRat(source.Radius)
	length := new(big.Rat).Sub(proofarith.FloatRat(source.XHi), proofarith.FloatRat(source.XLo))
	var maxGap, volumeGap *big.Rat
	maxGap, volumeGap = new(big.Rat), new(big.Rat)
	for i := range len(stations) - 1 {
		a, b := stations[i], stations[i+1]
		df := new(big.Rat).Sub(b.XUpper, proofarith.FloatRat(a.X))
		dh := new(big.Rat).Sub(a.YUpper, proofarith.FloatRat(b.Y))
		if df.Sign() < 0 || dh.Sign() < 0 {
			return 0, 0, fmt.Errorf("%w: cap rim intervals are not monotone", decaderr.ErrUnsupported)
		}
		gap := new(big.Rat).Add(df, dh)
		if gap.Cmp(maxGap) > 0 {
			maxGap = gap
		}
		dz := new(big.Rat).Sub(proofarith.FloatRat(b.Z), proofarith.FloatRat(a.Z))
		density := new(big.Rat).Add(new(big.Rat).Mul(r, df), new(big.Rat).Mul(length, dh))
		volumeGap.Add(volumeGap, new(big.Rat).Mul(dz, density))
	}
	bound, missing := proofbound.RatFloatUp(maxGap), proofbound.RatFloatUp(volumeGap)
	if proofbound.IsNonFinite(bound) || proofbound.IsNonFinite(missing) || bound >= 0.1 {
		return 0, 0, fmt.Errorf("%w: the rim's certified boundary bound exceeds 0.1 mm", decaderr.ErrUnsupported)
	}
	return bound, missing, nil
}
