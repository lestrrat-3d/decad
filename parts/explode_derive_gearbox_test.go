package parts_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

const (
	gbRing         = "ring"
	gbFrontCover   = "front cover"
	gbFrontBearing = "front bearing"
	gbInputShaft   = "input shaft"
	gbCarrier      = "carrier plate"
	gbRearBearing  = "rear bearing"
	gbRearCover    = "rear cover"
	gbOutputShaft  = "output shaft"
)

// gearboxSet is a planetary gearbox along Z. The ring (z 0..30) is the ground. The front cover and the
// front bearing sit below it, the rear cover and rear bearing above. Each shaft has a collar of radius
// 4.95 (9.9 mm across) that is wider than the 2.5 mm radius bore (5 mm) of its bearing and than the
// carrier's 3 mm bore: the input shaft's collar stands over the front bearing, so the shaft can only
// leave downward once that bearing has gone; the output shaft's collar stands under the rear bearing,
// so it can only leave upward once that bearing has gone. Three planet pins hang from the carrier.
func gearboxSet(t *testing.T) *parts.Set {
	t.Helper()
	doc := decad.New()
	cyl := func(r, z0, h float64) *decad.Body { return cylinderAt(t, doc, 0, 0, r, z0, h) }
	cut := func(b, tool *decad.Body) *decad.Body { return must(decad.Cut(t.Context(), b, tool)) }
	union := func(b, o *decad.Body) *decad.Body { return must(decad.Union(t.Context(), b, o)) }

	ring := cut(cyl(30, 0, 30), cyl(20, -1, 32))
	frontCover := cut(cyl(30, -5, 5), cyl(8, -6, 7))
	frontBearing := cut(cyl(7.9, -5, 5), cyl(2.5, -6, 7))
	inputShaft := union(cyl(2.4, -10, 16), cyl(4.95, 1, 2))
	carrier := cut(cyl(18, 9, 3), cyl(3, 8, 5))
	for k := range 3 {
		a := 2 * math.Pi * float64(k) / 3
		carrier = cut(carrier, cylinderAt(t, doc, 10*math.Cos(a), 10*math.Sin(a), 1.6, 8, 5))
	}
	rearCover := cut(cyl(30, 30, 4), cyl(8, 29, 6))
	rearBearing := cut(cyl(7.9, 30, 4), cyl(2.5, 29, 6))
	outputShaft := union(cyl(2.4, 7, 40), cyl(4.95, 28, 2))

	s := parts.NewSet(doc)
	mustAdd(t, s,
		parts.Part{Name: gbRing, Kind: parts.Manufactured, Body: ring},
		parts.Part{Name: gbFrontCover, Kind: parts.Manufactured, Body: frontCover},
		parts.Part{Name: gbFrontBearing, Kind: parts.Purchased, Body: frontBearing},
		parts.Part{Name: gbInputShaft, Kind: parts.Manufactured, Body: inputShaft},
		parts.Part{Name: gbCarrier, Kind: parts.Manufactured, Body: carrier},
		parts.Part{Name: gbRearBearing, Kind: parts.Purchased, Body: rearBearing},
		parts.Part{Name: gbRearCover, Kind: parts.Manufactured, Body: rearCover},
		parts.Part{Name: gbOutputShaft, Kind: parts.Manufactured, Body: outputShaft})
	for k := range 3 {
		a := 2 * math.Pi * float64(k) / 3
		x, y := 10*math.Cos(a), 10*math.Sin(a)
		pin := union(cylinderAt(t, doc, x, y, 1.5, 5, 8), cylinderAt(t, doc, x, y, 2.5, 12, 1))
		mustAdd(t, s, parts.Part{Name: pinName(k), Kind: parts.Purchased, Body: pin})
	}
	return s
}

func pinName(k int) string { return "planet pin " + string(rune('1'+k)) }

func TestDerivedGearbox(t *testing.T) {
	s := gearboxSet(t)
	ex, err := s.Explode(t.Context(), parts.WithDerivedExplode(), parts.WithExplodeGround(gbRing))
	require.NoError(t, err)
	p := func(name string) parts.ExplodedPart { return partByName(t, ex, name) }

	// Front side: the bearing leaves before the input shaft, both downward.
	require.Less(t, p(gbFrontBearing).Order, p(gbInputShaft).Order)
	require.Less(t, p(gbFrontBearing).Round, p(gbInputShaft).Round)
	require.InDelta(t, -1, p(gbFrontBearing).Dir.Z, 1e-9)
	require.InDelta(t, -1, p(gbInputShaft).Dir.Z, 1e-9)
	require.Less(t, p(gbInputShaft).Offset.Z, p(gbFrontBearing).Offset.Z, "the shaft ends beyond the bearing")
	require.Less(t, p(gbFrontBearing).Offset.Z, 0.0)

	// Rear side: the bearing leaves before the output shaft, both upward.
	require.Less(t, p(gbRearBearing).Order, p(gbOutputShaft).Order)
	require.Less(t, p(gbRearBearing).Round, p(gbOutputShaft).Round)
	require.InDelta(t, 1, p(gbRearBearing).Dir.Z, 1e-9)
	require.InDelta(t, 1, p(gbOutputShaft).Dir.Z, 1e-9)
	require.Greater(t, p(gbOutputShaft).Offset.Z, p(gbRearBearing).Offset.Z)
	require.Greater(t, p(gbRearBearing).Offset.Z, 0.0)

	// Nothing moves sideways, and the ground stays.
	for _, part := range ex.Parts {
		require.InDelta(t, 0, part.Offset.X, 1e-9, part.Name)
		require.InDelta(t, 0, part.Offset.Y, 1e-9, part.Name)
	}
	require.Equal(t, r3.Vec{}, p(gbRing).Offset)

	// The report names the pairs that forced the two orders, with the collar as the witness.
	for _, w := range []struct {
		mover, blocker string
		z              float64
	}{{gbInputShaft, gbFrontBearing, -1}, {gbOutputShaft, gbRearBearing, 1}} {
		pair, ok := findBlocked(ex, w.mover, w.blocker)
		require.True(t, ok, "%s blocked by %s", w.mover, w.blocker)
		require.InDelta(t, w.z, pair.Dir.Z, 1e-9)
		require.Greater(t, pair.Height.Base(), 0.0)
		require.Greater(t, math.Hypot(pair.Point.X, pair.Point.Y), 2.4, "the witness is on the collar, outside the neck")
		require.Less(t, math.Hypot(pair.Point.X, pair.Point.Y), 4.96)
	}
	// The planet pins are one shape in three places.
	for k := range 3 {
		require.InDelta(t, 1, p(pinName(k)).Dir.Z, 1e-9, pinName(k))
		require.Less(t, p(pinName(k)).Order, p(gbCarrier).Order, pinName(k))
	}
}

// Blocking by boxes alone cannot order the shafts: each shaft's box overlaps its bearing's in both senses.
func TestDerivedGearboxNeedsTheTessellation(t *testing.T) {
	ex, err := gearboxSet(t).Explode(t.Context(), parts.WithDerivedExplode(), parts.WithExplodeGround(gbRing),
		parts.BoxOnlyBlocking())
	if err != nil {
		require.ErrorIs(t, err, decad.ErrDegenerate)
		return
	}
	bearing, shaft := partByName(t, ex, gbFrontBearing), partByName(t, ex, gbInputShaft)
	ordered := bearing.Order < shaft.Order && bearing.Round < shaft.Round && shaft.Dir.Z < -0.99 && bearing.Dir.Z < -0.99
	require.False(t, ordered, "box-only blocking must not reproduce the collar order")
}
