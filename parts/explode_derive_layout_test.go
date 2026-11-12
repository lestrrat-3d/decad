package parts_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

const (
	psRing         = "ring"
	psFrontCover   = "front cover"
	psFrontBearing = "front bearing"
	psInputShaft   = "input shaft"
	psCarrier      = "carrier"
	psRearCover    = "rear cover"
	psRearBearing  = "rear bearing"
	psGap          = 4.0
)

func psPlanet(k int) string { return fmt.Sprintf("planet %d", k+1) }
func psBolt(k int) string   { return fmt.Sprintf("bolt %d", k+1) }

// planetStack is a planetary stack along Z with the ring (z 0..30, bore 20 above a shoulder of bore 17
// that ends at z 9) as the ground. Below it a front cover (bore 8) holds a front bearing (bore 2.5); a
// stepped input shaft (neck radius 2.4, collar radius 4.95 at z 1..3) passes through both, its collar over
// the bearing and under the carrier (z 9..12, bore 3) that rests on the shoulder. Three identical planets
// (radius 8 at radius 11, z 13..21) sit on the carrier. Above, a rear cover (z 30..34, bore 8) holds a
// rear bearing joined to it and is held by three identical bolts whose heads stand on it.
//
// The removal order this gives: the bolts, the rear bearing, the front cover and the front bearing
// first; then the rear cover and the input shaft; then the planets; then the carrier.
func planetStack(t *testing.T) (*parts.Set, map[string]*decad.Body) {
	t.Helper()
	doc := decad.New()
	cyl := func(r, z0, h float64) *decad.Body { return cylinderAt(t, doc, 0, 0, r, z0, h) }
	cut := func(b, tool *decad.Body) *decad.Body { return must(decad.Cut(t.Context(), b, tool)) }
	union := func(b, o *decad.Body) *decad.Body { return must(decad.Union(t.Context(), b, o)) }
	at := func(k int, radius, offset float64) (float64, float64) {
		a := offset + 2*math.Pi*float64(k)/3
		return radius * math.Cos(a), radius * math.Sin(a)
	}

	ring := cut(cut(cyl(30, 0, 30), cyl(20, 9, 22)), cyl(17, -1, 11))
	rearCover := cut(cyl(30, 30, 4), cyl(8, 29, 6))
	bodies := map[string]*decad.Body{
		psFrontCover:   cut(cyl(30, -5, 5), cyl(8, -6, 7)),
		psFrontBearing: cut(cyl(7.9, -5, 5), cyl(2.5, -6, 7)),
		psInputShaft:   union(cyl(2.4, -12, 20), cyl(4.95, 1, 2)),
		psCarrier:      cut(cyl(18, 9, 3), cyl(3, 8, 5)),
		psRearBearing:  cut(cyl(7.9, 30, 4), cyl(2.5, 29, 6)),
	}
	for k := range 3 {
		x, y := at(k, 25, math.Pi/3)
		ring = cut(ring, cylinderAt(t, doc, x, y, 1.6, -1, 32))
		rearCover = cut(rearCover, cylinderAt(t, doc, x, y, 1.6, 29, 6))
		bodies[psBolt(k)] = union(cylinderAt(t, doc, x, y, 1.5, 22, 12), cylinderAt(t, doc, x, y, 2.5, 34, 2))
		px, py := at(k, 11, 0)
		bodies[psPlanet(k)] = cylinderAt(t, doc, px, py, 8, 13, 8)
	}
	bodies[psRing], bodies[psRearCover] = ring, rearCover

	s := parts.NewSet(doc)
	names := []string{psRing, psFrontCover, psFrontBearing, psInputShaft, psCarrier, psRearCover, psRearBearing}
	for k := range 3 {
		names = append(names, psPlanet(k), psBolt(k))
	}
	for _, name := range names {
		mustAdd(t, s, parts.Part{Name: name, Kind: parts.Manufactured, Body: bodies[name]})
	}
	mustJoin(t, s,
		parts.Joint{Name: "rear bearing~rear cover", Kind: parts.PinInBore, A: psRearBearing, B: psRearCover, Motion: parts.Fixed},
		parts.Joint{Name: "input shaft~front bearing", Kind: parts.PinInBore, A: psInputShaft, B: psFrontBearing, Motion: parts.Turns})
	return s, bodies
}

// TestDerivedLayoutOrder checks the layout built from a derived removal order: the round removed last
// sits nearest the assembly and each earlier round beyond it, a part never stays inside a joined part
// it was removed past, and identical parts of one round travel the same distance.
func TestDerivedLayoutOrder(t *testing.T) {
	s, bodies := planetStack(t)
	ex, err := s.Explode(t.Context(), parts.WithDerivedExplode(), parts.WithExplodeGround(psRing),
		parts.WithExplodeGap(units.Millimeters(psGap)))
	require.NoError(t, err)
	p := func(name string) parts.ExplodedPart { return partByName(t, ex, name) }
	final := func(name string) decad.Box {
		b, err := bodies[name].Bounds()
		require.NoError(t, err)
		off := p(name).Offset
		return decad.Box{Min: b.Min.Add(off), Max: b.Max.Add(off)}
	}

	// The removal order the layout is built from.
	wantRound := map[string]int{psFrontCover: 1, psFrontBearing: 1, psRearBearing: 1, psRearCover: 2, psInputShaft: 2,
		psCarrier: 4}
	for k := range 3 {
		wantRound[psBolt(k)], wantRound[psPlanet(k)] = 1, 3
	}
	for name, r := range wantRound {
		require.Equal(t, r, p(name).Round, name)
		require.InDelta(t, 1, math.Abs(p(name).Dir.Z), 1e-9, name)
	}

	t.Run("last removed round sits nearest", func(t *testing.T) {
		// Two parts leaving the same way whose boxes overlap seen along the way: the part removed earlier
		// ends wholly beyond the part removed later.
		for _, a := range ex.Parts {
			for _, b := range ex.Parts {
				if a.Round == 0 || b.Round == 0 || a.Round >= b.Round || a.Dir.Z*b.Dir.Z <= 0 {
					continue
				}
				fa, fb := final(a.Name), final(b.Name)
				if fa.Min.X >= fb.Max.X || fb.Min.X >= fa.Max.X || fa.Min.Y >= fb.Max.Y || fb.Min.Y >= fa.Max.Y {
					continue
				}
				if a.Dir.Z > 0 {
					require.GreaterOrEqual(t, fa.Min.Z, fb.Max.Z-1e-6, "%s (round %d) beyond %s (round %d)", a.Name, a.Round,
						b.Name, b.Round)
					continue
				}
				require.LessOrEqual(t, fa.Max.Z, fb.Min.Z+1e-6, "%s (round %d) beyond %s (round %d)", a.Name, a.Round,
					b.Name, b.Round)
			}
		}
		// The carrier leaves last: it clears the ring, the only part still present, by the gap and no more.
		require.InDelta(t, 30+psGap-9, p(psCarrier).Offset.Z, 1e-6)
		// The input shaft clears the ring and the parts removed after it, not the front parts removed before it.
		require.InDelta(t, -(8 + psGap), p(psInputShaft).Offset.Z, 1e-6)
	})

	t.Run("a part removed before a joined part ends beyond it", func(t *testing.T) {
		rb, rc := final(psRearBearing), final(psRearCover)
		require.GreaterOrEqual(t, rb.Min.Z, rc.Max.Z+psGap-1e-6, "the rear bearing ends above the rear cover")
		fb, sh := final(psFrontBearing), final(psInputShaft)
		require.LessOrEqual(t, fb.Max.Z, sh.Min.Z-psGap+1e-6, "the front bearing ends below the input shaft")
	})

	t.Run("identical round members travel equally", func(t *testing.T) {
		for k := 1; k < 3; k++ {
			require.InDelta(t, p(psPlanet(0)).Offset.Z, p(psPlanet(k)).Offset.Z, 1e-9, psPlanet(k))
			require.InDelta(t, p(psBolt(0)).Offset.Z, p(psBolt(k)).Offset.Z, 1e-9, psBolt(k))
		}
		// The planets clear the carrier, which left after them, by the gap.
		require.InDelta(t, final(psCarrier).Max.Z+psGap, final(psPlanet(0)).Min.Z, 1e-6)
	})

	t.Run("nested round members separate", func(t *testing.T) {
		// The front bearing sits in the front cover's bore and both leave downward in one round: one ends
		// beyond the other, and no exploded boxes meet.
		fc, fb := final(psFrontCover), final(psFrontBearing)
		require.True(t, fc.Max.Z <= fb.Min.Z-psGap+1e-6 || fb.Max.Z <= fc.Min.Z-psGap+1e-6,
			"front cover z [%g, %g], front bearing z [%g, %g]", fc.Min.Z, fc.Max.Z, fb.Min.Z, fb.Max.Z)
		require.Empty(t, ex.Overlaps)
	})
}
