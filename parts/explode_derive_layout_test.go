package parts_test

import (
	"fmt"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
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
// The removal order this gives: the bolts and the front bearing first; then the rear cover, the rear
// bearing and the input shaft; then the planets; then the carrier and the front cover. The rear bearing
// and the front cover wait for nothing, so each leaves just before the first part it blocks: the rear
// bearing before the planets, the front cover, which blocks nothing, last.
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
	wantRound := map[string]int{psFrontCover: 4, psFrontBearing: 1, psRearBearing: 2, psRearCover: 2, psInputShaft: 2,
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
		// The front cover, removed last as well, clears the ring by the gap. The input shaft clears the front
		// cover removed after it, not the front bearing removed before it.
		require.InDelta(t, -psGap, p(psFrontCover).Offset.Z, 1e-6)
		require.InDelta(t, final(psFrontCover).Min.Z-psGap, final(psInputShaft).Max.Z, 1e-6)
	})

	t.Run("a part removed before a joined part ends beyond it", func(t *testing.T) {
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
		// The rear bearing sits in the rear cover's bore and both leave upward in one round: one ends beyond
		// the other, and no exploded boxes meet.
		rc, rb := final(psRearCover), final(psRearBearing)
		require.True(t, rc.Max.Z <= rb.Min.Z-psGap+1e-6 || rb.Max.Z <= rc.Min.Z-psGap+1e-6,
			"rear cover z [%g, %g], rear bearing z [%g, %g]", rc.Min.Z, rc.Max.Z, rb.Min.Z, rb.Max.Z)
		require.Empty(t, ex.Overlaps)
	})
}

// TestDerivedPinsBesideCarrier checks that a part with a free way out from the start still leaves just
// before the part it blocks. The ring (bore 20 above a shoulder of bore 17 that ends at z 9) is the ground
// and the carrier (radius 18, z 9..12) rests on the shoulder. Three pins (shank radius 1.5 through the
// carrier's 1.6 holes, head radius 2.5 on its top face) stand in the carrier's way. The rear cover above
// (z 30..34) is an open frame: its bore of 14 passes the pins (out to radius 12.5) but not the carrier,
// and a bolt whose head stands on the cover holds it. The pins could leave at once, but they belong with
// the round just before the carrier, so they end next to it and not beyond the cover.
func TestDerivedPinsBesideCarrier(t *testing.T) {
	const (
		ring, carrier, cover, bolt = "ring", "carrier", "rear cover", "bolt"
	)
	pin := func(k int) string { return fmt.Sprintf("pin %d", k+1) }
	doc := decad.New()
	cyl := func(x, y, r, z0, h float64) *decad.Body { return cylinderAt(t, doc, x, y, r, z0, h) }
	cut := func(b, tool *decad.Body) *decad.Body { return must(decad.Cut(t.Context(), b, tool)) }
	union := func(b, o *decad.Body) *decad.Body { return must(decad.Union(t.Context(), b, o)) }

	bodies := map[string]*decad.Body{
		ring:    cut(cut(cut(cyl(0, 0, 30, 0, 30), cyl(0, 0, 20, 9, 22)), cyl(0, 0, 17, -1, 11)), cyl(25, 0, 1.6, -1, 32)),
		cover:   cut(cut(cyl(0, 0, 30, 30, 4), cyl(0, 0, 14, 29, 6)), cyl(25, 0, 1.6, 29, 6)),
		bolt:    union(cyl(25, 0, 1.5, 22, 12), cyl(25, 0, 2.5, 34, 2)),
		carrier: cyl(0, 0, 18, 9, 3),
	}
	for k := range 3 {
		a := math.Pi/2 + 2*math.Pi*float64(k)/3
		x, y := 10*math.Cos(a), 10*math.Sin(a)
		bodies[carrier] = cut(bodies[carrier], cyl(x, y, 1.6, 8, 5))
		bodies[pin(k)] = union(cyl(x, y, 1.5, 5, 8), cyl(x, y, 2.5, 12, 1))
	}
	s := parts.NewSet(doc)
	names := []string{ring, carrier, cover, bolt, pin(0), pin(1), pin(2)}
	for _, name := range names {
		mustAdd(t, s, parts.Part{Name: name, Kind: parts.Manufactured, Body: bodies[name]})
	}
	ex, err := s.Explode(t.Context(), parts.WithDerivedExplode(), parts.WithExplodeGround(ring),
		parts.WithExplodeGap(units.Millimeters(psGap)))
	require.NoError(t, err)
	p := func(name string) parts.ExplodedPart { return partByName(t, ex, name) }
	final := func(name string) decad.Box {
		b, err := bodies[name].Bounds()
		require.NoError(t, err)
		off := p(name).Offset
		return decad.Box{Min: b.Min.Add(off), Max: b.Max.Add(off)}
	}
	for _, name := range names[1:] {
		require.InDelta(t, 1, p(name).Dir.Z, 1e-9, name)
	}

	// The bolt holds the cover and the cover the carrier; each pin leaves with the cover, one round before
	// the carrier it stands in the way of.
	require.Equal(t, 1, p(bolt).Round)
	require.Equal(t, 2, p(cover).Round)
	require.Equal(t, 3, p(carrier).Round)
	for k := range 3 {
		require.Equal(t, p(carrier).Round-1, p(pin(k)).Round, pin(k))
	}

	// The carrier, removed last, clears the ring by the gap. Each pin clears the carrier by the gap and no
	// more; the cover, which overlaps the pins seen along the axis, ends beyond them.
	require.InDelta(t, 30+psGap-9, p(carrier).Offset.Z, 1e-6)
	for k := range 3 {
		require.InDelta(t, final(carrier).Max.Z+psGap, final(pin(k)).Min.Z, 1e-6, pin(k))
		require.GreaterOrEqual(t, final(cover).Min.Z, final(pin(k)).Max.Z+psGap-1e-6, pin(k))
	}
	require.Empty(t, ex.Overlaps)
}

// TestDerivedCopiesAroundHubTravelEqually checks that identical parts placed around a hub in one round
// travel one distance. A hub (radius 6, z 0..8) stands on a base plate with three identical rings
// (radius 8, bore 3, z 1..7) at radius 15 around it, the first on the X axis, 1 mm clear of the hub. All
// four leave upward in one round and the hub goes first. Seen along Z the hub's box overlaps the boxes
// of the rings at 120 and 240 degrees at their corners but not the box of the ring on the X axis, so
// each ring alone would end at a different height. As copies of one shape all three end beyond the hub.
func TestDerivedCopiesAroundHubTravelEqually(t *testing.T) {
	const gap = 6.0
	ring := func(k int) string { return fmt.Sprintf("ring %d", k+1) }
	doc := decad.New()
	cyl := func(x, y, r, z0, h float64) *decad.Body { return cylinderAt(t, doc, x, y, r, z0, h) }
	cut := func(b, tool *decad.Body) *decad.Body { return must(decad.Cut(t.Context(), b, tool)) }
	bodies := map[string]*decad.Body{namePlate: cyl(0, 0, 30, -5, 5), "hub": cyl(0, 0, 6, 0, 8)}
	for k := range 3 {
		a := 2 * math.Pi * float64(k) / 3
		x, y := 15*math.Cos(a), 15*math.Sin(a)
		bodies[ring(k)] = cut(cyl(x, y, 8, 1, 6), cyl(x, y, 3, 0, 8))
	}
	s := parts.NewSet(doc)
	up := parts.Removal{Direction: r3.Vec{Z: 1}}
	opts := []parts.ExplodeOption{parts.WithDerivedExplode(), parts.WithExplodeGround(namePlate),
		parts.WithExplodeGap(units.Millimeters(gap)), parts.WithRemoval("hub", up)}
	for _, name := range []string{namePlate, "hub", ring(0), ring(1), ring(2)} {
		mustAdd(t, s, parts.Part{Name: name, Kind: parts.Manufactured, Body: bodies[name]})
		if strings.HasPrefix(name, "ring") {
			opts = append(opts, parts.WithRemoval(name, up))
		}
	}
	ex, err := s.Explode(t.Context(), opts...)
	require.NoError(t, err)
	p := func(name string) parts.ExplodedPart { return partByName(t, ex, name) }
	final := func(name string) decad.Box {
		b, err := bodies[name].Bounds()
		require.NoError(t, err)
		off := p(name).Offset
		return decad.Box{Min: b.Min.Add(off), Max: b.Max.Add(off)}
	}

	require.Equal(t, 1, p("hub").Round)
	require.InDelta(t, gap, p("hub").Offset.Z, 1e-6, "the hub clears the plate by the gap")
	for k := range 3 {
		require.Equal(t, 1, p(ring(k)).Round, ring(k))
		require.InDelta(t, 0, p(ring(k)).Offset.Sub(p(ring(0)).Offset).Len(), 1e-6, ring(k))
		require.InDelta(t, final("hub").Max.Z+gap, final(ring(k)).Min.Z, 1e-6, "%s ends beyond the hub", ring(k))
	}
	require.Empty(t, ex.Overlaps)
}
