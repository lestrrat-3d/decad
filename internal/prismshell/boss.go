package prismshell

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type circleSeg = sectionrecord.CircleSeg

type curveSegment = sectionrecord.CurveSegment

type lineSeg = sectionrecord.LineSeg

type loopRecord = sectionrecord.LoopRecord

// planePoint is the evaluator's plane-local coordinate. Point2 is its public
// spelling; both share one underlying struct, so a value assigns either way.
type planePoint = sectionrecord.Point2

type profileRecord = momentinput.Profile

// PocketShellBand records the rounded transition below an exact rectangular
// blind pocket or across a coaxial counterbore shoulder. A blind pocket has
// an outward-rounded side ring and a square cap ring; its four circular side
// walks collapse to sphere poles. A counterbore has a quarter-torus side ring
// and a circular cap ring.
type PocketShellBand struct {
	Outer, Pocket, Inner BossRect
	Round                *CounterboreShellRound
	Rounded              profileRecord
	Z0, FloorZ, TopZ     float64
	LowerZ, T            float64
	FloorFace            int
	Frame                r3.Frame
}

// CapProfile returns the profile of the band's cap ring.
func (b *PocketShellBand) CapProfile() profileRecord {
	if b.Round != nil {
		return b.Round.Large.Outer
	}
	return b.Pocket.Profile()
}

// CapOutward reports whether the cap ring rounds outward.
func (b *PocketShellBand) CapOutward() bool {
	return b.Round == nil || b.Round.Sign > 0
}

// CounterboreShellRound is the source-authenticated two-level through bore.
// Its shell uses PocketShellBand's whole-circle fillet between the larger
// bore's shoulder and the new cavity wall. Sign points from the kept far cap
// toward the removed cap in the producer's frame.
type CounterboreShellRound struct {
	Small, Large       RoundBossCircle
	Plate, Inset       BossRect
	FarZ, StepZ, NearZ float64
	Sign, T            float64
	Center             planePoint
	Frame              r3.Frame
}

// CounterboreHoleCircle reads the single hole of a profile as a whole circle.
func CounterboreHoleCircle(ctx context.Context, p profileRecord) (RoundBossCircle, bool, error) {
	if len(p.Holes) != 1 {
		return RoundBossCircle{}, false, nil
	}
	loop, err := offset2d.ReverseLoopRecordContext(ctx, p.Holes[0])
	if err != nil {
		return RoundBossCircle{}, false, err
	}
	c, ok := RecordedBossCircle(profileRecord{Outer: loop})
	return c, ok, nil
}

// PrismUnionBossShellCorners returns the rectangle's four corners in order.
func PrismUnionBossShellCorners(r BossRect) [4][2]float64 {
	return [4][2]float64{{r.U0, r.V0}, {r.U1, r.V0}, {r.U1, r.V1}, {r.U0, r.V1}}
}

// RoundBossCircle is the exact whole-circle variant of BossShellBand's two rings.
// The plate remains rectangular; the single transition is a quarter torus.
type RoundBossCircle struct {
	Outer, Inner profileRecord
	Center       planePoint
	Radius       float64
}

// RecordedBossCircle accepts a complete counter-clockwise whole circle only.
func RecordedBossCircle(p profileRecord) (RoundBossCircle, bool) {
	if len(p.Holes) != 0 || len(p.Outer.Segments) != 1 {
		return RoundBossCircle{}, false
	}
	c, ok := p.Outer.Segments[0].(circleSeg)
	if !ok || !c.CCW || c.TStart != 0 || c.TEnd != 1 {
		return RoundBossCircle{}, false
	}
	r, delta, err := extent.MagnitudeInBounded(c.Radius, units.Length, units.Millimeter, "boss radius")
	if err != nil || delta != 0 || !(r > 0) || proofbound.IsNonFinite(r) ||
		proofbound.IsNonFinite(c.Center.U) || proofbound.IsNonFinite(c.Center.V) {
		return RoundBossCircle{}, false
	}
	return RoundBossCircle{Outer: p, Center: c.Center, Radius: r}, true
}

// BossShellBand records the one rounded cavity transition of an inward shell
// on a two-slab rectangular boss. The two open rings belong to the inner
// ledge at LowerZ and the upper cavity walls at InterfaceZ.
type BossShellBand struct {
	Plate, Boss, Lower, Upper                BossRect
	Round                                    *RoundBossCircle
	Z0, InterfaceZ, TopZ, CavityTopZ, LowerZ float64
	T                                        float64
	LedgeFace                                int
	OuterFaceCount                           int
	Closed                                   bool
	Frame                                    r3.Frame
}

// SideProfile returns the profile of the band's side ring.
func (b *BossShellBand) SideProfile() profileRecord {
	if b.Round != nil {
		return b.Round.Outer
	}
	return b.Boss.Profile()
}

// CapProfile returns the profile of the band's cap ring.
func (b *BossShellBand) CapProfile() profileRecord {
	if b.Round != nil {
		return b.Round.Inner
	}
	return b.Upper.Profile()
}

// BossRect is an axis-aligned rectangle in the plane-local frame.
type BossRect struct{ U0, V0, U1, V1 float64 }

// Width returns the extent along U.
func (r BossRect) Width() float64 { return r.U1 - r.U0 }

// Height returns the extent along V.
func (r BossRect) Height() float64 { return r.V1 - r.V0 }

// Inset returns the rectangle moved inward by t on every side.
func (r BossRect) Inset(t float64) BossRect {
	return BossRect{r.U0 + t, r.V0 + t, r.U1 - t, r.V1 - t}
}

// Profile returns the rectangle as a counter-clockwise recorded profile.
func (r BossRect) Profile() profileRecord {
	pts := [4]planePoint{{U: r.U0, V: r.V0}, {U: r.U1, V: r.V0}, {U: r.U1, V: r.V1}, {U: r.U0, V: r.V1}}
	segs := make([]curveSegment, 4)
	for i := range segs {
		segs[i] = lineSeg{Start: pts[i], End: pts[(i+1)%4], TEnd: 1}
	}
	return profileRecord{Outer: loopRecord{Segments: segs}}
}

// RecordedBossRect accepts a complete, counter-clockwise axis-aligned
// rectangle only. It reads the producer's segments, never a sampled outline.
func RecordedBossRect(p profileRecord) (BossRect, bool) {
	if len(p.Holes) != 0 || len(p.Outer.Segments) != 4 {
		return BossRect{}, false
	}
	var r BossRect
	var pts [4]planePoint
	for i, seg := range p.Outer.Segments {
		line, ok := seg.(lineSeg)
		if !ok || line.TStart != 0 || line.TEnd != 1 {
			return BossRect{}, false
		}
		if (line.Start.U == line.End.U) == (line.Start.V == line.End.V) {
			return BossRect{}, false
		}
		pts[i] = line.Start
		if i > 0 {
			prev, ok := p.Outer.Segments[i-1].(lineSeg)
			if !ok || prev.End != line.Start {
				return BossRect{}, false
			}
		}
		if i == 0 {
			r = BossRect{line.Start.U, line.Start.V, line.Start.U, line.Start.V}
		}
		r.U0, r.U1 = math.Min(r.U0, line.Start.U), math.Max(r.U1, line.Start.U)
		r.V0, r.V1 = math.Min(r.V0, line.Start.V), math.Max(r.V1, line.Start.V)
		if i == 3 && line.End != pts[0] {
			return BossRect{}, false
		}
	}
	if !(r.Width() > 0 && r.Height() > 0) {
		return BossRect{}, false
	}
	seen := map[planePoint]bool{}
	for _, p := range pts {
		if seen[p] || !((p.U == r.U0 || p.U == r.U1) && (p.V == r.V0 || p.V == r.V1)) {
			return BossRect{}, false
		}
		seen[p] = true
	}
	a, b := pts[0], pts[1]
	c := pts[2]
	cross := (b.U-a.U)*(c.V-b.V) - (b.V-a.V)*(c.U-b.U)
	return r, cross > 0 && !proofbound.IsNonFinite(r.Width()) && !proofbound.IsNonFinite(r.Height())
}
